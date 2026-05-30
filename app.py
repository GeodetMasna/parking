# -*- coding: utf-8 -*-
"""
Generátor parkovacích stání z DXF (ČSN 73 6056)
================================================
Flask backend. Přijme DXF se zakreslenou uzavřenou křivkou (obrys plochy),
vygeneruje dovnitř obdélníky / kosodélníky stání, odfiltruje ty, které
přečnívají mimo obrys, a vrátí modifikované DXF ke stažení.

Endpointy:
    GET  /            – health check
    POST /generate    – multipart/form-data: file=<dxf>, + parametry

Spuštění lokálně:  python app.py
Spuštění produkce:  gunicorn app:app --bind 0.0.0.0:$PORT
"""

import io
import math
import os
import tempfile

import ezdxf
from ezdxf import bbox as ezbbox
from flask import Flask, request, send_file, jsonify
from flask_cors import CORS
from shapely.affinity import rotate as shp_rotate
from shapely.geometry import Polygon
from shapely.prepared import prep

app = Flask(__name__)

# --- CORS -------------------------------------------------------------------
# Frontend běží z file:// (lokální index.html), prohlížeč proto posílá
# hlavičku "Origin: null". Wildcard "*" tuto situaci pokrývá i pro null origin.
CORS(app, resources={r"/*": {"origins": "*"}})

# Vrstva, na kterou se kreslí vygenerovaná stání
STALL_LAYER = "PARKING_STANI"
STALL_LAYER_COLOR = 3  # zelená (ACI)

# ---------------------------------------------------------------------------
#  ČSN 73 6056 (rev. 2011) – tabulkové rozměry pro osobní vozidla [m]
#  width = šířka stání MĚŘENÁ PODÉL komunikace (rozteč),
#  length = délka stání = KOLMÁ hloubka řady,
#  aisle = šířka komunikace (jízdního pruhu mezi řadami).
#  (Podélné stání: width = šířka kolmo k vozovce, length = délka podél vozovky.)
# ---------------------------------------------------------------------------
CSN_PRESETS = {
    "kolme":   {"angle": 90, "width": 2.50, "length": 5.00, "aisle": 6.00},
    "sikme75": {"angle": 75, "width": 2.60, "length": 5.30, "aisle": 5.00},
    "sikme60": {"angle": 60, "width": 2.90, "length": 5.20, "aisle": 3.50},
    "sikme45": {"angle": 45, "width": 3.55, "length": 4.80, "aisle": 3.00},
    # Podélné: hodnoty běžně uváděné, ověř proti normě (krajní stání bývá delší).
    "podelne": {"angle": 0,  "width": 2.00, "length": 5.75, "aisle": 3.50},
}


# ---------------------------------------------------------------------------
#  Načtení obrysu z DXF
# ---------------------------------------------------------------------------
def _polyline_points(entity):
    """Vrátí seznam (x, y) bodů z LWPOLYLINE / POLYLINE, pokud je uzavřená."""
    dxftype = entity.dxftype()
    if dxftype == "LWPOLYLINE":
        if not entity.closed:
            return None
        return [(p[0], p[1]) for p in entity.get_points("xy")]
    if dxftype == "POLYLINE":
        if not entity.is_closed:
            return None
        return [(v.dxf.location.x, v.dxf.location.y) for v in entity.vertices]
    return None


def extract_boundary(doc, layer=None):
    """
    Vybere zpracovávanou uzavřenou křivku.
      - Pokud je zadán `layer`, hledá pouze na této vrstvě.
      - Jinak vybere uzavřenou křivku s největší plochou.
    Vrací (shapely.Polygon, počet_kandidátů).
    """
    msp = doc.modelspace()
    candidates = []
    for e in msp:
        if e.dxftype() not in ("LWPOLYLINE", "POLYLINE"):
            continue
        if layer and e.dxf.layer != layer:
            continue
        pts = _polyline_points(e)
        if not pts or len(pts) < 3:
            continue
        try:
            poly = Polygon(pts)
        except Exception:
            continue
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty or poly.area <= 0:
            continue
        candidates.append(poly)

    if not candidates:
        return None, 0
    # největší plocha = obrys parkoviště
    candidates.sort(key=lambda p: p.area, reverse=True)
    return candidates[0], len(candidates)


# ---------------------------------------------------------------------------
#  Generování stání
# ---------------------------------------------------------------------------
def _row_starts(y_min, y_max, row_depth, aisle):
    """
    Pozice začátků řad ve směru hloubky.
    Vzor:  řada – ulička – řada – řada(zády k sobě) – ulička – ...
    To odpovídá běžnému dvoupruhovému modulu parkoviště.
    """
    starts = []
    y = y_min
    gap_is_aisle = True
    eps = 1e-9
    while y + row_depth <= y_max + eps:
        starts.append(y)
        y += row_depth + (aisle if gap_is_aisle else 0.0)
        gap_is_aisle = not gap_is_aisle
    return starts


def _stall_polygons_in_frame(work_poly, pitch, depth, aisle, angle_deg, parallel):
    """
    Vygeneruje stání v pracovním (osově zarovnaném) systému dle konvence ČSN 73 6056:
        pitch = šířka stání měřená PODÉL komunikace (rozteč mezi stáními),
        depth = délka stání = KOLMÁ hloubka řady (od hrany komunikace k obrubníku),
        angle = úhel řazení (90° kolmé; 45/60/75° šikmé; podélné -> obdélník).

    Šikmé stání je rovnoběžník se svislou hloubkou `depth` a vodorovným posunutím
    horní hrany run = depth / tan(angle). Kolmá světlá šířka stání pak vychází
    pitch · sin(angle) ≈ 2,5 m, což odpovídá normě.
    Vrací seznam shapely.Polygon (kandidáti, ještě nefiltrováno).
    """
    minx, miny, maxx, maxy = work_poly.bounds
    stalls = []

    if parallel:
        run = 0.0  # podélné = obdélník (vůz rovnoběžně s komunikací)
    else:
        theta = math.radians(angle_deg)
        run = 0.0 if abs(theta - math.pi / 2) < 1e-6 else depth / math.tan(theta)

    for y0 in _row_starts(miny, maxy, depth, aisle):
        x = minx
        # poslední stání v řadě se nesmí dostat za pravý okraj rastru
        while x + pitch + max(run, 0.0) <= maxx + 1e-9:
            stalls.append(Polygon([
                (x, y0),
                (x + pitch, y0),
                (x + pitch + run, y0 + depth),
                (x + run, y0 + depth),
            ]))
            x += pitch
    return stalls


def generate_stalls(boundary, pitch, depth, aisle, angle_deg, parallel):
    """
    Hlavní generátor.
      1. Zarovná generační rastr s nejdelší hranou obrysu (lepší vyplnění).
      2. Vygeneruje kandidáty stání.
      3. Ponechá pouze ta, která leží zcela uvnitř obrysu.
    Vrací seznam stání jako seznamy (x, y) bodů ve světových souřadnicích.
    """
    centroid = boundary.centroid

    # Orientace nejmenšího opsaného obdélníku => natočení rastru
    mrr = boundary.minimum_rotated_rectangle
    coords = list(mrr.exterior.coords)
    edges = [(coords[i], coords[i + 1]) for i in range(len(coords) - 1)]
    longest = max(edges, key=lambda e: math.dist(e[0], e[1]))
    phi = math.degrees(math.atan2(longest[1][1] - longest[0][1],
                                  longest[1][0] - longest[0][0]))

    # Pracovní soustava: obrys natočíme o -phi kolem těžiště
    work_poly = shp_rotate(boundary, -phi, origin=centroid)
    candidates = _stall_polygons_in_frame(
        work_poly, pitch, depth, aisle, angle_deg, parallel
    )

    # Filtr přečnívajících – stání musí ležet celé uvnitř (s malou tolerancí)
    test_poly = work_poly.buffer(-min(pitch, depth) * 1e-3)
    if test_poly.is_empty:
        test_poly = work_poly
    prepared = prep(test_poly)

    kept = []
    for stall in candidates:
        if prepared.contains(stall):
            world = shp_rotate(stall, phi, origin=centroid)
            kept.append(list(world.exterior.coords)[:-1])
    return kept


# ---------------------------------------------------------------------------
#  Zápis stání do DXF
# ---------------------------------------------------------------------------
def add_stalls_to_doc(doc, stalls):
    if STALL_LAYER not in doc.layers:
        doc.layers.add(STALL_LAYER, color=STALL_LAYER_COLOR)
    msp = doc.modelspace()
    for pts in stalls:
        msp.add_lwpolyline(
            pts, format="xy", close=True,
            dxfattribs={"layer": STALL_LAYER},
        )


# ---------------------------------------------------------------------------
#  Endpointy
# ---------------------------------------------------------------------------
@app.get("/")
def health():
    return jsonify({
        "status": "ok",
        "service": "DXF parking generator (ČSN 73 6056)",
        "endpoint": "POST /generate",
    })


@app.get("/presets")
def presets():
    """Tabulkové rozměry ČSN 73 6056 pro osobní vozidla."""
    return jsonify(CSN_PRESETS)


@app.post("/generate")
def generate():
    if "file" not in request.files:
        return jsonify({"error": "Chybí soubor DXF (pole 'file')."}), 400

    upload = request.files["file"]
    if not upload.filename:
        return jsonify({"error": "Prázdný název souboru."}), 400

    # Parametry dle ČSN 73 6056 (jednotky = jednotky výkresu; ČSN udává metry)
    #   width  = šířka stání (u kolmého/šikmého měřeno PODÉL komunikace)
    #   length = délka stání (kolmá hloubka řady)
    try:
        width = float(request.form.get("width", 2.5))
        length = float(request.form.get("length", 5.0))
        aisle = float(request.form.get("aisle", 6.0))
        angle = float(request.form.get("angle", 90.0))
    except ValueError:
        return jsonify({"error": "Neplatná číselná hodnota parametru."}), 400

    typ = request.form.get("type", "kolme").lower()        # kolme|sikme*|podelne
    layer = request.form.get("layer") or None              # volitelná vrstva obrysu
    parallel = (typ == "podelne")
    if typ == "kolme":
        angle = 90.0

    # Mapování ČSN šířka/délka -> rozteč podél komunikace (pitch) a kolmá hloubka (depth)
    if parallel:
        # podélné: délka jede podél komunikace, šířka je kolmá hloubka
        pitch, depth = length, width
    else:
        # kolmé/šikmé: šířka = rozteč podél komunikace, délka = kolmá hloubka
        pitch, depth = width, length

    # Načtení DXF přes dočasný soubor (kvůli detekci kódování)
    tmp_in = tempfile.NamedTemporaryFile(suffix=".dxf", delete=False)
    try:
        upload.save(tmp_in.name)
        tmp_in.close()
        try:
            doc = ezdxf.readfile(tmp_in.name)
        except (IOError, ezdxf.DXFStructureError) as exc:
            return jsonify({"error": f"Neplatný DXF: {exc}"}), 400

        boundary, n = extract_boundary(doc, layer)
        if boundary is None:
            return jsonify({
                "error": "Nenalezena žádná uzavřená křivka (LWPOLYLINE/POLYLINE)."
                         + (f" na vrstvě '{layer}'." if layer else "")
            }), 422

        stalls = generate_stalls(boundary, pitch, depth, aisle, angle, parallel)
        add_stalls_to_doc(doc, stalls)

        # Uložení výstupu do paměti
        text_stream = io.StringIO()
        doc.write(text_stream)
        out = io.BytesIO(text_stream.getvalue().encode("utf-8"))
        out.seek(0)

        base = os.path.splitext(upload.filename)[0]
        resp = send_file(
            out,
            mimetype="application/dxf",
            as_attachment=True,
            download_name=f"{base}_stani.dxf",
        )
        # Počet stání předáme v hlavičce (frontend ji může přečíst)
        resp.headers["X-Stall-Count"] = str(len(stalls))
        resp.headers["Access-Control-Expose-Headers"] = "X-Stall-Count"
        return resp
    finally:
        try:
            os.unlink(tmp_in.name)
        except OSError:
            pass


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 5000))
    app.run(host="0.0.0.0", port=port, debug=True)
