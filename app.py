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
from collections import Counter

import ezdxf
from ezdxf import bbox as ezbbox
from flask import Flask, request, send_file, jsonify
from flask_cors import CORS
from shapely.affinity import rotate as shp_rotate
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
from shapely.prepared import prep

app = Flask(__name__)

# --- CORS -------------------------------------------------------------------
# Frontend běží z file:// (lokální index.html), prohlížeč proto posílá
# hlavičku "Origin: null". Wildcard "*" tuto situaci pokrývá i pro null origin.
CORS(app, resources={r"/*": {"origins": "*"}})

# Vrstvy, na které se kreslí vygenerovaná stání
STALL_LAYER = "PARKING_STANI"            # jednotný typ
STALL_LAYER_COLOR = 3                     # zelená (ACI)
KOLME_LAYER = "PARKING_KOLME"             # kombi: kolmé jádro
PODELNE_LAYER = "PARKING_PODELNE"         # kombi: podélné okraje
ALL_STALL_LAYERS = [STALL_LAYER, KOLME_LAYER, PODELNE_LAYER]

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
CLOSE_TOL_REL = 1e-3  # relativní tolerance pro geometricky uzavřenou křivku


def _points_from_entity(e):
    """
    Vrátí (body[(x,y)], je_uzavřená) nebo None.
    Podporuje LWPOLYLINE, POLYLINE, CIRCLE, ELLIPSE a uzavřený SPLINE.
    """
    t = e.dxftype()
    try:
        if t == "LWPOLYLINE":
            return [(p[0], p[1]) for p in e.get_points("xy")], bool(e.closed)
        if t == "POLYLINE":
            return [(v.dxf.location.x, v.dxf.location.y) for v in e.vertices], bool(e.is_closed)
        if t == "CIRCLE":
            c, r = e.dxf.center, e.dxf.radius
            n = 72
            return [(c.x + r * math.cos(2 * math.pi * i / n),
                     c.y + r * math.sin(2 * math.pi * i / n)) for i in range(n)], True
        if t == "ELLIPSE":
            return [(p.x, p.y) for p in e.flattening(0.05)], True
        if t == "SPLINE" and e.closed:
            return [(p.x, p.y) for p in e.flattening(0.05)], True
    except Exception:
        return None
    return None


def _iter_entities(doc):
    """Projde modelspace a rozbalí i obsah vložených bloků (INSERT)."""
    for e in doc.modelspace():
        if e.dxftype() == "INSERT":
            try:
                yield from e.virtual_entities()
            except Exception:
                continue
        else:
            yield e


def _is_geom_closed(pts):
    """Křivka je uzavřená, pokud první a poslední bod splývají (rel. tolerance)."""
    if len(pts) < 3:
        return False
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    diag = math.hypot(max(xs) - min(xs), max(ys) - min(ys)) or 1.0
    return math.dist(pts[0], pts[-1]) <= diag * CLOSE_TOL_REL


def extract_boundary(doc, layer=None):
    """
    Vybere zpracovávaný obrys (uzavřenou křivku s největší plochou, příp. na dané
    vrstvě). Akceptuje polylinie uzavřené příznakem i geometricky, CIRCLE, ELLIPSE
    a uzavřený SPLINE; prohledá i obsah bloků. Pokud žádná uzavřená křivka není,
    zkusí uzavřít největší otevřenou polylinii (s upozorněním).
    Vrací (shapely.Polygon | None, diagnostics: dict).
    """
    types = Counter()
    layers = set()
    closed_polys, open_polys = [], []

    for e in _iter_entities(doc):
        ent_layer = getattr(e.dxf, "layer", None)
        if ent_layer in ALL_STALL_LAYERS:
            continue  # přeskoč dříve vygenerovaná stání
        types[e.dxftype()] += 1
        if ent_layer:
            layers.add(ent_layer)
        if layer and ent_layer != layer:
            continue

        res = _points_from_entity(e)
        if not res:
            continue
        pts, is_closed = res
        if len(pts) < 3:
            continue
        try:
            poly = Polygon(pts)
            if not poly.is_valid:
                poly = poly.buffer(0)
        except Exception:
            continue
        if poly.is_empty or poly.area <= 0:
            continue

        (closed_polys if (is_closed or _is_geom_closed(pts)) else open_polys).append(poly)

    diag = {
        "types": dict(types),
        "layers": sorted(layers),
        "closed": len(closed_polys),
        "open": len(open_polys),
        "auto_closed": False,
    }

    if closed_polys:
        closed_polys.sort(key=lambda p: p.area, reverse=True)
        return closed_polys[0], diag
    if open_polys:  # nouzové uzavření největší otevřené polylinie
        open_polys.sort(key=lambda p: p.area, reverse=True)
        diag["auto_closed"] = True
        return open_polys[0], diag
    return None, diag


# ---------------------------------------------------------------------------
#  Generování stání
# ---------------------------------------------------------------------------
def _max_rows(depth_avail, d, a):
    """Max. počet řad v hloubce; uličky = ceil(n/2) (každá řada má přístup k uličce)."""
    n = 0
    while (n + 1) * d + math.ceil((n + 1) / 2) * a <= depth_avail + 1e-9:
        n += 1
    return n


def _row_starts(y_min, y_max, d, a):
    """
    Optimální dvoupruhové uspořádání řad dle ČSN: řada |ulička| řada–řada |ulička| …
    (mezery mezi řadami: a, 0, a, 0, …, a; krajní i poslední řada má přístup k uličce).
    Maximalizuje počet řad a blok vycentruje v dostupné hloubce.
    """
    D = y_max - y_min
    n = _max_rows(D, d, a)
    if n <= 0:
        return []
    gaps = [a if i % 2 == 0 else 0.0 for i in range(n - 1)]
    if gaps:
        gaps[-1] = a  # poslední řada musí mít uličku (přístup)
    starts, y = [], 0.0
    for i in range(n):
        starts.append(y)
        if i < n - 1:
            y += d + gaps[i]
    used = starts[-1] + d
    shift = y_min + (D - used) / 2.0
    return [s + shift for s in starts]


def _layout(work_poly, prepared, pitch, depth, aisle, angle_deg, parallel,
            row_shift, col_shift):
    """Vygeneruje a hned ořeže stání pro jedno konkrétní natočení/posun rastru."""
    minx, miny, maxx, maxy = work_poly.bounds
    if parallel:
        run = 0.0
    else:
        theta = math.radians(angle_deg)
        run = 0.0 if abs(theta - math.pi / 2) < 1e-6 else depth / math.tan(theta)

    kept = []
    for y0 in _row_starts(miny + row_shift, maxy, depth, aisle):
        x = minx + col_shift
        while x + pitch + max(run, 0.0) <= maxx + 1e-9:
            poly = Polygon([
                (x, y0), (x + pitch, y0),
                (x + pitch + run, y0 + depth), (x + run, y0 + depth),
            ])
            if prepared.contains(poly):
                kept.append(poly)
            x += pitch
    return kept


def generate_stalls(boundary, pitch, depth, aisle, angle_deg, parallel):
    """
    Optimalizační generátor – maximalizuje počet stání:
      1. Vyzkouší více NATOČENÍ rastru (hlavní směry obrysu + hrubý sweep po 15°).
      2. Pro každé natočení vyzkouší několik FÁZOVÝCH POSUNŮ řad i sloupců.
      3. Ponechá jen stání ležící celá uvnitř obrysu a vybere nejlepší variantu.
    Vrací seznam stání jako seznamy (x, y) bodů ve světových souřadnicích.
    """
    centroid = boundary.centroid

    # Kandidátní natočení: hlavní směry nejmenšího opsaného obdélníku + sweep
    mrr = boundary.minimum_rotated_rectangle
    cc = list(mrr.exterior.coords)
    edges = [(cc[i], cc[i + 1]) for i in range(len(cc) - 1)]
    longest = max(edges, key=lambda e: math.dist(e[0], e[1]))
    phi0 = math.degrees(math.atan2(longest[1][1] - longest[0][1],
                                   longest[1][0] - longest[0][0]))
    orients = sorted({round((phi0 + 15 * k) % 180, 3) for k in range(12)}
                     | {round((phi0 + 90) % 180, 3)})

    tol = min(pitch, depth) * 1e-3
    period = depth + aisle
    row_shifts = [-period / 2 + period * f for f in (0.0, 0.25, 0.5, 0.75)]
    col_shifts = [pitch * f for f in (0.0, 0.34, 0.67)]

    best_kept, best_ori = [], phi0
    for ori in orients:
        work = shp_rotate(boundary, -ori, origin=centroid)
        shrunk = work.buffer(-tol)
        prepared = prep(shrunk if not shrunk.is_empty else work)
        for rs in row_shifts:
            for cs in col_shifts:
                kept = _layout(work, prepared, pitch, depth, aisle,
                               angle_deg, parallel, rs, cs)
                if len(kept) > len(best_kept):
                    best_kept, best_ori = kept, ori

    out = []
    for poly in best_kept:
        world = shp_rotate(poly, best_ori, origin=centroid)
        out.append(list(world.exterior.coords)[:-1])
    return out


def parallel_along_edges(boundary, occupied, length, depth, clear=0.15, setback=0.03):
    """
    KOMBI: doplní podélná stání podél HRAN obrysu (frontáží) do míst, kde už
    není kolmé jádro. Stání leží dlouhou stranou na hraně (mírně odsazeno
    dovnitř), nepřesahují obrys a nekolidují s `occupied` (kolmé pole + již
    osazená podélná). Vnitřní ulička se tím nezaplňuje – ta není hranou obrysu.
    Pozn.: dojezd k těmto stáním je nutné ručně ověřit.
    Vrací seznam stání jako seznamy (x, y) bodů.
    """
    prepared = prep(boundary)
    coords = list(boundary.exterior.coords)
    out = []
    for i in range(len(coords) - 1):
        ax, ay = coords[i]
        bx, by = coords[i + 1]
        ex, ey = bx - ax, by - ay
        elen = math.hypot(ex, ey)
        if elen < length:
            continue
        ux, uy = ex / elen, ey / elen
        # vnitřní normála hrany
        nrm = None
        for nx, ny in ((-uy, ux), (uy, -ux)):
            if boundary.contains(Point(ax + ux * elen / 2 + nx * 0.5,
                                       ay + uy * elen / 2 + ny * 0.5)):
                nrm = (nx, ny)
                break
        if nrm is None:
            continue
        nx, ny = nrm
        ox, oy = ax + nx * setback, ay + ny * setback  # odsazení od hrany
        k = 0
        while (k + 1) * length <= elen + 1e-9:
            x0 = ox + ux * (k * length)
            y0 = oy + uy * (k * length)
            x1, y1 = x0 + ux * length, y0 + uy * length
            poly = Polygon([
                (x0, y0), (x1, y1),
                (x1 + nx * depth, y1 + ny * depth),
                (x0 + nx * depth, y0 + ny * depth),
            ])
            k += 1
            if not prepared.contains(poly):
                continue
            if not occupied.is_empty and poly.intersects(occupied.buffer(clear)):
                continue
            out.append(list(poly.exterior.coords)[:-1])
            occupied = unary_union([occupied, poly])
    return out


# ---------------------------------------------------------------------------
#  Zápis stání do DXF
# ---------------------------------------------------------------------------
def add_stalls_to_doc(doc, groups):
    """
    Zapíše stání do DXF. `groups` = seznam (layer, color, stalls).
    Před zápisem smaže všechna dříve vygenerovaná stání (idempotence).
    """
    msp = doc.modelspace()
    for lyr in ALL_STALL_LAYERS:
        for e in list(msp.query(f'LWPOLYLINE[layer=="{lyr}"]')):
            msp.delete_entity(e)
    for layer, color, stalls in groups:
        if layer not in doc.layers:
            doc.layers.add(layer, color=color)
        for pts in stalls:
            msp.add_lwpolyline(
                pts, format="xy", close=True, dxfattribs={"layer": layer},
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


@app.post("/inspect")
def inspect():
    """Diagnostika DXF: typy entit, vrstvy, počet uzavřených/otevřených křivek."""
    if "file" not in request.files or not request.files["file"].filename:
        return jsonify({"error": "Chybí soubor DXF (pole 'file')."}), 400
    upload = request.files["file"]
    tmp = tempfile.NamedTemporaryFile(suffix=".dxf", delete=False)
    try:
        upload.save(tmp.name)
        tmp.close()
        try:
            doc = ezdxf.readfile(tmp.name)
        except (IOError, ezdxf.DXFStructureError) as exc:
            return jsonify({"error": f"Neplatný DXF: {exc}"}), 400
        _, diag = extract_boundary(doc, request.form.get("layer") or None)
        return jsonify(diag)
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass


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

    typ = request.form.get("type", "kolme").lower()    # kolme|sikme*|podelne|kombi
    layer = request.form.get("layer") or None          # volitelná vrstva obrysu
    parallel = (typ == "podelne")
    kombi = (typ == "kombi")
    if typ in ("kolme", "kombi"):
        angle = 90.0  # kombi = kolmé jádro + podélné okraje

    # Mapování ČSN šířka/délka -> rozteč podél komunikace (pitch) a kolmá hloubka (depth)
    if parallel:
        # podélné: délka jede podél komunikace, šířka je kolmá hloubka
        pitch, depth = length, width
    else:
        # kolmé/šikmé/kombi(jádro): šířka = rozteč podél komunikace, délka = kolmá hloubka
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

        boundary, diag = extract_boundary(doc, layer)
        if boundary is None:
            found = ", ".join(f"{k}×{v}" for k, v in sorted(diag["types"].items())) or "nic"
            return jsonify({
                "error": "Nenalezena žádná uzavřená plocha. Obrys musí být uzavřená "
                         "polylinie (LWPOLYLINE/POLYLINE), kruh, elipsa nebo uzavřený "
                         "splajn."
                         + (f" Na vrstvě '{layer}'." if layer else "")
                         + f" V souboru jsem našel: {found}. "
                         "Tip: v CADu obrys spoj příkazem PEDIT/JOIN a zavři (Closed), "
                         "nebo zadej vrstvu obrysu.",
                "diagnostics": diag,
            }), 422

        if kombi:
            # kolmé jádro
            core = generate_stalls(boundary, pitch, depth, aisle, 90.0, False)
            field = unary_union([Polygon(p) for p in core]) if core else Polygon()
            # podélné podél volných hran (ČSN: šířka 2,0 / délka 5,75 m)
            p_pre = CSN_PRESETS["podelne"]
            edges = parallel_along_edges(boundary, field,
                                         length=p_pre["length"], depth=p_pre["width"])
            add_stalls_to_doc(doc, [
                (KOLME_LAYER, STALL_LAYER_COLOR, core),
                (PODELNE_LAYER, 30, edges),  # 30 = oranžová (ACI)
            ])
            total = len(core) + len(edges)
            counts = f"{len(core)}+{len(edges)}"
        else:
            stalls = generate_stalls(boundary, pitch, depth, aisle, angle, parallel)
            add_stalls_to_doc(doc, [(STALL_LAYER, STALL_LAYER_COLOR, stalls)])
            total = len(stalls)
            counts = str(total)

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
        resp.headers["X-Stall-Count"] = str(total)
        resp.headers["X-Stall-Breakdown"] = counts
        if diag.get("auto_closed"):
            resp.headers["X-Warning"] = "boundary-auto-closed"
        resp.headers["Access-Control-Expose-Headers"] = (
            "X-Stall-Count, X-Stall-Breakdown, X-Warning"
        )
        return resp
    finally:
        try:
            os.unlink(tmp_in.name)
        except OSError:
            pass


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 5000))
    app.run(host="0.0.0.0", port=port, debug=True)
