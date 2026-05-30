# Generátor parkovacích stání z DXF (ČSN 73 6056)

Frontend (`index.html`) běží lokálně z disku přes `file://`,
backend (Flask) běží online na Railway.

## Soubory
| Soubor | Popis |
|---|---|
| `app.py` | Flask backend (`/generate`) |
| `index.html` | Lokální frontend |
| `requirements.txt` | Závislosti pro Railway |
| `Procfile` | Start command pro Railway |
| `runtime.txt` | Verze Pythonu (volitelné) |

## Lokální test
```bash
pip install -r requirements.txt
python app.py          # běží na http://127.0.0.1:5000
```
Pak otevři `index.html` v prohlížeči (dvojklik / `file://`).

## Nasazení na Railway
1. Nahraj složku na GitHub (geodetmasna) a v Railway dej **New → Deploy from GitHub repo**.
   - Nebo CLI: `railway init` → `railway up`.
2. Railway sám detekuje Python, nainstaluje `requirements.txt`
   a spustí příkaz z `Procfile` (`gunicorn app:app --bind 0.0.0.0:$PORT`).
3. V **Settings → Networking** dej **Generate Domain** → dostaneš
   `https://<projekt>.up.railway.app`.
4. V `index.html` nahoře v JS přepiš:
   ```js
   const BACKEND_URL = "https://<projekt>.up.railway.app";
   ```

## Endpointy
- `GET /` — health check
- `GET /presets` — tabulkové rozměry ČSN 73 6056 (JSON)
- `POST /generate` — `multipart/form-data`:
  `file` (DXF), `type` (kolme|sikme75|sikme60|sikme45|podelne),
  `width`, `length`, `aisle`, `angle`, `layer` (volitelné).
  Vrací modifikované DXF + hlavičku `X-Stall-Count`.

## Rozměry dle ČSN 73 6056 (rev. 2011, osobní vozidla)
| Řazení | Šířka stání | Délka stání | Šířka komunikace |
|---|---|---|---|
| Kolmé 90° | 2,50 m | 5,00 m | 6,00 m |
| Šikmé 75° | 2,60 m | 5,30 m | 5,00 m |
| Šikmé 60° | 2,90 m | 5,20 m | 3,50 m |
| Šikmé 45° | 3,55 m | 4,80 m | 3,00 m |
| Podélné 0° | 2,00 m | 5,75 m* | 3,50 m |

*Konvence ČSN:* **šířka stání** = rozteč měřená *podél* komunikace,
**délka stání** = *kolmá* hloubka řady (od hrany komunikace k obrubníku).
Kolmá světlá šířka stání proto u všech variant vychází ≈ 2,50 m.
\*Podélné: krajní stání bývá delší — ověř proti normě.

## Algoritmus
1. Vybere uzavřenou křivku (LWPOLYLINE/POLYLINE) — největší plochu nebo zadanou vrstvu.
2. Zarovná rastr s nejdelší hranou obrysu.
3. Vygeneruje stání: 90° = obdélník, 45/60/75° = rovnoběžník
   (svislá hloubka = délka, kolmá světlá šířka ≈ 2,5 m), podélné = obdélník.
   Modul řad: stání – komunikace – stání – (zády k sobě) – komunikace – …
4. `shapely` ponechá pouze stání ležící **celá uvnitř** obrysu.
5. Zapíše je na vrstvu `PARKING_STANI` a vrátí DXF.

> Rozměry zadávej v jednotkách výkresu. ČSN 73 6056 udává metry — pokud
> je výkres v mm, zadej hodnoty v mm (např. 2500 / 5000 / 6000).
