"""Pure-Python layout planner for the ESP32-P4 "Extreme Performance" board.

This is layer (A) of TASK 3: it turns ``hardware.lib.board_spec`` into a
complete, *checked* layout plan made of plain data -- footprint placements
(x, y, rotation, side), thermal vias, zones, rule areas, board outline,
mechanical silhouettes, net classes and custom DRC rules. It never imports
``pcbnew``; the thin applier ``esp32p4_extreme_place.py`` (layer B) copies
the plan onto a KiCad board. Everything here is importable and testable on
a machine without KiCad.

Conventions (identical to board_spec and to KiCad):

* millimetres, design origin (0, 0) = ESP32-P4 body centre, +X right, +Y down;
* rotation in degrees, positive = counter-clockwise *on screen*, i.e. a
  footprint-local point p maps to ``R(rot) p`` with
  ``R = [[cos, sin], [-sin, cos]]``;
* bottom-side footprints are mirrored in their local Y before rotating,
  ``R(rot) (px, -py)`` -- this is what ``SetLayerAndFlip(B_Cu)`` followed by
  ``SetOrientationDegrees(rot)`` produces (both checked by reading pad
  positions back from pcbnew 7.0.11, 8.0.8, 9.0.8 and 10.0.6).

Footprint geometry (courtyard, fab outline, pads, reference text, silk
extent, "PCB Edge" markers) comes from ``footprint_courtyards.json``,
extracted from the official KiCad 10.0.6 library (``python3 -m
hardware.pcbnew.layout_plan courtyards``). Placements keep clear of the
*envelope* of every courtyard across the 10.0.6/9.0.9/8.0.9/7.0.11 releases
plus any silk that pokes out of it, so the plan stays legal whichever
library the applier loads. U1 is not in the official libraries; its geometry
is generated from ``esp32p4_pinout`` (see :func:`u1_geometry`).

Placement policy, in order (every decision is logged in ``Plan.log``):

1. **Fixed**: U1 at (0, 0) rot 0 F.Cu and the holes H1..H8 at their exact
   spec coordinates. Never moved.
2. **Absolute** (``Place``): spec coordinates. Edge connectors (``faces=``)
   get the rotation that points their mating side at that board edge (see
   :func:`mating_direction`); if the mating face then protrudes more than
   ``EDGE_OVERHANG_MAX_MM`` beyond (or sits more than ``EDGE_RECESS_MAX_MM``
   inside) the edge, it is snapped flush and reported. A part that collides
   with an already placed part, a hole keep-out or the heatsink keep-out is
   moved to the nearest legal position and reported as a *spec conflict*
   (board_spec.py is never modified).
3. **Near** (the SoC ring and everything else): deterministic greedy search on
   a 0.1 mm grid, most constrained parts first (for equal distance limits,
   high-frequency decoupling first), anchors before dependants, on the side the
   spec declares (``Near.side``). Top-side positions near U1 must keep every U1
   pad's straight *escape channel* routable (section 5b) and a U1 part must sit
   in line with its own pad. Then the radius is widened step by step and a
   WARNING is logged -- never a silent violation. A repair loop
   (:func:`build_plan`) re-runs the plan with failed parts promoted and their
   ideal spot reserved against movable absolute parts.
4. **Silkscreen**: J9 pin legend (before step 3, so parts avoid it), every
   reference designator on a pad-free spot or on Fab, the licence line last.

Distances for ``Near`` are "pad-to-pad": from the target pad centre to the
nearest pad centre of the placed part (XY distance, also for bottom-side
parts).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

from hardware.lib import board_spec as bs
from hardware.lib import esp32p4_pinout as p4
from hardware.lib import impedance as imp

HERE = Path(__file__).resolve().parent
COURTYARD_JSON = HERE / "footprint_courtyards.json"
DRU_FILE = HERE / "esp32p4_extreme.kicad_dru"

# --------------------------------------------------------------------------
# Policy constants
# --------------------------------------------------------------------------
SEARCH_STEP_MM = 0.1              # grid of searched (Near) positions
COURTYARD_GAP_MM = 0.02           # minimum gap the planner keeps between courtyards
EDGE_OVERHANG_MAX_MM = 1.0        # courtyard front may protrude this far past the edge
EDGE_RECESS_MAX_MM = 3.0          # ... or sit this far inside it (cable/FPC entries)
EDGE_MARKER_TOL_MM = 0.5          # footprints with a "PCB Edge" marker: marker within +/-0.5 mm
LIBRARY_TAG = "10.0.6"            # kicad-footprints release the JSON and the board use (KiCad 10)

U1_LIB_ID = "PCM_Espressif:ESP32-P4"
# Espressif ships no 3D model for the ESP32-P4. For renders/MCAD use the stock KiCad
# 10 x 10 mm QFN body (0.4 mm pitch, 88 pins): same outline and ~same height, pins are
# only representative.
U1_MODEL = "${KICAD10_3DMODEL_DIR}/Package_DFN_QFN.3dshapes/ArtInChip_QFN-88-1EP_10x10mm_P0.4mm_EP6.74x6.74mm.step"
U1_COURTYARD_HALF_MM = 5.60       # Espressif ESP32-P4.kicad_mod F.CrtYd
U1_FAB_HALF_MM = p4.BODY_MM[0] / 2
FIDUCIAL_LIB_ID = "Fiducial:Fiducial_1mm_Mask2mm"

HEATSINK_AREA = "HEATSINK_KEEPOUT"
EPAD_VIA_AREA = "U1_EPAD_VIAS"
HEAT_SPREADER_HALF_MM = getattr(bs, "BOTTOM_SPREADER_MM", 14.0) / 2   # B.Cu GND heat spreader
VDD_HP_ISLAND_HALF_MM = 4.5       # ~9 x 9 mm VDD_HP island on In2.Cu
FAN_MM = 30.0                     # 30 x 30 mm fan silhouette
BSIDE_MARGIN_MM = 0.15            # B-side courtyards stay this far off the via field
ZONE_CLEARANCE_MM = 0.15
ZONE_MIN_WIDTH_MM = 0.15

# Silkscreen texts (F.SilkS). The licence line is placed last, in the free spot closest to
# the bottom-right corner; header legends are reserved right after the absolute placements.
LICENSE_TEXT = "ESP32-P4 Extreme Perf  |  CERN-OHL-P-2.0  |  github.com/Andreuxxx1977/esp32-p4"
LICENSE_TEXT_SIZE_MM = 0.8
LEGEND_TEXT_SIZE_MM = 0.8
SILK_CLEAR_MM = 0.2               # silk boxes keep this far from courtyards, holes and the edge
HEADER_LEGENDS = {"J9": "LCD_HEADER_LABELS"}    # ref -> board_spec attribute with pin labels

# Where the design origin (SoC centre) sits on the KiCad sheet: the centre of an A4 page, so
# the board is drawn inside the frame. Aux (drill/place) and grid origins are set here, so
# every coordinate *relative to the origins* equals the design coordinate.
PAGE_ORIGIN_MM = (148.5, 105.0)

LAYERS = {  # copper layer -> board layer name
    "F.Cu": "Signal_Top", "In1.Cu": "GND", "In2.Cu": "VCC_3V3", "B.Cu": "Signal_Bottom"}
USER_LAYERS = {"User.1": "Heatsink", "User.2": "Fan"}

# Design rules (HDI, 1+2+1): values in mm.
DESIGN_RULES = {
    "min_track": 0.09, "min_clearance": 0.09,
    "via_drill": 0.20, "via_diameter": 0.45,
    "uvia_drill": 0.10, "uvia_diameter": 0.25,
    "min_annular": 0.10, "hole_to_hole": 0.25, "hole_clearance": 0.20,
    "copper_edge": 0.30, "board_thickness": imp.BOARD_THICKNESS_MM,
}

# Spec pin names that are mechanical (excluded from the mating-side heuristic).
MECH_PIN_NAMES = frozenset({"MP", "SH", "SHIELD", "EP"})

EPS = 1e-6

Rect = tuple  # (x0, y0, x1, y1)


# ==========================================================================
# 1. Geometry
# ==========================================================================

def _cos_sin(deg: float) -> tuple[float, float]:
    d = deg % 360.0
    exact = {0.0: (1.0, 0.0), 90.0: (0.0, 1.0), 180.0: (-1.0, 0.0), 270.0: (0.0, -1.0)}
    if d in exact:
        return exact[d]
    a = math.radians(d)
    return math.cos(a), math.sin(a)


def rotate(x: float, y: float, deg: float) -> tuple[float, float]:
    """KiCad rotation: +deg is counter-clockwise on screen (Y axis down)."""
    c, s = _cos_sin(deg)
    return x * c + y * s, -x * s + y * c


def to_board(px: float, py: float, x: float, y: float, rot: float, side: str) -> tuple[float, float]:
    """Footprint-local point -> design coordinates."""
    if side == "B":
        py = -py
    rx, ry = rotate(px, py, rot)
    return x + rx, y + ry


def xform_rect(r: Rect, x: float, y: float, rot: float, side: str) -> Rect:
    pts = [to_board(px, py, x, y, rot, side) for px in (r[0], r[2]) for py in (r[1], r[3])]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys))


def rects_overlap(a: Rect, b: Rect, gap: float = 0.0) -> bool:
    """True if the rectangles are closer than ``gap`` (touching is legal)."""
    return (a[0] < b[2] + gap - EPS and b[0] < a[2] + gap - EPS
            and a[1] < b[3] + gap - EPS and b[1] < a[3] + gap - EPS)


def rect_point_dist(r: Rect, x: float, y: float) -> float:
    dx = max(r[0] - x, 0.0, x - r[2])
    dy = max(r[1] - y, 0.0, y - r[3])
    return math.hypot(dx, dy)


def point_in_outline(x: float, y: float, outline: Rect = bs.BOARD_OUTLINE,
                     radius: float = bs.BOARD_CORNER_R) -> bool:
    x0, y0, x1, y1 = outline
    if not (x0 - EPS <= x <= x1 + EPS and y0 - EPS <= y <= y1 + EPS):
        return False
    cx = min(max(x, x0 + radius), x1 - radius)
    cy = min(max(y, y0 + radius), y1 - radius)
    return math.hypot(x - cx, y - cy) <= radius + EPS


@dataclass(frozen=True)
class Shape:
    """A courtyard in design coordinates: an axis-aligned rectangle or a circle."""

    rect: Rect                      # bounding box (always valid)
    circle: tuple | None = None     # (cx, cy, r) for round courtyards

    def overlaps(self, other: "Shape", gap: float = 0.0) -> bool:
        if not rects_overlap(self.rect, other.rect, gap):
            return False
        if self.circle and other.circle:
            (ax, ay, ar), (bx, by, br) = self.circle, other.circle
            return math.hypot(ax - bx, ay - by) < ar + br + gap - EPS
        if self.circle:
            cx, cy, r = self.circle
            return rect_point_dist(other.rect, cx, cy) < r + gap - EPS
        if other.circle:
            cx, cy, r = other.circle
            return rect_point_dist(self.rect, cx, cy) < r + gap - EPS
        return True

    def overlaps_rect(self, r: Rect) -> bool:
        return self.overlaps(Shape(r))

    def hits_circle(self, cx: float, cy: float, r: float) -> bool:
        if self.circle:
            sx, sy, sr = self.circle
            return math.hypot(sx - cx, sy - cy) < sr + r - EPS
        return rect_point_dist(self.rect, cx, cy) < r - EPS

    def inside_outline(self, outline: Rect = bs.BOARD_OUTLINE, radius: float = bs.BOARD_CORNER_R) -> bool:
        if self.circle:
            cx, cy, r = self.circle
            n = 32
            return all(point_in_outline(cx + r * math.cos(2 * math.pi * k / n),
                                        cy + r * math.sin(2 * math.pi * k / n), outline, radius)
                       for k in range(n))
        x0, y0, x1, y1 = self.rect
        ox0, oy0, ox1, oy1 = outline
        if x0 < ox0 - EPS or y0 < oy0 - EPS or x1 > ox1 + EPS or y1 > oy1 + EPS:
            return False
        if (x0 >= ox0 + radius and x1 <= ox1 - radius) or (y0 >= oy0 + radius and y1 <= oy1 - radius):
            return True
        return all(point_in_outline(px, py, outline, radius) for px in (x0, x1) for py in (y0, y1))


# ==========================================================================
# 2. Footprint geometry database
# ==========================================================================

@dataclass(frozen=True)
class PadGeom:
    number: str
    kind: str            # smd | tht | npth | aperture
    x: float
    y: float
    sx: float            # copper extent along X in the footprint frame (pad rotation applied)
    sy: float
    shape: str = "rect"  # KiCad pad shape (circle | oval | rect | roundrect | ...)


@dataclass(frozen=True)
class FootprintGeom:
    lib_id: str
    courtyard: Rect                  # F.CrtYd bounding box, footprint frame, rot 0
    courtyard_r: float | None        # set when the courtyard is one circle at the origin
    fab: Rect | None
    pads: tuple[PadGeom, ...]
    pcb_edge: tuple | None           # "PCB Edge" marker segment (x0, y0, x1, y1)
    tht: bool
    description: str = ""
    ref_text: tuple | None = None    # library Reference text: (x, y, size, layer)
    silk: Rect | None = None         # F.SilkS bounding box (stroke included)

    @property
    def envelope(self) -> Rect:
        """What the planner keeps clear: courtyard plus any silk poking out of it
        (pin-1 markers of QFN/SOT parts reach up to 0.4 mm past the courtyard)."""
        c, k = self.courtyard, self.silk
        if not k:
            return c
        return (min(c[0], k[0]), min(c[1], k[1]), max(c[2], k[2]), max(c[3], k[3]))

    def pad_numbers(self) -> list[str]:
        out: list[str] = []
        for p in self.pads:
            if p.number and p.number not in out:
                out.append(p.number)
        return out

    def copper_pads(self) -> list[PadGeom]:
        return [p for p in self.pads if p.kind in ("smd", "tht")]


def u1_geometry() -> FootprintGeom:
    """ESP32-P4 QFN-104 land pattern generated from ``esp32p4_pinout``.

    104 perimeter pads (0.20 x 0.65 mm, 0.35 mm pitch, rows at +/-4.875 mm)
    and one solid 7.5 x 7.5 mm EPAD (pad 105). The applier adds the 3 x 3
    paste windows (2.1 mm tiles on 2.7 mm pitch, -0.2 mm paste margin).
    """
    w, l = p4.PAD_SIZE_MM
    pads = []
    for n in range(1, 105):
        x, y = p4.pad_position(n)
        horizontal = p4.pad_side(n) in ("left", "right")
        pads.append(PadGeom(str(n), "smd", x, y, l if horizontal else w, w if horizontal else l,
                            "oval"))
    e = p4.EPAD_SIZE_MM
    pads.append(PadGeom("105", "smd", 0.0, 0.0, e, e))
    c, f = U1_COURTYARD_HALF_MM, U1_FAB_HALF_MM
    return FootprintGeom(U1_LIB_ID, (-c, -c, c, c), None, (-f, -f, f, f), tuple(pads), None, False,
                         "ESP32-P4 QFN-104 10x10 mm, generated from esp32p4_pinout",
                         (0.0, -6.4, 1.0, "F.SilkS"))


def load_geometry(path: Path = COURTYARD_JSON) -> dict[str, FootprintGeom]:
    data = json.loads(Path(path).read_text())
    out: dict[str, FootprintGeom] = {}
    for lib_id, d in data["footprints"].items():
        pads = tuple(PadGeom(*p) for p in d["pads"])
        out[lib_id] = FootprintGeom(
            lib_id, tuple(d["courtyard"]), d.get("courtyard_r"),
            tuple(d["fab"]) if d.get("fab") else None, pads,
            tuple(d["pcb_edge"]) if d.get("pcb_edge") else None, d.get("tht", False),
            d.get("descr", ""), tuple(d["ref"]) if d.get("ref") else None,
            tuple(d["silk"]) if d.get("silk") else None)
    out[U1_LIB_ID] = u1_geometry()
    return out


# ---- .kicad_mod parsing (only used to (re)generate the JSON) ---------------
_TOKEN = re.compile(r'\s*(\(|\)|"(?:[^"\\]|\\.)*"|[^\s()"]+)')


def parse_sexpr(text: str) -> list:
    stack: list[list] = [[]]
    for m in _TOKEN.finditer(text):
        tok = m.group(1)
        if tok == "(":
            stack.append([])
        elif tok == ")":
            node = stack.pop()
            stack[-1].append(node)
        else:
            stack[-1].append(tok[1:-1].replace('\\"', '"') if tok.startswith('"') else tok)
    return stack[0][0]


def _children(node: list, key: str) -> list[list]:
    return [c for c in node[1:] if isinstance(c, list) and c and c[0] == key]


def _child(node: list, key: str) -> list | None:
    found = _children(node, key)
    return found[0] if found else None


def _xy(node: list | None) -> tuple[float, float] | None:
    return (float(node[1]), float(node[2])) if node else None


def _graphic_points(g: list) -> list[tuple[float, float]]:
    if g[0] == "fp_circle":
        (cx, cy), (ex, ey) = _xy(_child(g, "center")), _xy(_child(g, "end"))
        r = math.hypot(ex - cx, ey - cy)
        return [(cx - r, cy - r), (cx + r, cy + r)]
    pts = [_xy(_child(g, k)) for k in ("start", "mid", "end")]
    poly = _child(g, "pts")
    if poly:
        pts += [_xy(xy) for xy in _children(poly, "xy")]
    return [p for p in pts if p]


def _bbox(points: list[tuple[float, float]]) -> Rect | None:
    if not points:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (round(min(xs), 4), round(min(ys), 4), round(max(xs), 4), round(max(ys), 4))


def extract_footprint(text: str) -> dict:
    """Courtyard / fab bbox, pads and PCB-edge marker of one .kicad_mod file."""
    fp = parse_sexpr(text)
    layers: dict[str, list[list]] = {}
    for kind in ("fp_line", "fp_rect", "fp_circle", "fp_arc", "fp_poly"):
        for g in _children(fp, kind):
            layers.setdefault(_child(g, "layer")[1], []).append(g)
    crt = layers.get("F.CrtYd", [])
    courtyard = _bbox([p for g in crt for p in _graphic_points(g)])
    courtyard_r = None
    if len(crt) == 1 and crt[0][0] == "fp_circle":
        (cx, cy) = _xy(_child(crt[0], "center"))
        if abs(cx) < EPS and abs(cy) < EPS:
            courtyard_r = round(courtyard[2], 4)
    fab = _bbox([p for g in layers.get("F.Fab", []) for p in _graphic_points(g)])
    silk_pts = []
    for g in layers.get("F.SilkS", []):
        w = _child(_child(g, "stroke") or [], "width")
        half = float(w[1]) / 2 if w else 0.06
        for x, y in _graphic_points(g):
            silk_pts += [(x - half, y - half), (x + half, y + half)]
    silk = _bbox(silk_pts)
    pads = []
    for pad in _children(fp, "pad"):
        at = _child(pad, "at")
        size = _child(pad, "size")
        ang = float(at[3]) if len(at) > 3 else 0.0
        w, h = float(size[1]), float(size[2])
        c, s = abs(math.cos(math.radians(ang))), abs(math.sin(math.radians(ang)))
        sx, sy = w * c + h * s, w * s + h * c
        lay = set(_child(pad, "layers")[1:])
        kind = {"thru_hole": "tht", "np_thru_hole": "npth"}.get(pad[2], "smd")
        if kind == "smd" and not any(layer.endswith(".Cu") for layer in lay):
            kind = "aperture"
        pads.append([pad[1], kind, float(at[1]), float(at[2]), round(sx, 4), round(sy, 4), pad[3]])
    edge = None
    marker_texts = [t for t in _children(fp, "fp_text") if "pcb edge" in t[2].lower()]
    if marker_texts:
        for layer in ("Dwgs.User", "Cmts.User"):
            for g in layers.get(layer, []):
                if g[0] == "fp_line":
                    (x0, y0), (x1, y1) = _xy(_child(g, "start")), _xy(_child(g, "end"))
                    edge = (x0, y0, x1, y1)
    ref = None
    for node in _children(fp, "property") + _children(fp, "fp_text"):
        if node[1] in ("Reference", "reference"):
            at, font = _child(node, "at"), _child(_child(node, "effects") or [], "font")
            size = _child(font, "size") if font else None
            layer = _child(node, "layer")
            ref = [float(at[1]), float(at[2]), float(size[1]) if size else 1.0,
                   layer[1] if layer else "F.SilkS"]
    attr = _child(fp, "attr")
    descr = _child(fp, "descr")
    return {"courtyard": courtyard, "courtyard_r": courtyard_r, "fab": fab, "pads": pads,
            "pcb_edge": edge, "tht": bool(attr and attr[1] == "through_hole"),
            "ref": ref, "silk": silk, "descr": descr[1] if descr else ""}


def required_lib_ids() -> list[str]:
    ids = {c.part.footprint for c in bs.COMPONENTS} | {FIDUCIAL_LIB_ID}
    ids.discard(U1_LIB_ID)
    return sorted(ids)


ENVELOPE_TAGS = ("9.0.9", "8.0.9", "7.0.11")   # older libraries the applier may still meet


DEFAULT_CACHE = Path(os.environ.get("KICAD_FP_CACHE", Path.home() / ".cache" / "esp32p4-footprints"))


def cached_footprint_path(cache: Path, tag: str, lib_id: str) -> Path:
    """master files live directly in the cache (same layout as hardware/skidl/check_footprints.py),
    other library releases under ``<cache>/<tag>/``."""
    lib, name = lib_id.split(":", 1)
    base = cache if tag == "master" else cache / tag
    return base / f"{lib}.pretty" / f"{name}.kicad_mod"


def _fetch(cache: Path, tag: str, lib_id: str, fetch: bool) -> Path | None:
    lib, name = lib_id.split(":", 1)
    path = cached_footprint_path(cache, tag, lib_id)
    if not path.exists() and fetch:
        url = (f"https://gitlab.com/kicad/libraries/kicad-footprints/-/raw/{tag}/"
               f"{lib}.pretty/{name}.kicad_mod")
        try:
            data = urllib.request.urlopen(url, timeout=60).read()
        except urllib.error.HTTPError:
            return None
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        print(f"fetched {tag} {lib_id}")
    return path if path.exists() else None


def refresh_courtyards(cache: Path, tag: str = LIBRARY_TAG, out: Path = COURTYARD_JSON,
                       fetch: bool = True, envelope: tuple[str, ...] = ENVELOPE_TAGS) -> dict:
    """Download (if needed) every footprint the spec uses and write the JSON.

    Pads, fab outline and markers come from ``tag`` (10.0.6: the pad names the
    spec uses, e.g. 'SH' for shields). The courtyard is the *envelope* over ``tag`` and the older
    library releases in ``envelope`` (courtyards grew or shrank between
    releases, e.g. SOT-363 and the 2.54 mm headers), so a plan that is legal
    with the envelope stays legal with whichever library the applier loads.
    Cache layout: see :func:`cached_footprint_path`.
    """
    result = {"_meta": {
        "source": f"gitlab.com/kicad/libraries/kicad-footprints, ref {tag}",
        "courtyard_envelope": [tag, *envelope],
        "generated_by": "python3 -m hardware.pcbnew.layout_plan courtyards",
        "units": "mm, footprint frame (rotation 0, top side); pads = "
                 "[number, smd|tht|npth|aperture, x, y, size_x, size_y, shape]"},
        "footprints": {}}
    for lib_id in required_lib_ids():
        path = _fetch(cache, tag, lib_id, fetch)
        if path is None:
            raise FileNotFoundError(f"{lib_id} not found in {tag}")
        entry = extract_footprint(path.read_text())
        found_in = [tag]
        for old in envelope:
            opath = _fetch(cache, old, lib_id, fetch)
            if opath is None:
                continue
            other = extract_footprint(opath.read_text())
            c, o = entry["courtyard"], other["courtyard"]
            entry["courtyard"] = [min(c[0], o[0]), min(c[1], o[1]), max(c[2], o[2]), max(c[3], o[3])]
            if entry["courtyard_r"] and other["courtyard_r"]:
                entry["courtyard_r"] = max(entry["courtyard_r"], other["courtyard_r"])
            elif entry["courtyard_r"] or other["courtyard_r"]:
                entry["courtyard_r"] = None
            found_in.append(old)
        entry["libraries"] = found_in
        result["footprints"][lib_id] = entry
    out.write_text(json.dumps(result, indent=1, sort_keys=True) + "\n")
    return result


# ==========================================================================
# 3. Plan data model
# ==========================================================================

@dataclass
class Placement:
    ref: str
    lib_id: str
    geom: FootprintGeom
    x: float
    y: float
    rot: float
    side: str = "F"
    method: str = ""                 # fixed | absolute | edge | fiducial | near | near-B
    spec_xy: tuple | None = None     # coordinates requested by the spec (Place only)
    target: str | None = None        # "U1.26" for Near parts
    max_mm: float | None = None
    dist_mm: float | None = None
    faces: str | None = None
    notes: list[str] = field(default_factory=list)
    ref_on_silk: bool = True         # False: reference text goes to the Fab layer
    ref_xy: tuple | None = None      # planned reference position (design mm), text upright
    ref_size: float | None = None    # planned reference text height

    @property
    def tht(self) -> bool:
        return self.geom.tht or any(p.kind == "tht" for p in self.geom.pads)

    def shape(self) -> Shape:
        return shape_at(self.geom, self.x, self.y, self.rot, self.side)

    def ref_box(self) -> Rect | None:
        """Ink box of the silkscreen reference designator (None when it lives on Fab)."""
        rt = self.geom.ref_text
        if not self.ref_on_silk or not rt or not rt[3].endswith("SilkS"):
            return None
        if self.ref_xy:
            return ref_ink_box(self.ref, self.ref_xy[0], self.ref_xy[1], self.ref_size, 0.0)
        cx, cy = to_board(rt[0], rt[1], self.x, self.y, self.rot, self.side)
        return ref_ink_box(self.ref, cx, cy, rt[2], self.rot)

    def pad_positions(self, number: str | None = None) -> list[tuple[str, float, float]]:
        return [(p.number, *to_board(p.x, p.y, self.x, self.y, self.rot, self.side))
                for p in self.geom.copper_pads() if number is None or p.number == number]

    def pad_shapes(self) -> list[Shape]:
        """Copper of every pad (circles for round pads, bounding rectangles otherwise)."""
        out = []
        for p in self.geom.copper_pads():
            r = xform_rect((p.x - p.sx / 2, p.y - p.sy / 2, p.x + p.sx / 2, p.y + p.sy / 2),
                           self.x, self.y, self.rot, self.side)
            if p.shape == "circle":
                cx, cy = to_board(p.x, p.y, self.x, self.y, self.rot, self.side)
                out.append(Shape(r, (cx, cy, p.sx / 2)))
            else:
                out.append(Shape(r))
        return out


_LOCAL_RECTS: dict[tuple, Rect] = {}


def shape_at(geom: FootprintGeom, x: float, y: float, rot: float, side: str) -> Shape:
    """Planning envelope of ``geom`` placed at (x, y, rot, side) (memoised rotated rects)."""
    key = (geom.lib_id, rot, side)
    r = _LOCAL_RECTS.get(key)
    if r is None:
        r = _LOCAL_RECTS[key] = xform_rect(geom.envelope, 0.0, 0.0, rot, side)
    rect = (x + r[0], y + r[1], x + r[2], y + r[3])
    if geom.courtyard_r:
        return Shape(rect, (x, y, geom.courtyard_r))
    return Shape(rect)


@dataclass(frozen=True)
class Via:
    x: float
    y: float
    drill: float
    diameter: float
    net: str
    kind: str = "through"


@dataclass(frozen=True)
class Zone:
    name: str
    layers: tuple[str, ...]
    polygon: tuple[tuple[float, float], ...]
    net: str | None = None
    priority: int = 0
    connection: str = "thermal"          # thermal | solid
    rule_area: bool = False
    keepout: tuple[str, ...] = ()        # tracks | vias | pads | footprints | pours
    note: str = ""


@dataclass(frozen=True)
class Graphic:
    kind: str                            # line | arc (start, mid, end) | rect | poly | text
    layer: str
    pts: tuple[tuple[float, float], ...]
    width: float = 0.1
    filled: bool = False
    text: str = ""


# KiCad stroke font ("newstroke") glyph advances per 1 mm of text width, measured with
# pcbnew 7.0.11 (PCB_TEXT bounding boxes; the font is unchanged in 8 and 9).
_GLYPH = dict(zip(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 _-|./:()+",
    (0.922, 0.922, 0.875, 0.922, 0.875, 0.589, 0.922, 0.922, 0.494, 0.494, 0.827, 0.541, 1.351,
     0.922, 0.922, 0.922, 0.922, 0.637, 0.827, 0.589, 0.922, 0.779, 1.065, 0.827, 0.779, 0.827,
     0.875, 1.018, 1.018, 1.018, 0.922, 0.875, 1.018, 1.065, 0.494, 0.779, 1.018, 0.827, 1.160,
     1.065, 1.065, 1.018, 1.065, 1.018, 0.970, 0.779, 1.065, 0.875, 1.160, 0.970, 0.875, 0.970,
     0.970, 0.970, 0.970, 0.970, 0.970, 0.970, 0.970, 0.970, 0.970, 0.970,
     0.762, 0.779, 1.256, 0.970, 0.494, 1.065, 0.494, 0.684, 0.684, 1.256)))


def text_width(text: str, size: float, width: float | None = None) -> float:
    """Bounding-box width of a KiCad stroke-font string (within ~0.01 mm of pcbnew 7).

    ``size`` is the glyph height, ``width`` the glyph width (condensed text), the
    stroke is 0.15 x height.
    """
    w = size if width is None else width
    return w * (sum(_GLYPH.get(c, 1.1) for c in text) + 0.15) + 0.15 * size


def text_height(size: float) -> float:
    return 1.7 * size


def ref_ink_box(text: str, cx: float, cy: float, size: float, angle: float) -> Rect:
    """Box around the strokes of a centred text (glyph height + stroke, measured width)."""
    w, h = text_width(text, size), 1.15 * size
    if round(angle) % 180 == 90:
        w, h = h, w
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


@dataclass(frozen=True)
class SilkText:
    text: str
    x: float                        # centre
    y: float
    size: float                     # glyph height (mm); stroke = 0.15 x size
    angle: float = 0.0
    layer: str = "F.SilkS"
    kind: str = ""                  # licence | legend:<ref>
    width: float | None = None      # glyph width if condensed (default = size)

    @property
    def glyph_width(self) -> float:
        return self.size if self.width is None else self.width

    @property
    def thickness(self) -> float:
        return round(0.15 * self.size, 4)

    @property
    def bbox(self) -> Rect:
        w, h = text_width(self.text, self.size, self.width), text_height(self.size)
        if round(self.angle) % 180 == 90:
            w, h = h, w
        return (self.x - w / 2, self.y - h / 2, self.x + w / 2, self.y + h / 2)


@dataclass(frozen=True)
class NetClassPlan:
    name: str
    track_width: float
    clearance: float
    via_diameter: float
    via_drill: float
    uvia_diameter: float
    uvia_drill: float
    dp_width: float | None = None
    dp_gap: float | None = None
    description: str = ""


@dataclass
class ConnectorCheck:
    ref: str
    lib_id: str
    faces: str
    direction: str                   # local mating direction, e.g. "+Y"
    evidence: str
    rot: float
    spec_rot: float
    edge_dev_mm: float               # mating face beyond (+) / inside (-) the edge at spec coords
    action: str


@dataclass
class Plan:
    placements: dict[str, Placement] = field(default_factory=dict)
    vias: list[Via] = field(default_factory=list)
    zones: list[Zone] = field(default_factory=list)
    graphics: list[Graphic] = field(default_factory=list)
    silk: list[SilkText] = field(default_factory=list)
    netclasses: list[NetClassPlan] = field(default_factory=list)
    net_assignments: dict[str, str] = field(default_factory=dict)
    pad_maps: dict[str, dict[str, tuple[str, ...]]] = field(default_factory=dict)
    connectors: list[ConnectorCheck] = field(default_factory=list)
    conflicts: list[str] = field(default_factory=list)   # spec coordinates that had to change
    warnings: list[str] = field(default_factory=list)
    log: list[str] = field(default_factory=list)
    allow_bottom: bool = True

    def warn(self, msg: str) -> None:
        self.warnings.append(msg)
        self.log.append("WARNING " + msg)


# ==========================================================================
# 4. Spec helpers
# ==========================================================================

def resolve_lib_id(lib_id: str, geometry: dict[str, FootprintGeom]) -> str:
    if lib_id in geometry:
        return lib_id
    raise KeyError(f"footprint {lib_id} not in {COURTYARD_JSON.name}; run "
                   "`python3 -m hardware.pcbnew.layout_plan courtyards`")


def pad_map(comp: bs.Component, geom: FootprintGeom) -> tuple[dict[str, tuple[str, ...]], list[str]]:
    """Spec pad number -> footprint pad number(s) for the planning geometry."""
    return map_pads(comp, geom.pad_numbers())


def map_pads(comp: bs.Component, fp_numbers: list[str]) -> tuple[dict[str, tuple[str, ...]], list[str]]:
    """Spec pad number -> footprint pad number(s), plus notes about remaps.

    Exact matches first; then a spec pad whose *pin name* is a footprint pad
    number (e.g. pin 'MP'); finally, if exactly one spec pad and one footprint
    pad number are left over, they are paired (the KiCad 7/8/9 libraries name
    the USB-C shell 'S1' and the microSD shield '11' where master uses 'SH').
    """
    fp_numbers = [n for n in dict.fromkeys(fp_numbers) if n]
    spec_numbers = [n for n, _ in comp.part.pins]
    names = dict(comp.part.pins)
    mapping = {n: (n,) for n in spec_numbers if n in fp_numbers}
    left_spec = [n for n in spec_numbers if n not in mapping]
    left_fp = [n for n in fp_numbers if n not in spec_numbers]
    notes = []
    for n in list(left_spec):
        if names[n] in left_fp:
            mapping[n] = (names[n],)
            left_spec.remove(n)
            left_fp.remove(names[n])
            notes.append(f"spec pad {n} ({names[n]}) -> footprint pad {names[n]}")
    if len(left_spec) == 1 and len(left_fp) == 1:
        mapping[left_spec[0]] = (left_fp[0],)
        notes.append(f"spec pad {left_spec[0]} ({names[left_spec[0]]}) -> footprint pad {left_fp[0]}")
        left_spec = []
    for n in left_spec:
        if n in comp.conns:
            notes.append(f"spec pad {n} has no footprint pad (net {comp.conns[n]} dropped)")
    return mapping, notes


def comp_sort_key(ref: str) -> tuple:
    m = re.match(r"([A-Z_]+?)(\d+)$", ref)
    return (m.group(1), int(m.group(2))) if m else (ref, 0)


# ==========================================================================
# 5. Edge-connector mating direction heuristic
# ==========================================================================

DIRS = {"+X": (1.0, 0.0), "-X": (-1.0, 0.0), "+Y": (0.0, 1.0), "-Y": (0.0, -1.0)}
EDGE_VECTORS = {"right": (1.0, 0.0), "left": (-1.0, 0.0), "bottom": (0.0, 1.0), "top": (0.0, -1.0)}


def mating_direction(geom: FootprintGeom, signal_pads: list[PadGeom]) -> tuple[str, str]:
    """Return (local direction, evidence) of the side a plug/card/cable enters.

    Heuristic, in order of trust:

    1. A "PCB Edge" marker line (Dwgs.User, e.g. GCT USB4105): the mating
       side is the direction from the signal-pad centroid toward the marker.
    2. Body overhang: for each of +X/-X/+Y/-Y, how far the body (F.Fab
       outline, else courtyard) extends beyond the outermost *signal* pad in
       that direction. Solder tails sit at the back of a right-angle
       connector and the housing extends toward the mating face, so the
       largest overhang wins (margin to the runner-up is reported).
    3. Cross-check: the signal-pad centroid lies on the opposite side of the
       body centre.
    """
    pads = signal_pads or geom.copper_pads()
    body = geom.fab or geom.courtyard
    cx = sum(p.x for p in pads) / len(pads)
    cy = sum(p.y for p in pads) / len(pads)
    ext = {"+X": max(p.x + p.sx / 2 for p in pads), "-X": min(p.x - p.sx / 2 for p in pads),
           "+Y": max(p.y + p.sy / 2 for p in pads), "-Y": min(p.y - p.sy / 2 for p in pads)}
    over = {"+X": body[2] - ext["+X"], "-X": ext["-X"] - body[0],
            "+Y": body[3] - ext["+Y"], "-Y": ext["-Y"] - body[1]}
    ranked = sorted(over, key=lambda k: -over[k])
    best = ranked[0]
    bcx, bcy = (body[0] + body[2]) / 2, (body[1] + body[3]) / 2
    centroid_dir = ("-X" if cx > bcx else "+X") if abs(cx - bcx) > abs(cy - bcy) else \
                   ("-Y" if cy > bcy else "+Y")
    evidence = (f"body overhang {best} {over[best]:.2f} mm (next {ranked[1]} {over[ranked[1]]:.2f}); "
                f"pad centroid ({cx:.2f},{cy:.2f}) vs body centre ({bcx:.2f},{bcy:.2f}) -> {centroid_dir}")
    if geom.pcb_edge:
        x0, y0, x1, y1 = geom.pcb_edge
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        dx, dy = mx - cx, my - cy
        marker = ("+X" if dx > 0 else "-X") if abs(dx) > abs(dy) else ("+Y" if dy > 0 else "-Y")
        evidence = f"'PCB Edge' marker -> {marker}; " + evidence
        if marker != best:
            evidence += " [marker overrides overhang]"
        return marker, evidence
    if centroid_dir != best:
        evidence += " [centroid disagrees]"
    return best, evidence


def rotation_for(direction: str, faces: str) -> float:
    dx, dy = DIRS[direction]
    for rot in (0.0, 90.0, 180.0, 270.0):
        rx, ry = rotate(dx, dy, rot)
        if (round(rx), round(ry)) == EDGE_VECTORS[faces]:
            return rot
    raise ValueError(direction)


def mating_face(geom: FootprintGeom, direction: str) -> tuple[float, str]:
    """Local coordinate of the mating face along ``direction`` and its source."""
    axis = 0 if direction[1] == "X" else 1
    sign = 1 if direction[0] == "+" else -1
    if geom.pcb_edge:
        vals = geom.pcb_edge[axis::2]
        return (max(vals) if sign > 0 else min(vals)), "PCB-edge marker"
    c = geom.courtyard
    return (c[2 + axis] if sign > 0 else c[axis]), "courtyard"


# ==========================================================================
# 5b. SoC escape channels
# ==========================================================================
#
# U1's pads are 0.20 mm wide on a 0.35 mm pitch. Inside its courtyard the custom rule
# "u1_fanout_*" allows 0.09 mm, and the router gets a locked 0.10 mm stub per pad out to
# U1_STUB_END_MM (hardware/pcbnew/autoroute.py). From there every pad continues as a straight
# radial track on the same 0.35 mm pitch, which meets every net class (0.15 mm + 0.20 mm
# gap). Those tracks are the pad's *escape channel*. A top-side pad of another net inside
# a channel would block it, so the planner only accepts top-side placements near the SoC
# for which all channels still fit: per side and radial band, the channels (in pad order,
# each at most ESCAPE_JOG_MAX_MM off its pad line, further only as far as the radial run
# before the band allows at 45 degrees) must pack between the other nets' pads at netclass
# width + clearance. Pads of a channel's own nets (the pad's net and nets it continues into
# through series resistors) are passed through, not around. A decoupled supply pad's channel
# ends at its first shunt part (the cap); the rail goes on through the planes.

U1_STUB_END_MM = U1_COURTYARD_HALF_MM - 0.10   # the autorouter's locked U1 stubs end here
ESCAPE_REACH_MM = 12.0            # channels are kept free this far out (square half-size)
ESCAPE_JOG_MAX_MM = 1.0
ESCAPE_ALIGN_WEIGHT = 1.0         # search score per mm of sideways offset from the target pad
# Track width / clearance per net class for the channel model (the router's values).
ESCAPE_TRACKS = {"Default": (0.15, 0.10), "SE_50": (0.16, 0.15), "USB_90": (0.15, 0.20),
                 "MIPI_100": (0.135, 0.20), "ETH_100": (0.135, 0.20), "POWER": (0.20, 0.15)}


@dataclass(frozen=True)
class Channel:
    pad: str
    net: str
    side: str            # left | right | top | bottom (of U1)
    t: float             # coordinate along the side (y for left/right, x for top/bottom)
    width: float
    clearance: float
    owners: frozenset    # nets allowed inside the channel
    decoupled: bool      # supply pad: the channel ends at its first shunt part


@dataclass(frozen=True)
class Obstacle:
    side: str
    t0: float
    t1: float
    r0: float
    r1: float
    net: str
    shunt: bool          # pad of a part that has a GND pad (decoupling / filter cap)
    ref: str = ""


def polar(x: float, y: float) -> tuple[str, float, float]:
    """(side, t, r) of a point relative to U1: side by the larger coordinate."""
    if abs(x) >= abs(y):
        return ("left" if x < 0 else "right"), y, abs(x)
    return ("top" if y < 0 else "bottom"), x, abs(y)


def rect_polar(r: Rect) -> tuple[str, float, float, float, float]:
    """(side, t0, t1, r0, r1) of a pad rectangle, side from its centre."""
    side, _, _ = polar((r[0] + r[2]) / 2, (r[1] + r[3]) / 2)
    if side in ("left", "right"):
        rs = sorted((abs(r[0]), abs(r[2]))) if r[0] * r[2] > 0 else (0.0, max(abs(r[0]), abs(r[2])))
        return side, r[1], r[3], rs[0], rs[1]
    rs = sorted((abs(r[1]), abs(r[3]))) if r[1] * r[3] > 0 else (0.0, max(abs(r[1]), abs(r[3])))
    return side, r[0], r[2], rs[0], rs[1]


def series_links() -> dict[str, set[str]]:
    """Net -> nets it continues into through a 2-pad resistor (0R/22R series parts),
    excluding supplies and GND (pull-ups and rail links are not in-line)."""
    def signal(n: str) -> bool:
        return bool(n) and n != bs.GND and not n.startswith(("+", "VDD", "VSYS", "VBUS"))
    links: dict[str, set[str]] = {}
    for c in bs.COMPONENTS:
        nets = list(c.conns.values())
        if c.part.kind == "res" and len(c.part.pins) == 2 and len(nets) == 2 and all(map(signal, nets)):
            a, b = nets
            links.setdefault(a, set()).add(b)
            links.setdefault(b, set()).add(a)
    return links


def decoupled_nets() -> set[str]:
    """Supply nets with a shunt cap next to U1 (their pads end their channel at the cap)."""
    out = set()
    for c in bs.COMPONENTS:
        if isinstance(c.place, bs.Near) and c.place.ref == "U1" and c.part.kind == "cap" \
                and bs.GND in c.conns.values():
            out |= {n for n in c.conns.values() if n.startswith(("+", "VDD"))}
    return out


def escape_channels() -> dict[str, list[Channel]]:
    u1 = next(c for c in bs.COMPONENTS if c.ref == "U1")
    links, dec = series_links(), decoupled_nets()
    out: dict[str, list[Channel]] = {s: [] for s in ("left", "right", "top", "bottom")}
    for pad, net in u1.conns.items():
        if not net or net == bs.GND or not pad.isdigit() or int(pad) > 104:
            continue
        x, y = p4.pad_position(int(pad))
        side = p4.pad_side(int(pad))
        w, c = ESCAPE_TRACKS.get(bs.netclass_of(net), ESCAPE_TRACKS["Default"])
        owners = {net}
        todo = [net]
        while todo:                                  # whole series chain (A - R - B - R - C)
            for m in links.get(todo.pop(), ()):
                if m not in owners:
                    owners.add(m)
                    todo.append(m)
        out[side].append(Channel(pad, net, side, y if side in ("left", "right") else x, w, c,
                                 frozenset(owners), net in dec))
    for chans in out.values():
        chans.sort(key=lambda ch: ch.t)
    return out


CLEARANCE_MAX_MM = max(c for _, c in ESCAPE_TRACKS.values())


def net_clearance(net: str) -> float:
    return ESCAPE_TRACKS.get(bs.netclass_of(net), ESCAPE_TRACKS["Default"])[1] if net else 0.0


def channel_keep(ch: "Channel", o: "Obstacle") -> float:
    """Track centre to pad edge: half width + the larger of the two nets' clearances."""
    return ch.width / 2 + max(ch.clearance, net_clearance(o.net))


def channel_pitch(a: "Channel", b: "Channel") -> float:
    return (a.width + b.width) / 2 + max(a.clearance, b.clearance)


def channel_forbidden(ch: "Channel", obs: list["Obstacle"], r: float) -> list[tuple[float, float]]:
    """Tangential intervals a channel's track centre must avoid at radius r (exact
    point-to-rectangle distance: rounded ends in front of / behind a pad)."""
    out = []
    for o in obs:
        if o.net in ch.owners:
            continue
        k = channel_keep(ch, o)
        dr = o.r0 - r if r < o.r0 else (r - o.r1 if r > o.r1 else 0.0)
        if dr >= k:
            continue
        ext = math.sqrt(k * k - dr * dr)
        out.append((o.t0 - ext, o.t1 + ext))
    return out


class EscapeModel:
    """Placed top-side pads near U1 versus the U1 escape channels (see above)."""

    def __init__(self):
        self.channels = escape_channels()
        self.placed: dict[str, list[Obstacle]] = {s: [] for s in self.channels}

    @staticmethod
    def in_zone(r: Rect) -> bool:
        k = ESCAPE_REACH_MM
        return r[0] < k and r[2] > -k and r[1] < k and r[3] > -k

    def obstacles(self, pl: "Placement", nets_by_pad: dict[str, str], shunt: bool) -> list[Obstacle]:
        out = []
        for p, shape in zip(pl.geom.copper_pads(), pl.pad_shapes()):
            if not self.in_zone(shape.rect):
                continue
            side, t0, t1, r0, r1 = rect_polar(shape.rect)
            if r1 <= U1_STUB_END_MM:
                continue
            out.append(Obstacle(side, t0, t1, r0, r1, nets_by_pad.get(p.number, ""), shunt, pl.ref))
        return out

    def add(self, obstacles: list[Obstacle]) -> None:
        for o in obstacles:
            self.placed[o.side].append(o)

    def _end(self, ch: Channel, obs: list[Obstacle]) -> float:
        """Radius where the channel ends: its first own shunt part for a supply pad."""
        if not ch.decoupled:
            return ESCAPE_REACH_MM
        ends = [o.r1 for o in obs if o.shunt and o.net in ch.owners
                and o.t0 - ESCAPE_JOG_MAX_MM <= ch.t <= o.t1 + ESCAPE_JOG_MAX_MM]
        return min(ends, default=ESCAPE_REACH_MM)

    @staticmethod
    def _jog(r: float) -> float:
        """How far a 45-degree track can be off its pad line at radius r."""
        return max(0.0, min(ESCAPE_JOG_MAX_MM, r - U1_STUB_END_MM))

    def _band(self, o: Obstacle, obs: list[Obstacle]) -> str | None:
        """Do the channels passing ``o`` still pack between ``obs``? Checked at a few radii
        from where ``o``'s clearance zone starts to its inner edge, with the exact
        point-to-rectangle clearance and the jog a 45-degree track can have made by then
        (the same model :func:`escape_tracks` routes with)."""
        kmax = max(ch.width / 2 + CLEARANCE_MAX_MM for ch in self.channels[o.side]) \
            if self.channels[o.side] else 0.0
        for f in (1.0, 0.75, 0.5, 0.25, 0.0):
            r = max(o.r0 - f * kmax, U1_STUB_END_MM)
            why = self._pack(o.side, obs, r)
            if why:
                return why
        return None

    def _pack(self, side: str, obs: list[Obstacle], r: float) -> str | None:
        chans = [ch for ch in self.channels[side] if self._end(ch, obs) > r]
        jog = self._jog(r)
        x_prev, ch_prev = None, None
        for ch in chans:
            x = ch.t - jog
            if ch_prev is not None:
                x = max(x, x_prev + channel_pitch(ch_prev, ch))
            for a, b in sorted(channel_forbidden(ch, obs, r)):
                if a < x < b - EPS:
                    x = b
            if x > ch.t + jog + EPS:
                return f"blocks the escape channel of U1.{ch.pad} ({ch.net})"
            x_prev, ch_prev = x, ch
        return None

    def blocked(self, new: list[Obstacle]) -> str | None:
        """None if every channel still fits with ``new`` added, else which one does not.
        The bands of placed pads that ``new`` overlaps are re-checked too, so the final
        layout passes :meth:`violations` whatever the placement order."""
        for o in new:
            obs = self.placed[o.side] + [q for q in new if q.side == o.side]
            bands = [o] + [q for q in self.placed[o.side]
                           if q.r1 > o.r0 - 0.05 and q.r0 < o.r1 + 0.05]
            for b in bands:
                why = self._band(b, obs)
                if why:
                    return why
        return None

    def violations(self) -> list[str]:
        """Re-check every placed obstacle (for tests and the report)."""
        return [f"{o.ref}: {why}" for obs in self.placed.values() for o in obs
                if (why := self._band(o, obs))]


# --------------------------------------------------------------------------
# Escape tracks: the channels drawn out as real copper
# --------------------------------------------------------------------------
# The router is not told about the channel model, so it happily lays a neighbour straight
# through the room a channel needed for its jog. The escape tracks are therefore routed
# here, deterministically, and handed to the router as fixed copper: per side, channels in
# side: one linear program over all its channels (scipy / HiGHS). Variables are each
# channel's tangential position every ESCAPE_STEP_MM of radius; constraints: start on the
# pad line at the stub end, move at most 45 degrees, keep netclass pitch to the neighbour,
# keep netclass clearance to every other-net pad (the side of each pad a channel passes is
# taken from the leftmost packing where that pad is widest), and land in its first own-net
# pad in line (cap, series resistor) or run out to ESCAPE_REACH_MM. Objective: stay as close
# to the pad line as possible. The result is written to escape_tracks.json (the router's
# Python has no scipy) and re-checked geometrically by the tests.

ESCAPE_STEP_MM = 0.025
ESCAPE_DROP_COST = 1000.0        # a channel without escape track costs this much deviation
ESCAPE_MARGIN_MM = 0.005          # on top of the netclass rules (grid / linearisation slack)
ESCAPE_BEND_COST = 200.0          # per mm of second difference (one 45-degree bend ~ 5)
ESCAPE_MILP_SECONDS = 300


@dataclass
class EscapeTrack:
    pad: str
    net: str
    width: float
    points: list          # design mm, from the U1 stub end outwards
    end: str              # "pad <ref>" | "reach"


def _to_xy(side: str, t: float, r: float) -> tuple[float, float]:
    return {"left": (-r, t), "right": (r, t), "top": (t, -r), "bottom": (t, r)}[side]


def _leftmost(chans, obs, r, active) -> dict[str, float]:
    """Leftmost packing of the active channels at radius r (the planner's own check)."""
    jog = EscapeModel._jog(r)
    pos, x_prev, ch_prev = {}, None, None
    for ch in chans:
        if not active(ch, r):
            continue
        x = ch.t - jog
        if ch_prev is not None:
            x = max(x, x_prev + channel_pitch(ch_prev, ch))
        for a, b in sorted(channel_forbidden(ch, obs, r)):
            if a < x < b - EPS:
                x = b
        pos[ch.pad] = x
        x_prev, ch_prev = x, ch
    return pos


def escape_tracks(plan: "Plan") -> tuple[list[EscapeTrack], list[str]]:
    """Escape tracks for every U1 channel (see above). Returns (tracks, problems)."""
    import numpy as np                                   # only needed here
    from scipy.optimize import Bounds, LinearConstraint, milp
    from scipy.sparse import coo_matrix

    planner = Planner()
    planner.plan = plan
    model = EscapeModel()
    for ref, p in plan.placements.items():
        if ref != "U1" and (p.side == "F" or p.tht) and model.in_zone(p.shape().rect):
            model.add(planner._escape_obstacles(p))
    d = ESCAPE_STEP_MM
    radii = [U1_STUB_END_MM + k * d for k in range(int(round((ESCAPE_REACH_MM - U1_STUB_END_MM) / d)) + 1)]
    out: list[EscapeTrack] = []
    problems: list[str] = []
    def solve(side, chans, r_limit):
        obs = model.placed[side]
        ends = {}
        for ch in chans:                     # first own pad in line with the channel
            own = sorted((o for o in obs if o.net in ch.owners and o.t0 - ESCAPE_JOG_MAX_MM <= ch.t
                          <= o.t1 + ESCAPE_JOG_MAX_MM), key=lambda o: o.r0)
            ends[ch.pad] = own[0] if own else None
        k_limit = int(round((r_limit - radii[0]) / d))
        for ch in chans:                     # an own pad beyond the limit is not reached
            if ends[ch.pad] is not None and ends[ch.pad].r0 + 0.1 > r_limit:
                ends[ch.pad] = None
        last = {ch.pad: (min(k_limit, int((ends[ch.pad].r0 + 0.1 - radii[0]) / d))
                         if ends[ch.pad] else k_limit) for ch in chans}

        def active(ch, r):
            return r <= radii[last[ch.pad]] + EPS

        # variables: x[i,k] (position), u[i,k] (|x - t|), one binary per interacting
        # (channel, pad) -- 1 = the channel passes that pad on its right (higher t) side --
        # and one binary per channel: 1 = dropped (no escape track; left to the router).
        index = {}
        for ch in chans:
            for k in range(last[ch.pad] + 1):
                index[(ch.pad, k)] = len(index)
        n = len(index)
        pairs = {}
        for ch in chans:
            for o in obs:
                if o.net in ch.owners:
                    continue
                kk = channel_keep(ch, o)
                if o.t0 - kk - ESCAPE_JOG_MAX_MM < ch.t < o.t1 + kk + ESCAPE_JOG_MAX_MM \
                        and o.r0 - kk < radii[last[ch.pad]]:
                    pairs[(ch.pad, id(o))] = 2 * n + len(pairs)
        drop = {ch.pad: 2 * n + len(pairs) + i for i, ch in enumerate(chans)}
        # sigma[i,k] = |slope| of the segment arriving at sample k (0 straight .. 1 = 45 deg)
        base = 2 * n + len(pairs) + len(chans)
        sigma = {key: base + v for key, v in index.items()}
        # bend[i,k] = |x[k] - 2 x[k-1] + x[k-2]|: an L1 price on bends keeps the tracks to a
        # few straight runs (and FreeRouting fast: it chokes on thousands of tiny segments)
        bend = {key: base + n + v for key, v in index.items()}
        nv = base + 2 * n
        rows, cols, vals, rhs = [], [], [], []

        def le(terms, b):                    # sum(c * var) <= b
            r = len(rhs)
            for var, c in terms:
                rows.append(r)
                cols.append(var)
                vals.append(c)
            rhs.append(b)
        big = 2 * ESCAPE_JOG_MAX_MM + 4.0
        lower = [-np.inf] * nv
        upper = [np.inf] * nv
        for ch in chans:
            z = drop[ch.pad]
            for k in range(last[ch.pad] + 1):
                v, u = index[(ch.pad, k)], n + index[(ch.pad, k)]
                lower[u] = 0.0
                le([(v, 1), (u, -1)], ch.t)          # u >= x - t
                le([(v, -1), (u, -1)], -ch.t)        # u >= t - x
                lower[v], upper[v] = ch.t - ESCAPE_JOG_MAX_MM, ch.t + ESCAPE_JOG_MAX_MM
                if k == 0:
                    lower[v] = upper[v] = ch.t
                    continue
                w = index[(ch.pad, k - 1)]
                if k >= 2:
                    w2 = index[(ch.pad, k - 2)]
                    bn = bend[(ch.pad, k)]
                    le([(v, 1), (w, -2), (w2, 1), (bn, -1)], 0.0)
                    le([(v, -1), (w, 2), (w2, -1), (bn, -1)], 0.0)
                sg = sigma[(ch.pad, k)]
                lower[sg], upper[sg] = 0.0, 1.0      # 45 degrees at most (unless dropped)
                le([(v, 1), (w, -1), (sg, -d), (z, -big)], 0.0)
                le([(v, -1), (w, 1), (sg, -d), (z, -big)], 0.0)
                r = radii[k]
                for o in obs:
                    s = pairs.get((ch.pad, id(o)))
                    if s is None:
                        continue
                    kk = channel_keep(ch, o) + ESCAPE_MARGIN_MM
                    dr = o.r0 - r if r < o.r0 else (r - o.r1 if r > o.r1 else 0.0)
                    if dr >= kk:
                        continue
                    ext = math.sqrt(kk * kk - dr * dr)
                    le([(v, -1), (s, big), (z, -big)], big - (o.t1 + ext))   # s=1 -> x >= t1 + ext
                    le([(v, 1), (s, -big), (z, -big)], o.t0 - ext)           # s=0 -> x <= t0 - ext
                e = ends[ch.pad]
                if e is not None and k == last[ch.pad]:
                    le([(v, -1), (z, -big)], -(e.t0 + ch.width / 2))        # land in the own pad
                    le([(v, 1), (z, -big)], e.t1 - ch.width / 2)
        for s in pairs.values():
            lower[s], upper[s] = 0.0, 1.0
        for z in drop.values():
            lower[z], upper[z] = 0.0, 1.0
        for sg in sigma.values():
            lower[sg], upper[sg] = 0.0, 1.0
        for bn in bend.values():
            lower[bn], upper[bn] = 0.0, np.inf
        # pitch to the next few channels; slanted neighbours need more tangential room
        # (perpendicular = tangential / sqrt(1 + s^2) <= ... ; sqrt(1 + s^2) <= 1 + 0.414 s)
        slant = (math.sqrt(2) - 1) / 2
        for k in range(1, len(radii)):
            act = [ch for ch in chans if last[ch.pad] >= k]
            for ia, a in enumerate(act):
                for b in act[ia + 1:ia + 5]:
                    pab = channel_pitch(a, b) + ESCAPE_MARGIN_MM
                    le([(index[(a.pad, k)], 1), (index[(b.pad, k)], -1),
                        (sigma[(a.pad, k)], pab * slant), (sigma[(b.pad, k)], pab * slant),
                        (drop[a.pad], -big), (drop[b.pad], -big)], -pab)
        A = coo_matrix((vals, (rows, cols)), shape=(len(rhs), nv)).tocsr()
        cost = np.concatenate([np.zeros(n), np.ones(n), np.zeros(len(pairs)),
                               np.full(len(chans), ESCAPE_DROP_COST), np.full(n, 0.002),
                               np.full(n, ESCAPE_BEND_COST)])
        integrality = np.concatenate([np.zeros(2 * n), np.ones(len(pairs) + len(chans)), np.zeros(2 * n)])
        res = milp(cost, constraints=LinearConstraint(A, -np.inf, np.array(rhs)),
                   integrality=integrality, bounds=Bounds(lower, upper),
                   options={"time_limit": ESCAPE_MILP_SECONDS, "mip_rel_gap": 0.02})
        if res.x is None:
            return None, res.message
        dropped = {pad for pad, z in drop.items() if res.x[z] > 0.5}
        x = res.x
        tracks = []
        for ch in chans:
            if ch.pad in dropped:
                continue
            e = ends[ch.pad]
            pts = []
            prev_slope = None
            for k in range(last[ch.pad] + 1):
                xv = x[index[(ch.pad, k)]]
                if k == 0 or k == last[ch.pad]:
                    pts.append((radii[k], xv))
                    continue
                slope = x[index[(ch.pad, k + 1)]] - xv
                if prev_slope is None or abs(slope - prev_slope) > 2e-4:
                    pts.append((radii[k], xv))
                prev_slope = slope
            tracks.append(EscapeTrack(ch.pad, ch.net, ch.width,
                                      [tuple(round(v, 4) for v in _to_xy(side, xv, r)) for r, xv in pts],
                                      f"pad {e.ref}" if e is not None else "reach"))
        return tracks, sorted(dropped, key=int)

    for side, chans in model.channels.items():
        tracks, dropped = solve(side, chans, ESCAPE_REACH_MM)
        if tracks is None:
            problems.append(f"{side}: no escape tracks ({dropped})")
            continue
        out += tracks
        if dropped:
            problems.append(f"{side}: no escape track for U1 pads {', '.join(dropped)} (left to the router)")
    return out, problems


ESCAPE_FILE = Path(__file__).with_name("escape_tracks.json")


def _seg_rect(p, q, r) -> float:
    """Distance between segment pq and axis-aligned rectangle r (0 if they touch)."""
    x0, y0, x1, y1 = r
    if (x0 <= p[0] <= x1 and y0 <= p[1] <= y1) or (x0 <= q[0] <= x1 and y0 <= q[1] <= y1):
        return 0.0
    corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    edges = list(zip(corners, corners[1:] + corners[:1]))
    return min([_seg_seg(p, q, a, b) for a, b in edges])


def _seg_seg(p, q, a, b) -> float:
    def pt(s0, s1, c):
        vx, vy = s1[0] - s0[0], s1[1] - s0[1]
        L = vx * vx + vy * vy
        u = 0.0 if L == 0 else max(0.0, min(1.0, ((c[0] - s0[0]) * vx + (c[1] - s0[1]) * vy) / L))
        return math.hypot(s0[0] + u * vx - c[0], s0[1] + u * vy - c[1])

    def cross(o, a1, b1):
        return (a1[0] - o[0]) * (b1[1] - o[1]) - (a1[1] - o[1]) * (b1[0] - o[0])
    if (cross(p, q, a) * cross(p, q, b) < 0) and (cross(a, b, p) * cross(a, b, q) < 0):
        return 0.0
    return min(pt(p, q, a), pt(p, q, b), pt(a, b, p), pt(a, b, q))


def check_escape_tracks(plan: "Plan", tracks: list[EscapeTrack]) -> list[str]:
    """Exact geometry: every escape track keeps netclass clearance to other-net top pads
    and to the other escape tracks (inside U1's courtyard the 0.09 mm fan-out rule)."""
    planner = Planner()
    planner.plan = plan
    pads = []
    for ref, p in plan.placements.items():
        if ref == "U1" or not (p.side == "F" or p.tht):
            continue
        comp = planner.comps.get(ref)
        mapping = plan.pad_maps.get(ref, {})
        nets = {fp: net for spec, net in (comp.conns.items() if comp else ()) for fp in mapping.get(spec, (spec,))}
        for pg, sh in zip(p.geom.copper_pads(), p.pad_shapes()):
            if EscapeModel.in_zone(sh.rect):
                pads.append((sh.rect, nets.get(pg.number, ""), ref))
    owners = {ch.pad: ch.owners for chans in escape_channels().values() for ch in chans}
    rules = {ch.pad: (ch.width, ch.clearance) for chans in escape_channels().values() for ch in chans}
    problems = []
    segs = []
    for tr in tracks:
        w, c = rules[tr.pad]
        for a, b in zip(tr.points, tr.points[1:]):
            segs.append((a, b, tr, w, c))
            for rect, net, ref in pads:
                if net in owners[tr.pad]:
                    continue
                need = w / 2 + max(c, net_clearance(net))
                gap = _seg_rect(a, b, rect)
                if gap < need - 1e-3:
                    problems.append(f"U1.{tr.pad} ({tr.net}) {gap:.3f} mm from {ref} [{net}] (needs {need:.3f})")
    for i, (a, b, t1, w1, c1) in enumerate(segs):
        for a2, b2, t2, w2, c2 in segs[i + 1:]:
            if t2.net == t1.net:
                continue
            if max(abs(v) for v in (*a, *b, *a2, *b2)) > ESCAPE_REACH_MM + 1:
                continue
            gap = _seg_seg(a, b, a2, b2) - (w1 + w2) / 2
            inside = all(max(abs(v[0]), abs(v[1])) <= U1_COURTYARD_HALF_MM for v in (a, a2)) or \
                all(max(abs(v[0]), abs(v[1])) <= U1_COURTYARD_HALF_MM for v in (b, b2))
            need = DESIGN_RULES["min_clearance"] if inside else max(c1, c2)
            if gap < need - 1e-3:
                problems.append(f"U1.{t1.pad} / U1.{t2.pad}: {gap:.3f} mm (needs {need:.3f})")
    return sorted(set(problems))


def drop_failing_escapes(plan: "Plan", tracks: list[EscapeTrack], problems: list[str]) -> list[EscapeTrack]:
    """The solver works on a linearised model; the exact check has the last word. A track
    in a clearance problem is dropped (its pad is left to the router), later pad first."""
    while True:
        bad = check_escape_tracks(plan, tracks)
        if not bad:
            return tracks
        pads = [int(x) for x in re.findall(r"U1\.(\d+)", bad[0])]
        victim = str(max(pads))
        problems.append(f"escape track for U1.{victim} dropped after the exact check: {bad[0]}")
        tracks = [tr for tr in tracks if tr.pad != victim]


def escape_json(plan: "Plan", tracks: list[EscapeTrack] | None = None,
                problems: list[str] | None = None) -> dict:
    if tracks is None:
        tracks, problems = escape_tracks(plan)
    problems = list(problems or [])
    tracks = drop_failing_escapes(plan, tracks, problems)
    return {"comment": "GENERATED by `python3 -m hardware.pcbnew.layout_plan escapes` from "
                       "board_spec + the layout plan; routed as fixed copper by autoroute.py",
            "problems": problems,
            "tracks": [{"pad": t.pad, "net": t.net, "width": t.width, "end": t.end,
                        "points": [[round(x, 4), round(y, 4)] for x, y in t.points]}
                       for t in sorted(tracks, key=lambda t: int(t.pad))]}


def load_escape_tracks(path: Path = ESCAPE_FILE) -> list[EscapeTrack]:
    return load_escape_tracks_from(json.loads(Path(path).read_text()))


def load_escape_tracks_from(data: dict) -> list[EscapeTrack]:
    return [EscapeTrack(t["pad"], t["net"], t["width"], [tuple(p) for p in t["points"]], t["end"])
            for t in data["tracks"]]


# ==========================================================================
# 6. The planner
# ==========================================================================

class Planner:
    """One placement pass. :func:`build_plan` runs several passes (repair loop).

    ``promoted``: Near parts that failed in an earlier pass; they are placed
    first. ``reservations``: courtyards those parts could have used if the
    movable absolute parts were out of the way; absolute parts and fiducials
    treat them as obstacles (and get nudged, which is reported as a conflict).
    """

    def __init__(self, geometry: dict[str, FootprintGeom] | None = None, allow_bottom: bool = True,
                 promoted: list[str] | None = None, reservations: dict[str, Shape] | None = None):
        self.geometry = geometry or load_geometry()
        self.plan = Plan(allow_bottom=allow_bottom)
        self.promoted = list(promoted or [])
        self.reservations = dict(reservations or {})
        self.use_reservations = True
        self.comps = {c.ref: c for c in bs.COMPONENTS}
        self.order = {c.ref: i for i, c in enumerate(bs.COMPONENTS)}
        self._offset_cache: dict[tuple, list] = {}
        self._shapes: dict[str, Shape] = {}          # courtyards of placed parts
        self._grid: dict[tuple[int, int], list[str]] = {}   # spatial hash of those courtyards
        self._silk_boxes: list[Shape] = []                  # reserved silkscreen (F side)
        k = bs.KEEPOUT_HALF
        self.keepout = (-k, -k, k, k)
        tv = bs.THERMAL_VIA
        half = (tv["grid"] - 1) / 2 * tv["pitch_mm"] + tv["pad_mm"] / 2
        self.via_field_half = half
        self.bside_forbidden = (-(half + BSIDE_MARGIN_MM), -(half + BSIDE_MARGIN_MM),
                                half + BSIDE_MARGIN_MM, half + BSIDE_MARGIN_MM)
        self.escape = EscapeModel()

    # ------------------------------------------------------------ helpers
    def geom_of(self, comp: bs.Component) -> FootprintGeom:
        return self.geometry[resolve_lib_id(comp.part.footprint, self.geometry)]

    def exempt(self, ref: str) -> bool:
        comp = self.comps.get(ref)
        return bool(comp and bs.is_keepout_exempt(comp))

    def legal(self, cand: Placement, ignore: frozenset = frozenset(),
              shape: Shape | None = None) -> str | None:
        """Why ``cand`` cannot go where it is (None == legal). Cheapest tests first."""
        shape = shape or cand.shape()
        cand_tht = cand.tht
        for ref in self._nearby(shape.rect):
            if ref == cand.ref or ref in ignore:
                continue
            other = self.plan.placements[ref]
            if other.side != cand.side and not (other.tht or cand_tht):
                continue
            if shape.overlaps(self._shapes[ref], COURTYARD_GAP_MM):
                return f"courtyard overlaps {ref}"
        if self.use_reservations:
            for ref, res in self.reservations.items():
                if ref != cand.ref and cand.side == "F" and shape.overlaps(res, COURTYARD_GAP_MM):
                    return f"courtyard overlaps the spot reserved for {ref}"
        if cand.side == "F" or cand_tht:
            for box in self._silk_boxes:
                if shape.overlaps(box):
                    return "courtyard overlaps a silkscreen legend"
        if cand.side == "F" and cand.ref != "U1" and shape.overlaps_rect(self.keepout) \
                and not self.exempt(cand.ref):
            return "inside the 25 x 25 mm heatsink keep-out"
        for h, (hx, hy) in bs.HEATSINK_HOLES.items():
            if h != cand.ref and shape.hits_circle(hx, hy, bs.HEATSINK_HOLE_KEEPOUT_R):
                return f"inside the {h} hole keep-out circle"
        if cand.side == "B" and rects_overlap(shape.rect, self.bside_forbidden):
            return "on the bottom-side thermal-via field / thermal-pad window"
        if not self._inside_outline(shape, cand.faces):
            return "outside the board outline"
        if (cand.side == "F" or cand_tht) and cand.ref != "U1" and self.escape.in_zone(shape.rect):
            why = self.escape.blocked(self._escape_obstacles(cand))
            if why:
                return why
        return None

    def _escape_obstacles(self, pl: Placement) -> list[Obstacle]:
        comp = self.comps.get(pl.ref)
        mapping = self.plan.pad_maps.get(pl.ref, {})
        nets = {fp: net for spec, net in (comp.conns.items() if comp else ())
                for fp in mapping.get(spec, (spec,))}
        shunt = bool(comp and len(comp.part.pins) == 2 and bs.GND in comp.conns.values())
        return self.escape.obstacles(pl, nets, shunt)

    _CELL = 4.0

    def _cells(self, r: Rect):
        c = self._CELL
        for i in range(math.floor((r[0] - 1) / c), math.floor((r[2] + 1) / c) + 1):
            for j in range(math.floor((r[1] - 1) / c), math.floor((r[3] + 1) / c) + 1):
                yield (i, j)

    def _nearby(self, r: Rect) -> dict[str, None]:
        out: dict[str, None] = {}          # ordered set
        for cell in self._cells(r):
            for ref in self._grid.get(cell, ()):
                out[ref] = None
        return out

    @staticmethod
    def _inside_outline(shape: Shape, faces: str | None) -> bool:
        x0, y0, x1, y1 = bs.BOARD_OUTLINE
        if faces:  # the mating face may overhang its own edge
            o = EDGE_OVERHANG_MAX_MM
            x0, y0, x1, y1 = (x0 - o * (faces == "left"), y0 - o * (faces == "top"),
                              x1 + o * (faces == "right"), y1 + o * (faces == "bottom"))
        return shape.inside_outline((x0, y0, x1, y1), bs.BOARD_CORNER_R)

    def _offsets(self, radius: float, step: float) -> list[tuple[float, float]]:
        """Grid offsets sorted by distance (deterministic tie-break)."""
        key = (radius, step)
        if key not in self._offset_cache:
            n = int(radius / step)
            pts = [(i * step, j * step) for i in range(-n, n + 1) for j in range(-n, n + 1)
                   if math.hypot(i, j) * step <= radius + EPS]
            pts.sort(key=lambda p: (round(math.hypot(*p), 6), abs(p[1]), p[1], p[0]))
            self._offset_cache[key] = pts
        return self._offset_cache[key]

    def add(self, pl: Placement) -> Placement:
        assert pl.ref not in self.plan.placements, pl.ref
        comp = self.comps.get(pl.ref)
        # Dense HDI convention: 0402 passives, holes and fiducials carry their reference on
        # the Fab layer (assembly drawing) instead of the silkscreen.
        pl.ref_on_silk = not (pl.ref.startswith("FID") or (comp and (
            comp.part.kind == "mech" or (comp.part.kind in ("res", "cap") and comp.part.package == "0402"))))
        self.plan.placements[pl.ref] = pl
        shape = pl.shape()
        self._shapes[pl.ref] = shape
        if (pl.side == "F" or pl.tht) and pl.ref != "U1" and self.escape.in_zone(shape.rect):
            self.escape.add(self._escape_obstacles(pl))
        for cell in self._cells(shape.rect):
            self._grid.setdefault(cell, []).append(pl.ref)
        return pl

    # ------------------------------------------------------------ 6.1 fixed
    def place_fixed(self) -> None:
        u1 = self.comps["U1"]
        p = u1.place
        assert isinstance(p, bs.Place) and (p.x, p.y, p.rot, p.side) == (0.0, 0.0, 0.0, "F"), \
            "the design origin is the SoC centre: U1 must be Place(0, 0, 0, 'F')"
        self.add(Placement("U1", U1_LIB_ID, self.geometry[U1_LIB_ID], 0.0, 0.0, 0.0, "F",
                           "fixed", (0.0, 0.0)))
        holes = {**bs.HEATSINK_HOLES, **bs.BOARD_HOLES}
        for ref, (hx, hy) in holes.items():
            comp = self.comps[ref]
            assert isinstance(comp.place, bs.Place) and (comp.place.x, comp.place.y) == (hx, hy), \
                f"{ref}: spec placement disagrees with the hole table"
            pl = Placement(ref, comp.part.footprint, self.geom_of(comp), hx, hy, 0.0, "F", "fixed",
                           (hx, hy))
            why = self.legal(pl)
            if why:
                self.plan.conflicts.append(f"{ref} at ({hx}, {hy}) (fixed): {why}")
            self.add(pl)

    # ------------------------------------------------------------ 6.2 absolute
    def place_absolute(self) -> None:
        todo = [c for c in bs.COMPONENTS if isinstance(c.place, bs.Place)
                and c.ref not in self.plan.placements]

        def priority(c: bs.Component) -> tuple:
            g = self.geom_of(c)
            area = (g.courtyard[2] - g.courtyard[0]) * (g.courtyard[3] - g.courtyard[1])
            return (0 if c.place.faces else 1, 0 if g.tht else 1, -round(area, 3), self.order[c.ref])

        for comp in sorted(todo, key=priority):
            self._place_one_absolute(comp)

    def _signal_pads(self, comp: bs.Component, geom: FootprintGeom) -> list[PadGeom]:
        mapping, _ = pad_map(comp, geom)
        names = dict(comp.part.pins)
        numbers = {fp for spec, fps in mapping.items() if names[spec] not in MECH_PIN_NAMES
                   for fp in fps}
        return [p for p in geom.copper_pads() if p.number in numbers]

    def _place_one_absolute(self, comp: bs.Component) -> None:
        p: bs.Place = comp.place
        geom = self.geom_of(comp)
        x, y, rot = p.x, p.y, p.rot
        pl = Placement(comp.ref, geom.lib_id, geom, x, y, rot, p.side, "absolute", (p.x, p.y),
                       faces=p.faces)
        if p.faces:
            direction, evidence = mating_direction(geom, self._signal_pads(comp, geom))
            rot = rotation_for(direction, p.faces)
            pl.rot, pl.method = rot, "edge"
            dev, source = self._edge_deviation(pl, direction)
            action = f"kept spec coordinates ({source} {dev:+.2f} mm)"
            lo, hi = ((-EDGE_MARKER_TOL_MM, EDGE_MARKER_TOL_MM) if source == "PCB-edge marker"
                      else (-EDGE_RECESS_MAX_MM, EDGE_OVERHANG_MAX_MM))
            if not lo <= dev <= hi:
                ex, ey = EDGE_VECTORS[p.faces]
                pl.x, pl.y = round(x - ex * dev, 4), round(y - ey * dev, 4)
                action = f"snapped {source} flush: ({x}, {y}) -> ({pl.x}, {pl.y})"
                self.plan.conflicts.append(
                    f"{comp.ref} ({geom.lib_id.split(':')[1]}): at spec ({x}, {y}) rot {rot:g} the "
                    f"{source} is {dev:+.2f} mm past the {p.faces} edge (allowed {lo:+g}..{hi:+g}); "
                    f"suggest ({pl.x}, {pl.y})")
            if rot != p.rot:
                pl.notes.append(f"rotation {p.rot:g} -> {rot:g} (faces {p.faces})")
            self.plan.connectors.append(ConnectorCheck(
                comp.ref, geom.lib_id, p.faces, direction, evidence, rot, p.rot, round(dev, 3), action))
        why = self.legal(pl)
        if why:
            self._nudge(pl, why)
        self.add(pl)

    def _edge_deviation(self, pl: Placement, direction: str) -> tuple[float, str]:
        """How far the mating face protrudes (+) past its board edge, and its source."""
        face, source = mating_face(pl.geom, direction)
        axis = 0 if direction[1] == "X" else 1
        local = [0.0, 0.0]
        local[axis] = face
        bx, by = to_board(local[0], local[1], pl.x, pl.y, pl.rot, pl.side)
        x0, y0, x1, y1 = bs.BOARD_OUTLINE
        dev = {"right": bx - x1, "left": x0 - bx, "bottom": by - y1, "top": y0 - by}[pl.faces]
        return round(dev, 4), source

    def _nudge(self, pl: Placement, why: str, max_radius: float = 12.0) -> None:
        """Move ``pl`` to the nearest legal grid position (spec conflict)."""
        x0, y0 = pl.x, pl.y
        for radius, step in ((4.0, 0.05), (max_radius, 0.25)):
            for dx, dy in self._offsets(radius, step):
                if pl.faces:  # edge connectors only slide along their edge
                    if (pl.faces in ("top", "bottom") and dy) or (pl.faces in ("left", "right") and dx):
                        continue
                pl.x, pl.y = round(x0 + dx, 4), round(y0 + dy, 4)
                if not self.legal(pl):
                    msg = (f"{pl.ref} ({pl.lib_id.split(':')[1]}) at spec ({x0}, {y0}): {why}; "
                           f"moved to ({pl.x}, {pl.y}) [{math.hypot(dx, dy):.2f} mm]")
                    self.plan.conflicts.append(msg)
                    self.plan.log.append("CONFLICT " + msg)
                    pl.method += "+nudged"
                    return
        pl.x, pl.y = x0, y0
        self.plan.warn(f"{pl.ref}: no legal position within {max_radius} mm of spec "
                       f"({x0}, {y0}); left in place ({why})")

    # ------------------------------------------------------------ 6.3 fiducials
    def place_fiducials(self) -> None:
        geom = self.geometry[FIDUCIAL_LIB_ID]
        for ref, (fx, fy) in bs.FIDUCIALS.items():
            pl = Placement(ref, FIDUCIAL_LIB_ID, geom, fx, fy, 0.0, "F", "fiducial", (fx, fy))
            why = self.legal(pl)
            if why:
                self._nudge(pl, why)
            self.add(pl)

    def _target_points(self, near: bs.Near, comp: bs.Component) -> tuple[list, set]:
        """Target pad centres and their nets for a Near placement."""
        anchor = self.plan.placements[near.ref]
        acomp = self.comps.get(near.ref)
        amap = self.plan.pad_maps.get(near.ref) or (pad_map(acomp, anchor.geom)[0] if acomp else {})
        mine = set(comp.conns.values())
        if near.pad is not None:
            spec_pads = [near.pad]
        else:  # the anchor pads that share a net with this part (else all pads)
            spec_pads = [n for n, net in (acomp.conns.items() if acomp else []) if net in mine] \
                or [n for n, _ in acomp.part.pins]
        pts, nets = [], set()
        for sp in spec_pads:
            for fpn in amap.get(sp, (sp,)):
                for _, px, py in anchor.pad_positions(fpn):
                    pts.append((px, py))
            if acomp and sp in acomp.conns:
                nets.add(acomp.conns[sp])
        if not pts:
            raise KeyError(f"{comp.ref}: anchor {near.ref} has no pad {near.pad}")
        return pts, nets

    def _near_dist(self, pl: Placement, targets: list) -> float:
        return min(math.hypot(px - tx, py - ty)
                   for _, px, py in pl.pad_positions() for tx, ty in targets)

    @staticmethod
    def _weight(comp: bs.Component) -> float:
        """Assignment weight: high-frequency decoupling caps want the closest slots."""
        if comp.dnp:
            return 0.5
        if comp.part.kind == "cap":
            m = re.match(r"([\d.]+)(pF|nF|uF)", comp.value)
            farads = float(m.group(1)) * {"pF": 1e-12, "nF": 1e-9, "uF": 1e-6}[m.group(2)] if m else 1e-6
            return 1.5 if farads <= 100e-9 else 1.0
        return 0.8

    # ------------------------------------------------------------ 6.4 Near (incl. the SoC ring)
    def place_near_all(self) -> None:
        pending = [c for c in bs.COMPONENTS if isinstance(c.place, bs.Near)
                   and c.ref not in self.plan.placements]
        while pending:
            ready = [c for c in pending if c.place.ref in self.plan.placements]
            if not ready:
                raise RuntimeError(f"Near anchors never placed: {[c.ref for c in pending]}")
            ready.sort(key=lambda c: ((0, self.promoted.index(c.ref)) if c.ref in self.promoted
                                      else (1, c.place.max_mm), -self._weight(c),
                                      self.order[c.ref]))
            for comp in ready:
                self._place_near(comp)
            pending = [c for c in pending if c.ref not in self.plan.placements]

    def side_of(self, comp: bs.Component) -> str:
        """Side declared in the spec (Near.side); --top-only forces everything to F.Cu."""
        side = getattr(comp.place, "side", "F")
        return side if self.plan.allow_bottom else "F"

    def _place_near(self, comp: bs.Component) -> None:
        """Tiers: F.Cu within max_mm -> B.Cu within max_mm (small passives) -> widen + WARNING."""
        near: bs.Near = comp.place
        geom = self.geom_of(comp)
        mapping = self.plan.pad_maps[comp.ref]
        targets, nets = self._target_points(near, comp)
        anchor = self.plan.placements[near.ref]
        same = {fp for spec, net in comp.conns.items() if net in nets for fp in mapping.get(spec, ())}
        gnd = {fp for spec, net in comp.conns.items() if net == bs.GND for fp in mapping.get(spec, ())}
        side = self.side_of(comp)
        tiers = [(side, near.max_mm)]
        tiers += [(side, round(near.max_mm * k, 3)) for k in (1.5, 2.0, 3.0, 4.0, 8.0)]
        for side, radius in tiers:
            # within max_mm: 0.1 mm grid; widened fall-back tiers: coarser as the disc grows
            step = SEARCH_STEP_MM if radius <= near.max_mm + EPS else \
                round(max(SEARCH_STEP_MM, radius / 25), 2)
            pl = self._search(comp, geom, targets, same, gnd, anchor, radius, step, side=side)
            if pl:
                break
        else:
            raise RuntimeError(f"{comp.ref}: no legal position near {near.ref}")
        pl.method = "near" if side == "F" else "near-B"
        pl.target = f"{near.ref}.{near.pad}" if near.pad else f"{near.ref}.*"
        pl.max_mm = near.max_mm
        pl.dist_mm = round(self._near_dist(pl, targets), 3)
        if pl.dist_mm > near.max_mm + EPS:
            self.plan.warn(f"{comp.ref}: nothing legal within {near.max_mm} mm of {pl.target}; "
                           f"placed at {pl.dist_mm:.2f} mm (search radius widened to {radius} mm)")
        self.add(pl)

    def _int_disc(self, radius: float, step: float) -> list[tuple[int, int, float]]:
        key = ("int", radius, step)
        if key not in self._offset_cache:
            self._offset_cache[key] = [(round(dx / step), round(dy / step), math.hypot(dx, dy))
                                       for dx, dy in self._offsets(radius, step)]
        return self._offset_cache[key]

    def _search(self, comp, geom, targets, same, gnd, anchor, radius, step,
                ignore: frozenset = frozenset(), side: str = "F") -> Placement | None:
        """Best legal position with a same-net pad within ``radius`` of a target.

        Candidates lie on a ``step`` grid. Score = distance of that pad to the
        target, +0.25 mm if the part's GND pad would point *toward* the anchor
        instead of away from it. Candidates are tried best-first against a
        pre-fetched list of nearby obstacles; the first one that also passes the
        full :meth:`legal` check wins (tie-break: rotation, then y, then x).
        """
        pads = geom.copper_pads()
        q_idx = [i for i, p in enumerate(pads) if p.number in same] or list(range(len(pads)))
        g_idx = [i for i, p in enumerate(pads) if p.number in gnd]
        tht = geom.tht or any(p.kind == "tht" for p in pads)
        ext = max(abs(v) for v in geom.courtyard) + 1.0
        tx0 = min(t[0] for t in targets) - radius - ext
        ty0 = min(t[1] for t in targets) - radius - ext
        tx1 = max(t[0] for t in targets) + radius + ext
        ty1 = max(t[1] for t in targets) + radius + ext
        obstacles: list[Shape] = []
        for ref in self._nearby((tx0, ty0, tx1, ty1)):
            other = self.plan.placements[ref]
            if ref != comp.ref and ref not in ignore and (other.side == side or other.tht or tht):
                obstacles.append(self._shapes[ref])
        if side == "F" or tht:
            obstacles += self._silk_boxes
        inv = 1.0 / step
        disc = self._int_disc(radius, step)
        align = anchor.ref == "U1"
        best: dict[tuple, float] = {}
        local_by_rot = {}
        for rot in (0.0, 90.0, 180.0, 270.0):
            local = [to_board(p.x, p.y, 0.0, 0.0, rot, side) for p in pads]
            local_by_rot[rot] = local
            for qi in q_idx:
                qx, qy = local[qi]
                for tx, ty in targets:
                    penalty = 0.0
                    if g_idx and same:  # GND pad pointing away from the anchor centre?
                        gx, gy = local[g_idx[0]]
                        if (gx - qx) * (tx - anchor.x) + (gy - qy) * (ty - anchor.y) < -EPS:
                            penalty = 0.25
                    bx, by = round((tx - qx) * inv), round((ty - qy) * inv)
                    # Near U1: stay in line with the target pad (inside its own escape
                    # channel) rather than drift in front of a neighbouring pad.
                    along_y = align and polar(tx, ty)[0] in ("left", "right")
                    for i, j, r in disc:
                        key = (rot, bx + i, by + j)
                        score = r + penalty + (ESCAPE_ALIGN_WEIGHT * abs(j if along_y else i) * step
                                               if align else 0.0)
                        if score < best.get(key, 1e9):
                            best[key] = score
        order = sorted(best.items(), key=lambda kv: (round(kv[1], 6), kv[0][0], kv[0][2], kv[0][1]))
        for (rot, ix, iy), _ in order:
            cx, cy = round(ix * step, 4), round(iy * step, 4)
            shape = shape_at(geom, cx, cy, rot, side)
            if any(shape.overlaps(o, COURTYARD_GAP_MM) for o in obstacles):
                continue
            local = local_by_rot[rot]
            d_q = min(math.hypot(cx + local[i][0] - tx, cy + local[i][1] - ty)
                      for i in q_idx for tx, ty in targets)
            if d_q > radius + EPS:
                continue
            if align and side == "F" and not self._in_channel(
                    [(cx + local[i][0], cy + local[i][1]) for i in q_idx], targets):
                continue
            pl = Placement(comp.ref, geom.lib_id, geom, cx, cy, rot, side)
            if not self.legal(pl, ignore, shape):
                return pl
        return None

    @staticmethod
    def _in_channel(pads: list[tuple[float, float]], targets: list[tuple[float, float]]) -> bool:
        """A top-side part of a U1 pad must be reachable along that pad's escape channel:
        a same-net pad on the same side of U1, at most the channel's jog off the pad line
        (plus the pad's own half-width). Outside the escape zone anything goes."""
        for px, py in pads:
            side, t, r = polar(px, py)
            if r > ESCAPE_REACH_MM:
                return True
            for tx, ty in targets:
                tside, tt, _ = polar(tx, ty)
                if side == tside and abs(t - tt) <= EscapeModel._jog(r - 0.28) + 0.2 + EPS:
                    return True
        return False

    def ideal_spot(self, ref: str) -> Shape | None:
        """Best spot for a failed Near part if movable parts were out of the way.

        Movable = absolute parts that are not edge connectors, fiducials and all
        other Near parts. Used by the repair loop in :func:`build_plan`.
        """
        comp = self.comps[ref]
        near: bs.Near = comp.place
        if near.ref not in self.plan.placements:
            return None
        geom = self.geom_of(comp)
        mapping = self.plan.pad_maps[comp.ref]
        targets, nets = self._target_points(near, comp)
        same = {fp for spec, net in comp.conns.items() if net in nets for fp in mapping.get(spec, ())}
        gnd = {fp for spec, net in comp.conns.items() if net == bs.GND for fp in mapping.get(spec, ())}
        movable = frozenset(p.ref for p in self.plan.placements.values()
                            if p.method.startswith(("absolute", "fiducial", "near")) and p.ref != near.ref)
        saved, self.use_reservations = self.use_reservations, False
        pl = self._search(comp, geom, targets, same, gnd, self.plan.placements[near.ref],
                          near.max_mm, 0.1, movable)
        self.use_reservations = saved
        return pl.shape() if pl else None

    # ------------------------------------------------------------ 6.5 silkscreen
    def place_header_legends(self) -> None:
        """Pin labels next to a header, laid out like the header itself.

        One text line per pin row, on the courtyard side with more room to the
        board edge; the line nearest the header labels the nearest pin row. The
        legend is reserved before the Near parts are placed, so they avoid it.
        """
        for ref, attr in HEADER_LEGENDS.items():
            labels = getattr(bs, attr, None)
            pl = self.plan.placements.get(ref)
            if not labels or not pl:
                continue
            pads = {n: (x, y) for n, x, y in pl.pad_positions()}
            pins = [(str(i), lab) for i, lab in enumerate(labels, 1) if str(i) in pads]
            xs = [pads[n][0] for n, _ in pins]
            ys = [pads[n][1] for n, _ in pins]
            along_x = (max(xs) - min(xs)) >= (max(ys) - min(ys))
            ax, px = (0, 1) if along_x else (1, 0)          # axis along the rows / across them
            court = pl.shape().rect
            lo, hi = (bs.BOARD_OUTLINE[1], bs.BOARD_OUTLINE[3]) if along_x else \
                     (bs.BOARD_OUTLINE[0], bs.BOARD_OUTLINE[2])
            side = 1 if (hi - court[px + 2]) >= (court[px] - lo) else -1
            rows = sorted({round(pads[n][px], 3) for n, _ in pins}, key=lambda r: -side * r)
            size = LEGEND_TEXT_SIZE_MM
            rowmap: dict[float, list[tuple[float, str]]] = {}
            for n, lab in pins:
                rowmap.setdefault(round(pads[n][px], 3), []).append((pads[n][ax], lab))

            def fits(w: float) -> bool:   # neighbours in a line keep >= 0.15 mm of air
                for items in rowmap.values():
                    items = sorted(items)
                    for (a, la), (b, lb) in zip(items, items[1:]):
                        if (text_width(la, size, w) + text_width(lb, size, w)) / 2 + 0.15 > b - a:
                            return False
                return True
            width = size
            while width > 0.45 and not fits(width):
                width = round(width - 0.05, 2)     # condense the glyphs, keep the height
            line = size * 1.35
            edge = court[px + 2] if side > 0 else court[px]
            texts = []
            for k, row in enumerate(rows):
                c = edge + side * (SILK_CLEAR_MM + line / 2 + k * line)
                for n, lab in pins:
                    if round(pads[n][px], 3) == row:
                        pos = [0.0, 0.0]
                        pos[ax], pos[px] = pads[n][ax], c
                        texts.append(SilkText(lab, round(pos[0], 4), round(pos[1], 4), size,
                                              0.0 if along_x else 90.0, kind=f"legend:{ref}",
                                              width=None if width == size else width))
            self.plan.silk.extend(texts)
            box = (min(t.bbox[0] for t in texts), min(t.bbox[1] for t in texts),
                   max(t.bbox[2] for t in texts), max(t.bbox[3] for t in texts))
            self._silk_boxes.append(Shape(box))
            self.plan.log.append(f"{ref}: {len(texts)} pin labels ({size} mm high, {width} mm "
                                 f"glyph width) in {len(rows)} lines, "
                                 f"{'below' if side > 0 else 'above'} the header, nearest line = nearest row")

    def place_references(self) -> None:
        """Put every silkscreen reference where it touches no pad, courtyard or other silk.

        Tried in order: the library position (rotated with the part, text kept
        upright), then centred above / below / left / right of the courtyard, at
        1.0 mm and then 0.8 mm text height. Parts with no legal spot -- and every
        part inside the heatsink keep-out, where silk would sit under the
        heatsink -- get their reference on the Fab layer instead.
        """
        k = bs.KEEPOUT_HALF
        keep = Shape((-k, -k, k, k))
        pads: dict[str, list[tuple[Shape, bool]]] = {}
        for p in self.plan.placements.values():
            pads[p.ref] = [(sh, p.tht) for sh in p.pad_shapes()]
        taken: list[Shape] = [Shape(t.bbox) for t in self.plan.silk]
        moved = 0
        for ref in sorted(self.plan.placements, key=comp_sort_key):
            pl = self.plan.placements[ref]
            rt = pl.geom.ref_text
            if not pl.ref_on_silk or not rt or not rt[3].endswith("SilkS"):
                continue
            court = self._shapes[ref].rect
            lx, ly = to_board(rt[0], rt[1], pl.x, pl.y, pl.rot, pl.side)
            cands = [(lx, ly, rt[2])]
            for size in (1.0, 0.8):
                h, w = 1.15 * size, text_width(ref, size)
                cx, cy = (court[0] + court[2]) / 2, (court[1] + court[3]) / 2
                g = 0.15
                cands += [(cx, court[1] - g - h / 2, size), (cx, court[3] + g + h / 2, size),
                          (court[0] - g - w / 2, cy, size), (court[2] + g + w / 2, cy, size)]
            best = None
            for x, y, size in cands:
                box = Shape(ref_ink_box(ref, x, y, size, 0.0))
                if self._ref_spot_ok(pl, box, keep, pads, taken):
                    best = (x, y, size, box)
                    break
            if best is None:
                pl.ref_on_silk = False
                moved += 1
                continue
            pl.ref_xy, pl.ref_size = (round(best[0], 4), round(best[1], 4)), best[2]
            taken.append(best[3])
        self.plan.log.append(f"references: {moved} moved to Fab (no clean silk spot)")

    def _ref_spot_ok(self, pl: Placement, box: Shape, keep: Shape, pads, taken) -> bool:
        g = 0.1
        grown = Shape((box.rect[0] - g, box.rect[1] - g, box.rect[2] + g, box.rect[3] + g))
        if not Shape((box.rect[0] - 0.2, box.rect[1] - 0.2, box.rect[2] + 0.2, box.rect[3] + 0.2)
                     ).inside_outline():
            return False
        if box.overlaps(keep):
            return False
        if pl.geom.silk and box.overlaps_rect(xform_rect(pl.geom.silk, pl.x, pl.y, pl.rot, pl.side)):
            return False
        for ref in self._nearby(grown.rect):
            other = self.plan.placements[ref]
            same_side = other.side == pl.side or other.tht
            if ref != pl.ref and same_side and box.overlaps(self._shapes[ref]):
                return False
            if same_side or ref == pl.ref:
                if any(grown.overlaps(sh) for sh, _ in pads[ref]):
                    return False
        return not any(grown.overlaps(t) for t in taken)

    def place_license_text(self) -> None:
        """The licence line: free F.SilkS spot closest to the bottom-right corner."""
        x0, y0, x1, y1 = bs.BOARD_OUTLINE
        m = SILK_CLEAR_MM + 0.3
        best = None
        for size in (LICENSE_TEXT_SIZE_MM, 0.7):
            for angle in (0.0, 90.0):
                probe = SilkText(LICENSE_TEXT, 0.0, 0.0, size, angle, kind="licence")
                hw, hh = probe.bbox[2], probe.bbox[3]
                ys = [round(y1 - m - hh - 0.25 * k, 3) for k in range(int((y1 - y0) / 0.25))]
                xs = [round(x1 - m - hw - 0.25 * k, 3) for k in range(int((x1 - x0) / 0.25))]
                for cy in ys:
                    if cy - hh < y0 + m:
                        break
                    for cx in xs:
                        if cx - hw < x0 + m:
                            break
                        t = SilkText(LICENSE_TEXT, cx, cy, size, angle, kind="licence")
                        if self._silk_free(t):
                            score = math.hypot(x1 - (cx + hw), y1 - (cy + hh))
                            if best is None or score < best[0] - EPS:
                                best = (score, t)
                            break           # further left on this line only scores worse
                if best:
                    break
            if best:
                break
        if not best:
            self.plan.warn("no free spot for the licence text on F.SilkS")
            return
        self.plan.silk.append(best[1])
        self.plan.log.append(f"licence text at ({best[1].x}, {best[1].y}) size {best[1].size} "
                             f"angle {best[1].angle:g}")

    def _silk_free(self, t: SilkText) -> bool:
        r = t.bbox
        g = SILK_CLEAR_MM
        box = Shape((r[0] - g, r[1] - g, r[2] + g, r[3] + g))
        if not box.inside_outline():
            return False
        if box.overlaps_rect(self.keepout):
            return False
        for hx, hy in list(bs.HEATSINK_HOLES.values()):
            if box.hits_circle(hx, hy, bs.HEATSINK_HOLE_KEEPOUT_R):
                return False
        for ref in self._nearby(box.rect):
            p = self.plan.placements[ref]
            if (p.side == "F" or p.tht) and box.overlaps(self._shapes[ref]):
                return False
            rb = p.ref_box() if p.side == "F" else None
            if rb and box.overlaps_rect(rb):
                return False
        return not any(box.overlaps_rect(s.bbox) for s in self.plan.silk)

    # ------------------------------------------------------------ 6.6 the rest
    def build_vias(self) -> None:
        tv = bs.THERMAL_VIA
        n, pitch = tv["grid"], tv["pitch_mm"]
        for i in range(n):
            for j in range(n):
                x = round((i - (n - 1) / 2) * pitch, 4)
                y = round((j - (n - 1) / 2) * pitch, 4)
                self.plan.vias.append(Via(x, y, tv["drill_mm"], tv["pad_mm"], bs.GND))

    def build_zones(self) -> None:
        x0, y0, x1, y1 = bs.BOARD_OUTLINE
        board = rounded_rect_polygon(bs.BOARD_OUTLINE, bs.BOARD_CORNER_R)
        z = self.plan.zones
        z.append(Zone("GND_L2", ("In1.Cu",), board, bs.GND, 0, "solid",
                      note="solid GND reference plane"))
        z.append(Zone("3V3_L3", ("In2.Cu",), board, "+3V3", 0, "solid", note="+3V3 plane"))
        z.append(Zone("VDD_HP_ISLAND", ("In2.Cu",), square(VDD_HP_ISLAND_HALF_MM), "VDD_HP", 1,
                      "solid", note="VDD_HP island under U1 (higher priority than +3V3)"))
        z.append(Zone("GND_TOP", ("F.Cu",), board, bs.GND, 0, "thermal",
                      note="top GND pour; U1 EPAD overridden to solid"))
        z.append(Zone("GND_BOTTOM", ("B.Cu",), board, bs.GND, 0, "thermal", note="bottom GND pour"))
        z.append(Zone("GND_HEAT_SPREADER", ("B.Cu",), square(HEAT_SPREADER_HALF_MM), bs.GND, 1,
                      "thermal", note="14 x 14 mm heat spreader under U1 (thermal relief on pads "
                                      "protects bottom 0402s; the vias connect solid)"))
        # rule areas
        k = bs.KEEPOUT_HALF
        z.append(Zone(HEATSINK_AREA, ("F.Cu",), square(k), rule_area=True,
                      note="named area for the custom DRC rule (no built-in restrictions)"))
        e = p4.EPAD_SIZE_MM / 2
        z.append(Zone(EPAD_VIA_AREA, ("F.Cu", "B.Cu"), square(e), rule_area=True,
                      note="named area: thermal-via size rule"))
        for h, (hx, hy) in bs.HEATSINK_HOLES.items():
            z.append(Zone(f"{h}_KEEPOUT", ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu"),
                          circle_polygon(hx, hy, bs.HEATSINK_HOLE_KEEPOUT_R), rule_area=True,
                          keepout=("tracks", "vias"),
                          note=f"standoff/washer keep-out R={bs.HEATSINK_HOLE_KEEPOUT_R} mm"))

    def build_graphics(self) -> None:
        g = self.plan.graphics
        g.extend(rounded_rect_outline(bs.BOARD_OUTLINE, bs.BOARD_CORNER_R))
        k = bs.KEEPOUT_HALF
        hs = bs.HEATSINK
        g.append(Graphic("rect", "User.1", ((-k, -k), (k, k)), 0.15))
        g.append(Graphic("text", "User.1", ((0.0, -k - 1.2),),
                         text=f"HEATSINK {hs['base_mm'][0]:g}x{hs['base_mm'][1]:g} KEEP-OUT "
                              f"(U1 + 0402 <= {bs.KEEPOUT_MAX_HEIGHT_MM} mm only)"))
        f = FAN_MM / 2
        g.append(Graphic("rect", "User.2", ((-f, -f), (f, f)), 0.15))
        g.append(Graphic("text", "User.2", ((0.0, f + 1.2),), text=f"FAN {FAN_MM:g}x{FAN_MM:g}"))
        for hx, hy in bs.HEATSINK_HOLES.values():
            g.append(Graphic("circle", "User.1", ((hx, hy), (hx + bs.HEATSINK_HOLE_KEEPOUT_R, hy)), 0.1))
        w = self.via_field_half
        g.append(Graphic("rect", "B.Mask", ((-w, -w), (w, w)), 0.0, filled=True,
                         text="exposed-copper window for the optional bottom thermal pad"))

    def build_netclasses(self) -> None:
        r = DESIGN_RULES
        rules = {x.netclass: x for x in imp.impedance_rules()}
        std = dict(via_diameter=r["via_diameter"], via_drill=r["via_drill"],
                   uvia_diameter=r["uvia_diameter"], uvia_drill=r["uvia_drill"])
        ncs = [NetClassPlan("Default", 0.15, 0.10, description="HDI default", **std)]
        se = rules["SE_50"]
        ncs.append(NetClassPlan("SE_50", se.width_mm, 0.15,
                                description=f"{se.target_ohm:g} ohm SE ({se.nets_hint})", **std))
        for name in ("USB_90", "MIPI_100", "ETH_100"):
            d = rules[name]
            ncs.append(NetClassPlan(name, d.width_mm, 0.20, dp_width=d.width_mm, dp_gap=d.gap_mm,
                                    description=f"{d.target_ohm:g} ohm diff ({d.nets_hint})", **std))
        ncs.append(NetClassPlan("POWER", 0.50, 0.15, 0.60, 0.30, r["uvia_diameter"], r["uvia_drill"],
                                description="power rails"))
        self.plan.netclasses = ncs
        self.plan.net_assignments = {net: bs.netclass_of(net) for net in sorted(bs.nets())
                                     if bs.netclass_of(net) != "Default"}

    # ------------------------------------------------------------ driver
    def run(self) -> Plan:
        for comp in bs.COMPONENTS:
            geom = self.geom_of(comp)
            mapping, notes = pad_map(comp, geom)
            self.plan.pad_maps[comp.ref] = mapping
            for note in notes:
                self.plan.warn(f"{comp.ref} ({geom.lib_id}): {note}")
        self.place_fixed()
        self.place_absolute()
        self.place_fiducials()
        self.place_header_legends()
        # (SoC ring parts are placed by the Near search, under the escape-channel rule)
        self.place_near_all()
        self.place_references()
        self.place_license_text()
        self.build_vias()
        self.build_zones()
        self.build_graphics()
        self.build_netclasses()
        return self.plan


# ==========================================================================
# 7. Small geometry builders
# ==========================================================================

def square(half: float) -> tuple[tuple[float, float], ...]:
    return ((-half, -half), (half, -half), (half, half), (-half, half))


def circle_polygon(cx: float, cy: float, r: float, n: int = 48) -> tuple[tuple[float, float], ...]:
    """Polygon that *contains* the circle (vertices on the circumscribed radius)."""
    rr = r / math.cos(math.pi / n)
    return tuple((round(cx + rr * math.cos(2 * math.pi * k / n), 4),
                  round(cy + rr * math.sin(2 * math.pi * k / n), 4)) for k in range(n))


def rounded_rect_polygon(r: Rect, radius: float, n: int = 8) -> tuple[tuple[float, float], ...]:
    x0, y0, x1, y1 = r
    pts = []
    for cx, cy, a0 in ((x1 - radius, y0 + radius, -90), (x1 - radius, y1 - radius, 0),
                       (x0 + radius, y1 - radius, 90), (x0 + radius, y0 + radius, 180)):
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((round(cx + radius * math.cos(a), 4), round(cy + radius * math.sin(a), 4)))
    return tuple(pts)


def rounded_rect_outline(r: Rect, radius: float) -> list[Graphic]:
    x0, y0, x1, y1 = r
    m = radius * (1 - math.sqrt(0.5))
    g = [Graphic("line", "Edge.Cuts", ((x0 + radius, y0), (x1 - radius, y0)), 0.1),
         Graphic("line", "Edge.Cuts", ((x1, y0 + radius), (x1, y1 - radius)), 0.1),
         Graphic("line", "Edge.Cuts", ((x1 - radius, y1), (x0 + radius, y1)), 0.1),
         Graphic("line", "Edge.Cuts", ((x0, y1 - radius), (x0, y0 + radius)), 0.1)]
    arcs = (((x1 - radius, y0), (x1 - m, y0 + m), (x1, y0 + radius)),
            ((x1, y1 - radius), (x1 - m, y1 - m), (x1 - radius, y1)),
            ((x0 + radius, y1), (x0 + m, y1 - m), (x0, y1 - radius)),
            ((x0, y0 + radius), (x0 + m, y0 + m), (x0 + radius, y0)))
    for a in arcs:
        g.append(Graphic("arc", "Edge.Cuts", tuple((round(px, 4), round(py, 4)) for px, py in a), 0.1))
    return g


# ==========================================================================
# 8. Public entry points: plan, checks, DRC rules, report
# ==========================================================================

_PLAN_CACHE: dict[bool, Plan] = {}


def _near_failures(plan: Plan) -> list[str]:
    return [c.ref for c in bs.COMPONENTS if isinstance(c.place, bs.Near)
            and (plan.placements[c.ref].dist_mm or 0) > c.place.max_mm + EPS]


def build_plan(allow_bottom: bool = True, use_cache: bool = True, max_passes: int = 6) -> Plan:
    """Run the planner with a deterministic repair loop.

    After each pass, Near parts that missed their ``max_mm`` are *promoted*
    (placed first next time) and their ideal spot -- computed as if movable
    absolute parts and other Near parts were absent -- is *reserved*, so that
    absolute parts in its way are nudged (and reported as spec conflicts).
    """
    if use_cache and allow_bottom in _PLAN_CACHE:
        return _PLAN_CACHE[allow_bottom]
    promoted: list[str] = []
    reservations: dict[str, Shape] = {}
    ring = {c.ref for c in bs.COMPONENTS if isinstance(c.place, bs.Near) and c.place.ref == "U1"
            and bs.is_keepout_exempt(c)}       # solved globally by the slot assignment
    best: tuple[int, Plan] | None = None
    for n in range(1, max_passes + 1):
        planner = Planner(allow_bottom=allow_bottom, promoted=promoted, reservations=reservations)
        plan = planner.run()
        failed = _near_failures(plan)
        if best is not None and len(failed) >= best[0]:
            plan = best[1]                      # no progress: keep the better pass
            break
        best = (len(failed), plan)
        new = [r for r in failed if r not in promoted and r not in ring]
        if not failed or not new:
            break
        for ref in new:
            promoted.append(ref)
            spot = planner.ideal_spot(ref)
            if spot:
                reservations[ref] = spot
    plan.log.append(f"repair loop: {n} pass(es); promoted {promoted or 'none'}")
    _PLAN_CACHE[allow_bottom] = plan
    return plan


def check_overlaps(plan: Plan) -> list[str]:
    out = []
    items = sorted(plan.placements.values(), key=lambda p: comp_sort_key(p.ref))
    shapes = [(p, p.shape()) for p in items]
    for i, (a, sa) in enumerate(shapes):
        for b, sb in shapes[i + 1:]:
            if a.side != b.side and not (a.tht or b.tht):
                continue
            if sa.overlaps(sb):
                out.append(f"courtyards overlap: {a.ref} / {b.ref}")
    return out


def check_keepout(plan: Plan) -> list[str]:
    k = bs.KEEPOUT_HALF
    comps = {c.ref: c for c in bs.COMPONENTS}
    out = []
    for p in plan.placements.values():
        if p.side != "F" or p.ref == "U1":
            continue
        comp = comps.get(p.ref)
        if comp and bs.is_keepout_exempt(comp):
            continue
        if p.shape().overlaps_rect((-k, -k, k, k)):
            out.append(f"{p.ref} ({p.lib_id}) intrudes into the heatsink keep-out")
    return out


def check_holes(plan: Plan) -> list[str]:
    out = []
    for p in plan.placements.values():
        for h, (hx, hy) in bs.HEATSINK_HOLES.items():
            if p.ref != h and p.shape().hits_circle(hx, hy, bs.HEATSINK_HOLE_KEEPOUT_R):
                out.append(f"{p.ref} inside the {h} keep-out circle")
    for v in plan.vias:
        for h, (hx, hy) in bs.HEATSINK_HOLES.items():
            if math.hypot(v.x - hx, v.y - hy) < bs.HEATSINK_HOLE_KEEPOUT_R + v.diameter / 2:
                out.append(f"via at ({v.x}, {v.y}) inside the {h} keep-out circle")
    return out


def check_outline(plan: Plan) -> list[str]:
    out = []
    for p in plan.placements.values():
        if not Planner._inside_outline(p.shape(), p.faces):
            out.append(f"{p.ref} courtyard outside the board outline")
        for pad in p.pad_shapes():
            if not pad.inside_outline():
                out.append(f"{p.ref} has copper outside the board outline")
                break
    return out


def check_near(plan: Plan) -> list[str]:
    out = []
    for comp in bs.COMPONENTS:
        if not isinstance(comp.place, bs.Near):
            continue
        p = plan.placements[comp.ref]
        if p.dist_mm is None or p.dist_mm > comp.place.max_mm + EPS:
            out.append(f"{comp.ref} is {p.dist_mm} mm from {p.target} (max {comp.place.max_mm})")
    return out


def check_bottom(plan: Plan) -> list[str]:
    tv = bs.THERMAL_VIA
    half = (tv["grid"] - 1) / 2 * tv["pitch_mm"] + tv["pad_mm"] / 2
    return [f"{p.ref} on the bottom thermal-via field" for p in plan.placements.values()
            if p.side == "B" and rects_overlap(p.shape().rect, (-half, -half, half, half))]


def validate(plan: Plan) -> list[str]:
    return (check_overlaps(plan) + check_keepout(plan) + check_holes(plan) + check_outline(plan)
            + check_near(plan) + check_bottom(plan))


# ---- custom DRC rules -----------------------------------------------------

def keepout_exempt_refs() -> list[str]:
    return sorted((c.ref for c in bs.COMPONENTS if bs.is_keepout_exempt(c)), key=comp_sort_key)


def _or_nets(nets: list[str]) -> str:
    return " || ".join(f"A.NetName == '{n}'" for n in nets)


def dru_text(plan: Plan | None = None) -> str:
    """KiCad custom design rules (version 1 syntax, KiCad 7/8/9)."""
    exempt = keepout_exempt_refs()
    ex_cond = " && ".join(f"A.Reference != '{r}'" for r in exempt)
    holes = " || ".join(f"A.memberOfFootprint('{h}')" for h in bs.HEATSINK_HOLES)
    tv = bs.THERMAL_VIA
    all_nets = set(bs.nets())
    L = [
        "(version 1)",
        "# ESP32-P4 Extreme Performance -- custom DRC rules.",
        "# GENERATED from hardware/lib/board_spec.py by hardware/pcbnew/layout_plan.py -- do not edit;",
        "# re-run `python3 -m hardware.pcbnew.layout_plan dru` after changing the spec.",
        "# Syntax: KiCad 7+ custom rules (intersectsArea / intersectsCourtyard / inDiffPair).",
        "",
        "# ---------------------------------------------------------------- (1) heatsink keep-out",
        f"# Only U1 and 0402 passives (<= {bs.KEEPOUT_MAX_HEIGHT_MM} mm, board_spec.is_keepout_exempt) "
        f"may touch the {2 * bs.KEEPOUT_HALF:g} x {2 * bs.KEEPOUT_HALF:g} mm area",
        f"# '{HEATSINK_AREA}' (F.Cu rule area). intersectsArea() tests a footprint's courtyard on the",
        "# area's side, so bottom-side parts under the SoC are not affected (the heatsink is on top).",
        f"# {len(exempt)} exempt references are listed explicitly (no wildcards, so the condition is",
        "# unambiguous in every KiCad version).",
        '(rule "heatsink_keepout_footprints"',
        "    (constraint disallow footprint)",
        f"    (condition \"A.intersectsArea('{HEATSINK_AREA}') && {ex_cond}\"))",
        "",
        "# ---------------------------------------------------------------- footprint-intrinsic",
    ]
    usb = [c.ref for c in bs.COMPONENTS if "USB4105" in c.part.footprint]
    if usb:
        cond = " || ".join(f"(A.memberOfFootprint('{r}') && B.memberOfFootprint('{r}'))" for r in usb)
        L += ["# GCT USB4105 land pattern: its own NPTH locating pegs sit 0.194 mm from pads A1/B12/",
              "# A12/B1 (manufacturer drawing). Relax hole clearance only *inside* those footprints.",
              '(rule "usb4105_own_npth_clearance"',
              "    (constraint hole_clearance (min 0.15mm))",
              f"    (condition \"{cond}\"))", ""]
    L += ["# Dense 0402 arrays, the USB-C GND pins and the 0.5 mm-pitch FPC / QFN GND pins leave room",
          "# for only one thermal spoke once the tracks are in; accept 1 instead of the board default 2",
          "# (the plane connection of the 0402 and larger GND pads is their via in pad anyway).",
          '(rule "single_spoke_ok"',
          "    (constraint min_resolved_spokes 1)",
          "    (condition \"A.Type == 'Pad'\"))", ""]
    geo = load_geometry()
    ep = []
    for c in bs.COMPONENTS:            # exposed pad = SMD pad > 1.5 mm^2 at the footprint origin
        if c.ref == "U1":
            continue
        numbers = sorted({p.number for p in geo[c.part.footprint].copper_pads()
                          if p.kind == "smd" and abs(p.x) < 0.01 and abs(p.y) < 0.01
                          and p.sx * p.sy > 1.5 and p.number})
        ep += [f"(A.memberOfFootprint('{c.ref}') && A.Pad_Number == '{n}')" for n in numbers]
    if ep:
        L += ["# Exposed pads (with their thermal-via pads, same number) connect solid, like U1's EPAD.",
              '(rule "exposed_pads_solid"',
              "    (constraint zone_connection solid)",
              f"    (condition \"A.Type == 'Pad' && ({' || '.join(ep)})\"))", ""]
    L += ["# ---------------------------------------------------------------- (2) differential pairs"]
    for nc in plan.netclasses if plan else build_plan().netclasses:
        if nc.dp_gap is None:
            continue
        uncoupled = {"MIPI_100": 1.5, "USB_90": 3.0, "ETH_100": 5.0}[nc.name]
        L += [f'(rule "{nc.name.lower()}_diff_pair"',
              f"    (constraint diff_pair_gap (min {nc.dp_gap * 0.9:.3f}mm) (opt {nc.dp_gap:.3f}mm) "
              f"(max {nc.dp_gap * 1.1:.3f}mm))",
              f"    (constraint diff_pair_uncoupled (max {uncoupled}mm))",
              f"    (constraint track_width (min {nc.dp_width * 0.9:.3f}mm) (opt {nc.dp_width:.3f}mm))",
              f"    (condition \"A.NetClass == '{nc.name}' && A.inDiffPair('*')\"))"]
    L += ["",
          "# ---------------------------------------------------------------- SoC fan-out (0.35 mm pitch)",
          "# Inside the U1 courtyard the netclass clearances cannot be met: neck down to the HDI minimum",
          "# (pads, tracks, vias only -- pours keep their own clearance). The pairs leave U1 as 0.10 mm",
          "# stubs on the pad pitch. These rules come after the pair rules: later rules take precedence.",
          '(rule "u1_fanout_clearance"',
          f"    (constraint clearance (min {DESIGN_RULES['min_clearance']}mm))",
          "    (condition \"A.intersectsCourtyard('U1') && A.Type != 'Zone' && B.Type != 'Zone'\"))",
          '(rule "u1_fanout_track_width"',
          f"    (constraint track_width (min {DESIGN_RULES['min_track']}mm))",
          "    (condition \"A.Type == 'Track' && A.intersectsCourtyard('U1')\"))",
          '(rule "u1_fanout_diff_pair_gap"',
          f"    (constraint diff_pair_gap (min {DESIGN_RULES['min_clearance']}mm) (opt 0.25mm) (max 0.5mm))",
          "    (condition \"A.inDiffPair('*') && A.intersectsCourtyard('U1')\"))",
          "",
          "# ---------------------------------------------------------------- (3) skew / length matching",
          "# Intra-pair skew <= 0.254 mm (10 mil). One rule per pair: a skew rule measures all nets it",
          "# matches as one group (KiCad 8 adds '(within_diff_pairs)'; per-pair rules also work in 7).",
          ]
    for p_net, n_net, cls in bs.DIFF_PAIRS:
        if p_net in all_nets and n_net in all_nets:
            L += [f'(rule "skew_{p_net.lower()}"', "    (constraint skew (max 0.254mm))",
                  f"    (condition \"{_or_nets([p_net, n_net])}\"))"]
    L += ["",
          "# MIPI D-PHY inter-lane matching (Espressif: lanes of one link within 0.762 mm / 30 mil).",
          "# KiCad measures each net separately and cannot sum across the 0R series resistors, so the",
          "# SoC-side and connector-side segments are matched separately (tighten on the long side).",
          ]
    for pfx in ("CSI", "CAM", "DSI", "DISP"):
        link = [f"{pfx}_{lane}_{pol}" for lane in ("D0", "D1", "CLK") for pol in ("P", "N")]
        if all(n in all_nets for n in link):
            L += [f'(rule "mipi_{pfx.lower()}_lane_matching"', "    (constraint skew (max 0.762mm))",
                  f"    (condition \"{_or_nets(link)}\"))"]
    sd_c = [f"SD_{s}_C" for s in ("CLK", "CMD", "D0", "D1", "D2", "D3")]
    sd_s = [f"SD_{s}" for s in ("CLK", "CMD", "D0", "D1", "D2", "D3")]
    L += ["",
          "# SDIO: CMD/D0-D3 within +/-1.27 mm (50 mil) of SD_CLK. KiCad's skew is measured against the",
          "# longest net of the group (not against CLK), so a 1.27 mm group skew is the closest",
          "# expressible form; it is stricter than or equal to the intent.",
          '(rule "sdio_card_side_matching"', "    (constraint skew (max 1.27mm))",
          f"    (condition \"{_or_nets([n for n in sd_c if n in all_nets])}\"))",
          '(rule "sdio_soc_side_matching"', "    (constraint skew (max 1.27mm))",
          f"    (condition \"{_or_nets([n for n in sd_s if n in all_nets])}\"))",
          "",
          "# ---------------------------------------------------------------- (4) thermal vias",
          f"# {tv['grid']}x{tv['grid']} GND vias in the EPAD ('{EPAD_VIA_AREA}' rule area): "
          f"{tv['drill_mm']} mm drill / {tv['pad_mm']} mm pad, {tv['fill']}.",
          '(rule "thermal_via_size"',
          f"    (constraint hole_size (min {tv['drill_mm']}mm) (max {tv['drill_mm']}mm))",
          f"    (constraint via_diameter (min {tv['pad_mm']}mm))",
          f"    (condition \"A.Type == 'Via' && A.intersectsArea('{EPAD_VIA_AREA}')\"))",
          "",
          "# ---------------------------------------------------------------- (5) M2.5 heatsink holes",
          f"# H1..H4 also carry '{list(bs.HEATSINK_HOLES)[0]}_KEEPOUT'.. rule areas "
          f"(R = {bs.HEATSINK_HOLE_KEEPOUT_R} mm, no tracks/vias).",
          '(rule "m25_hole_to_hole"', "    (constraint hole_to_hole (min 0.4mm))",
          f"    (condition \"{holes}\"))",
          '(rule "m25_hole_clearance"', "    (constraint hole_clearance (min 0.5mm))",
          "    (constraint clearance (min 0.3mm))",
          f"    (condition \"({holes}) && B.NetName != 'GND'\"))",
          "",
          "# ---------------------------------------------------------------- aspirational (comments only)",
          "# * Keep-out height limit (0.60 mm) is mechanical; DRC cannot see component height.",
          "# * Microvia-only fan-out inside U1: (condition \"A.Via_Type == 'Micro'\") -- property value",
          "#   spelling differs between versions, so the board minimums in the .kicad_pro cover it.",
          "# * Total MIPI/SDIO length through the 0R resistors needs net chaining (not in KiCad 8/9).",
          ""]
    return "\n".join(L)


# ---- report ---------------------------------------------------------------

def report(plan: Plan) -> str:
    L = ["ESP32-P4 Extreme Performance -- layout plan", "=" * 60]
    n = len(plan.placements)
    by_method: dict[str, int] = {}
    for p in plan.placements.values():
        by_method[p.method] = by_method.get(p.method, 0) + 1
    L.append(f"{n} footprints placed: " + ", ".join(f"{k} {v}" for k, v in sorted(by_method.items())))
    L.append(f"{len(plan.vias)} thermal vias, {len(plan.zones)} zones/rule areas, "
             f"{len(plan.graphics)} graphics, {len(plan.netclasses)} net classes "
             f"({len(plan.net_assignments)} nets assigned)")
    L.append("")
    L.append("Edge connectors (mating-side heuristic)")
    for c in plan.connectors:
        L.append(f"  {c.ref:6s} faces {c.faces:6s} local {c.direction} -> rot {c.rot:g} "
                 f"(spec rot {c.spec_rot:g}); edge dev {c.edge_dev_mm:+.2f} mm; {c.action}")
        L.append(f"         {c.lib_id.split(':')[1]}: {c.evidence}")
    L.append("")
    L.append(f"Spec conflicts resolved ({len(plan.conflicts)})")
    L += [f"  - {c}" for c in plan.conflicts] or ["  none"]
    L.append("")
    L.append(f"Warnings ({len(plan.warnings)})")
    L += [f"  - {w}" for w in plan.warnings] or ["  none"]
    L.append("")
    L.append(f"{'ref':7s}{'footprint':44s}{'x':>9s}{'y':>9s}{'rot':>6s} side  method        target"
             "       dist/max")
    for p in sorted(plan.placements.values(), key=lambda p: comp_sort_key(p.ref)):
        tgt = f"{p.target:12s} {p.dist_mm:.2f}/{p.max_mm:g}" if p.target else ""
        L.append(f"{p.ref:7s}{p.lib_id.split(':')[1][:43]:44s}{p.x:9.3f}{p.y:9.3f}{p.rot:6.0f}"
                 f"  {p.side}   {p.method:13s} {tgt}")
    problems = validate(plan)
    L.append("")
    L.append("validate(): " + ("clean" if not problems else f"{len(problems)} problem(s)"))
    L += [f"  ! {x}" for x in problems]
    return "\n".join(L)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd")
    r = sub.add_parser("report", help="print the placement report (default)")
    r.add_argument("--top-only", action="store_true", help="never use B.Cu for the SoC ring")
    d = sub.add_parser("dru", help="write the custom DRC rules")
    d.add_argument("-o", "--output", type=Path, default=DRU_FILE)
    e = sub.add_parser("escapes", help="solve and write the U1 escape tracks (needs scipy)")
    e.add_argument("-o", "--output", type=Path, default=ESCAPE_FILE)
    c = sub.add_parser("courtyards", help="(re)generate footprint_courtyards.json")
    c.add_argument("--cache", type=Path, default=DEFAULT_CACHE,
                   help="footprint cache ($KICAD_FP_CACHE, default ~/.cache/esp32p4-footprints)")
    c.add_argument("--tag", default=LIBRARY_TAG)
    c.add_argument("--offline", action="store_true", help="use only files already in --cache")
    args = ap.parse_args(argv)
    if args.cmd == "courtyards":
        res = refresh_courtyards(args.cache, args.tag, fetch=not args.offline)
        print(f"wrote {COURTYARD_JSON} ({len(res['footprints'])} footprints)")
        return 0
    if args.cmd == "escapes":
        plan = build_plan()
        data = escape_json(plan)
        bad = check_escape_tracks(plan, load_escape_tracks_from(data))
        args.output.write_text(json.dumps(data, indent=1) + "\n")
        print(f"wrote {args.output}: {len(data['tracks'])} escape tracks; "
              f"{len(data['problems'])} notes, {len(bad)} clearance problems")
        for x in data["problems"] + bad:
            print("  " + x)
        return 1 if bad else 0
    if args.cmd == "dru":
        args.output.write_text(dru_text(build_plan()))
        print(f"wrote {args.output}")
        return 0
    plan = build_plan(allow_bottom=not getattr(args, "top_only", False))
    print(report(plan))
    return 1 if validate(plan) else 0


if __name__ == "__main__":
    sys.exit(main())
