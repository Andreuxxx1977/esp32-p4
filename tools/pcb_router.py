"""Route the board with a router written for this design (no FreeRouting needed).

Input: the placed board plus its pre-routing (``hardware/pcbnew/preroute.py``: U1
fan-out stubs, SoC escape tracks, inner / plane / thermal vias).
Output: the same board with the remaining connections routed. KiCad's DRC is the judge
(``python -m tools.check_drc_report --routed``); nothing here is trusted on its own.

How it routes -- the usual practice for a 4-layer board with GND on L2 and power on L3,
and Espressif's ESP32-P4 PCB layout guide:

* F.Cu and B.Cu carry the signals. In1.Cu (GND) and In2.Cu (+3V3 with the VDD_HP island)
  stay whole planes: a plane pad gets its own via next to it (laser microvia F->In1 for
  GND on top, a through via otherwise) on a short, wide track.
* Differential pairs (USB 90 ohm, MIPI D-PHY and Ethernet 100 ohm) go first and are
  routed *coupled*: one centre line as wide as the pair plus its clearance, offset into
  the P and N tracks at the netclass gap, on F.Cu over the unbroken GND plane, without
  vias and without right-angle bends; short uncoupled fan-ins join the terminals.
* Then clocks and 50-ohm single-ended nets (crystal without vias), power rails as wide
  as they fit (25 mil for 3.3 V, 20 mil for the SoC core rail, >= 10 mil for the other
  supplies), and the rest, shortest first.
* Tracks are octilinear. The search pays for every bend (45 degrees a little, right
  angles more, acute angles never), so a track is a few long straight runs.
* A connection that finds no room rips up the routes in its way; they go back into the
  queue and the contested cells get a growing history cost (rip-up and reroute, with
  PathFinder-style negotiation).
* Clearances are exact: the distance from every grid cell to every copper shape is
  computed from the shape itself (rounded rectangles, circles, track capsules), with
  the larger of the two nets' clearances plus a small margin.

The search core (A* with a direction per state, exact distance fields) is C
(``tools/router_core.c``), compiled with the system C compiler on first use.

Usage::

    python -m tools.pcb_router PREROUTED.kicad_pcb -o ROUTED.kicad_pcb [--grid 0.05]
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import math
import os
import re
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from hardware.lib import board_spec as bs  # noqa: E402
from hardware.pcbnew import layout_plan as lp  # noqa: E402

ORIGIN = lp.PAGE_ORIGIN_MM          # board file coordinates = design coordinates + ORIGIN
LAYER_NAMES = ("F.Cu", "B.Cu", "In2.Cu")   # routing layers 0, 1, 2 (In1.Cu stays a solid GND plane)
TOP, BOT, IN2 = 1, 2, 4                    # layer bits
NL = 3
IN2_COST = 0.6                      # extra cost per step on In2 (the +3V3 plane layer): only when needed
GRID_MM = 0.05
MARGIN_MM = 0.01                    # on top of every clearance (grid slack)
CLASSES = (0.10, 0.15, 0.20)        # clearance classes of obstacles; slot 3 = the pair partner,
PARTNER = 3                         # slot 4 = unplated holes (hole clearance, never necked down),
HOLE = 4                            # slot 5 = outlines of rule areas that only forbid vias
VIAKO = 5
NCLS = 6
HOLE_CLEARANCE = lp.DESIGN_RULES["hole_clearance"]
EDGE_MM = lp.DESIGN_RULES["copper_edge"]
HOLE_TO_HOLE = lp.DESIGN_RULES["hole_to_hole"]
VIA = (lp.DESIGN_RULES["via_diameter"], lp.DESIGN_RULES["via_drill"])      # 0.45 / 0.20
MICRO = (0.30, 0.10)                # laser microvia (autoroute.MICROVIA_MM)
NPTH, KEEPOUT = -1, -2              # pseudo net ids

# search costs, in grid steps
TURN45, TURN90 = 3.0, 20.0
VIA_COST = 40.0                     # a via is worth 2 mm of track
RIP_COST = 25.0                     # per step over a routed track of another net (rip-up search)
HISTORY_STEP = 10.0
MAX_RIPS = 12
PAIR_PUSH_MAX = 2                   # times a pair may be pushed aside by a walled-in connection
WINDOWS_MM = (3.0, 8.0, 20.0)       # search windows around a connection, tried in turn
MAX_WINDOW_CELLS = 2_600_000

PAIR_CLASSES = {"USB_90": (0.15, 0.15), "MIPI_100": (0.135, 0.20), "ETH_100": (0.135, 0.20)}
PAIR_GAP_SLACK = 0.004              # gap routed above the netclass gap (rounding to nm)
PAIR_LAUNCH_MM = (0.8, 1.6, 3.0)    # where the coupled part may start, around the terminals
PAIR_MIN_COUPLED_MM = 2.5           # shorter pair connections are two plain tracks
PLANE_NETS = (bs.GND, "+3V3")         # routed to a via into their plane (VDD_HP: a real track)
BREAKOUT_HALF = bs.KEEPOUT_HALF        # DRU 'soc_breakout_clearance' (F.Cu, heatsink area)
BREAKOUT_CLEARANCE = lp.BREAKOUT_CLEARANCE_MM
SUPPLY_PREFIXES = ("VDD", "VDDO", "PHY_AVDD", "PHY_VDDCR", "VBUS", "VSYS", "SD_VDD", "FAN_5V")
UUID_NS = uuid.UUID("0b6c2f55-7d0e-4a8e-9a57-3f1f0c9d2e41")


# ==========================================================================
# C core
# ==========================================================================

def _core():
    src = Path(__file__).with_name("router_core.c")
    tag = hashlib.sha1(src.read_bytes()).hexdigest()[:12]
    cache = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache") / "esp32p4_router"
    so = cache / f"router_core_{tag}.so"
    if not so.exists():
        cache.mkdir(parents=True, exist_ok=True)
        tmp = so.with_name(f"{so.name}.{os.getpid()}.tmp")
        subprocess.run([os.environ.get("CC", "cc"), "-O2", "-std=gnu99", "-shared", "-fPIC",
                        "-o", str(tmp), str(src), "-lm"], check=True)
        tmp.replace(so)
    lib = ctypes.CDLL(str(so))
    P = np.ctypeslib.ndpointer
    i32, f32, f64, u8 = (P(dtype=t, flags="C_CONTIGUOUS") for t in (np.int32, np.float32, np.float64, np.uint8))
    lib.fields.argtypes = [ctypes.c_int, i32, i32, i32, i32, f64, f64, f64,
                           ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_int,
                           ctypes.c_double, ctypes.c_int, ctypes.c_int, f32, f32]
    lib.fields.restype = None
    lib.astar.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, f32, ctypes.c_int, i32, u8, f32, u8, u8, i32,
                          ctypes.c_float, ctypes.c_float, ctypes.c_long, i32, ctypes.c_int]
    lib.astar.restype = ctypes.c_int
    return lib


_LIB = None


def core():
    global _LIB
    if _LIB is None:
        _LIB = _core()
    return _LIB


# ==========================================================================
# Board file
# ==========================================================================

_TOK = re.compile(r'"(?:[^"\\]|\\.)*"|[()]|[^\s()"]+')


def sexpr(text: str) -> list:
    stack: list[list] = [[]]
    for t in _TOK.findall(text):
        if t == "(":
            stack.append([])
        elif t == ")":
            node = stack.pop()
            stack[-1].append(node)
        else:
            stack[-1].append(t[1:-1] if t[0] == '"' else t)
    return stack[0][0]


def child(node: list, key: str):
    for x in node[1:]:
        if isinstance(x, list) and x and x[0] == key:
            return x
    return None


def children(node: list, key: str) -> list:
    return [x for x in node[1:] if isinstance(x, list) and x and x[0] == key]


def rot(x: float, y: float, deg: float) -> tuple[float, float]:
    """KiCad rotation (y down, positive = counter-clockwise on screen)."""
    if deg == 0:
        return x, y
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return x * c + y * s, -x * s + y * c


def layer_bits(names) -> int:
    bits = 0
    for n in names:
        if n in ("F.Cu", "*.Cu", "F&B.Cu"):
            bits |= TOP
        if n in ("B.Cu", "*.Cu", "F&B.Cu"):
            bits |= BOT
        if n in ("In2.Cu", "*.Cu"):
            bits |= IN2
    return bits


@dataclass
class Item:
    """A piece of copper (or a hole / keep-out) in design coordinates."""
    kind: int                  # 0 rounded rect (cx cy hx hy angle r), 1 circle (cx cy r), 2 capsule (x0 y0 x1 y1 r)
    par: tuple
    layers: int
    net: str
    what: str                  # pad | smd | tht | track | via | micro | npth | keepout
    hole: tuple = (0.0, 0.0, 0.0)
    fixed: bool = True
    conn: int = -1             # connection that routed it
    ref: str = ""
    planes: str = ""           # inner layers reached: "1" (In1), "2" (In2), "12"
    width: float = 0.0
    rewrite: bool = False      # fixed, but written by the router (a trimmed pre-routed track)

    def bbox(self) -> tuple[float, float, float, float]:
        p = self.par
        if self.kind == 0:
            e = math.hypot(p[2], p[3]) if p[4] else None
            hx, hy = (e, e) if e else (p[2], p[3])
            return p[0] - hx, p[1] - hy, p[0] + hx, p[1] + hy
        if self.kind == 1:
            return p[0] - p[2], p[1] - p[2], p[0] + p[2], p[1] + p[2]
        return min(p[0], p[2]) - p[4], min(p[1], p[3]) - p[4], max(p[0], p[2]) + p[4], max(p[1], p[3]) + p[4]

    def anchors(self) -> list[tuple[float, float]]:
        p = self.par
        if self.kind == 2:
            return [(p[0], p[1]), (p[2], p[3])]
        return [(p[0], p[1])]

    def dist(self, x: float, y: float) -> float:
        p = self.par
        if self.kind == 0:
            dx, dy = x - p[0], y - p[1]
            if p[4]:
                c, s = math.cos(p[4]), math.sin(p[4])
                dx, dy = dx * c + dy * s, -dx * s + dy * c
            r = p[5]
            ex, ey = max(abs(dx) - (p[2] - r), 0.0), max(abs(dy) - (p[3] - r), 0.0)
            return max(math.hypot(ex, ey) - r, 0.0)
        if self.kind == 1:
            return max(math.hypot(x - p[0], y - p[1]) - p[2], 0.0)
        vx, vy = p[2] - p[0], p[3] - p[1]
        L = vx * vx + vy * vy
        u = 0.0 if L == 0 else min(max(((x - p[0]) * vx + (y - p[1]) * vy) / L, 0.0), 1.0)
        return max(math.hypot(p[0] + u * vx - x, p[1] + u * vy - y) - p[4], 0.0)


def item_dist(it: Item, X: np.ndarray, Y: np.ndarray) -> np.ndarray:
    """Item.dist for arrays of points."""
    p = it.par
    if it.kind == 0:
        dx, dy = X - p[0], Y - p[1]
        if p[4]:
            c, s = math.cos(p[4]), math.sin(p[4])
            dx, dy = dx * c + dy * s, -dx * s + dy * c
        r = p[5]
        ex = np.maximum(np.abs(dx) - (p[2] - r), 0.0)
        ey = np.maximum(np.abs(dy) - (p[3] - r), 0.0)
        return np.maximum(np.hypot(ex, ey) - r, 0.0)
    if it.kind == 1:
        return np.maximum(np.hypot(X - p[0], Y - p[1]) - p[2], 0.0)
    vx, vy = p[2] - p[0], p[3] - p[1]
    L = vx * vx + vy * vy
    u = np.zeros_like(X) if L == 0 else np.clip(((X - p[0]) * vx + (Y - p[1]) * vy) / L, 0.0, 1.0)
    return np.maximum(np.hypot(p[0] + u * vx - X, p[1] + u * vy - Y) - p[4], 0.0)


def _pad_items(fp: list, ref: str) -> list[Item]:
    at = child(fp, "at")
    fx, fy = float(at[1]) - ORIGIN[0], float(at[2]) - ORIGIN[1]
    frot = float(at[3]) if len(at) > 3 else 0.0
    out = []
    for pad in children(fp, "pad"):
        ptype, pshape = pad[2], pad[3]
        pat = child(pad, "at")
        dx, dy = rot(float(pat[1]), float(pat[2]), frot)
        x, y = fx + dx, fy + dy
        pa = float(pat[3]) if len(pat) > 3 else 0.0
        sx, sy = (float(v) for v in child(pad, "size")[1:3])
        net_node = child(pad, "net")
        net = net_node[-1] if net_node else ""
        layers = layer_bits(child(pad, "layers")[1:])
        drill = child(pad, "drill")
        hole = (0.0, 0.0, 0.0)
        if drill:
            vals = [float(v) for v in drill[1:] if isinstance(v, str) and re.match(r"^[\d.]+$", v)]
            if vals:
                hole = (x, y, max(vals) / 2)
        if ptype == "np_thru_hole":
            r = max(sx, sy, 2 * hole[2]) / 2
            out.append(Item(1, (x, y, r), TOP | BOT | IN2, "", "npth", hole=(x, y, r), ref=ref))
            continue
        if not layers:
            continue
        what = "tht" if ptype == "thru_hole" else "smd"
        planes = "12" if ptype == "thru_hole" else ""
        if pshape == "custom":
            xs, ys = [-sx / 2, sx / 2], [-sy / 2, sy / 2]
            prim = child(pad, "primitives")
            for poly in children(prim, "gr_poly") if prim else ():
                for xy in children(child(poly, "pts"), "xy"):
                    xs.append(float(xy[1]))
                    ys.append(float(xy[2]))
            cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
            ox, oy = rot(cx, cy, pa)
            sx, sy = max(xs) - min(xs), max(ys) - min(ys)
            x, y = x + ox, y + oy
            pshape = "rect"
        a = pa % 180
        swap = abs(a - 90) < 1e-6
        ang = 0.0 if (a < 1e-6 or swap or abs(a - 180) < 1e-6) else -math.radians(pa)
        hx, hy = (sy / 2, sx / 2) if swap else (sx / 2, sy / 2)
        if pshape in ("circle",) or (pshape == "oval" and abs(sx - sy) < 1e-9):
            out.append(Item(1, (x, y, sx / 2), layers, net, what, hole=hole, ref=ref, planes=planes))
        elif pshape == "oval":
            r = min(sx, sy) / 2
            L = (max(sx, sy) - min(sx, sy)) / 2
            ux, uy = rot(L, 0.0, pa) if sx >= sy else rot(0.0, L, pa)
            out.append(Item(2, (x - ux, y - uy, x + ux, y + uy, r), layers, net, what, hole=hole, ref=ref,
                            planes=planes))
        else:
            rr = 0.0
            if pshape == "roundrect":
                rn = child(pad, "roundrect_rratio")
                rr = float(rn[1]) * min(sx, sy) if rn else 0.0
            out.append(Item(0, (x, y, hx, hy, ang, rr), layers, net, what, hole=hole, ref=ref, planes=planes))
    return out


def parse_board(text: str, routed_unlocked: bool = False) -> tuple[list[Item], list[str]]:
    """Pads, tracks, vias and keep-outs of a .kicad_pcb (KiCad 8-10 syntax).
    ``routed_unlocked``: tracks and vias that are not locked count as earlier routing
    (may be ripped up); otherwise all copper on the board is fixed."""
    root = sexpr(text)
    items: list[Item] = []
    notes = []
    for fp in children(root, "footprint"):
        ref = next((p[2] for p in children(fp, "property") if p[1] == "Reference"), "")
        items += _pad_items(fp, ref)
    ox, oy = ORIGIN
    for seg in children(root, "segment"):
        layer = child(seg, "layer")[1]
        if layer not in LAYER_NAMES:
            continue
        s, e = child(seg, "start"), child(seg, "end")
        w = float(child(seg, "width")[1])
        net = child(seg, "net")[-1]
        items.append(Item(2, (float(s[1]) - ox, float(s[2]) - oy, float(e[1]) - ox, float(e[2]) - oy, w / 2),
                          layer_bits([layer]), net, "track",
                          fixed=not routed_unlocked or child(seg, "locked") is not None,
                          width=w))
    for via in children(root, "via"):
        at = child(via, "at")
        x, y = float(at[1]) - ox, float(at[2]) - oy
        size, drill = float(child(via, "size")[1]), float(child(via, "drill")[1])
        span = child(via, "layers")[1:]
        order = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
        idx = sorted(order.index(n) for n in span if n in order)
        reach = order[idx[0]:idx[-1] + 1] if idx else order
        planes = ("1" if "In1.Cu" in reach else "") + ("2" if "In2.Cu" in reach else "")
        net = child(via, "net")[-1]
        items.append(Item(1, (x, y, size / 2), layer_bits(reach), net,
                          "micro" if "micro" in via[1:2] else "via", hole=(x, y, drill / 2), planes=planes,
                          fixed=not routed_unlocked or child(via, "locked") is not None))
    for bits, segs, name, tracks in keepout_polygons(root):
        for x0, y0, x1, y1 in segs:                  # the outline as zero-width "copper"
            items.append(Item(2, (x0, y0, x1, y1, 0.0), bits, "", "keepout", ref="" if tracks else "vias-only"))
        notes.append(f"keep-out {name}: {len(segs)} edges")
    return items, notes


def keepout_polygons(root: list) -> list[tuple[int, list, str, bool]]:
    """Rule areas of the board and of its footprints that forbid tracks or vias:
    (layer bits, outline segments, name, tracks forbidden). U1_EPAD_VIAS is listed too
    (tracks allowed): the DRU only accepts the thermal-via size inside it."""
    ox, oy = ORIGIN
    zones = [(z, "") for z in children(root, "zone")]
    for fp in children(root, "footprint"):
        ref = next((p[2] for p in children(fp, "property") if p[1] == "Reference"), "")
        zones += [(z, ref) for z in children(fp, "zone")]
    out = []
    for zone, ref in zones:
        ko = child(zone, "keepout")
        name_node = child(zone, "name")
        name = name_node[1] if name_node else (f"{ref} keep-out" if ref else "keep-out")
        if not ko:
            continue
        tracks = (child(ko, "tracks") or ["", ""])[1] == "not_allowed"
        vias = (child(ko, "vias") or ["", ""])[1] == "not_allowed"
        if not (tracks or vias or name == "U1_EPAD_VIAS"):
            continue
        ln = child(zone, "layers") or child(zone, "layer")
        bits = layer_bits(ln[1:])
        for poly in children(zone, "polygon"):
            pts = [(float(p[1]) - ox, float(p[2]) - oy) for p in children(child(poly, "pts"), "xy")]
            segs = [(*pts[k], *pts[(k + 1) % len(pts)]) for k in range(len(pts))]
            out.append((bits, segs, name, tracks))
    return out


def parse_courtyards(text: str) -> dict[str, tuple[int, list]]:
    """ref -> (layer bit, courtyard segments [(x0, y0, x1, y1)] in design coordinates)."""
    root = sexpr(text)
    out = {}
    for fp in children(root, "footprint"):
        ref = next((p[2] for p in children(fp, "property") if p[1] == "Reference"), "")
        at = child(fp, "at")
        fx, fy = float(at[1]) - ORIGIN[0], float(at[2]) - ORIGIN[1]
        frot = float(at[3]) if len(at) > 3 else 0.0

        def tr(x, y):
            dx, dy = rot(float(x), float(y), frot)
            return fx + dx, fy + dy
        segs, bits = [], 0
        for g in fp[1:]:
            if not isinstance(g, list) or g[0] not in ("fp_line", "fp_rect", "fp_poly", "fp_arc"):
                continue
            layer = child(g, "layer")
            if not layer or layer[1] not in ("F.CrtYd", "B.CrtYd"):
                continue
            bits |= TOP if layer[1] == "F.CrtYd" else BOT
            if g[0] == "fp_rect":
                (x0, y0), (x1, y1) = (child(g, "start")[1:3], child(g, "end")[1:3])
                pts = [tr(x0, y0), tr(x1, y0), tr(x1, y1), tr(x0, y1)]
                segs += [(*pts[k], *pts[(k + 1) % 4]) for k in range(4)]
            elif g[0] == "fp_poly":
                pts = [tr(p[1], p[2]) for p in children(child(g, "pts"), "xy")]
                segs += [(*pts[k], *pts[(k + 1) % len(pts)]) for k in range(len(pts))]
            elif g[0] == "fp_arc":
                pts = [tr(*child(g, k)[1:3]) for k in ("start", "mid", "end")]
                segs += [(*pts[0], *pts[1]), (*pts[1], *pts[2])]
            else:
                segs.append((*tr(*child(g, "start")[1:3]), *tr(*child(g, "end")[1:3])))
        if ref and segs:
            out[ref] = (bits, segs)
    return out


def dru_fine_pitch_refs(dru_text: str) -> list[str]:
    """The parts named by the DRU's fine-pitch fan-out rule (its clearance neck-down)."""
    m = re.search(r'\(rule "fine_pitch_fanout_clearance".*?\(condition "([^"]*)"', dru_text, re.S)
    return re.findall(r"intersectsCourtyard\('([^']+)'\)", m.group(1)) if m else []


# ==========================================================================
# Net rules
# ==========================================================================

def netclass(net: str) -> str:
    return bs.netclass_of(net) if net else "Default"


def is_supply(net: str) -> bool:
    return net.startswith("+") or net.startswith(SUPPLY_PREFIXES) or netclass(net) == "POWER"


def clearance(net: str) -> float:
    cls = netclass(net)
    if cls in PAIR_CLASSES:
        return 0.20
    if cls in ("SE_50", "POWER"):
        return 0.15
    return 0.10


def widths(net: str) -> tuple[float, ...]:
    """Track widths to try, widest first (Espressif: 3.3 V main >= 25 mil, VDD_HP >= 20 mil,
    the other supplies >= 10 mil); signals use their netclass width."""
    cls = netclass(net)
    if net == "+3V3":
        return (0.635, 0.5, 0.4, 0.3, 0.25)
    if cls == "POWER":
        return (0.5, 0.4, 0.3, 0.25)
    if is_supply(net) and net not in ("VDD_HP_EN", "VDD_HP_FB"):
        return (0.4, 0.3, 0.254)
    if cls in PAIR_CLASSES:
        return (PAIR_CLASSES[cls][0],)
    if cls == "SE_50":
        return (0.16,)
    return (0.15, 0.125)


NECK_MM = 1.0          # neck-down allowed this close to a connection's terminals (fine-pitch pins)


def neck_width(net: str) -> float | None:
    """Narrowest track a net may neck down to at a fine-pitch pin (None: no neck-down;
    the pair nets have a DRU minimum width)."""
    if partner(net):
        return None
    if is_supply(net) and net not in ("VDD_HP_EN", "VDD_HP_FB"):
        return 0.15
    if netclass(net) == "SE_50":
        return 0.12
    return 0.10


def cls_index(c: float) -> int:
    return min(range(len(CLASSES)), key=lambda k: abs(CLASSES[k] - c))


def pair_base(net: str) -> str | None:
    if netclass(net) in PAIR_CLASSES and net.endswith(("_P", "_N")):
        return net[:-2]
    return None


def partner(net: str) -> str | None:
    b = pair_base(net)
    return None if b is None else b + ("_N" if net.endswith("_P") else "_P")


def pair_gap(net: str) -> float:
    return PAIR_CLASSES[netclass(net)][1] + PAIR_GAP_SLACK


def partner_clearance(net: str) -> float:
    """Clearance to the other net of the pair: the gap the pair is routed at (the DRU
    relaxes the USB pairs' own clearance to their gap)."""
    return PAIR_CLASSES[netclass(net)][1]


# ==========================================================================
# Copper store
# ==========================================================================

class Copper:
    """All items as flat numpy arrays (for the C field code) plus the Item objects."""

    def __init__(self, items: list[Item]):
        self.items: list[Item] = []
        self.net_id: dict[str, int] = {"": 0}
        n = len(items) + 4096
        self.kind = np.zeros(n, np.int32)
        self.lay = np.zeros(n, np.int32)
        self.cls = np.zeros(n, np.int32)
        self.net = np.zeros(n, np.int32)
        self.par = np.zeros((n, 6))
        self.box = np.zeros((n, 4))
        self.hole = np.zeros((n, 3))
        self.alive = np.zeros(n, bool)
        self.fixed = np.zeros(n, bool)
        self.by_net: dict[str, set[int]] = {}
        self.version: dict[str, int] = {}
        self.in2_version = 0
        self.dropped: set[tuple] = set()     # pre-routed segments to leave out of the output
        for it in items:
            self.add(it)

    def nid(self, net: str) -> int:
        if net not in self.net_id:
            self.net_id[net] = len(self.net_id)
        return self.net_id[net]

    def _grow(self):
        n = len(self.kind) * 2
        for name in ("kind", "lay", "cls", "net", "alive", "fixed"):
            a = getattr(self, name)
            b = np.zeros(n, a.dtype)
            b[:len(a)] = a
            setattr(self, name, b)
        for name, w in (("par", 6), ("box", 4), ("hole", 3)):
            a = getattr(self, name)
            b = np.zeros((n, w))
            b[:len(a)] = a
            setattr(self, name, b)

    def add(self, it: Item) -> int:
        k = len(self.items)
        if k >= len(self.kind):
            self._grow()
        self.items.append(it)
        self.kind[k] = it.kind
        self.lay[k] = it.layers if it.what != "npth" or True else 0
        if it.what == "npth":
            self.net[k], self.cls[k] = NPTH, HOLE
        elif it.what == "keepout":
            self.net[k], self.cls[k] = KEEPOUT, (VIAKO if it.ref == "vias-only" else 0)
        else:
            self.net[k], self.cls[k] = self.nid(it.net), cls_index(clearance(it.net))
        p = list(it.par) + [0.0] * (6 - len(it.par))
        self.par[k] = p
        self.box[k] = it.bbox()
        self.hole[k] = it.hole
        self.alive[k] = True
        self.fixed[k] = it.fixed
        if it.what not in ("npth", "keepout"):
            self.by_net.setdefault(it.net, set()).add(k)
            self.version[it.net] = self.version.get(it.net, 0) + 1
        if it.layers & IN2:
            self.in2_version += 1
        return k

    def remove(self, k: int) -> None:
        self.alive[k] = False
        net = self.items[k].net
        self.by_net.get(net, set()).discard(k)
        self.version[net] = self.version.get(net, 0) + 1
        if self.items[k].layers & IN2:
            self.in2_version += 1

    def restore(self, k: int) -> None:
        self.alive[k] = True
        net = self.items[k].net
        self.by_net.setdefault(net, set()).add(k)
        self.version[net] = self.version.get(net, 0) + 1
        if self.items[k].layers & IN2:
            self.in2_version += 1

    def select(self, win: tuple, reach: float) -> np.ndarray:
        n = len(self.items)
        b = self.box[:n]
        x0, y0, x1, y1 = win
        m = self.alive[:n] & (b[:, 2] >= x0 - reach) & (b[:, 0] <= x1 + reach) & \
            (b[:, 3] >= y0 - reach) & (b[:, 1] <= y1 + reach)
        return np.nonzero(m)[0]


# ==========================================================================
# Connectivity
# ==========================================================================

VDD_HP_VIA_HALF = lp.VDD_HP_ISLAND_HALF_MM - 0.5       # a VDD_HP via inside the island
PLUS_3V3_KEEP_HALF = lp.VDD_HP_ISLAND_HALF_MM + lp.ZONE_CLEARANCE_MM + 0.35


def on_plane(it: Item) -> bool:
    """Does the item reach its net's plane (In1 GND, In2 +3V3 / VDD_HP island)?"""
    if it.net == bs.GND:
        return "1" in it.planes
    if it.net in ("+3V3", "VDD_HP") and "2" in it.planes:
        x, y = it.par[0], it.par[1]
        inside = max(abs(x), abs(y)) < lp.VDD_HP_ISLAND_HALF_MM
        outside = max(abs(x), abs(y)) > lp.VDD_HP_ISLAND_HALF_MM + lp.ZONE_CLEARANCE_MM
        return outside if it.net == "+3V3" else inside
    return False


def touching(a: Item, b: Item) -> bool:
    if not (a.layers & b.layers):
        return False
    ab, bb = a.bbox(), b.bbox()
    if ab[2] < bb[0] - 1e-6 or bb[2] < ab[0] - 1e-6 or ab[3] < bb[1] - 1e-6 or bb[3] < ab[1] - 1e-6:
        return False
    tol = 1e-4
    if any(b.dist(x, y) <= tol for x, y in a.anchors()) or any(a.dist(x, y) <= tol for x, y in b.anchors()):
        return True
    if a.kind != 2 and b.kind != 2:           # two pads / vias: overlapping copper
        return a.dist(*b.anchors()[0]) <= (b.par[2] if b.kind == 1 else 0.0) + tol
    return False


class DSU:
    def __init__(self, n: int):
        self.p = list(range(n))

    def find(self, x: int) -> int:
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x

    def union(self, a: int, b: int) -> None:
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[rb] = ra


def net_islands(cu: Copper, net: str) -> tuple[list[list[int]], int | None]:
    """Connected copper groups of a net; the index of the group on the plane (if any)."""
    idx = sorted(cu.by_net.get(net, ()))
    if not idx:
        return [], None
    items = [cu.items[k] for k in idx]
    n = len(items)
    d = DSU(n + 1)                           # n = the plane
    boxes = cu.box[idx]
    for i in range(n):
        if on_plane(items[i]):
            d.union(n, i)
        cand = np.nonzero((boxes[i + 1:, 0] <= boxes[i, 2] + 1e-6) & (boxes[i + 1:, 2] >= boxes[i, 0] - 1e-6) &
                          (boxes[i + 1:, 1] <= boxes[i, 3] + 1e-6) & (boxes[i + 1:, 3] >= boxes[i, 1] - 1e-6))[0]
        for j in cand + i + 1:
            if d.find(i) != d.find(j) and touching(items[i], items[j]):
                d.union(i, j)
    groups: dict[int, list[int]] = {}
    for i in range(n):
        groups.setdefault(d.find(i), []).append(idx[i])
    root_plane = d.find(n)
    out, plane = [], None
    for r, g in groups.items():
        if r == root_plane:
            plane = len(out)
        out.append(g)
    return out, plane


@dataclass
class Conn:
    id: int
    net: str
    a: list[int]               # items of one island
    b: list[int] | None        # items of the other island; None = the net's plane
    ta: tuple                  # terminal points (closest anchors)
    tb: tuple
    prio: int = 5
    rips: int = 0
    items: list[int] = field(default_factory=list)   # routed copper
    done: bool = False
    pair: int = -1             # id of the partner connection (coupled routing)
    note: str = ""

    @property
    def length(self) -> float:
        return math.dist(self.ta, self.tb)


def _anchor_array(cu: Copper, group: list[int]) -> np.ndarray:
    return np.array([a for k in group for a in cu.items[k].anchors()])


def closest(cu: Copper, ga: list[int], gb: list[int]) -> tuple[float, tuple, tuple]:
    A, B = _anchor_array(cu, ga), _anchor_array(cu, gb)
    d = np.hypot(A[:, None, 0] - B[None, :, 0], A[:, None, 1] - B[None, :, 1])
    i, j = np.unravel_index(np.argmin(d), d.shape)
    return float(d[i, j]), tuple(A[i]), tuple(B[j])


PAIR_RANK = {"MIPI_100": 0, "USB_90": 1, "ETH_100": 2}    # fastest first (1.5 Gb/s, 480 Mb/s, 100 Mb/s)
SOC_RANK = 2.5                     # any other connection that starts at a U1 pad
LOCAL_MM = 2.5                     # connections this short are fan-out, routed before anything else


def priority(net: str) -> int:
    """Routing order; a connection may only rip up connections of the same or a later rank."""
    cls = netclass(net)
    if cls in PAIR_RANK:
        return PAIR_RANK[cls]
    if net.startswith("XTAL"):
        return 3
    if cls == "SE_50":
        return 4
    if net in PLANE_NETS:
        return 6
    if is_supply(net):
        return 5
    return 7


def in2_ok(net: str) -> bool:
    """May the net use In2.Cu? Espressif: power traces on the inner (power) layer, and
    signals there too if needed; the differential pairs stay on F.Cu over the GND plane
    and the plane nets use their planes."""
    return not partner(net) and net not in PLANE_NETS


def rip_rank(c: "Conn") -> float:
    """Who may rip up whom: differential pairs by their rank, everything else is equal
    (the routing order puts the SoC's connections first, but a short local connection
    they wall off must be able to push them aside)."""
    return PAIR_RANK[netclass(c.net)] if partner(c.net) else 3


def may_rip(by: "Conn", victim: "Conn") -> bool:
    if victim.note == "escape":                  # pre-routed escape: only pushed (Router.may_rip)
        return False
    if victim.net == partner(by.net):          # the two halves of a pair never push each other
        return False
    return rip_rank(victim) >= rip_rank(by) and victim.rips < MAX_RIPS


def net_connections(cu: Copper, net: str, next_id) -> list[Conn]:
    """Minimum spanning tree over the net's islands (plane nets: every island not on the
    plane connects to the plane)."""
    groups, plane = net_islands(cu, net)
    out = []
    if net in PLANE_NETS:
        for g_i, g in enumerate(groups):
            if g_i == plane:
                continue
            it = cu.items[g[0]]
            p = it.anchors()[0]
            out.append(Conn(next_id(), net, g, None, p, p, prio=priority(net)))
        return out
    if len(groups) < 2:
        return out
    edges = []
    for i in range(len(groups)):
        for j in range(i + 1, len(groups)):
            dd, pa, pb = closest(cu, groups[i], groups[j])
            edges.append((dd, i, j, pa, pb))
    edges.sort(key=lambda e: e[0])
    d = DSU(len(groups))
    for dd, i, j, pa, pb in edges:
        if d.find(i) != d.find(j):
            d.union(i, j)
            out.append(Conn(next_id(), net, groups[i], groups[j], pa, pb, prio=priority(net)))
    return out


def pair_connections(cu: Copper, base: str, next_id) -> list[Conn] | None:
    """The connections of a differential pair, P and N alike: P islands are matched one to
    one with N islands (same footprint first, then the nearest), the spanning tree is
    computed once over the matched islands and applied to both nets, so each P connection
    has an N connection between the same parts (coupled routing). Islands of one net
    without a partner island (a pull-down on one line only) join it by a single track.
    None if fewer than two islands match."""
    from scipy.optimize import linear_sum_assignment
    pn, nn = base + "_P", base + "_N"
    gp, _ = net_islands(cu, pn)
    gq, _ = net_islands(cu, nn)
    if len(gp) < 2 and len(gq) < 2:
        return []

    def refs(g):
        return {cu.items[k].ref for k in g if cu.items[k].ref}
    cost = np.array([[closest(cu, a, b)[0] - 10.0 * len(refs(a) & refs(b)) for b in gq] for a in gp])
    rows, cols = linear_sum_assignment(cost)
    sib = {int(i): int(j) for i, j in zip(rows, cols)}
    if len(sib) < 2:
        return None
    matched = sorted(sib)
    edges = []
    for x, i in enumerate(matched):
        for j in matched[x + 1:]:
            dp, pa, pb = closest(cu, gp[i], gp[j])
            dq, qa, qb = closest(cu, gq[sib[i]], gq[sib[j]])
            edges.append((dp + dq, i, j, pa, pb, qa, qb))
    edges.sort(key=lambda e: e[0])
    dp_, dq_ = DSU(len(gp)), DSU(len(gq))
    out = []
    for _, i, j, pa, pb, qa, qb in edges:
        if dp_.find(i) != dp_.find(j):
            dp_.union(i, j)
            dq_.union(sib[i], sib[j])
            c_p = Conn(next_id(), pn, gp[i], gp[j], pa, pb, prio=priority(pn))
            c_n = Conn(next_id(), nn, gq[sib[i]], gq[sib[j]], qa, qb, prio=priority(nn))
            c_p.pair, c_n.pair = c_n.id, c_p.id
            out += [c_p, c_n]
    for net, groups, d in ((pn, gp, dp_), (nn, gq, dq_)):          # unmatched islands
        extra = [i for i in range(len(groups)) if (i not in sib if groups is gp else i not in sib.values())]
        for i in extra:
            best = min((closest(cu, groups[i], groups[j]) + (j,) for j in range(len(groups))
                        if j != i and d.find(j) != d.find(i)), default=None, key=lambda t: t[0])
            if best is None:
                continue
            d.union(i, best[3])
            out.append(Conn(next_id(), net, groups[i], groups[best[3]], best[1], best[2], prio=priority(net)))
    return out


# ==========================================================================
# Router
# ==========================================================================

@dataclass
class Window:
    i0: int
    j0: int
    nx: int
    ny: int
    x0: float
    y0: float
    g: float

    def xy(self, i, j):
        return self.x0 + i * self.g, self.y0 + j * self.g

    def mesh(self):
        xs = self.x0 + np.arange(self.nx) * self.g
        ys = self.y0 + np.arange(self.ny) * self.g
        return np.meshgrid(xs, ys, indexing="ij")


class Router:
    def __init__(self, items: list[Item], grid: float = GRID_MM, log=print, necks=(), keepouts=(),
                 neck_zones=()):
        """``necks``: (layer bits, courtyard segments) of the parts whose courtyard has the
        DRU's breakout clearance (fine-pitch fan-out). ``neck_zones``: the same, where a
        track may also neck down anywhere, not only near its terminals (the SoC courtyard:
        the ring between its pads and the thermal-via field is the only way past it)."""
        self.necks = [(bits, np.array(segs, float)) for bits, segs in necks]
        self.neck_zones = [(bits, np.array(segs, float)) for bits, segs in neck_zones]
        self.keepouts = [(bits, np.array(segs, float)) for bits, segs, _, tr in keepouts if tr]
        self.via_keepouts = [(bits, np.array(segs, float)) for bits, segs, _, tr in keepouts]
        self.cu = Copper(items)
        self.enclosed = False
        self.queue: list[Conn] = []
        self.g = grid
        self.log = log
        x0, y0, x1, y1 = bs.BOARD_OUTLINE
        self.bx0, self.by0 = x0, y0
        self.NX, self.NY = int(round((x1 - x0) / grid)) + 1, int(round((y1 - y0) / grid)) + 1
        xs = x0 + np.arange(self.NX) * grid
        ys = y0 + np.arange(self.NY) * grid
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        r = bs.BOARD_CORNER_R
        cx, cy = np.clip(X, x0 + r, x1 - r), np.clip(Y, y0 + r, y1 - r)
        corner = r - np.hypot(X - cx, Y - cy)
        straight = np.minimum(np.minimum(X - x0, x1 - X), np.minimum(Y - y0, y1 - Y))
        self.edge = np.where((np.abs(X - cx) > 1e-9) & (np.abs(Y - cy) > 1e-9), corner, straight).astype(np.float32)
        self.hist = np.zeros((NL, self.NX, self.NY), np.float32)
        tv = bs.THERMAL_VIA                  # bottom solder-mask window over the thermal vias
        h = (tv["grid"] - 1) / 2 * tv["pitch_mm"] + tv["pad_mm"] / 2
        self.cu.add(Item(0, (0.0, 0.0, h, h, 0.0, 0.0), BOT, bs.GND, "keepout"))
        self.conns: list[Conn] = []
        self._isl: dict = {}
        self._next = 0
        self.stats = {"routed": 0, "ripped": 0, "failed": 0}
        self.push_pairs = False
        self.soft_escapes()

    def soft_escapes(self) -> int:
        """Pre-routed SoC escape tracks (fixed tracks between the U1 stub ends and the escape
        reach, not of a pair or plane net) become routing that only a last-resort search may
        push aside (``may_rip``): the planner lays them out one side of U1 at a time and can
        leave a neighbour walled in by a few hundredths of a millimetre. They stay locked in
        the output file. Returns the number of pieces."""
        cu = self.cu
        lo, hi = lp.U1_STUB_END_MM - 1e-6, lp.ESCAPE_REACH_MM + 0.5
        ox, oy = ORIGIN
        by_net: dict[str, list[int]] = {}
        for k, it in enumerate(cu.items):
            if not (cu.alive[k] and it.fixed and it.kind == 2 and it.what == "track") or partner(it.net) \
                    or it.net in PLANE_NETS or not it.net:
                continue
            x0, y0, x1, y1 = it.par[:4]
            if all(lo <= max(abs(x), abs(y)) <= hi for x, y in ((x0, y0), (x1, y1))):
                by_net.setdefault(it.net, []).append(k)
        n = 0
        for net, ks in by_net.items():
            d = DSU(len(ks))
            for a in range(len(ks)):
                for b in range(a + 1, len(ks)):
                    if touching(cu.items[ks[a]], cu.items[ks[b]]):
                        d.union(a, b)
            groups: dict[int, list[int]] = {}
            for a, k in enumerate(ks):
                groups.setdefault(d.find(a), []).append(k)
            for g in groups.values():
                c = Conn(self.next_id(), net, [], None, cu.items[g[0]].anchors()[0], cu.items[g[0]].anchors()[0],
                         prio=priority(net), items=list(g), done=True, note="escape")
                for k in g:
                    it = cu.items[k]
                    L = LAYER_NAMES[0 if it.layers & TOP else (1 if it.layers & BOT else 2)]
                    cu.dropped.add(segment_key(net, L, it.par[0] + ox, it.par[1] + oy, it.par[2] + ox, it.par[3] + oy))
                    it.fixed, it.rewrite, it.conn = False, True, c.id
                    cu.fixed[k] = False
                self.conns.append(c)
                n += len(g)
        return n

    def next_id(self) -> int:
        self._next += 1
        return self._next

    # ------------------------------------------------------------------ geometry
    def window(self, pts: list, pad: float) -> Window:
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        g = self.g
        i0 = max(0, int(math.floor((min(xs) - pad - self.bx0) / g)))
        j0 = max(0, int(math.floor((min(ys) - pad - self.by0) / g)))
        i1 = min(self.NX - 1, int(math.ceil((max(xs) + pad - self.bx0) / g)))
        j1 = min(self.NY - 1, int(math.ceil((max(ys) + pad - self.by0) / g)))
        nx, ny = i1 - i0 + 1, j1 - j0 + 1
        while nx * ny > MAX_WINDOW_CELLS and pad > 1:
            return self.window(pts, pad * 0.7)
        return Window(i0, j0, nx, ny, self.bx0 + i0 * g, self.by0 + j0 * g, g)

    def fields(self, w: Window, own: set[str], partner_net: str | None, reach: float,
               ripper: "Conn | None" = None):
        """Distance fields in the window: obs[grp][cls][layer] for other nets (grp 0 fixed,
        1 routed -- routing that ``ripper`` may not rip up counts as fixed), own[layer] for the
        nets in ``own``, holes (all nets)."""
        cu = self.cu
        sel = cu.select((w.x0, w.y0, w.x0 + (w.nx - 1) * w.g, w.y0 + (w.ny - 1) * w.g), reach)
        own_ids = {cu.net_id[n] for n in own if n in cu.net_id}
        nets = cu.net[sel]
        is_own = np.isin(nets, list(own_ids)) if own_ids else np.zeros(len(sel), bool)
        pid = cu.net_id.get(partner_net, -99) if partner_net else -99
        other = sel[~is_own]
        mine = sel[is_own]
        N = w.nx * w.ny
        obs = np.full((2, NCLS, NL, N), 1e9, np.float32)
        holes = np.full(N, 1e9, np.float32)
        cls = cu.cls[other].copy()
        cls[cu.net[other] == pid] = PARTNER
        grp = (~cu.fixed[other]).astype(np.int32)
        if ripper is not None:
            for x, k in enumerate(other):
                if grp[x]:
                    owner = self.by_id.get(cu.items[k].conn) if hasattr(self, "by_id") else None
                    if owner is not None and not self.may_rip(ripper, owner):
                        grp[x] = 0
        lib = core()
        lib.fields(len(other), cu.kind[other], cu.lay[other], cls, grp,
                   np.ascontiguousarray(cu.par[other]), np.ascontiguousarray(cu.box[other]),
                   np.ascontiguousarray(cu.hole[other]), w.x0, w.y0, w.g, w.nx, w.ny, reach, NCLS, NL,
                   obs.reshape(-1), holes)
        ownf = np.full((1, 1, NL, N), 1e9, np.float32)
        lib.fields(len(mine), cu.kind[mine], cu.lay[mine], np.zeros(len(mine), np.int32),
                   np.zeros(len(mine), np.int32), np.ascontiguousarray(cu.par[mine]),
                   np.ascontiguousarray(cu.box[mine]), np.ascontiguousarray(cu.hole[mine]),
                   w.x0, w.y0, w.g, w.nx, w.ny, 0.05, 1, NL, ownf.reshape(-1), holes)
        shp = (w.nx, w.ny)
        return obs.reshape(2, NCLS, NL, *shp), ownf.reshape(NL, *shp), holes.reshape(shp)

    def group_field(self, w: Window, group: list[int]) -> np.ndarray:
        """Distance to the copper of one island (per layer)."""
        cu = self.cu
        g = np.array([k for k in group if cu.alive[k]], np.int64)
        f = np.full((1, 1, NL, w.nx * w.ny), 1e9, np.float32)
        holes = np.full(w.nx * w.ny, 1e9, np.float32)
        if len(g):
            core().fields(len(g), cu.kind[g], cu.lay[g], np.zeros(len(g), np.int32), np.zeros(len(g), np.int32),
                          np.ascontiguousarray(cu.par[g]), np.ascontiguousarray(cu.box[g]),
                          np.ascontiguousarray(cu.hole[g]), w.x0, w.y0, w.g, w.nx, w.ny, 0.05, 1, NL,
                          f.reshape(-1), holes)
        return f.reshape(NL, w.nx, w.ny)

    def need(self, net: str, half: float, margin: float = MARGIN_MM, breakout: bool = False) -> list[float]:
        """Centre distance a shape of half-size ``half`` of ``net`` keeps per obstacle class
        (inside the SoC breakout area every clearance is the DRU's breakout clearance)."""
        c = clearance(net)
        hole = half + max(HOLE_CLEARANCE, c) + margin
        if breakout:
            return [half + BREAKOUT_CLEARANCE + margin] * len(CLASSES) + \
                [half + min(BREAKOUT_CLEARANCE, partner_clearance(net) if partner(net) else c) + min(margin, 0.002),
                 hole, 0.0]
        out = [half + max(c, k) + margin for k in CLASSES]
        out.append(half + (partner_clearance(net) if partner(net) else c) + min(margin, 0.002))
        out.append(hole)
        out.append(0.0)                      # via-only rule areas: see via_maps
        return out

    def breakout(self, w: Window) -> np.ndarray:
        """Per layer: cells where the DRU necks the clearance down (SoC breakout area on
        F.Cu, courtyards of the fine-pitch parts on their side)."""
        X, Y = w.mesh()
        out = self.poly_mask(w, self.necks)
        out[0] |= (np.abs(X) < BREAKOUT_HALF) & (np.abs(Y) < BREAKOUT_HALF)
        out[2] = False                       # the DRU areas and courtyards are outer-layer only,
        if self.neck_zones:                  # but the U1 courtyard rule holds on every layer
            out[2] |= self.poly_mask(w, self.neck_zones)[2]
        return out

    def poly_mask(self, w: Window, polys) -> np.ndarray:
        """Per layer: cells inside any of the (layer bits, outline segments) polygons."""
        X, Y = w.mesh()
        out = np.zeros((NL, w.nx, w.ny), bool)
        x1, y1 = w.x0 + (w.nx - 1) * w.g, w.y0 + (w.ny - 1) * w.g
        for bits, segs in polys:
            bx0, by0 = segs[:, [0, 2]].min(), segs[:, [1, 3]].min()
            bx1, by1 = segs[:, [0, 2]].max(), segs[:, [1, 3]].max()
            if bx1 < w.x0 or bx0 > x1 or by1 < w.y0 or by0 > y1:
                continue
            i0, i1 = max(0, int((bx0 - w.x0) / w.g)), min(w.nx, int((bx1 - w.x0) / w.g) + 2)
            j0, j1 = max(0, int((by0 - w.y0) / w.g)), min(w.ny, int((by1 - w.y0) / w.g) + 2)
            Xs, Ys = X[i0:i1, j0:j1], Y[i0:i1, j0:j1]
            inside = np.zeros(Xs.shape, bool)
            for xa, ya, xb, yb in segs:                    # even-odd rule
                if ya == yb:
                    continue
                c = ((ya > Ys) != (yb > Ys)) & (Xs < (xb - xa) * (Ys - ya) / (yb - ya) + xa)
                inside ^= c
            for L in range(NL):
                if bits >> L & 1:
                    out[L, i0:i1, j0:j1] |= inside
        return out

    def free_maps(self, w: Window, obs, net: str, half: float, margin: float = MARGIN_MM):
        """(free w.r.t. fixed copper, free w.r.t. routed copper) per layer."""
        edge = self.edge[w.i0:w.i0 + w.nx, w.j0:w.j0 + w.ny]
        bo = self.breakout(w)
        inside_ko = self.poly_mask(w, self.keepouts)
        fixed, routed = [], []
        for L in range(NL):
            maps = []
            for brk in ((False, True) if bo[L].any() else (False,)):
                nd = self.need(net, half, margin, brk)
                f = edge >= EDGE_MM + half + margin
                r = np.ones_like(f)
                for k in range(NCLS):
                    f &= obs[0, k, L] >= nd[k]
                    r &= obs[1, k, L] >= nd[k]
                maps.append((f, r))
            if len(maps) == 2:
                f = np.where(bo[L], maps[1][0], maps[0][0])
                r = np.where(bo[L], maps[1][1], maps[0][1])
            else:
                f, r = maps[0]
            fixed.append(f & ~inside_ko[L])
            routed.append(r)
        return fixed, routed

    def via_maps(self, w: Window, obs, holes, net: str, diameter: float, drill: float, layers: tuple):
        """Cells where a via with copper on ``layers`` fits: (w.r.t. fixed copper, routed copper)."""
        f, r = self.free_maps(w, obs, net, diameter / 2)
        hf = holes >= drill / 2 + HOLE_TO_HOLE + MARGIN_MM
        vko = self.poly_mask(w, self.via_keepouts)      # includes U1_EPAD_VIAS (thermal vias only)
        hf &= ~vko.any(axis=0)
        vf = hf.copy()
        vr = np.ones_like(hf)
        for L in layers:
            vf &= f[L] & (obs[0, VIAKO, L] >= diameter / 2 + MARGIN_MM)    # nor across their edge
            vr &= r[L]
        return vf, vr

    # ------------------------------------------------------------------ search
    @staticmethod
    def reachable(free: np.ndarray, vias, src: np.ndarray, dst: np.ndarray) -> tuple[bool, bool]:
        """Cheap necessary condition for a path: src and dst in one 8-connected region of
        free cells (layers joined where a via may stand). Saves the full A* on hopeless
        connections, which would explore the whole window. Returns (connected, the source
        region reaches the window border)."""
        from scipy import ndimage
        st = np.ones((3, 3), bool)
        labs, off = [], 0
        for L in range(len(free)):
            lab, n = ndimage.label(free[L], structure=st)
            labs.append(np.where(lab > 0, lab + off, 0))
            off += n
        parent = np.arange(off + 1)

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        for bits, mask, _ in vias:
            ls = [L for L in range(len(free)) if bits >> L & 1]
            for a_l, b_l in zip(ls, ls[1:]):
                both = mask & (labs[a_l] > 0) & (labs[b_l] > 0)
                for a, b in set(zip(labs[a_l][both].tolist(), labs[b_l][both].tolist())):
                    ra, rb = find(a), find(b)
                    if ra != rb:
                        parent[rb] = ra
        roots = np.array([find(x) for x in range(len(parent))])
        roots[0] = -1

        def roots_of(mask3):
            out = set()
            for L in range(len(free)):
                out |= set(roots[labs[L][mask3[L]]].tolist())
            out.discard(-1)
            return out
        s_roots, d_roots = roots_of(src), roots_of(dst)
        if s_roots & d_roots:
            return True, True
        edge = np.zeros(free.shape[1:], bool)
        edge[0, :] = edge[-1, :] = edge[:, 0] = edge[:, -1] = True
        return False, bool(s_roots & roots_of(np.stack([edge] * len(free))))

    def search(self, w: Window, cost: np.ndarray, vias, src: np.ndarray, dst: np.ndarray,
               turn90: float = TURN90, turn45: float = TURN45):
        """A* over the window; ``vias``: [(layer bits, cell mask, cost)] layer changes."""
        if not src.any() or not dst.any():
            return None
        ok, border = self.reachable(cost >= 0, vias, src, dst)
        if not ok:
            self.enclosed = not border
            return None
        ii, jj = np.nonzero(dst.any(axis=0))
        tbox = np.array([ii.min(), jj.min(), ii.max(), jj.max()], np.int32)
        max_path = 4 * (w.nx + w.ny) * 4
        path = np.zeros(3 * max_path, np.int32)
        nvk = len(vias)
        vlay = np.array([b for b, _, _ in vias] or [0], np.int32)
        vmask = np.ascontiguousarray(np.stack([m for _, m, _ in vias]) if vias else
                                     np.zeros((1, w.nx, w.ny), bool), np.uint8).reshape(-1)
        vcost = np.array([c for _, _, c in vias] or [0.0], np.float32)
        n = core().astar(len(cost), w.nx, w.ny, np.ascontiguousarray(cost, np.float32).reshape(-1),
                         nvk, vlay, vmask, vcost,
                         np.ascontiguousarray(src, np.uint8).reshape(-1),
                         np.ascontiguousarray(dst, np.uint8).reshape(-1), tbox,
                         turn45, turn90, 60_000_000, path, max_path)
        self.enclosed = n == -3
        if n <= 0:
            return None
        return path[:3 * n].reshape(n, 3)

    def geometry(self, w: Window, path: np.ndarray, narrow=None, micro=None):
        """Grid path -> straight runs per layer ((layer, (x0, y0), (x1, y1), narrow)) and vias
        (i, j, kind). ``narrow[k]``: path cell k only has room for the neck-down width;
        ``micro``: cells where an L4-L3 laser microvia may take a B.Cu <-> In2.Cu change."""
        segs, vias = [], []
        k = 0
        n = len(path)
        nar = narrow if narrow is not None else [False] * n
        while k < n - 1:
            L, i, j = path[k]
            L2, i2, j2 = path[k + 1]
            if L2 != L:
                visited = {int(L)}
                m = k
                while m + 1 < n and path[m + 1][1] == i and path[m + 1][2] == j:
                    m += 1
                    visited.add(int(path[m][0]))
                kind = "micro_bottom" if (visited <= {1, 2} and micro is not None and micro[i, j]) else "through"
                vias.append((i, j, kind))
                k = m
                continue
            d = (i2 - i, j2 - j)
            flag = nar[k] or nar[k + 1]
            m = k + 1
            while m + 1 < n and path[m + 1][0] == L and (path[m + 1][1] - path[m][1], path[m + 1][2] - path[m][2]) == d \
                    and (nar[m] or nar[m + 1]) == flag:
                m += 1
            segs.append((int(L), w.xy(i, j), w.xy(path[m][1], path[m][2]), flag))
            k = m
        return segs, vias

    # ------------------------------------------------------------------ commit / rip
    def commit(self, conn: Conn, segs, width: float, vias, neck: float | None = None) -> None:
        """Add routed copper; a segment (layer, a, b[, narrow]) is ``neck`` wide if narrow."""
        cu = self.cu
        for seg in segs:
            L, a, b = seg[:3]
            wd = neck if (len(seg) > 3 and seg[3] and neck) else width
            it = Item(2, (a[0], a[1], b[0], b[1], wd / 2), 1 << L, conn.net, "track",
                      fixed=False, conn=conn.id, width=wd)
            conn.items.append(cu.add(it))
        for x, y, kind in vias:
            if kind == "through":
                it = Item(1, (x, y, VIA[0] / 2), TOP | BOT | IN2, conn.net, "via", hole=(x, y, VIA[1] / 2),
                          fixed=False, conn=conn.id, planes="12")
            elif kind == "micro_top":
                it = Item(1, (x, y, MICRO[0] / 2), TOP, conn.net, "micro", hole=(x, y, MICRO[1] / 2),
                          fixed=False, conn=conn.id, planes="1")
            else:
                it = Item(1, (x, y, MICRO[0] / 2), BOT | IN2, conn.net, "micro", hole=(x, y, MICRO[1] / 2),
                          fixed=False, conn=conn.id, planes="2")
            conn.items.append(cu.add(it))
        conn.done = True

    def rip(self, conn: Conn) -> None:
        for k in conn.items:
            self.cu.remove(k)
        conn.items = []
        conn.done = False
        conn.rips += 1
        self.stats["ripped"] += 1

    def victims(self, w: Window, conn: Conn, path: np.ndarray, half: float, vias, via_half: float,
                own: set[str] | None = None, halves=None) -> set[int]:
        """Routed connections of other nets too close to a rip-up path (with the clearance
        the path cell actually needs: necked down inside the breakout areas; ``halves``: the
        track half-width per path cell where it necks down)."""
        cu = self.cu
        own = own or {conn.net}
        box = (w.x0, w.y0, w.x0 + (w.nx - 1) * w.g, w.y0 + (w.ny - 1) * w.g)
        sel = [k for k in cu.select(box, 1.0) if not cu.fixed[k] and cu.items[k].net not in own]
        bo = self.breakout(w)
        P = np.array([(L, *w.xy(i, j), bo[L][i, j]) for L, i, j in path], float).reshape(-1, 4)
        V = np.array([(*w.xy(i, j), bo[0][i, j]) for i, j in vias], float).reshape(-1, 3)
        out = set()
        nd = np.array([self.need(conn.net, 0.0), self.need(conn.net, 0.0, breakout=True)])
        hv = np.full(len(P), half) if halves is None else np.asarray(halves, float)
        nv = np.array([self.need(conn.net, via_half), self.need(conn.net, via_half, breakout=True)])
        pn = partner(conn.net)
        for k in sel:
            it = cu.items[k]
            if it.conn in out:
                continue
            ci = PARTNER if it.net == pn else cu.cls[k]
            on = np.zeros(len(P), bool)
            for L in range(NL):
                if (it.layers >> L) & 1:
                    on |= P[:, 0] == L
            if on.any():
                lim = nd[P[on, 3].astype(int), ci] + hv[on]
                if (item_dist(it, P[on, 1], P[on, 2]) < lim).any():
                    out.add(it.conn)
                    continue
            if len(V):
                hit = item_dist(it, V[:, 0], V[:, 1]) < nv[V[:, 2].astype(int), ci]
                if it.hole[2]:
                    hit |= np.hypot(V[:, 0] - it.hole[0], V[:, 1] - it.hole[1]) < \
                        it.hole[2] + VIA[1] / 2 + HOLE_TO_HOLE + MARGIN_MM
                if hit.any():
                    out.add(it.conn)
        return out

    # ------------------------------------------------------------------ one connection
    def plane_targets(self, w: Window, obs, holes, net: str, free, rip: bool = False):
        """Cells where a via into the net's plane fits (GND: microvia on top, through via
        on the bottom; +3V3: through via on top, microvia on the bottom)."""
        X, Y = w.mesh()
        vt, vtr = self.via_maps(w, obs, holes, net, VIA[0], VIA[1], (0, 1, 2))
        m_top, m_top_r = self.via_maps(w, obs, holes, net, MICRO[0], MICRO[1], (0,))
        m_bot, m_bot_r = self.via_maps(w, obs, holes, net, MICRO[0], MICRO[1], (1, 2))
        if rip:
            vtr, m_top_r, m_bot_r = (np.ones_like(vtr),) * 3
        a = np.maximum(np.abs(X), np.abs(Y))
        if net == bs.GND:
            region = np.ones(X.shape, bool)
            top, bot = m_top & m_top_r, vt & vtr
            kinds = ("micro_top", "through")
        else:
            region = a > PLUS_3V3_KEEP_HALF
            top, bot = vt & vtr, m_bot & m_bot_r
            kinds = ("through", "micro_bottom")
        return np.stack([top & region & free[0], bot & region & free[1], np.zeros_like(top)]), kinds

    def route(self, conn: Conn, allow_rip: bool = False, layers=(0, 1, 2), no_vias: bool = False,
              width_list=None, target=None) -> bool:
        """Route one connection in growing windows: first the narrowest track the net may use
        (necked down at fine-pitch pins if needed), then the widest that fits the same way.
        With ``allow_rip`` one search in the largest window may cross routed copper of
        lower-priority connections, which is ripped up and queued again. ``target``:
        (point, mask builder) for the fan-ins of a coupled pair."""
        net = conn.net
        wl = width_list or widths(net)
        neck = neck_width(net) if target is None else None
        # Espressif: no vias on the crystal traces -- unless the placement makes the two cross
        xtal_retry = net.startswith("XTAL") and not no_vias and target is None
        opts = dict(layers=layers, no_vias=no_vias or net.startswith("XTAL"), target=target)
        if allow_rip:
            ctx = self._context(conn, WINDOWS_MM[-1], wl, target)
            plan = self._attempt(conn, ctx, wl[-1], neck, True, **opts)
            if plan is not None and self._apply(conn, plan):
                return True
            if 2 not in layers or not in2_ok(net):
                return False
            # it would have split a plane on In2.Cu (or its way through In2 met routing it may
            # not rip up): try again on the outer layers only
            plan = self._attempt(conn, ctx, wl[-1], neck, True, **{**opts, "layers": (0, 1)})
            return plan is not None and self._apply(conn, plan)
        for pad in WINDOWS_MM:
            ctx = self._context(conn, pad, wl, target)
            if conn.b is None and target is None and self._via_in_pad(conn, ctx, wl[-1]):
                return True
            plan = self._attempt(conn, ctx, wl[-1], None, False, **opts)
            enclosed = self.enclosed
            if plan is None and neck:
                plan = self._attempt(conn, ctx, wl[-1], neck, False, **opts)
                enclosed = enclosed and self.enclosed
            if plan is None:
                if enclosed:
                    break                     # the search never reached the window border
                continue
            for wd in wl[:-1]:                # widest first
                better = self._attempt(conn, ctx, wd, neck, False, **opts)
                if better is not None:
                    plan = better
                    break
            if self._apply(conn, plan):
                return True
            # it would have split a plane on In2.Cu: try again on the outer layers only
            plan = self._attempt(conn, ctx, wl[-1], neck, False, **{**opts, "layers": (0, 1)})
            if plan is not None and self._apply(conn, plan):
                return True
        if xtal_retry:
            ok = self._route_xtal_with_vias(conn, wl)
            if ok:
                conn.note += " (crossing: vias on the crystal net)"
            return ok
        return False

    def _route_xtal_with_vias(self, conn: Conn, wl) -> bool:
        for pad in WINDOWS_MM:
            ctx = self._context(conn, pad, wl, None)
            plan = self._attempt(conn, ctx, wl[-1], None, False)
            if plan is not None:
                return self._apply(conn, plan)
        return False

    def _context(self, conn: Conn, pad: float, wl, target):
        pts = [conn.ta, conn.tb] + ([target[0]] if target else [])
        w = self.window(pts, pad)
        reach = max(wl) / 2 + max(CLASSES) + VIA[0] + 0.1
        obs, _, holes = self.fields(w, {conn.net}, partner(conn.net), reach, ripper=conn)
        fa = self.group_field(w, conn.a)
        fb = self.group_field(w, conn.b) if conn.b is not None else None
        if conn.b is None and target is None:            # plane pad: its plane's copper counts too
            groups, plane = self.islands(conn.net)
            if plane is not None:
                fb = self.group_field(w, groups[plane])
        return w, obs, holes, fa, fb

    def _via_in_pad(self, conn: Conn, ctx, wd: float) -> bool:
        """Plane connection: a via that fits in the pad itself needs no track."""
        w, obs, holes, fa, fb = ctx
        fixed, routed = self.free_maps(w, obs, conn.net, wd / 2)
        free = [fixed[L] & routed[L] for L in range(NL)]
        pt, kinds = self.plane_targets(w, obs, holes, conn.net, free)
        inpad = pt & (fa <= 0)
        if not inpad.any():
            return False
        L, i, j = min(zip(*np.nonzero(inpad)), key=lambda c: math.dist(w.xy(c[1], c[2]), conn.ta))
        self.commit(conn, [], wd, [(*w.xy(i, j), kinds[L])])
        conn.note = "via in pad"
        return True

    def _attempt(self, conn: Conn, ctx, wd: float, nk, rip: bool, layers=(0, 1, 2), no_vias=False, target=None):
        """One search; returns a plan (segments, vias, victims, width, neck) or None."""
        w, obs, holes, fa, fb = ctx
        net = conn.net
        pn = partner(net)
        plane = conn.b is None and target is None
        self.enclosed = False
        layers = [L for L in layers if L != 2 or in2_ok(net)]
        fixed, routed = self.free_maps(w, obs, net, wd / 2)
        vf, vr = self.via_maps(w, obs, holes, net, VIA[0], VIA[1], (0, 1, 2))
        wide = [fixed[L] & (routed[L] if not rip else True) for L in range(NL)]
        if nk:                       # neck down to ``nk`` within NECK_MM of the two terminals
            fn, rn = self.free_maps(w, obs, net, nk / 2)
            near = [(fa[L] <= NECK_MM) | ((fb[L] <= NECK_MM) if fb is not None else False) for L in range(NL)]
            if self.neck_zones:
                nz = self.poly_mask(w, self.neck_zones)
                near = [near[L] | nz[L] for L in range(NL)]
            fixed = [fixed[L] | (fn[L] & near[L]) for L in range(NL)]
            routed = [routed[L] | (rn[L] & near[L]) for L in range(NL)]
        free = list(fixed) if rip else [fixed[L] & routed[L] for L in range(NL)]
        for L in range(NL):
            if L not in layers:
                free[L] = np.zeros_like(free[L])
        src = np.stack([(fa[L] <= 0) & free[L] for L in range(NL)])
        dst = np.stack([(fb[L] <= 0) & free[L] for L in range(NL)]) if fb is not None else np.zeros_like(src)
        kinds = None
        if plane:
            pt, kinds = self.plane_targets(w, obs, holes, net, free, rip)
            dst = dst | pt
        if target is not None:
            dst = target[1](w, free)
        if not src.any() or not dst.any():
            return None
        hist = self.hist[:, w.i0:w.i0 + w.nx, w.j0:w.j0 + w.ny]
        cost = np.where(np.stack(free), hist, -1.0).astype(np.float32)
        if pn or netclass(net) == "SE_50":        # high-speed: F.Cu over the GND plane
            cost[1] = np.where(cost[1] >= 0, cost[1] + 0.3, cost[1])
        cost[2] = np.where(cost[2] >= 0, cost[2] + IN2_COST, cost[2])
        if rip:
            clean = np.stack(routed)
            cost = np.where((cost >= 0) & ~clean, cost + RIP_COST, cost)
        via_cost = VIA_COST * (4 if pn else 3 if netclass(net) == "SE_50" else 1)
        vias = []
        micro = None
        if not no_vias:
            through_bits = sum(1 << L for L in layers)
            vias.append((through_bits, vf if rip else vf & vr, via_cost))
            if 2 in layers:          # L4 -> L3 laser microvia between B.Cu and In2.Cu
                mf, mr = self.via_maps(w, obs, holes, net, MICRO[0], MICRO[1], (1, 2))
                micro = mf if rip else mf & mr
                vias.append((BOT | IN2, micro, via_cost * 0.6))
        path = self.search(w, cost, vias, src, dst)
        if path is None:
            return None
        narrow = [not wide[L][i, j] for L, i, j in path] if nk else None
        segs, vcells = self.geometry(w, path, narrow, micro)
        vias_out = [(*w.xy(i, j), kind) for i, j, kind in vcells]
        L, i, j = path[-1]
        if plane and not (fb is not None and fb[L][i, j] <= 0):
            vias_out.append((*w.xy(i, j), kinds[L]))
        victims = []
        if rip:
            vic = self.victims(w, conn, path, wd / 2, [(i_, j_) for i_, j_, _ in vcells] + ([(i, j)] if plane else []),
                               VIA[0] / 2, halves=[nk / 2 if n else wd / 2 for n in narrow] if narrow else None)
            vic.discard(-1)
            victims = [self.by_id[v] for v in vic if v in self.by_id]
            if any(not self.may_rip(conn, b) for b in victims):
                return None
        return segs, vias_out, victims, wd, nk, bool(narrow and any(narrow))

    # ------------------------------------------------------------------ plane integrity
    PLANE_GRID = 0.1
    ZONE_MIN_THICKNESS = 0.15

    def plane_parts(self) -> dict[str, int]:
        """In2.Cu carries the +3V3 plane and the VDD_HP island. Count, per plane, the pieces
        its vias / pins end up in once the fill flows around the other nets' In2 copper
        (conservative raster: zone clearance, half the minimum fill width, half a cell)."""
        if getattr(self, "_parts_version", None) == self.cu.in2_version:
            return self._parts
        from scipy import ndimage
        g = self.PLANE_GRID
        if not hasattr(self, "_pX"):
            x0, y0, x1, y1 = bs.BOARD_OUTLINE
            xs = np.arange(x0, x1 + g / 2, g)
            ys = np.arange(y0, y1 + g / 2, g)
            self._pX, self._pY = np.meshgrid(xs, ys, indexing="ij")
            inset = EDGE_MM + self.ZONE_MIN_THICKNESS / 2
            a = np.maximum(np.abs(self._pX), np.abs(self._pY))
            board = (self._pX > x0 + inset) & (self._pX < x1 - inset) & (self._pY > y0 + inset) & \
                (self._pY < y1 - inset)
            isl = lp.VDD_HP_ISLAND_HALF_MM
            self._pregion = {"+3V3": board & (a > isl + lp.ZONE_CLEARANCE_MM + self.ZONE_MIN_THICKNESS / 2),
                             "VDD_HP": a < isl - self.ZONE_MIN_THICKNESS / 2}
        X, Y = self._pX, self._pY
        cu = self.cu
        n = len(cu.items)
        idx = np.nonzero(cu.alive[:n] & ((cu.lay[:n] & IN2) > 0))[0]
        out = {}
        for plane, region in self._pregion.items():
            blocked = np.zeros(X.shape, bool)
            for k in idx:
                it = cu.items[k]
                if it.net == plane or it.what == "keepout":
                    continue
                c = max(lp.ZONE_CLEARANCE_MM, clearance(it.net) if it.what != "npth" else HOLE_CLEARANCE)
                lim = c + self.ZONE_MIN_THICKNESS / 2 + g / 2
                bx0, by0, bx1, by1 = it.bbox()
                i0 = max(0, int((bx0 - lim - X[0, 0]) / g))
                i1 = min(X.shape[0], int((bx1 + lim - X[0, 0]) / g) + 2)
                j0 = max(0, int((by0 - lim - Y[0, 0]) / g))
                j1 = min(X.shape[1], int((by1 + lim - Y[0, 0]) / g) + 2)
                if i0 >= i1 or j0 >= j1:
                    continue
                blocked[i0:i1, j0:j1] |= item_dist(it, X[i0:i1, j0:j1], Y[i0:i1, j0:j1]) < lim
            lab, _ = ndimage.label(region & ~blocked)
            parts = set()
            for k in cu.by_net.get(plane, ()):
                it = cu.items[k]
                if it.layers & IN2 and it.kind != 2:
                    i = int(round((it.par[0] - X[0, 0]) / g))
                    j = int(round((it.par[1] - Y[0, 0]) / g))
                    if 0 <= i < X.shape[0] and 0 <= j < X.shape[1] and lab[i, j]:
                        parts.add(int(lab[i, j]))
            out[plane] = len(parts)
        self._parts, self._parts_version = out, self.cu.in2_version
        return out

    def _apply(self, conn: Conn, plan) -> bool:
        """Commit a plan (ripping up its victims). A route that would cut the +3V3 plane or
        the VDD_HP island on In2.Cu into more pieces is refused (returns False)."""
        segs, vias, victims, wd, nk, necked = plan
        touches_in2 = any(seg[0] == 2 for seg in segs) or any(k in ("through", "micro_bottom") for *_, k in vias)
        touches_in2 = touches_in2 and conn.net not in PLANE_NETS
        before = self.plane_parts() if touches_in2 else None
        mark = len(conn.items)
        self.commit(conn, segs, wd, vias, neck=nk)
        if touches_in2:
            after = self.plane_parts()
            if any(after[p] > before[p] for p in before):
                for k in conn.items[mark:]:
                    self.cu.remove(k)
                del conn.items[mark:]
                conn.done = bool(conn.items)
                self.stats["plane_rejects"] = self.stats.get("plane_rejects", 0) + 1
                return False
        for b in victims:
            for k in b.items:
                self._bump_history(self.cu.items[k])
            self.rip(b)
            self.requeue(b)
            mate = self.by_id.get(b.pair) if b.pair >= 0 else None
            if self.push_pairs and mate is not None and mate.done and mate.id != conn.id:
                self.rip(mate)                   # a pushed pair is re-routed coupled
                self.requeue(mate)
        if victims:
            self.log(f"    {conn.net}: ripped {', '.join(sorted({b.net for b in victims}))}")
        conn.note = f"w={wd:g}" + (f" (neck {nk:g})" if necked else "") + (f", {len(vias)} via" if vias else "")
        return True

    def may_rip(self, by: Conn, victim: Conn) -> bool:
        """``may_rip``, and in a last-resort search (``push_pairs``) a connection that has no
        other way may also push a differential pair aside (both halves are re-routed) or a
        pre-routed SoC escape track (see ``soft_escapes``)."""
        if victim.note == "escape":
            return self.push_pairs and victim.rips < PAIR_PUSH_MAX
        if may_rip(by, victim):
            return True
        return (self.push_pairs and bool(partner(victim.net)) and victim.net != partner(by.net)
                and victim.rips < PAIR_PUSH_MAX)

    def requeue(self, c: Conn) -> None:
        """Put a ripped-up connection back; earlier routing (a "legacy" piece) has no
        endpoints of its own, so its net's missing connections are derived afresh."""
        if c.note not in ("legacy", "escape"):
            self.queue.append(c)
            return
        for nc in net_connections(self.cu, c.net, self.next_id):
            self.conns.append(nc)
            self.by_id[nc.id] = nc
            self.queue.append(nc)

    def _bump_history(self, it: Item) -> None:
        """Raise the history cost on the cells a ripped-up item covered (plus its clearance)."""
        x0, y0, x1, y1 = it.bbox()
        g, pad = self.g, max(CLASSES)
        i0 = max(0, int((x0 - pad - self.bx0) / g))
        i1 = min(self.NX - 1, int((x1 + pad - self.bx0) / g) + 1)
        j0 = max(0, int((y0 - pad - self.by0) / g))
        j1 = min(self.NY - 1, int((y1 + pad - self.by0) / g) + 1)
        if i0 > i1 or j0 > j1:
            return
        xs = self.bx0 + np.arange(i0, i1 + 1) * g
        ys = self.by0 + np.arange(j0, j1 + 1) * g
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        p = it.par
        if it.kind == 2:
            vx, vy = p[2] - p[0], p[3] - p[1]
            L = vx * vx + vy * vy
            u = np.zeros_like(X) if L == 0 else np.clip(((X - p[0]) * vx + (Y - p[1]) * vy) / L, 0, 1)
            d = np.hypot(p[0] + u * vx - X, p[1] + u * vy - Y) - p[4]
        else:
            d = np.hypot(X - p[0], Y - p[1]) - (p[2] if it.kind == 1 else max(p[2], p[3]))
        near = d <= pad
        for L in range(NL):
            if (it.layers >> L) & 1:
                self.hist[L, i0:i1 + 1, j0:j1 + 1] += np.where(near, HISTORY_STEP / 10, 0).astype(np.float32)

    # ------------------------------------------------------------------ pairs
    def route_pair(self, cp: Conn, cn: Conn, rip: bool = False) -> bool:
        """Coupled routing of a P / N connection pair (see the module doc)."""
        why: list[str] = []
        ok = self._route_pair(cp, cn, why, rip)
        if not ok:
            cp.note = "; ".join(dict.fromkeys(why))
        return ok

    def _route_pair(self, cp: Conn, cn: Conn, why: list, rip: bool = False) -> bool:
        net = cp.net
        wd = PAIR_CLASSES[netclass(net)][0]
        gap = pair_gap(net)
        h = (wd + gap) / 2
        # terminals: A end (cp.ta / cn.ta), B end (cp.tb / cn.tb)
        ma = ((cp.ta[0] + cn.ta[0]) / 2, (cp.ta[1] + cn.ta[1]) / 2)
        mb = ((cp.tb[0] + cn.tb[0]) / 2, (cp.tb[1] + cn.tb[1]) / 2)
        if math.dist(ma, mb) < PAIR_MIN_COUPLED_MM:
            why.append("short: routed as two tracks")
            return False
        span = math.dist(ma, mb)
        launches = [x for x in PAIR_LAUNCH_MM if x <= max(PAIR_LAUNCH_MM[0], 0.3 * span)]
        launches = launches[-1:] if rip else launches
        pads = WINDOWS_MM[-1:] if rip else WINDOWS_MM
        # F.Cu over the GND plane first; B.Cu (over the +3V3 plane, same stack-up distance)
        # only when F.Cu has no coupled path -- the fan-ins then drop to it with vias
        for PL, launch, pad in [(PL, la, pa) for PL in (0, 1) for la in launches for pa in pads]:
            w = self.window([cp.ta, cp.tb, cn.ta, cn.tb], pad)
            half = h + wd / 2
            reach = half + max(CLASSES) + 0.1
            obs, ownf, holes = self.fields(w, {"#none"}, None, reach, ripper=cp)
            # fat centre line: every other copper, the pair's own included, is an obstacle
            fixed, routed = self.free_maps(w, obs, net, half, margin=MARGIN_MM + 0.015)
            free0 = fixed[PL] if rip else fixed[PL] & routed[PL]
            X, Y = w.mesh()
            ra = launch + math.dist(cp.ta, cn.ta) / 2
            rb = launch + math.dist(cp.tb, cn.tb) / 2
            src = np.zeros((NL, w.nx, w.ny), bool)
            dst = np.zeros((NL, w.nx, w.ny), bool)
            src[PL] = free0 & (np.hypot(X - ma[0], Y - ma[1]) <= ra)
            dst[PL] = free0 & (np.hypot(X - mb[0], Y - mb[1]) <= rb)
            if not src.any() or not dst.any():
                why.append(f"no launch room at the {'A' if not src.any() else 'B'} end (r {launch})")
                continue
            if (src & dst).any():
                why.append("launch regions overlap")
                continue
            hist = self.hist[:, w.i0:w.i0 + w.nx, w.j0:w.j0 + w.ny]
            cost = np.full((NL, w.nx, w.ny), -1.0, np.float32)
            cost[PL] = np.where(free0, hist[PL], -1.0)
            if rip:
                cost[PL] = np.where((cost[PL] >= 0) & ~routed[PL], cost[PL] + RIP_COST, cost[PL])
            path = self.search(w, cost, [], src, dst, turn90=-1.0, turn45=8.0)
            if path is None:
                why.append(f"no coupled path on {LAYER_NAMES[PL]} (window {pad} mm)")
                continue
            pts = [w.xy(i, j) for L, i, j in path]
            line = simplify(pts)
            offs = offset_polyline(line, h) if len(line) >= 2 else None
            if offs is None:
                why.append("degenerate centre line")
                continue
            victims = []
            if rip:
                vic = self.victims(w, cp, path, half, [], half, own={cp.net, cn.net})
                vic.discard(-1)
                victims = [self.by_id[v] for v in vic if v in self.by_id]
                if not victims or any(not may_rip(cp, v) for v in victims):
                    why.append("rip-up refused")
                    continue
                for v in victims:
                    for k in v.items:
                        self._bump_history(self.cu.items[k])
                    self.rip(v)
                    if not partner(v.net):
                        self.requeue(v)          # (ripped pairs are queued by the pair loop)
                self.log(f"    {net[:-2]}: ripped {', '.join(sorted({v.net for v in victims}))}")
                self.ripped_pairs += [v for v in victims if partner(v.net)]
            left, right = offs
            # which side is P: the side of the A-end P terminal relative to the start direction
            d0 = (line[1][0] - line[0][0], line[1][1] - line[0][1])
            side_p = cross(d0, (cp.ta[0] - cn.ta[0], cp.ta[1] - cn.ta[1]))
            pl, nl = (left, right) if side_p > 0 else (right, left)
            d1 = (line[-1][0] - line[-2][0], line[-1][1] - line[-2][1])
            side_b = cross(d1, (cp.tb[0] - cn.tb[0], cp.tb[1] - cn.tb[1]))
            swapped = side_b * side_p < 0
            cpp = Conn(self.next_id(), cp.net, [], None, pl[0], pl[-1], prio=cp.prio)
            cnn = Conn(self.next_id(), cn.net, [], None, nl[0], nl[-1], prio=cn.prio)
            for c_, poly in ((cpp, pl), (cnn, nl)):
                segs = [(PL, a, b) for a, b in zip(poly, poly[1:])]
                self.commit(c_, segs, wd, [])
            ok = True
            fans = []
            # the P / N order is reversed at the B end: one fan-in crosses under the other
            # (two vias; Espressif: add GND return vias next to them -- the GND pour does)
            for c, poly, first in ((cp, pl, True), (cn, nl, True), (cp, pl, False), (cn, nl, False)):
                end = poly[0] if first else poly[-1]
                grp = c.a if first else c.b
                fc = Conn(self.next_id(), c.net, grp, None, c.ta if first else c.tb, end, prio=c.prio)

                def tgt(w2, free, end=end, PL=PL):
                    X2, Y2 = w2.mesh()
                    m = np.zeros((NL, w2.nx, w2.ny), bool)
                    m[PL] = (np.hypot(X2 - end[0], Y2 - end[1]) <= wd * 0.45) & free[PL]
                    return m
                cross_under = (swapped and not first and c is cn) or PL == 1
                if not self.route(fc, layers=(0, 1) if cross_under else (0,), no_vias=not cross_under,
                                  target=(end, tgt), width_list=(wd,)):
                    why.append(f"fan-in of {c.net} at the {'A' if first else 'B'} end"
                               + (" (P/N order reversed)" if swapped and not first else ""))
                    ok = False
                    break
                last = next(self.cu.items[k] for k in reversed(fc.items) if self.cu.items[k].kind == 2)
                tip = (last.par[2], last.par[3])
                if math.dist(tip, end) > 1e-6:          # last grid cell -> exact offset end
                    self.commit(fc, [(PL, tip, end)], wd, [])
                fans.append(fc)
            if not ok:
                for c_ in [cpp, cnn] + fans:
                    for k in c_.items:
                        self.cu.remove(k)
                continue                                # (ripped victims are queued again)
            cp.items = cpp.items + [k for f in fans if f.net == cp.net for k in f.items]
            cn.items = cnn.items + [k for f in fans if f.net == cn.net for k in f.items]
            for c in (cp, cn):
                for k in c.items:
                    self.cu.items[k].conn = c.id
                c.done = True
            bends = len(line) - 2
            length = sum(math.dist(a, b) for a, b in zip(line, line[1:]))
            cp.note = cn.note = (f"coupled, {length:.1f} mm, {bends} bends"
                                 + (f" on {LAYER_NAMES[PL]}" if PL else "")
                                 + (", P/N crossed with vias" if swapped else ""))
            return True
        return False

    # ------------------------------------------------------------------ driver
    def adopt_routing(self) -> int:
        """Routing that came with the board (unlocked copper, ``parse_board(...,
        routed_unlocked=True)``): one pseudo connection per connected piece, so it can be
        ripped up like the router's own. Ripping one re-derives its net's connections."""
        cu = self.cu
        n = 0
        by_net: dict[str, list[int]] = {}
        for k, it in enumerate(cu.items):
            if cu.alive[k] and not it.fixed and it.what not in ("keepout", "npth") and it.conn < 0:
                by_net.setdefault(it.net, []).append(k)
        for net, ks in by_net.items():
            d = DSU(len(ks))
            for x in range(len(ks)):
                for y in range(x + 1, len(ks)):
                    if touching(cu.items[ks[x]], cu.items[ks[y]]):
                        d.union(x, y)
            groups: dict[int, list[int]] = {}
            for x, k in enumerate(ks):
                groups.setdefault(d.find(x), []).append(k)
            for g in groups.values():
                c = Conn(self.next_id(), net, [], None, cu.items[g[0]].anchors()[0], cu.items[g[0]].anchors()[0],
                         prio=priority(net), items=list(g), done=True, note="legacy")
                for k in g:
                    cu.items[k].conn = c.id
                self.conns.append(c)
                n += 1
        return n

    def run(self, max_iter: int = 20000) -> None:
        cu = self.cu
        nets = sorted(n for n in cu.by_net if n)
        seen = set()
        for net in nets:
            base = pair_base(net)
            if base and partner(net) in cu.by_net:
                if base in seen:
                    continue
                seen.add(base)
                pc = pair_connections(cu, base, self.next_id)
                if pc is not None:
                    self.conns += pc
                    continue
                self.log(f"  pair {base}: P and N islands differ, routed as two nets")
                self.conns += net_connections(cu, base + "_P", self.next_id)
                self.conns += net_connections(cu, base + "_N", self.next_id)
                continue
            self.conns += net_connections(cu, net, self.next_id)
        self.by_id = {c.id: c for c in self.conns}
        # Connections leaving the SoC go right after the pairs: other nets must not wall off
        # the ends of the escape tracks (they are packed at the 0.35 mm pad pitch).
        for c in self.conns:
            if c.prio > SOC_RANK and self._touches_u1(c):
                c.prio = SOC_RANK
        self.log(f"{len(self.conns)} connections to route")
        pairs = {c.id: self.by_id[c.pair] for c in self.conns if c.pair >= 0 and c.net.endswith("_P")}
        self.log(f"{len(pairs)} differential-pair connections routed coupled first")
        t0 = time.time()
        # Fan-out first (usual practice): the pin-to-neighbour connections of the fine-pitch
        # parts and every plane pad's via. They have no alternative route; long connections do.
        local = [c for c in self.conns if c.b is None or c.length <= LOCAL_MM]
        for c in sorted(local, key=lambda c: (c.length, c.prio)):
            if not c.done:
                self.route(c)
        self.log(f"  fan-out: {sum(c.done for c in local)} of {len(local)} local connections")
        # U1 pads whose escape track the planner could not fit end at a bare stub inside the
        # breakout: route those connections first, while the breakout still has room.
        bare = [c for c in self.conns if self._bare_stub(c.a) or (c.b is not None and self._bare_stub(c.b))]
        for c in sorted(bare, key=lambda c: (c.prio, c.length)):
            if c.done:
                continue
            ok = self.route(c) or self.route(c, allow_rip=True)
            self.log(f"  SoC stub {c.net}: {c.note if ok else 'not yet'}")
        self.ripped_pairs: list[Conn] = []
        todo = sorted(pairs.items(), key=lambda kv: (self.by_id[kv[0]].prio, -self.by_id[kv[0]].length))
        tries: dict[int, int] = {}
        while todo:
            cid, dn = todo.pop(0)
            cp = self.by_id[cid]
            if cp.done or dn.done:
                continue
            tries[cid] = tries.get(cid, 0) + 1
            ok = self.route_pair(cp, dn)
            if not ok and tries[cid] <= 3 and not cp.note.startswith("short"):
                self.ripped_pairs = []
                first = cp.note
                ok = self.route_pair(cp, dn, rip=True)
                if not ok:
                    cp.note = f"{first} | rip-up: {cp.note}"
                for v in self.ripped_pairs:              # ripped pairs go back into the queue
                    p_id = v.id if v.net.endswith("_P") else v.pair
                    if p_id in pairs and all(p_id != q for q, _ in todo):
                        todo.append((p_id, pairs[p_id]))
                        other = self.by_id.get(self.by_id[p_id].pair if v.net.endswith("_P") else p_id)
                        for c in (self.by_id[p_id], other):
                            if c is not None and c.done:
                                self.rip(c)
            if ok:
                self.log(f"  pair {cp.net[:-2]}: {cp.note}")
            else:
                self.log(f"  pair {cp.net[:-2]}: not coupled ({cp.note}), routed as two nets")
        self.queue = sorted((c for c in self.conns if not c.done), key=lambda c: (c.prio, c.length))
        self.t0 = t0
        self.drain(max_iter)

    def drain(self, max_iter: int = 20000) -> None:
        """Route the queue (rip-ups put connections back into it)."""
        t0 = getattr(self, "t0", time.time())
        self.queue += [c for c in getattr(self, "failed", []) if not c.done]   # room may have opened
        self.failed = []
        self.queue.sort(key=lambda c: (c.prio, c.length))
        it = 0
        failed = []
        failed_gaps: set = set()       # the same gap is often queued twice (own list + KiCad's)
        while self.queue and it < max_iter:
            it += 1
            conn = self.queue.pop(0)
            if conn.done:
                continue
            # the islands may have merged or split meanwhile: re-derive this net's needs
            if not self._still_needed(conn):
                continue
            gap = (conn.net, min(conn.a), min(conn.b) if conn.b else None)
            if gap in failed_gaps:
                continue
            mate = self.by_id.get(conn.pair) if conn.pair >= 0 else None
            if mate is not None and not mate.done and self._still_needed(mate):
                cp, cn = (conn, mate) if conn.net.endswith("_P") else (mate, conn)
                if self.route_pair(cp, cn) or self.route_pair(cp, cn, rip=True):
                    self.stats["routed"] += 2
                    self.log(f"  pair {cp.net[:-2]} re-routed: {cp.note}")
                    continue
            ok = self.route(conn) or self.route(conn, allow_rip=True)
            if not ok:
                self.push_pairs = True           # last resort: push a differential pair aside
                try:
                    ok = self.route(conn, allow_rip=True)
                finally:
                    self.push_pairs = False
                if ok:
                    self.log(f"    {conn.net}: pushed a differential pair aside")
            if not ok:
                failed_gaps.add(gap)
            if ok:
                self.stats["routed"] += 1
                if it % 25 == 0:
                    self.log(f"  [{it}] {len(self.queue)} queued, {time.time() - t0:.0f} s")
            else:
                failed.append(conn)
                self.log(f"  FAILED {conn.net} ({conn.ta[0]:.2f}, {conn.ta[1]:.2f}) -> "
                         f"({conn.tb[0]:.2f}, {conn.tb[1]:.2f})")
        self.failed = [c for c in getattr(self, "failed", []) + failed if not c.done]
        self.stats["failed"] = len(self.failed)

    def _touches_u1(self, c: Conn) -> bool:
        return any(self.cu.items[k].ref == "U1" for g in (c.a, c.b or []) for k in g)

    def _bare_stub(self, group: list[int]) -> bool:
        """An island that is only a U1 pad and its fan-out stub (no escape track)."""
        lim = lp.U1_STUB_END_MM + 0.05
        for k in group:
            it = self.cu.items[k]
            if it.what == "track":
                if max(abs(it.par[0]), abs(it.par[1]), abs(it.par[2]), abs(it.par[3])) > lim:
                    return False
            elif it.ref != "U1":
                return False
        return True

    def islands(self, net: str):
        key = (net, self.cu.version.get(net, 0))
        if self._isl.get(net, (None,))[0] != key:
            self._isl[net] = (key, net_islands(self.cu, net))
        return self._isl[net][1]

    def _still_needed(self, conn: Conn) -> bool:
        """Refresh the connection's islands from the current copper (routes of this net may
        have been ripped or added since it was queued). An island is found by the copper it
        held when the connection was made (its pads stay; a point lookup would confuse a
        bottom pad with the top pad above it)."""
        groups, plane = self.islands(conn.net)
        where = {k: gi for gi, g in enumerate(groups) for k in g}

        def group_of(items):
            for k in items:
                if self.cu.alive[k] and k in where:
                    return where[k]
            return None
        ga = group_of(conn.a)
        if ga is None:
            return False
        if conn.b is None:
            if ga == plane:
                return False
            conn.a = groups[ga]
            return True
        gb = group_of(conn.b)
        if gb is None or ga == gb:
            return False
        conn.a, conn.b = groups[ga], groups[gb]
        return True


# ==========================================================================
# Polyline helpers
# ==========================================================================

def cross(a, b) -> float:
    return a[0] * b[1] - a[1] * b[0]


def simplify(pts: list) -> list:
    out = [pts[0]]
    for p in pts[1:]:
        if len(out) >= 2:
            a, b = out[-2], out[-1]
            if abs(cross((b[0] - a[0], b[1] - a[1]), (p[0] - b[0], p[1] - b[1]))) < 1e-9 and \
                    (b[0] - a[0]) * (p[0] - b[0]) + (b[1] - a[1]) * (p[1] - b[1]) > 0:
                out[-1] = p
                continue
        if math.dist(p, out[-1]) > 1e-9:
            out.append(p)
    return out


def offset_polyline(line: list, h: float):
    """Both offsets (left, right) of an octilinear polyline at distance h (mitred)."""
    normals = []
    for a, b in zip(line, line[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        normals.append((-dy / L, dx / L))
    left, right = [], []
    for k, p in enumerate(line):
        if k == 0:
            n = normals[0]
            s = 1.0
        elif k == len(line) - 1:
            n = normals[-1]
            s = 1.0
        else:
            n1, n2 = normals[k - 1], normals[k]
            m = (n1[0] + n2[0], n1[1] + n2[1])
            dot = 1 + n1[0] * n2[0] + n1[1] * n2[1]
            if dot < 0.5:                                   # sharper than 90 degrees
                return None
            n = (m[0] / dot, m[1] / dot)
            s = 1.0
        left.append((p[0] + h * n[0] * s, p[1] + h * n[1] * s))
        right.append((p[0] - h * n[0] * s, p[1] - h * n[1] * s))
    return left, right


# ==========================================================================
# Output
# ==========================================================================

def sexpr_items(cu: Copper) -> list[str]:
    ox, oy = ORIGIN
    out = []
    for k, it in enumerate(cu.items):
        if not cu.alive[k] or (it.fixed and not it.rewrite) or it.what == "keepout":
            continue
        if it.kind == 2:
            x0, y0, x1, y1, r = it.par[:5]
            if math.hypot(x1 - x0, y1 - y0) < 1e-6:
                continue
            L = 0 if it.layers & TOP else (1 if it.layers & BOT else 2)
            key = f"{it.net}:{L}:{x0:.4f},{y0:.4f}:{x1:.4f},{y1:.4f}"
            lock = "\t\t(locked yes)\n" if it.rewrite else ""
            out.append(f'\t(segment\n\t\t(start {x0 + ox:.4f} {y0 + oy:.4f})\n\t\t(end {x1 + ox:.4f} {y1 + oy:.4f})\n'
                       f'\t\t(width {2 * r:.4g})\n{lock}\t\t(layer "{LAYER_NAMES[L]}")\n\t\t(net "{it.net}")\n'
                       f'\t\t(uuid "{uuid.uuid5(UUID_NS, key)}")\n\t)')
        else:
            x, y, r = it.par[:3]
            key = f"{it.net}:via:{x:.4f},{y:.4f}"
            if it.what == "via":
                head, layers = "(via", '"F.Cu" "B.Cu"'
            elif it.layers & TOP:
                head, layers = "(via micro", '"F.Cu" "In1.Cu"'
            else:
                head, layers = "(via micro", '"In2.Cu" "B.Cu"'
            out.append(f'\t{head}\n\t\t(at {x + ox:.4f} {y + oy:.4f})\n\t\t(size {2 * r:.4g})\n'
                       f'\t\t(drill {2 * it.hole[2]:.4g})\n\t\t(layers {layers})\n\t\t(net "{it.net}")\n'
                       f'\t\t(uuid "{uuid.uuid5(UUID_NS, key)}")\n\t)')
    return out


def _blocks(text: str, kinds: str = "segment|via|arc"):
    """(start, end) of the top-level blocks of these kinds in a .kicad_pcb text."""
    pos = 0
    for m in re.finditer(rf"\n\t\(({kinds})\b", text):
        start = m.start() + 1
        if start < pos:
            continue
        depth, k = 0, start
        while True:
            ch = text[k]
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    break
            elif ch == '"':
                k = text.index('"', k + 1)
            k += 1
        pos = k + 1
        yield start, pos


def _cut(text: str, drop) -> str:
    """The text without the blocks for which ``drop(block)`` is true."""
    out, pos = [], 0
    for a, b in _blocks(text):
        if drop(text[a:b]):
            out.append(text[pos:a - 1])
            pos = b
    out.append(text[pos:])
    return "".join(out)


def strip_unlocked(text: str) -> str:
    """The board text without its unlocked tracks and vias (``--resume`` re-writes them)."""
    return _cut(text, lambda block: "(locked yes)" not in block)


def segment_key(net: str, layer: str, x0: float, y0: float, x1: float, y1: float) -> tuple:
    """Identifies a segment of the board file (board coordinates)."""
    return (net, layer, round(x0, 4), round(y0, 4), round(x1, 4), round(y1, 4))


_SEG = re.compile(r'\(start ([-\d.]+) ([-\d.]+)\).*?\(end ([-\d.]+) ([-\d.]+)\).*?\(layer "([^"]+)"\)'
                  r'.*?\(net "((?:[^"\\]|\\.)*)"\)', re.S)


def write_board(text: str, cu: Copper, out: Path) -> int:
    if cu.dropped:
        def drop(block):
            m = block.lstrip().startswith("(segment") and _SEG.search(block)
            return bool(m) and segment_key(m[6], m[5], *(float(m[i]) for i in range(1, 5))) in cu.dropped
        text = _cut(text, drop)
    body = text.rstrip()
    assert body.endswith(")")
    new = sexpr_items(cu)
    out.write_text(body[:-1].rstrip() + "\n" + "\n".join(new) + "\n)\n")
    return len(new)


# ==========================================================================
# Dangling track ends
# ==========================================================================

def zone_layers(root: list) -> dict[str, int]:
    """Net -> layer bits of its copper zones (a track end in a zone is not dangling)."""
    out: dict[str, int] = {}
    for z in children(root, "zone"):
        if child(z, "keepout") is not None:
            continue
        net = child(z, "net_name") or child(z, "net")
        names = (child(z, "layers") or child(z, "layer") or [None])[1:]
        if net and isinstance(net[-1], str) and net[-1]:
            out[net[-1]] = out.get(net[-1], 0) | layer_bits(names)
    return out


def _dangling_ends(t: Item, others: list[Item]) -> tuple[bool, bool]:
    """KiCad's test (CONNECTIVITY_DATA::TestTrackEndpointDangling): an end is connected
    when another item comes within half the track width of it; an item that reaches
    both ends of a short track counts only for the end nearer to its own anchors."""
    x0, y0, x1, y1, rr = t.par[:5]
    ends = [0, 0]
    for o in others:
        hs, he = o.dist(x0, y0) <= rr + 1e-6, o.dist(x1, y1) <= rr + 1e-6
        if hs and he:
            near = [min(math.dist(a, p) for a in o.anchors()) for p in ((x0, y0), (x1, y1))]
            ends[0 if near[0] < near[1] else 1] += 1
        elif hs or he:
            ends[0 if hs else 1] += 1
        if ends[0] and ends[1]:
            break
    return not ends[0], not ends[1]


def trim_stubs(cu: Copper, skip: set[str], zones: dict[str, int]):
    """Cut back track ends that connect to nothing (where a route joined a pre-routed
    escape track part-way along it, the rest of the escape is a dead stub). A track end
    counts as connected the way KiCad counts it: another item of the net comes within
    half the track width of it. A dangling end moves back along its own track to the
    farthest point where another item of the net is anchored in the track (a track end,
    via or pad centre); a track with nothing anchored in it goes. Nets in ``skip`` (not
    fully routed) and tracks on a layer where their net has a zone are left alone.
    Returns an undo function (None if nothing changed)."""
    ox, oy = ORIGIN
    log: list[tuple[str, int]] = []
    dropped: list[tuple] = []
    for net in sorted(cu.by_net):
        if not net or net in skip:
            continue
        changed = True
        while changed:
            changed = False
            idx = [k for k in cu.by_net.get(net, ()) if cu.alive[k]]
            for k in idx:
                it = cu.items[k]
                if not cu.alive[k] or it.kind != 2 or it.what != "track" or it.layers & zones.get(net, 0):
                    continue
                x0, y0, x1, y1, rr = it.par[:5]
                others = [cu.items[j] for j in idx if j != k and cu.alive[j] and cu.items[j].layers & it.layers]
                dang = _dangling_ends(it, others)
                if not any(dang):
                    continue
                vx, vy = x1 - x0, y1 - y0
                L2 = vx * vx + vy * vy
                us = [((qx - x0) * vx + (qy - y0) * vy) / L2 if L2 > 0 else 0.0
                      for o in others for qx, qy in o.anchors() if it.dist(qx, qy) <= 1e-4]
                us = [min(max(u, 0.0), 1.0) for u in us]
                ua = min(us) if us and dang[0] else 0.0
                ub = max(us) if us and dang[1] else 1.0
                cu.remove(k)
                log.append(("removed", k))
                if not it.rewrite:
                    key = segment_key(net, LAYER_NAMES[0 if it.layers & TOP else (1 if it.layers & BOT else 2)],
                                      x0 + ox, y0 + oy, x1 + ox, y1 + oy)
                    cu.dropped.add(key)
                    dropped.append(key)
                if us and (ub - ua) * math.sqrt(L2) > 1e-3:
                    part = Item(2, (x0 + ua * vx, y0 + ua * vy, x0 + ub * vx, y0 + ub * vy, rr), it.layers, net,
                                "track", fixed=it.fixed, conn=it.conn, width=it.width, rewrite=it.fixed or it.rewrite)
                    log.append(("added", cu.add(part)))
                changed = True
                break                        # the neighbours changed: look again
    if not log:
        return None

    def undo():
        for what, k in reversed(log):
            (cu.remove if what == "added" else cu.restore)(k)
        cu.dropped.difference_update(dropped)
    undo.count = sum(1 for what, _ in log if what == "removed")
    return undo


# ==========================================================================
# Length matching
# ==========================================================================

SKEW_TARGET_MM = 0.15          # tune a pair when its P / N lengths differ by more (DRU max 0.254)
BUMP_GAIN = 2 * (math.sqrt(2) - 1)      # extra length of a 45-degree bump per mm of height


def net_length(cu: Copper, net: str) -> float:
    return sum(math.dist(cu.items[k].par[:2], cu.items[k].par[2:4])
               for k in cu.by_net.get(net, ()) if cu.items[k].kind == 2)


def clear_path(r: Router, net: str, pts: list, width: float) -> bool:
    """Exact check of a polyline of ``net`` against all other copper (sampled every 0.02 mm)."""
    cu = r.cu
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    reach = width / 2 + max(CLASSES) + 0.05
    sel = cu.select((min(xs), min(ys), max(xs), max(ys)), reach)
    pn = partner(net)
    samples = []
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.dist(a, b) / 0.02))
        samples += [(a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n) for t in range(n + 1)]
    x0, y0, x1, y1 = bs.BOARD_OUTLINE
    for x, y in samples:
        if min(x - x0, x1 - x, y - y0, y1 - y) < EDGE_MM + width / 2 + MARGIN_MM + bs.BOARD_CORNER_R:
            return False
    for k in sel:
        it = cu.items[k]
        if it.net == net or not (it.layers & TOP):
            continue
        brk = [max(abs(x), abs(y)) < BREAKOUT_HALF for x, y in samples]
        if it.net == pn:
            lim = [width / 2 + partner_clearance(net) + 0.002] * len(samples)
        else:
            c = CLASSES[cu.cls[k]] if it.what not in ("npth", "keepout") else CLASSES[2]
            base = width / 2 + max(clearance(net), c) + MARGIN_MM
            lim = [width / 2 + BREAKOUT_CLEARANCE + MARGIN_MM if b_ else base for b_ in brk]
        bb = it.bbox()
        for (x, y), L in zip(samples, lim):
            if bb[0] - L <= x <= bb[2] + L and bb[1] - L <= y <= bb[3] + L and it.dist(x, y) < L:
                return False
    return True


def tune_pair(r: Router, pn: str, nn: str) -> str:
    """Add 45-degree bumps to the shorter net of a pair until the skew is within target."""
    cu = r.cu
    lp_, ln_ = net_length(cu, pn), net_length(cu, nn)
    skew = abs(lp_ - ln_)
    if skew <= SKEW_TARGET_MM:
        return f"skew {skew:.3f} mm"
    short = pn if lp_ < ln_ else nn
    other = nn if short == pn else pn
    width = PAIR_CLASSES[netclass(short)][0]
    added = 0.0
    for _ in range(6):
        need = abs(net_length(cu, pn) - net_length(cu, nn))
        if need <= SKEW_TARGET_MM * 0.5:
            break
        cand = sorted((k for k in cu.by_net.get(short, ()) if not cu.items[k].fixed and cu.items[k].kind == 2
                       and cu.items[k].layers & TOP),
                      key=lambda k: -math.dist(cu.items[k].par[:2], cu.items[k].par[2:4]))
        done = False
        for k in cand:
            it = cu.items[k]
            a, b = it.par[:2], it.par[2:4]
            L = math.dist(a, b)
            h = min(need / BUMP_GAIN, 0.6)
            if L < 2 * h + 0.4:
                h = (L - 0.4) / 2
                if h < 0.08:
                    continue
            ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
            # bump away from the partner: side where the partner's copper is farther
            mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            part = [cu.items[q] for q in cu.by_net.get(other, ())]
            best_side = None
            for sgn in (1, -1):
                nx_, ny_ = -uy * sgn, ux * sgn
                probe = (mid[0] + nx_ * 0.3, mid[1] + ny_ * 0.3)
                dpart = min((q.dist(*probe) for q in part), default=9.0)
                if best_side is None or dpart > best_side[0]:
                    best_side = (dpart, sgn)
            for sgn in (best_side[1], -best_side[1]):
                nx_, ny_ = -uy * sgn, ux * sgn
                t0 = (L - 2 * h) / 2 - 0.1
                t0 = max(0.2, t0 - (L / 2 - h - 0.2) * 0.0)
                p1 = (a[0] + ux * t0, a[1] + uy * t0)
                p2 = (p1[0] + (ux + nx_) * h, p1[1] + (uy + ny_) * h)
                p3 = (p2[0] + ux * 0.2, p2[1] + uy * 0.2)
                p4 = (p3[0] + (ux - nx_) * h, p3[1] + (uy - ny_) * h)
                if math.dist(a, p4) > L - 0.1:
                    continue
                pts = [a, p1, p2, p3, p4, b]
                if not clear_path(r, short, [p1, p2, p3, p4], width):
                    continue
                conn = r.by_id.get(it.conn)
                cu.remove(k)
                segs = [(0, u, v) for u, v in zip(pts, pts[1:])]
                if conn is not None:
                    conn.items.remove(k)
                    r.commit(conn, segs, width, [])
                else:
                    for u, v in zip(pts, pts[1:]):
                        cu.add(Item(2, (u[0], u[1], v[0], v[1], width / 2), TOP, short, "track", fixed=False,
                                    width=width))
                added += BUMP_GAIN * h
                done = True
                break
            if done:
                break
        if not done:
            break
    skew2 = abs(net_length(cu, pn) - net_length(cu, nn))
    return f"skew {skew:.3f} -> {skew2:.3f} mm"


RETURN_VIA_MM = (0.55, 1.2)     # a GND return via this far from a high-speed signal via


def return_vias(r: Router) -> int:
    """Espressif: add ground return vias at each layer change of the USB / MIPI / SDIO
    lines. For every through via of a pair or 50-ohm net, a GND through via (In1 plane and
    both outer GND pours) goes as close as it fits."""
    cu = r.cu
    sig = [k for k, it in enumerate(cu.items)
           if cu.alive[k] and not it.fixed and it.what == "via" and (partner(it.net) or netclass(it.net) == "SE_50")]
    added = 0
    for k in sig:
        x, y = cu.items[k].par[:2]
        w = r.window([(x, y)], RETURN_VIA_MM[1] + 0.5)
        obs, _, holes = r.fields(w, {bs.GND}, None, VIA[0] + max(CLASSES) + 0.1)
        vf, vr = r.via_maps(w, obs, holes, bs.GND, VIA[0], VIA[1], (0, 1, 2))
        X, Y = w.mesh()
        d = np.hypot(X - x, Y - y)
        ok = vf & vr & (d >= RETURN_VIA_MM[0]) & (d <= RETURN_VIA_MM[1])
        if not ok.any():
            continue
        i, j = np.unravel_index(np.argmin(np.where(ok, d, np.inf)), d.shape)
        c = Conn(r.next_id(), bs.GND, [], None, (x, y), (x, y), prio=8)
        r.commit(c, [], VIA[0], [(*w.xy(i, j), "through")])
        c.note = "return via"
        r.conns.append(c)
        r.by_id[c.id] = c
        added += 1
    return added


def tune_all(r: Router) -> None:
    for p_net, n_net, _ in bs.DIFF_PAIRS:
        if p_net in r.cu.by_net and n_net in r.cu.by_net:
            r.log(f"  length match {p_net[:-2]}: {tune_pair(r, p_net, n_net)}")


# ==========================================================================
# KiCad as the judge
# ==========================================================================

# DRC errors caused by copper the router placed (the SI rules are handled separately)
REPAIR_TYPES = {"clearance", "shorting_items", "tracks_crossing", "hole_clearance", "hole_to_hole",
                "copper_edge_clearance", "solder_mask_bridge", "track_width", "via_diameter",
                "annular_width", "items_not_allowed", "drill_out_of_range"}


def kicad_drc(cli: str, board: Path, out: Path, save: bool = False) -> dict:
    cmd = [cli, "pcb", "drc", "--format", "json", "--severity-all", "--units", "mm", "--refill-zones",
           "-o", str(out)]
    if save:
        cmd.append("--save-board")
    subprocess.run(cmd + [str(board)], capture_output=True, text=True)
    import json
    return json.loads(out.read_text())


_BRACKET = re.compile(r"\[([^\]]*)\]")


def unconnected_nets(report: dict) -> set[str]:
    """Nets of KiCad's unconnected items."""
    return {m[0] for u in report.get("unconnected_items", []) for it in u.get("items", [])
            for m in [_BRACKET.findall(it.get("description", ""))] if m}


def repair(r: Router, report: dict) -> tuple[int, int]:
    """Rip up the routed connections KiCad's DRC blames and queue KiCad's unconnected
    items. Returns (ripped, new connections)."""
    cu = r.cu
    blamed: set[int] = set()
    for v in report.get("violations", []):
        if v.get("severity") != "error" or v.get("type") not in REPAIR_TYPES:
            continue
        for item in v.get("items", []):
            m = _BRACKET.search(item.get("description", ""))
            if not m:
                continue
            net = m.group(1)
            x, y = item["pos"]["x"] - ORIGIN[0], item["pos"]["y"] - ORIGIN[1]
            for k in cu.by_net.get(net, ()):
                it = cu.items[k]
                if it.fixed or it.conn < 0 or it.rewrite:
                    continue
                if it.dist(x, y) <= 0.05 or any(math.dist(a, (x, y)) < 0.05 for a in it.anchors()):
                    blamed.add(it.conn)
    ripped = 0
    for cid in blamed:
        c = r.by_id.get(cid)
        if c is None or not c.done:
            continue
        for k in c.items:
            r._bump_history(cu.items[k])
        r.rip(c)
        if c.note != "return via":           # a return via that does not fit is just dropped
            r.requeue(c)
        ripped += 1
    new = 0
    for u in report.get("unconnected_items", []):
        ends = []
        for item in u.get("items", []):
            desc = item.get("description", "")
            m = _BRACKET.search(desc)
            if m:
                bits = (TOP if "Signal_Top" in desc else 0) | (BOT if "Signal_Bottom" in desc else 0) | \
                    (IN2 if "VCC_3V3" in desc else 0)
                if desc.startswith("PTH") or not bits:
                    bits = TOP | BOT
                ends.append((m.group(1), (item["pos"]["x"] - ORIGIN[0], item["pos"]["y"] - ORIGIN[1]), bits))
        if len(ends) != 2 or ends[0][0] != ends[1][0]:
            continue
        net = ends[0][0]
        groups, plane = r.islands(net)

        def group_of(pt, bits):
            best, bd = None, 0.05
            for gi, g in enumerate(groups):
                for k in g:
                    it = cu.items[k]
                    if it.layers & bits:
                        d = it.dist(*pt)
                        if d < bd:
                            best, bd = gi, d
            return best
        ga, gb = group_of(*ends[0][1:]), group_of(*ends[1][1:])
        if ga is None or gb is None:
            continue
        if ga == gb:
            r.log(f"  KiCad sees {net} unconnected at {ends[0][1]} / {ends[1][1]}: one island here")
            continue
        c = Conn(r.next_id(), net, groups[ga], groups[gb], ends[0][1], ends[1][1], prio=priority(net))
        r.conns.append(c)
        r.by_id[c.id] = c
        r.queue.append(c)
        new += 1
    return ripped, new


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("board", type=Path, help="pre-routed board (hardware/pcbnew/preroute.py)")
    ap.add_argument("-o", "--out", type=Path, required=True)
    ap.add_argument("--grid", type=float, default=GRID_MM)
    ap.add_argument("--kicad-cli", help="judge each pass with KiCad's DRC and repair what it reports")
    ap.add_argument("--resume", action="store_true",
                    help="the board is already (partly) routed: keep its unlocked tracks and vias as "
                         "rip-up-able routing and route what is missing")
    ap.add_argument("--rounds", type=int, default=10,
                    help="at most this many DRC repair rounds (with --kicad-cli); stops when they stop helping")
    args = ap.parse_args(argv)
    text = args.board.read_text()
    items, notes = parse_board(text, routed_unlocked=args.resume)
    if args.resume:
        text = strip_unlocked(text)         # re-written from the router's copper
    for n in notes:
        print("  " + n)
    t0 = time.time()
    dru = args.board.with_suffix(".kicad_dru")
    fine = dru_fine_pitch_refs(dru.read_text()) if dru.exists() else []
    yards = parse_courtyards(text)
    necks = [yards[ref] for ref in fine if ref in yards]
    u1 = [(TOP | BOT | IN2, yards["U1"][1])] if "U1" in yards else []
    necks += u1                        # DRU 'u1_fanout_clearance' / '_track_width': U1 courtyard, either side
    print(f"  fine-pitch fan-out neck-down in the courtyards of {', '.join(fine) or '-'}"
          + (", U1 (both sides)" if "U1" in yards else ""))
    root = sexpr(text)
    zones = zone_layers(root)
    r = Router(items, args.grid, necks=necks, keepouts=keepout_polygons(root), neck_zones=u1)
    if args.resume:
        print(f"  resume: {r.adopt_routing()} pieces of earlier routing adopted")
    r.run()
    out = args.out.resolve()
    for suffix in (".kicad_pro", ".kicad_dru"):
        src = args.board.with_suffix(suffix)
        if src.exists() and src.resolve() != out.with_suffix(suffix):
            out.with_suffix(suffix).write_text(src.read_text())
    tune_all(r)
    print(f"GND return vias next to high-speed signal vias: {return_vias(r)}")
    n = write_board(text, r.cu, out)
    if args.kicad_cli:
        report_path = out.with_suffix(".drc.json")
        best, stale = None, 0
        for rnd in range(1, args.rounds + 1):
            rep = kicad_drc(args.kicad_cli, out, report_path)
            errs = [v for v in rep.get("violations", []) if v.get("severity") == "error"
                    and v.get("type") in REPAIR_TYPES]
            un = rep.get("unconnected_items", [])
            print(f"DRC round {rnd}: {len(un)} unconnected, {len(errs)} copper errors", flush=True)
            if not errs and not un:
                break
            score = len(un) + len(errs)
            stale = stale + 1 if best is not None and score >= best else 0
            best = score if best is None else min(best, score)
            if stale >= 2:
                break
            ripped, new = repair(r, rep)
            print(f"  repair: {ripped} connections ripped, {new} new from KiCad's unconnected list")
            if not ripped and not new:
                break
            r.drain()
            n = write_board(text, r.cu, out)
        rep = kicad_drc(args.kicad_cli, out, report_path)
        undo = trim_stubs(r.cu, unconnected_nets(rep) | {c.net for c in r.failed}, zones)
        if undo:
            write_board(text, r.cu, out)
        rep2 = kicad_drc(args.kicad_cli, out, report_path, save=True)
        if undo and len(rep2.get("unconnected_items", [])) > len(rep.get("unconnected_items", [])):
            print("  dangling-end trim undone: it left more unconnected items")
            undo()
            write_board(text, r.cu, out)
            rep2 = kicad_drc(args.kicad_cli, out, report_path, save=True)
        elif undo:
            print(f"  {undo.count} dangling track ends cut back")
        rep = rep2
        print(f"final DRC: {len(rep.get('unconnected_items', []))} unconnected, "
              f"{sum(1 for v in rep.get('violations', []) if v.get('severity') == 'error')} errors "
              f"(report {report_path})")
    else:
        undo = trim_stubs(r.cu, {c.net for c in r.failed}, zones)
        if undo:
            print(f"  {undo.count} dangling track ends cut back")
            n = write_board(text, r.cu, out)
    print(f"routed {r.stats['routed']} connections ({r.stats['ripped']} rip-ups), "
          f"{r.stats['failed']} failed, {n} new items, {time.time() - t0:.0f} s -> {out}")
    for c in r.failed:
        print(f"  unrouted: {c.net} ({c.ta[0]:.2f}, {c.ta[1]:.2f}) - ({c.tb[0]:.2f}, {c.tb[1]:.2f})")
    return 0 if not r.failed else 1


if __name__ == "__main__":
    sys.exit(main())
