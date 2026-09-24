#!/usr/bin/env python3
"""Autoroute the placed board with FreeRouting and write a routed copy.

The committed board (``hardware/output/esp32p4_extreme.kicad_pcb``) is placed but
not routed. This script produces a routed copy for prototype fabrication:

1. load the placed board and build a *routing view* of it for the DSN export:

   * rule areas that restrict neither tracks nor vias are dropped (KiCad's Specctra
     exporter turns every rule area into a full keep-out, which would block the
     whole 25 x 25 mm heatsink area around the SoC);
   * the outer-layer GND pours are dropped, so every GND pad gets an explicit
     via to the In1.Cu plane (the router would otherwise treat the unfilled pour
     polygon as solid copper and leave pads that the real fill can isolate);
   * the +3V3 plane gets a hole where the VDD_HP island sits (KiCad resolves the
     overlap by zone priority, the router needs it spelled out);
   * U1 fan-out stubs: locked 0.1 mm tracks from each SoC pad to just inside its
     courtyard, where the custom DRC rule ``u1_fanout_*`` allows the 0.35 mm pitch.
     At netclass clearance (0.2 mm for the MIPI/USB pairs) the router cannot reach
     a 0.2 mm pad at that pitch; it can reach the end of a 0.1 mm stub.

2. export Specctra DSN, adjust it for the router (power nets at the SoC routed at a
   width that fits a 0.2 mm pad, see ``SOC_POWER_WIDTH_UM``), run FreeRouting
   headless;
3. import the session into the *unmodified* placed board, fill every zone, save.

Nothing here checks the result: run ``kicad-cli pcb drc`` and
``python -m tools.check_drc_report --routed`` on the output (CI does).

Usage (a Python that can ``import pcbnew``; Java >= 25 for FreeRouting 2.4)::

    python3 hardware/pcbnew/autoroute.py --freerouting freerouting-2.4.1-executable.jar \
        [--board hardware/output/esp32p4_extreme.kicad_pcb] [--out routed/esp32p4_extreme.kicad_pcb]
        [--java java] [--passes 100] [--threads N] [--timeout 3600] [--work DIR]
    python3 hardware/pcbnew/autoroute.py --ses board.ses ...      # import an existing session
    python3 hardware/pcbnew/autoroute.py --verify routed.kicad_pcb  # placement/nets unchanged?
"""

from __future__ import annotations

import argparse
import math
import re
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import pcbnew  # noqa: E402  (KiCad)

from hardware.lib import board_spec as bs  # noqa: E402
from hardware.lib import esp32p4_pinout as p4  # noqa: E402
from hardware.pcbnew import layout_plan as lp  # noqa: E402

DEFAULT_BOARD = REPO / "hardware" / "output" / "esp32p4_extreme.kicad_pcb"
DEFAULT_OUT = REPO / "hardware" / "output" / "routed" / "esp32p4_extreme.kicad_pcb"
OUTER_POURS = ("GND_TOP", "GND_BOTTOM", "GND_HEAT_SPREADER")
PLANE_3V3, VDD_HP_ISLAND = "3V3_L3", "VDD_HP_ISLAND"

# Power nets that reach U1's 0.2 mm wide, 0.35 mm pitch pads. KiCad's netclass width
# is a default, not a DRC limit; at the SoC these rails carry < 0.6 A, which a 0.25 mm
# outer-layer track (1 oz) carries with ~10 degC rise (IPC-2221).
SOC_POWER_NETS = ("+3V3", "VDD_HP", "VDD_HP1_PAD")
SOC_POWER_WIDTH_UM = 250
SOC_POWER_VIA = "Via[0-3]_450:200_um"

U1_STUB_WIDTH_MM = 0.10
INNER_TRACK_MM = 0.15             # U1 pad -> inner via
INNER_VIA_PAD_GAP_MM = 0.10       # inner via copper to another U1 pad (also clears its mask opening)
INNER_VIA_SHIFTS_MM = (0.0, -0.10, 0.10, -0.20, 0.20)
MICROVIA_MM = (0.30, 0.10)        # laser microvia pad / drill (annular ring 0.10 mm)
PLANE_VIA_CLEARANCE_MM = 0.20     # the largest netclass clearance



def nm(mm: float) -> int:
    return int(round(mm * 1_000_000))


# ==========================================================================
# 1. Routing view of the board
# ==========================================================================

def zone_name(zone) -> str:
    return zone.GetZoneName() if hasattr(zone, "GetZoneName") else ""


def footprint(board, ref: str):
    # (FindFootprintByReference returns an unwrapped pointer in some SWIG builds)
    return next(f for f in board.GetFootprints() if f.GetReference() == ref)


def origin_of(board) -> tuple[int, int]:
    """Board coordinates (nm) of the design origin = U1's position."""
    p = footprint(board, "U1").GetPosition()
    return p.x, p.y


def square_chain(cx: int, cy: int, half: int):
    chain = pcbnew.SHAPE_LINE_CHAIN()
    for x, y in ((-half, -half), (half, -half), (half, half), (-half, half)):
        chain.Append(cx + x, cy + y)
    chain.SetClosed(True)
    return chain


@dataclass(frozen=True)
class PreTrack:
    start: tuple[int, int]
    end: tuple[int, int]
    width: int
    layer: int
    net: str


@dataclass(frozen=True)
class PreVia:
    pos: tuple[int, int]
    diameter: int
    drill: int
    top: int
    bottom: int
    net: str
    micro: bool = False


def u1_stubs(board) -> list[PreTrack]:
    """Tracks from every connected U1 pad out to U1_STUB_END_MM (inside the courtyard)."""
    u1 = footprint(board, "U1")
    ox, oy = origin_of(board)
    reach = nm(lp.U1_STUB_END_MM)       # the planner keeps the escape channels open from here
    stubs = []
    for pad in u1.Pads():
        net = pad.GetNetname()
        if not net or net == bs.GND:
            continue
        p = pad.GetPosition()
        dx, dy = p.x - ox, p.y - oy
        if abs(dx) >= abs(dy):          # left / right rows run along y
            end = (ox + (reach if dx > 0 else -reach), p.y)
        else:
            end = (p.x, oy + (reach if dy > 0 else -reach))
        stubs.append(PreTrack((p.x, p.y), end, nm(U1_STUB_WIDTH_MM), pcbnew.F_Cu, net))
    return stubs


def escape_tracks(board, path: Path | None = None) -> list[PreTrack]:
    """The planner's deterministic SoC escape tracks (layout_plan escape_tracks.json),
    from each U1 stub end through the 0402 ring, as fixed copper for the router."""
    ox, oy = origin_of(board)
    out = []
    for tr in lp.load_escape_tracks(path or lp.ESCAPE_FILE):
        pts = [(ox + nm(x), oy + nm(y)) for x, y in tr.points]
        out += [PreTrack(a, b, nm(tr.width), pcbnew.F_Cu, tr.net) for a, b in zip(pts, pts[1:]) if a != b]
    return out


def board_vias(board) -> list[PreVia]:
    """The vias the placed board carries: the EPAD thermal-via array of the layout plan
    (from the spec, so this also works on an already routed board)."""
    ox, oy = origin_of(board)
    tv = bs.THERMAL_VIA
    n, pitch = tv["grid"], tv["pitch_mm"]
    return [PreVia((ox + nm(round((i - (n - 1) / 2) * pitch, 4)), oy + nm(round((j - (n - 1) / 2) * pitch, 4))),
                   nm(tv["pad_mm"]), nm(tv["drill_mm"]), pcbnew.F_Cu, pcbnew.B_Cu, bs.GND)
            for i in range(n) for j in range(n)]


def _rect_dist(r, x: int, y: int) -> float:
    dx = max(r[0] - x, 0, x - r[2])
    dy = max(r[1] - y, 0, y - r[3])
    return math.hypot(dx, dy)


def _seg_dist(a, b, x: int, y: int) -> float:
    (ax, ay), (bx, by) = a, b
    vx, vy = bx - ax, by - ay
    L = vx * vx + vy * vy
    u = 0.0 if L == 0 else max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L))
    return math.hypot(ax + u * vx - x, ay + u * vy - y)


def plane_fanout(board, stubs: list[PreTrack], existing: list[PreVia]) -> tuple[list[PreVia], list[str]]:
    """Filled-and-capped vias in the pads of plane nets, so the router does not have to
    find room for them: top GND pads get an L1-L2 laser microvia, top +3V3 pads a
    through via, bottom +3V3 (VDD_HP inside its island) pads an L4-L3 microvia. A via
    is only placed where it clears every other-net pad on the layers it reaches, the
    U1 stubs, other holes and the board edge; the rest is left to the router."""
    ox, oy = origin_of(board)
    island = nm(lp.VDD_HP_ISLAND_HALF_MM + lp.ZONE_CLEARANCE_MM)
    clr, h2h, edge = nm(PLANE_VIA_CLEARANCE_MM), nm(lp.DESIGN_RULES["hole_to_hole"]), \
        nm(lp.DESIGN_RULES["copper_edge"])
    bb = board.GetBoardEdgesBoundingBox()
    outline = (bb.GetLeft(), bb.GetTop(), bb.GetRight(), bb.GetBottom())
    # Plain-data snapshot of every pad first (holding pcbnew proxies across many calls
    # upsets some SWIG builds): (ref, number, net, x, y, sx, sy, rect, on F, on B, drill, SMD).
    info = []
    for fp in board.GetFootprints():
        ref, skip = fp.GetReference(), "ThermalVias" in str(fp.GetFPIDAsString())
        for pad in fp.Pads():
            b = pad.GetBoundingBox()
            p = pad.GetPosition()
            size = pad.GetSize(pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu)
            info.append((ref, str(pad.GetNumber()), str(pad.GetNetname()), p.x, p.y, size.x, size.y,
                         (b.GetLeft(), b.GetTop(), b.GetRight(), b.GetBottom()),
                         pad.IsOnLayer(pcbnew.F_Cu), pad.IsOnLayer(pcbnew.B_Cu),
                         pad.GetDrillSize().x, pad.GetAttribute() == pcbnew.PAD_ATTRIB_SMD, skip))
    pads = {pcbnew.F_Cu: [(i[7], i[2], i[:2]) for i in info if i[8]],
            pcbnew.B_Cu: [(i[7], i[2], i[:2]) for i in info if i[9]]}
    holes = [(v.pos[0], v.pos[1], v.drill) for v in existing] + \
        [(i[3], i[4], i[10]) for i in info if i[10] > 0]

    def clear(x, y, d, drill, layers, net, own) -> bool:
        r = d / 2
        if not (outline[0] + edge + r <= x <= outline[2] - edge - r
                and outline[1] + edge + r <= y <= outline[3] - edge - r):
            return False
        for layer in layers:
            for rect, n, key in pads[layer]:
                if key != own and n != net and _rect_dist(rect, x, y) < r + clr:
                    return False
        if pcbnew.F_Cu in layers and any(_seg_dist(s.start, s.end, x, y) < r + clr + s.width / 2
                                         for s in stubs if s.net != net):
            return False
        return all(math.hypot(hx - x, hy - y) >= (hd + drill) / 2 + h2h for hx, hy, hd in holes)

    vias, skipped = [], 0
    for ref, number, net, x, y, sx, sy, _, top, _, _, smd, skip in info:
        if ref == "U1" or skip or not smd or net not in (bs.GND, "+3V3", "VDD_HP") \
                or min(sx, sy) < nm(0.45):
            continue
        inside = abs(x - ox) < island and abs(y - oy) < island
        if top and net == bs.GND:
            kind = ("micro", pcbnew.F_Cu, pcbnew.In1_Cu, (pcbnew.F_Cu,))
        elif top and net == "+3V3" and not inside:
            kind = ("through", pcbnew.F_Cu, pcbnew.B_Cu, (pcbnew.F_Cu, pcbnew.B_Cu))
        elif not top and ((net == "+3V3" and not inside) or
                          (net == "VDD_HP" and abs(x - ox) < island - nm(0.4)
                           and abs(y - oy) < island - nm(0.4))):
            kind = ("micro", pcbnew.B_Cu, pcbnew.In2_Cu, (pcbnew.B_Cu,))
        else:
            continue
        micro = kind[0] == "micro"
        d, drill = (nm(MICROVIA_MM[0]), nm(MICROVIA_MM[1])) if micro else \
            (nm(lp.DESIGN_RULES["via_diameter"]), nm(lp.DESIGN_RULES["via_drill"]))
        if not clear(x, y, d, drill, kind[3], net, (ref, number)):
            skipped += 1
            continue
        vias.append(PreVia((x, y), d, drill, kind[1], kind[2], net, micro))
        holes.append((x, y, drill))
    n_micro = sum(v.micro for v in vias)
    return vias, [f"plane fan-out: {len(vias)} vias in pad ({n_micro} laser microvias, "
                  f"{len(vias) - n_micro} through), {skipped} pads left to the router"]


def inner_vias(board, existing: list[PreVia]) -> tuple[list[PreTrack], list[PreVia], list[str]]:
    """Vias *inside* U1's pad ring (between the EPAD and the pad row) for the pads whose
    decoupling sits on the bottom side under them, and for the VDD_HP pads (the via lands
    in the VDD_HP island on In2.Cu). A short inward track joins pad and via. Neighbouring
    pads are 0.35 mm apart, so at most every other pad gets one; VDD_HP pads first."""
    ox, oy = origin_of(board)
    u1_nets = next(c for c in bs.COMPONENTS if c.ref == "U1").conns
    want = [p for p, n in sorted(u1_nets.items(), key=lambda kv: int(kv[0])) if n == "VDD_HP"]
    want += sorted({c.place.pad for c in bs.COMPONENTS if isinstance(c.place, bs.Near)
                    and c.place.ref == "U1" and c.place.side == "B"} - set(want), key=int)
    r_via = (p4.EPAD_SIZE_MM / 2 + p4.ROW_OFFSET_MM - p4.PAD_SIZE_MM[1] / 2) / 2   # mid-band
    d, drill = nm(lp.DESIGN_RULES["via_diameter"]), nm(lp.DESIGN_RULES["via_drill"])
    spacing = d + nm(lp.DESIGN_RULES["min_clearance"]) + nm(0.02)
    bottom = []
    for fp in board.GetFootprints():
        if fp.GetLayer() != pcbnew.B_Cu:
            continue
        for pad in fp.Pads():
            b = pad.GetBoundingBox()
            bottom.append(((b.GetLeft(), b.GetTop(), b.GetRight(), b.GetBottom()), str(pad.GetNetname())))
    # U1's own pads as rectangles (design mm -> board nm), for copper / hole / mask clearance
    w, l = p4.PAD_SIZE_MM
    u1_pads = []
    for n in range(1, 105):
        x, y = p4.pad_position(n)
        hx, hy = (l / 2, w / 2) if p4.pad_side(n) in ("left", "right") else (w / 2, l / 2)
        u1_pads.append(((ox + nm(x - hx), oy + nm(y - hy), ox + nm(x + hx), oy + nm(y + hy)),
                        u1_nets.get(str(n), "")))
    holes = [(v.pos[0], v.pos[1], v.drill) for v in existing]
    tracks, vias, skipped = [], [], []
    clr = nm(PLANE_VIA_CLEARANCE_MM)
    to_pad = max(d / 2 + nm(INNER_VIA_PAD_GAP_MM), drill / 2 + nm(lp.DESIGN_RULES["hole_clearance"]))

    def fits(vx: int, vy: int, net: str) -> bool:
        return (all(math.hypot(v.pos[0] - vx, v.pos[1] - vy) >= spacing for v in vias)
                and all(math.hypot(hx - vx, hy - vy) >= (hd + drill) / 2 + nm(lp.DESIGN_RULES["hole_to_hole"])
                        for hx, hy, hd in holes)
                and not any(n != net and _rect_dist(r, vx, vy) < to_pad for r, n in u1_pads)
                and not any(n != net and _rect_dist(r, vx, vy) < d / 2 + clr for r, n in bottom))

    for pad in want:
        x, y = p4.pad_position(int(pad))
        along_y = p4.pad_side(int(pad)) in ("left", "right")
        net = u1_nets[pad]
        for shift in INNER_VIA_SHIFTS_MM:             # slide along the row if a corner is in the way
            vx, vy = (math.copysign(r_via, x), y + shift) if along_y else (x + shift, math.copysign(r_via, y))
            vx, vy = ox + nm(vx), oy + nm(vy)
            if fits(vx, vy, net):
                break
        else:
            skipped.append(pad)
            continue
        px, py = ox + nm(x), oy + nm(y)
        # the pad's inner end first, then (if slid) a short dog-leg to the via
        ix, iy = ((ox + nm(math.copysign(p4.ROW_OFFSET_MM - l / 2, x)), py) if along_y
                  else (px, oy + nm(math.copysign(p4.ROW_OFFSET_MM - l / 2, y))))
        vias.append(PreVia((vx, vy), d, drill, pcbnew.F_Cu, pcbnew.B_Cu, net))
        tracks.append(PreTrack((px, py), (ix, iy), nm(INNER_TRACK_MM), pcbnew.F_Cu, net))
        tracks.append(PreTrack((ix, iy), (vx, vy), nm(INNER_TRACK_MM), pcbnew.F_Cu, net))
    return tracks, vias, [f"U1 inner vias: {len(vias)} ({', '.join(sorted({v.net for v in vias}))});"
                          f" not possible for pads {', '.join(skipped) or '-'}"]


def add_items(board, tracks: list[PreTrack], vias: list[PreVia]) -> None:
    nets = {n: board.FindNet(n) for n in {x.net for x in (*tracks, *vias)}}
    for s in tracks:
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(*s.start))
        t.SetEnd(pcbnew.VECTOR2I(*s.end))
        t.SetWidth(s.width)
        t.SetLayer(s.layer)
        t.SetNet(nets[s.net])
        t.SetLocked(True)
        board.Add(t)
    for v in vias:
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(pcbnew.VECTOR2I(*v.pos))
        via.SetViaType(pcbnew.VIATYPE_MICROVIA if v.micro else pcbnew.VIATYPE_THROUGH)
        via.SetLayerPair(v.top, v.bottom)
        via.SetWidth(v.diameter)
        via.SetDrill(v.drill)
        via.SetNet(nets[v.net])
        via.SetLocked(True)
        board.Add(via)


def _is_prerouted(item, tracks: list[PreTrack], vias: list[PreVia]) -> bool:
    if item.GetClass() == "PCB_VIA":
        p = item.GetPosition()
        return any(_near((p.x, p.y), v.pos) for v in vias)
    s, e = item.GetStart(), item.GetEnd()
    return any(item.GetLayer() == pt.layer and ((_near((s.x, s.y), pt.start) and _near((e.x, e.y), pt.end))
                                               or (_near((s.x, s.y), pt.end) and _near((e.x, e.y), pt.start)))
               for pt in tracks)


def routing_view(board, zones: list, tracks: list[PreTrack], vias: list[PreVia],
                 resume: bool = False) -> list[str]:
    """Modify ``board`` in place into what the router should see. Returns a log.
    ``zones`` is ``list(board.Zones())`` taken right after loading: some pcbnew/SWIG
    builds crash walking the zones (or footprints) once items were added or removed.
    ``resume``: ``board`` is an earlier routed result; its pre-routing is locked, the
    rest of its copper stays as ordinary (rip-up-able) routing for more passes."""
    log = []
    ox, oy = origin_of(board)
    if resume:
        kept = 0
        for item in board.GetTracks():
            pre = _is_prerouted(item, tracks, vias)
            item.SetLocked(pre)
            kept += not pre
        log.append(f"resume: {kept} routed tracks/vias kept as rip-up-able routing")
    else:
        for item in list(board.GetTracks()):     # replaced by the pre-routing (same vias, locked)
            board.Delete(item)
        add_items(board, tracks, vias)
    # The bottom solder-mask window over the thermal-via field (exposed copper for a thermal
    # pad): no tracks or new vias of other nets in it.
    tv = bs.THERMAL_VIA                       # same square as layout_plan's B.Mask window
    half = nm((tv["grid"] - 1) / 2 * tv["pitch_mm"] + tv["pad_mm"] / 2 + PLANE_VIA_CLEARANCE_MM)
    keep = pcbnew.ZONE(board)
    keep.SetIsRuleArea(True)
    keep.SetLayer(pcbnew.B_Cu)
    keep.SetDoNotAllowTracks(True)
    keep.SetDoNotAllowVias(True)
    keep.Outline().AddOutline(square_chain(ox, oy, half))
    board.Add(keep)
    log.append("bottom thermal window: track/via keep-out for the router")
    by_name = {zone_name(z): z for z in zones}
    if PLANE_3V3 in by_name and VDD_HP_ISLAND in by_name:
        half = nm(lp.VDD_HP_ISLAND_HALF_MM + lp.ZONE_CLEARANCE_MM)
        by_name[PLANE_3V3].Outline().AddHole(square_chain(ox, oy, half), 0)
        log.append(f"cut the {VDD_HP_ISLAND} footprint out of {PLANE_3V3}")
    for zone in zones:
        name = zone_name(zone)
        if zone.GetIsRuleArea():
            if not (zone.GetDoNotAllowTracks() or zone.GetDoNotAllowVias()):
                board.Delete(zone)
                log.append(f"dropped marker rule area {name}")
        elif name in OUTER_POURS:
            board.Delete(zone)
            log.append(f"dropped outer pour {name}")
    return log


# ==========================================================================
# 2. DSN adjustments and FreeRouting
# ==========================================================================

_CLASS = re.compile(r"\(class (\S+)((?:\s+[^\s()]+)+)\s*\(circuit", re.S)


def adjust_dsn(text: str) -> tuple[str, list[str]]:
    """Move the SoC power nets into their own, narrower routing class."""
    log = []
    m = next((m for m in _CLASS.finditer(text) if m.group(1) == "POWER"), None)
    if not m:
        return text, log
    nets = m.group(2).split()
    moved = [n for n in nets if n.strip('"') in SOC_POWER_NETS]
    if not moved:
        return text, log
    kept = [n for n in nets if n not in moved]
    head = "(class POWER " + " ".join(kept) + "\n      (circuit"
    text = text[:m.start()] + head + text[m.end():]
    new = (f"    (class POWER_SOC {' '.join(moved)}\n"
           f"      (circuit\n        (use_via \"{SOC_POWER_VIA}\")\n      )\n"
           f"      (rule\n        (width {SOC_POWER_WIDTH_UM})\n        (clearance 150)\n      )\n    )\n")
    i = text.index("    (class POWER ")
    text = text[:i] + new + text[i:]
    log.append(f"routing class POWER_SOC ({SOC_POWER_WIDTH_UM} um): {', '.join(moved)}")
    return text, log


# HDI 1+2+1 laser microvias (0.25 / 0.10 mm, board rules in the .kicad_pro): L1-L2 lands
# on the GND plane, L4-L3 on the +3V3 plane / VDD_HP island. KiCad's DSN export only lists
# the net classes' through vias, so they are added here for the nets that use planes.
MICROVIAS = {"Via[0-1]_300:100_um": ("Signal_Top", "GND"),
             "Via[2-3]_300:100_um": ("VCC_3V3", "Signal_Bottom")}
MICROVIA_CLASSES = ("kicad_default", "POWER_SOC", "POWER")


def add_microvias(text: str) -> tuple[str, list[str]]:
    size = round(MICROVIA_MM[0] * 1000)
    stacks = "".join(
        f"    (padstack \"{name}\"\n"
        + "".join(f"      (shape (circle {layer} {size}))\n" for layer in layers)
        + "      (attach off)\n    )\n" for name, layers in MICROVIAS.items()
        if f'(padstack "{name}"' not in text)          # pre-placed microvias define it already
    i = text.index("    (padstack \"Via[")
    text = text[:i] + stacks + text[i:]
    def extend(m):
        have = m.group(1)
        return have + "".join(f' "{n}"' for n in MICROVIAS if f'"{n}"' not in have) + ")"
    text = re.sub(r"(\(structure.*?\(via [^)]*)\)", extend, text, count=1, flags=re.S)
    done = []
    for cls in MICROVIA_CLASSES:
        pat = re.compile(r"(\(class " + re.escape(cls) + r" .*?\(circuit\s*)(\(use_via [^)]*\))",
                         re.S)
        text, n = pat.subn(lambda m: m.group(1) + m.group(2) + "".join(
            f'\n        (use_via "{v}")' for v in MICROVIAS), text, count=1)
        if n:
            done.append(cls)
    return text, [f"HDI microvias L1-L2 / L3-L4 allowed for classes {', '.join(done)}"]


def run_freerouting(java: str, jar: Path, dsn: Path, ses: Path, passes: int,
                    threads: int | None, timeout: int, logfile: Path,
                    extra: list[str] = ()) -> int:
    cmd = [java, "-Djava.awt.headless=true", "-jar", str(jar),
           "--gui.enabled=false", "--usage_and_diagnostic_data.disable_analytics=true",
           "-de", str(dsn), "-do", str(ses), "-mp", str(passes)]
    if threads:
        cmd += ["-mt", str(threads)]
    cmd += list(extra)
    t0 = time.time()
    with logfile.open("w") as fh:
        try:
            rc = subprocess.run(cmd, stdout=fh, stderr=subprocess.STDOUT, timeout=timeout).returncode
        except subprocess.TimeoutExpired:
            rc = -1
    print(f"FreeRouting exited {rc} after {time.time() - t0:.0f} s (log: {logfile})")
    return rc


# ==========================================================================
# 3. Import, fill, save
# ==========================================================================

# Via drill by pad diameter: KiCad's session import takes the drill from the net class,
# which is wrong for the SoC power nets (routed with the 0.45 mm via, POWER says 0.3 mm).
VIA_DRILL_BY_DIAMETER_MM = {0.45: 0.20, 0.60: 0.30, MICROVIA_MM[0]: MICROVIA_MM[1]}


def _near(a, b, tol: int = 2000) -> bool:
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol


def import_and_fill(board_path: Path, ses: Path, out: Path, tracks: list[PreTrack],
                    vias: list[PreVia]) -> list[str]:
    """Import the session into a copy of the placed board, put the pre-routing back
    exactly as planned (the session drops or rewrites some fixed items), fix via
    drills / microvia layers, fill the zones and save."""
    out.parent.mkdir(parents=True, exist_ok=True)
    for suffix in (".kicad_pro", ".kicad_dru"):     # DRC + fill read the project rules
        src = board_path.with_suffix(suffix)
        if src.exists() and src.resolve() != out.with_suffix(suffix).resolve():
            shutil.copyfile(src, out.with_suffix(suffix))
    shutil.copyfile(board_path, out)
    board = pcbnew.LoadBoard(str(out))
    if not pcbnew.ImportSpecctraSES(board, str(ses)):
        raise SystemExit(f"importing {ses} failed")
    fixed, dropped = 0, 0
    drills = {nm(k): nm(v) for k, v in VIA_DRILL_BY_DIAMETER_MM.items()}
    for item in list(board.GetTracks()):
        if item.GetClass() == "PCB_VIA":
            p = item.GetPosition()
            if any(_near((p.x, p.y), v.pos) for v in vias):
                board.Delete(item)
                dropped += 1
                continue
            d = item.GetWidth(pcbnew.F_Cu)
            want = drills.get(min(drills, key=lambda k: abs(k - d)))
            if item.GetDrillValue() != want:
                item.SetDrill(want)
                fixed += 1
            if d == nm(MICROVIA_MM[0]):                 # a microvia the router placed
                item.SetViaType(pcbnew.VIATYPE_MICROVIA)
                if item.GetNetname() == bs.GND:
                    item.SetLayerPair(pcbnew.F_Cu, pcbnew.In1_Cu)
                else:
                    item.SetLayerPair(pcbnew.B_Cu, pcbnew.In2_Cu)
        else:
            s, e = item.GetStart(), item.GetEnd()
            if any(item.GetLayer() == pt.layer and ((_near((s.x, s.y), pt.start) and _near((e.x, e.y), pt.end))
                                                   or (_near((s.x, s.y), pt.end) and _near((e.x, e.y), pt.start)))
                   for pt in tracks):
                board.Delete(item)
                dropped += 1
    add_items(board, tracks, vias)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(str(out), board)
    return [f"import: pre-routing restored ({len(tracks)} tracks, {len(vias)} vias; "
            f"{dropped} session copies replaced), {fixed} via drills corrected"]


# ==========================================================================
# 4. The routed board must be the placed board plus copper
# ==========================================================================

def design_signature(board) -> dict[str, tuple]:
    """Per footprint: position, rotation, side and the net of every pad; per zone:
    net and layers. Tracks and vias (the routing) and zone fills are left out."""
    sig = {}
    for fp in board.GetFootprints():
        p = fp.GetPosition()
        pads = tuple(sorted((pad.GetNumber(), pad.GetNetname()) for pad in fp.Pads()))
        sig[fp.GetReference()] = (p.x, p.y, round(fp.GetOrientationDegrees(), 3),
                                  fp.GetLayer(), fp.GetFPID().GetUniStringLibId(), pads)
    for z in board.Zones():
        sig[f"zone:{zone_name(z)}:{z.GetNetname()}"] = (tuple(z.GetLayerSet().CuStack()),
                                                        z.GetIsRuleArea())
    return sig


def verify_same_design(placed_path: Path, routed_path: Path) -> list[str]:
    placed = design_signature(pcbnew.LoadBoard(str(placed_path)))
    routed = design_signature(pcbnew.LoadBoard(str(routed_path)))
    problems = [f"{k}: only in the {'placed' if k in placed else 'routed'} board"
                for k in sorted(set(placed) ^ set(routed))]
    problems += [f"{k}: placed {placed[k][:5]} vs routed {routed[k][:5]}"
                 if placed[k][:5] != routed[k][:5] else f"{k}: pad nets differ"
                 for k in sorted(set(placed) & set(routed)) if placed[k] != routed[k]]
    return problems


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--freerouting", type=Path, help="FreeRouting executable jar")
    ap.add_argument("--java", default="java")
    ap.add_argument("--passes", type=int, default=100)
    ap.add_argument("--threads", type=int)
    ap.add_argument("--timeout", type=int, default=3600, help="seconds for FreeRouting")
    ap.add_argument("--no-u1-stubs", dest="u1_stubs", action="store_false")
    ap.add_argument("--no-escape-tracks", dest="escape_tracks", action="store_false",
                    help="do not pre-route the planner's SoC escape tracks")
    ap.add_argument("--microvias", action="store_true",
                    help="let the router use the HDI laser microvias (L1-L2, L3-L4)")
    ap.add_argument("--no-inner-vias", dest="inner_vias", action="store_false",
                    help="no vias inside U1's pad ring for bottom-side decoupling / VDD_HP")
    ap.add_argument("--no-plane-vias", dest="plane_vias", action="store_false",
                    help="do not pre-place vias in the pads of plane nets")
    ap.add_argument("--work", type=Path, help="keep DSN/SES/log here (default: temp dir)")
    ap.add_argument("--fr-option", action="append", default=[], metavar="--KEY=VALUE",
                    help="extra FreeRouting setting, e.g. --fr-option=--router.fanout.enabled=false")
    ap.add_argument("--ses", type=Path, help="import this session instead of running FreeRouting")
    ap.add_argument("--resume", type=Path, metavar="ROUTED",
                    help="start from an earlier routed board (more passes on its routing)")
    ap.add_argument("--verify", type=Path, metavar="ROUTED",
                    help="only check that ROUTED has the placement, pad nets and zones of --board")
    args = ap.parse_args(argv)

    if args.verify:
        problems = verify_same_design(args.board.resolve(), args.verify.resolve())
        for p in problems[:100]:
            print("  - " + p)
        print(f"{'FAIL' if problems else 'PASS'}: {args.verify} "
              f"{'differs from' if problems else 'is'} {args.board} plus routing "
              f"({len(problems)} difference(s))")
        return 1 if problems else 0
    if not (args.freerouting or args.ses):
        ap.error("--freerouting JAR (or --ses SESSION) is required to route")

    work = args.work or Path(tempfile.mkdtemp(prefix="autoroute-"))
    work.mkdir(parents=True, exist_ok=True)
    board_path = args.board.resolve()
    view = pcbnew.LoadBoard(str(args.resume.resolve() if args.resume else board_path))
    zones = list(view.Zones())
    tracks = u1_stubs(view) if args.u1_stubs else []
    vias = board_vias(view)
    log = [f"pre-routing: {len(tracks)} U1 fan-out stubs, {len(vias)} planned vias kept"]
    if args.escape_tracks and args.u1_stubs:
        esc = escape_tracks(view)
        tracks += esc
        log.append(f"SoC escape tracks: {len(esc)} segments ({lp.ESCAPE_FILE.name})")
    if args.inner_vias:
        more_tracks, more_vias, note = inner_vias(view, vias)
        tracks += more_tracks
        vias += more_vias
        log += note
    if args.plane_vias:
        more_vias, note = plane_fanout(view, tracks, vias)
        vias += more_vias
        log += note
    log += routing_view(view, zones, tracks, vias, resume=bool(args.resume))
    dsn, ses = work / "board.dsn", work / "board.ses"
    if not pcbnew.ExportSpecctraDSN(view, str(dsn)):
        raise SystemExit("DSN export failed")
    text, more = adjust_dsn(dsn.read_text())
    if args.microvias:
        text, mv = add_microvias(text)
        more += mv
    dsn.write_text(text)
    for line in log + more:
        print("  " + line)
    if args.ses:
        ses = args.ses.resolve()
    else:
        rc = run_freerouting(args.java, args.freerouting, dsn, ses, args.passes, args.threads,
                             args.timeout, work / "freerouting.log", args.fr_option)
        if not ses.exists():
            print(f"no session written (FreeRouting rc={rc})")
            return 1
    for line in import_and_fill(board_path, ses, args.out.resolve(), tracks, vias):
        print("  " + line)
    problems = verify_same_design(board_path, args.out.resolve())
    if problems:
        print("the routed board no longer matches the placed one:\n  " + "\n  ".join(problems[:50]))
        return 1
    print(f"routed board: {args.out} (placement, pad nets and zones unchanged)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
