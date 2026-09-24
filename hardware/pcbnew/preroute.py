#!/usr/bin/env python3
"""Pre-route the placed board (the SoC fan-out) and check routed boards against it.

The committed board (``hardware/output/esp32p4_extreme.kicad_pcb``) is placed but not
routed. The SoC's 0.35 mm-pitch fan-out is laid down here, deterministically, before the
board router (``python -m tools.pcb_router``) routes everything else:

* U1 fan-out stubs: 0.1 mm tracks from each SoC pad to just inside its courtyard, where
  the custom DRC rule ``u1_fanout_*`` allows the 0.35 mm pitch;
* the SoC escape tracks solved by the layout planner (``layout_plan escapes``) from the
  stub ends through the 0402 decoupling ring;
* vias inside U1's pad ring for the bottom-side decoupling and the VDD_HP island, the
  EPAD thermal-via array, and filled-and-capped vias in the pads of plane nets.

``--verify ROUTED`` checks that a routed board is this placed board plus copper (same
footprints, positions, pad nets and zones); CI runs it on the committed routed board.

Usage (a Python that can ``import pcbnew``)::

    python3 hardware/pcbnew/preroute.py [--board PLACED] --out PREROUTED.kicad_pcb
    python -m tools.pcb_router PREROUTED.kicad_pcb -o ROUTED.kicad_pcb --kicad-cli kicad-cli
    python3 hardware/pcbnew/preroute.py --verify ROUTED.kicad_pcb
"""

from __future__ import annotations

import argparse
import math
import shutil
import sys
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
DEFAULT_OUT = REPO / "build" / "prerouted" / "esp32p4_extreme.kicad_pcb"

U1_STUB_WIDTH_MM = 0.10
INNER_TRACK_MM = 0.15             # U1 pad -> inner via
INNER_VIA_PAD_GAP_MM = 0.10       # inner via copper to another U1 pad (also clears its mask opening)
INNER_VIA_SHIFTS_MM = (0.0, -0.10, 0.10, -0.20, 0.20)
MICROVIA_MM = (0.30, 0.10)        # laser microvia pad / drill (annular ring 0.10 mm)
PLANE_VIA_CLEARANCE_MM = 0.20     # the largest netclass clearance


def nm(mm: float) -> int:
    return int(round(mm * 1_000_000))


# ==========================================================================
# 1. Pre-routing: the SoC fan-out
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


ESCAPE_SIMPLIFY_MM = 0.002        # well inside the solver's 5 um margin


def simplify(points: list, tol: float) -> list:
    """Douglas-Peucker: drop vertices closer than ``tol`` to the line of their neighbours
    (the solver samples every 25 um of radius, which leaves many tiny collinear steps)."""
    if len(points) < 3:
        return list(points)
    (ax, ay), (bx, by) = points[0], points[-1]
    L = math.hypot(bx - ax, by - ay) or 1e-12
    dmax, idx = -1.0, 0
    for k, (x, y) in enumerate(points[1:-1], 1):
        d = abs((bx - ax) * (ay - y) - (ax - x) * (by - ay)) / L
        if d > dmax:
            dmax, idx = d, k
    if dmax <= tol:
        return [points[0], points[-1]]
    return simplify(points[:idx + 1], tol)[:-1] + simplify(points[idx:], tol)


def escape_tracks(board, path: Path | None = None) -> list[PreTrack]:
    """The planner's deterministic SoC escape tracks (layout_plan escape_tracks.json),
    from each U1 stub end through the 0402 ring, as fixed copper for the router."""
    ox, oy = origin_of(board)
    out = []
    for tr in lp.load_escape_tracks(path or lp.ESCAPE_FILE):
        pts = [(ox + nm(x), oy + nm(y)) for x, y in simplify(tr.points, ESCAPE_SIMPLIFY_MM)]
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


def write_prerouted(board_path: Path, out: Path, tracks: list[PreTrack], vias: list[PreVia]) -> None:
    """The placed board with only the pre-routing added (existing copper replaced)."""
    out.parent.mkdir(parents=True, exist_ok=True)
    for suffix in (".kicad_pro", ".kicad_dru"):
        src = board_path.with_suffix(suffix)
        if src.exists() and src.resolve() != out.with_suffix(suffix).resolve():
            shutil.copyfile(src, out.with_suffix(suffix))
    shutil.copyfile(board_path, out)
    board = pcbnew.LoadBoard(str(out))
    zones = list(board.Zones())
    for item in list(board.GetTracks()):
        board.Delete(item)
    add_items(board, tracks, vias)
    pcbnew.ZONE_FILLER(board).Fill(zones)
    pcbnew.SaveBoard(str(out), board)


# ==========================================================================
# 2. The routed board must be the placed board plus copper
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
    ap.add_argument("--no-u1-stubs", dest="u1_stubs", action="store_false")
    ap.add_argument("--no-escape-tracks", dest="escape_tracks", action="store_false",
                    help="do not pre-route the planner's SoC escape tracks")
    ap.add_argument("--no-inner-vias", dest="inner_vias", action="store_false",
                    help="no vias inside U1's pad ring for bottom-side decoupling / VDD_HP")
    ap.add_argument("--no-plane-vias", dest="plane_vias", action="store_false",
                    help="do not pre-place vias in the pads of plane nets")
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

    board_path = args.board.resolve()
    board = pcbnew.LoadBoard(str(board_path))
    tracks = u1_stubs(board) if args.u1_stubs else []
    vias = board_vias(board)
    log = [f"pre-routing: {len(tracks)} U1 fan-out stubs, {len(vias)} planned vias kept"]
    if args.escape_tracks and args.u1_stubs:
        esc = escape_tracks(board)
        tracks += esc
        log.append(f"SoC escape tracks: {len(esc)} segments ({lp.ESCAPE_FILE.name})")
    if args.inner_vias:
        more_tracks, more_vias, note = inner_vias(board, vias)
        tracks += more_tracks
        vias += more_vias
        log += note
    if args.plane_vias:
        more_vias, note = plane_fanout(board, tracks, vias)
        vias += more_vias
        log += note
    for line in log:
        print("  " + line)
    write_prerouted(board_path, args.out.resolve(), tracks, vias)
    print(f"pre-routed board: {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
