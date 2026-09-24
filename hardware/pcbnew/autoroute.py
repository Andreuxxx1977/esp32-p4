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
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import pcbnew  # noqa: E402  (KiCad)

from hardware.lib import board_spec as bs  # noqa: E402
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


def u1_stubs(board) -> list:
    """Short tracks from every connected U1 pad out to just inside the courtyard."""
    u1 = footprint(board, "U1")
    ox, oy = origin_of(board)
    reach = nm(lp.U1_STUB_END_MM)       # the planner keeps the escape channels open from here
    stubs = []
    for pad in u1.Pads():
        net = pad.GetNet()
        if not net or not net.GetNetname() or net.GetNetname() == bs.GND:
            continue
        p = pad.GetPosition()
        dx, dy = p.x - ox, p.y - oy
        if abs(dx) >= abs(dy):          # left / right rows run along y
            end = pcbnew.VECTOR2I(ox + (reach if dx > 0 else -reach), p.y)
        else:
            end = pcbnew.VECTOR2I(p.x, oy + (reach if dy > 0 else -reach))
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(p)
        t.SetEnd(end)
        t.SetWidth(nm(U1_STUB_WIDTH_MM))
        t.SetLayer(pcbnew.F_Cu)
        t.SetNet(net)
        t.SetLocked(True)
        stubs.append(t)
    return stubs


def routing_view(board, add_stubs: bool) -> list[str]:
    """Modify ``board`` in place into what the router should see. Returns a log."""
    log = []
    # Everything that walks the footprints runs before any zone is removed (removing
    # a zone invalidates the footprint iterator in some pcbnew/SWIG builds).
    ox, oy = origin_of(board)
    stubs = u1_stubs(board) if add_stubs else []
    for item in board.GetTracks():       # the planned thermal vias stay where they are
        item.SetLocked(True)
    for t in stubs:
        board.Add(t)
    if stubs:
        log.append(f"added {len(stubs)} U1 fan-out stubs")
    zones = list(board.Zones())
    by_name = {zone_name(z): z for z in zones}
    if PLANE_3V3 in by_name and VDD_HP_ISLAND in by_name:
        half = nm(lp.VDD_HP_ISLAND_HALF_MM + lp.ZONE_CLEARANCE_MM)
        by_name[PLANE_3V3].Outline().AddHole(square_chain(ox, oy, half), 0)
        log.append(f"cut the {VDD_HP_ISLAND} footprint out of {PLANE_3V3}")
    for zone in zones:
        name = zone_name(zone)
        if zone.GetIsRuleArea():
            if not (zone.GetDoNotAllowTracks() or zone.GetDoNotAllowVias()):
                board.Remove(zone)
                log.append(f"dropped marker rule area {name}")
        elif name in OUTER_POURS:
            board.Remove(zone)
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

def import_and_fill(board_path: Path, ses: Path, out: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    for suffix in (".kicad_pro", ".kicad_dru"):     # DRC + fill read the project rules
        src = board_path.with_suffix(suffix)
        if src.exists() and src.resolve() != out.with_suffix(suffix).resolve():
            shutil.copyfile(src, out.with_suffix(suffix))
    shutil.copyfile(board_path, out)
    board = pcbnew.LoadBoard(str(out))
    if not pcbnew.ImportSpecctraSES(board, str(ses)):
        raise SystemExit(f"importing {ses} failed")
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(str(out), board)


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
    ap.add_argument("--work", type=Path, help="keep DSN/SES/log here (default: temp dir)")
    ap.add_argument("--fr-option", action="append", default=[], metavar="--KEY=VALUE",
                    help="extra FreeRouting setting, e.g. --fr-option=--router.fanout.enabled=false")
    ap.add_argument("--ses", type=Path, help="import this session instead of running FreeRouting")
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
    view = pcbnew.LoadBoard(str(board_path))
    log = routing_view(view, args.u1_stubs)
    dsn, ses = work / "board.dsn", work / "board.ses"
    if not pcbnew.ExportSpecctraDSN(view, str(dsn)):
        raise SystemExit("DSN export failed")
    text, more = adjust_dsn(dsn.read_text())
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
    import_and_fill(board_path, ses, args.out.resolve())
    problems = verify_same_design(board_path, args.out.resolve())
    if problems:
        print("the routed board no longer matches the placed one:\n  " + "\n  ".join(problems[:50]))
        return 1
    print(f"routed board: {args.out} (placement, pad nets and zones unchanged)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
