"""Export the fabrication package (Gerbers, drills, IPC-356, pick-and-place, BOM).

Runs ``kicad-cli`` on a routed board and writes everything PCBWay (or any fab)
needs into one directory plus a zip of it. All files use the drill/place origin,
which this design puts at the SoC centre, so coordinates match the docs (U1 = 0,0).

``--check`` regenerates into a temporary directory and compares with the committed
package, ignoring only the lines that carry a date or the KiCad build string, so CI
proves the committed Gerbers are exactly what the committed routed board produces.

Usage::

    python -m tools.export_fab [BOARD] [--out DIR] [--kicad-cli kicad-cli] [--drc DRC.json] [--check]
"""

from __future__ import annotations

import argparse
import difflib
import json
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DEFAULT_BOARD = REPO / "hardware" / "output" / "routed" / "esp32p4_extreme.kicad_pcb"
DEFAULT_OUT = REPO / "hardware" / "output" / "fab"
BOM = REPO / "hardware" / "output" / "bom_pcbway.csv"
ZIP_NAME = "esp32p4_extreme_gerbers.zip"
LAYERS = ("F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,"
          "F.Mask,B.Mask,Edge.Cuts")
# Lines that differ between two exports of the same board (time stamps, build string).
VOLATILE = re.compile(r"CreationDate|GenerationSoftware|Created by|DRILL file|"
                      r'"(Version|Vendor|Application)"|^Date:|date\s*[:=]', re.I)
TEXT_SUFFIXES = {".gtl", ".gbl", ".g1", ".g2", ".gtp", ".gbp", ".gto", ".gbo", ".gts",
                 ".gbs", ".gm1", ".drl", ".gbr", ".gbrjob", ".csv", ".ipc", ".txt"}


def board_stats(board: Path) -> dict:
    """Counts straight from the .kicad_pcb text (no pcbnew needed)."""
    text = board.read_text()
    vias = re.findall(r"\(via( micro| blind| buried)?\s+\(at [^)]*\)\s*\(size ([\d.]+)\)\s*"
                      r"\(drill ([\d.]+)\)", text)
    kinds: dict[str, int] = {}
    for kind, size, drill in vias:
        key = f"{(kind.strip() or 'through')} {float(size):g}/{float(drill):g} mm"
        kinds[key] = kinds.get(key, 0) + 1
    widths = sorted({float(w) for w in re.findall(r"\(segment\s.*?\(width ([\d.]+)\)", text, re.S)})
    return {"segments": len(re.findall(r"\(segment\s", text)), "vias": kinds,
            "min_track_mm": widths[0] if widths else None,
            "footprints": len(re.findall(r"^\t\(footprint ", text, re.M))}


def fab_notes(board: Path, drc: Path | None) -> str:
    """README of the fabrication package: what to order and what is NOT verified."""
    sys.path.insert(0, str(REPO))
    from hardware.lib import board_spec as bs          # noqa: E402
    from hardware.lib import impedance as imp          # noqa: E402
    st = board_stats(board)
    x0, y0, x1, y1 = bs.BOARD_OUTLINE
    report = json.loads(drc.read_text()) if drc and drc.exists() else None
    blocking = None
    if report is not None:
        from tools.check_drc_report import SI_RULE_TYPES, si_summary, summarize   # noqa: E402
        blocking = summarize(report, False, SI_RULE_TYPES)[1]
    L = ["# ESP32-P4 Extreme -- fabrication package", "",
         "> **Prototype files, routed by the project's own router (`tools/pcb_router.py`).**",
         "> **Not fabricated or bench-tested yet.**"]
    if blocking == []:
        L += ["> The copper is fully connected and passes KiCad's DRC for manufacturing and electrical",
              "> rules (0 errors)."]
        if si_summary(report):
            L += ["> Some high-speed rules listed at the end are **not** met (coupling, gap or length",
                  "> matching of some differential pairs / length groups). Review those nets before",
                  "> ordering: they can limit USB 2.0 High-Speed, MIPI CSI/DSI or Ethernet margins."]
    else:
        L += [f"> **Do not order:** KiCad's DRC reports {len(blocking) if blocking else 'unchecked'}"
              " blocking problem(s) (see below)."]
    L += ["", "## Files", "",
         f"- `{ZIP_NAME}`: Gerber X2 (Protel extensions) + Excellon drills (PTH and NPTH separate)"
         " + drill maps + Gerber job file. Upload this to PCBWay.",
         "- `esp32p4_extreme.ipc`: IPC-D-356 netlist for the fab's electrical test.",
         "- `pick_and_place.csv`: centroids, both sides, origin = SoC centre (same as the Gerbers).",
         "- `bom_pcbway.csv`: turnkey BOM (MPN, manufacturer, LCSC).",
         "- Source: `hardware/output/routed/esp32p4_extreme.kicad_pcb` (KiCad 10).", "",
         "## Board", "",
         f"- {x1 - x0:g} x {y1 - y0:g} mm, {imp.BOARD_THICKNESS_MM:g} mm, 4 layers, "
         f"{st['footprints']} footprints, {st['segments']} track segments.",
         "- Stack-up (1+2+1 HDI, controlled impedance 50 / 90 / 100 ohm, ask PCBWay to confirm widths):", "",
         "  | Layer | Thickness | Material | Function |", "  |---|---|---|---|"]
    L += [f"  | {l.name} | {l.thickness_mm:g} mm | {l.material} | {l.function or ('Er ' + format(l.er, 'g'))} |"
          for l in imp.STACKUP]
    L += ["",
          "- Vias: " + ", ".join(f"{n} x {k}" for k, n in sorted(st["vias"].items())) + ".",
          "  Laser microvias L1-L2 (to the GND plane) and L4-L3 (to the power planes) need an HDI",
          "  1+2+1 build. Every via inside a pad (the EPAD array and the plane vias of the 0402 pads)",
          "  must be **filled and capped** (IPC-4761 Type VII, \"via in pad\" / VIPPO).",
          f"- Minimum track {st['min_track_mm']} mm (0.09 mm only in the SoC fan-out, inside U1's "
          "courtyard), minimum clearance 0.09 mm there, 0.10 mm elsewhere.",
          "- Surface finish ENIG (0.35 mm-pitch QFN), green mask, white silkscreen.", ""]
    if report is not None:
        errors = [v for v in report.get("violations", [])
                  if v.get("severity") == "error" and v.get("type") not in SI_RULE_TYPES]
        L += ["## Design-rule check (KiCad 10)", "",
              f"- Unconnected items: {len(report.get('unconnected_items', []))}; "
              f"manufacturing/electrical errors: {len(errors)}"
              + (" (" + ", ".join(sorted({v['type'] for v in errors})) + ")" if errors else "") + ".",
              "- High-speed rules **not met**:", ""]
        L += [f"  - {line.strip()}" for line in si_summary(report)] or ["  - none"]
        L.append("")
    return "\n".join(L) + "\n"


def run(cmd: list[str]) -> None:
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode:
        sys.stderr.write(res.stdout + res.stderr)
        raise SystemExit(f"failed: {' '.join(cmd)}")


def export(board: Path, out: Path, cli: str, drc: Path | None = None) -> list[Path]:
    """Write the package into ``out`` (emptied first); returns the files for the zip."""
    if out.exists():
        shutil.rmtree(out)
    gerb = out / "gerbers"
    gerb.mkdir(parents=True)
    b = str(board)
    run([cli, "pcb", "export", "gerbers", "--layers", LAYERS, "--use-drill-file-origin",
         "--subtract-soldermask", "-o", f"{gerb}/", b])
    run([cli, "pcb", "export", "drill", "--drill-origin", "plot", "--excellon-units", "mm",
         "--excellon-separate-th", "--generate-map", "--map-format", "gerberx2",
         "-o", f"{gerb}/", b])
    run([cli, "pcb", "export", "ipcd356", "-o", str(out / "esp32p4_extreme.ipc"), b])
    run([cli, "pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "both",
         "--use-drill-file-origin", "-o", str(out / "pick_and_place.csv"), b])
    if BOM.exists():
        shutil.copyfile(BOM, out / BOM.name)
    (out / "README.md").write_text(fab_notes(board, drc))
    files = sorted(gerb.iterdir())
    with zipfile.ZipFile(out / ZIP_NAME, "w", zipfile.ZIP_DEFLATED) as z:
        for f in files:
            # fixed timestamps: the zip only changes when a Gerber does
            info = zipfile.ZipInfo(f.name, date_time=(2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, f.read_bytes())
    return files


def normalized(path: Path) -> list[str]:
    return [line for line in path.read_text(errors="replace").splitlines()
            if not VOLATILE.search(line)]


def compare(committed: Path, fresh: Path) -> list[str]:
    """Differences between two packages (volatile lines ignored)."""
    problems = []
    a = {p.relative_to(committed) for p in committed.rglob("*") if p.is_file()}
    b = {p.relative_to(fresh) for p in fresh.rglob("*") if p.is_file()}
    for rel in sorted(a ^ b):
        problems.append(f"{rel}: only in {'committed' if rel in a else 'regenerated'} package")
    for rel in sorted(a & b):
        # the zip mirrors gerbers/; README.md is prose (its DRC summary is re-checked by CI's gate)
        if rel.name in (ZIP_NAME, "README.md") or rel.suffix.lower() not in TEXT_SUFFIXES:
            continue
        x, y = normalized(committed / rel), normalized(fresh / rel)
        if x != y:
            diff = list(difflib.unified_diff(x, y, "committed", "regenerated", n=0, lineterm=""))
            problems.append(f"{rel}: {len(diff)} diff lines, e.g. " + " | ".join(diff[2:6]))
    return problems


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("board", nargs="?", type=Path, default=DEFAULT_BOARD)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--kicad-cli", default="kicad-cli")
    ap.add_argument("--check", action="store_true",
                    help="regenerate and compare with --out instead of writing it")
    ap.add_argument("--drc", type=Path, help="KiCad DRC JSON of the board, summarised in README.md")
    args = ap.parse_args(argv)
    if args.check:
        with tempfile.TemporaryDirectory() as tmp:
            fresh = Path(tmp) / "fab"
            export(args.board.resolve(), fresh, args.kicad_cli, args.drc)
            problems = compare(args.out, fresh)
        for p in problems:
            print("  - " + p)
        print(f"{'FAIL' if problems else 'PASS'}: committed fabrication package "
              f"{'differs from' if problems else 'matches'} {args.board}")
        return 1 if problems else 0
    files = export(args.board.resolve(), args.out, args.kicad_cli, args.drc)
    print(f"{len(files)} Gerber/drill files + IPC-356 + pick-and-place + BOM in {args.out}; "
          f"zip: {args.out / ZIP_NAME}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
