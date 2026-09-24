"""Export the fabrication package (Gerbers, drills, IPC-356, pick-and-place, BOM).

Runs ``kicad-cli`` on a routed board and writes everything PCBWay (or any fab)
needs into one directory plus a zip of it. All files use the drill/place origin,
which this design puts at the SoC centre, so coordinates match the docs (U1 = 0,0).

``--check`` regenerates into a temporary directory and compares with the committed
package, ignoring only the lines that carry a date or the KiCad build string, so CI
proves the committed Gerbers are exactly what the committed routed board produces.

Usage::

    python -m tools.export_fab [BOARD] [--out DIR] [--kicad-cli kicad-cli] [--check]
"""

from __future__ import annotations

import argparse
import difflib
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


def run(cmd: list[str]) -> None:
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode:
        sys.stderr.write(res.stdout + res.stderr)
        raise SystemExit(f"failed: {' '.join(cmd)}")


def export(board: Path, out: Path, cli: str) -> list[Path]:
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
        if rel.name == ZIP_NAME or rel.suffix.lower() not in TEXT_SUFFIXES:
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
    args = ap.parse_args(argv)
    if args.check:
        with tempfile.TemporaryDirectory() as tmp:
            fresh = Path(tmp) / "fab"
            export(args.board.resolve(), fresh, args.kicad_cli)
            problems = compare(args.out, fresh)
        for p in problems:
            print("  - " + p)
        print(f"{'FAIL' if problems else 'PASS'}: committed fabrication package "
              f"{'differs from' if problems else 'matches'} {args.board}")
        return 1 if problems else 0
    files = export(args.board.resolve(), args.out, args.kicad_cli)
    print(f"{len(files)} Gerber/drill files + IPC-356 + pick-and-place + BOM in {args.out}; "
          f"zip: {args.out / ZIP_NAME}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
