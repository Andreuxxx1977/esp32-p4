"""Finish an autorouted board with the grid router, with KiCad as the judge.

Loop: KiCad DRC (unconnected items) -> ``tools.maze_route`` on them -> refill zones and
save (``kicad-cli pcb drc --refill-zones --save-board``) -> repeat while it helps.
The last DRC report is left next to the board (``<board>.drc.json``).

Usage::

    python -m tools.finish_routing ROUTED.kicad_pcb [--kicad-cli kicad-cli] [--rounds 3]
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def drc(cli: str, board: Path, out: Path, refill: bool = False) -> dict:
    cmd = [cli, "pcb", "drc", "--format", "json", "--severity-all", "--units", "mm", "-o", str(out)]
    if refill:
        cmd += ["--refill-zones", "--save-board"]
    subprocess.run(cmd + [str(board)], capture_output=True, text=True)
    return json.loads(out.read_text())


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("board", type=Path)
    ap.add_argument("--kicad-cli", default="kicad-cli")
    ap.add_argument("--rounds", type=int, default=3)
    args = ap.parse_args(argv)
    board = args.board.resolve()
    report = board.with_suffix(".drc.json")
    rep = drc(args.kicad_cli, board, report, refill=True)
    before = len(rep.get("unconnected_items", []))
    print(f"start: {before} unconnected items")
    for rnd in range(1, args.rounds + 1):
        n = len(rep.get("unconnected_items", []))
        if n == 0:
            break
        tmp = board.with_suffix(".maze.kicad_pcb")
        res = subprocess.run([sys.executable, "-m", "tools.maze_route", str(board), str(report), "-o", str(tmp)],
                             cwd=REPO, capture_output=True, text=True)
        print(res.stdout.strip().splitlines()[-1] if res.stdout.strip() else res.stderr[-2000:])
        if not tmp.exists():
            break
        shutil.move(tmp, board)
        rep = drc(args.kicad_cli, board, report, refill=True)
        after = len(rep.get("unconnected_items", []))
        print(f"round {rnd}: {n} -> {after} unconnected items")
        if after >= n:
            break
    errors = [v for v in rep.get("violations", []) if v.get("severity") == "error"]
    print(f"final: {len(rep.get('unconnected_items', []))} unconnected items, {len(errors)} DRC errors "
          f"(report: {report})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
