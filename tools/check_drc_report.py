"""Gate a KiCad DRC JSON report (``kicad-cli pcb drc --format json``).

The board is currently *placed but not routed*, so every net still shows up
as "unconnected items". Those are reported and counted but do not fail the
build; any other violation of severity ``error`` does (courtyard overlaps,
clearance, holes, edge clearance, keep-out/rule-area hits ...).

Once the board is routed, run with ``--strict`` so unconnected items fail too.

Usage::

    python -m tools.check_drc_report drc.json [--strict] [--allow TYPE ...]
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

# Violation types that are expected while the board is only placed.
PLACEMENT_PHASE_ALLOWED = frozenset({"unconnected_items"})


def summarize(report: dict, strict: bool = False,
              allowed: frozenset[str] = PLACEMENT_PHASE_ALLOWED) -> tuple[list[str], list[str]]:
    """Return (summary lines, blocking problems)."""
    allowed = frozenset() if strict else allowed
    violations = list(report.get("violations", []))
    unconnected = list(report.get("unconnected_items", []))
    parity = list(report.get("schematic_parity", []))

    by_type = Counter((v.get("type", "?"), v.get("severity", "?")) for v in violations)
    lines = [f"KiCad {report.get('kicad_version', '?')} DRC of {report.get('source', '?')}",
             f"violations: {len(violations)}, unconnected items: {len(unconnected)}, "
             f"schematic parity: {len(parity)}"]
    lines += [f"  {n:5d}  {sev:8s} {typ}" for (typ, sev), n in sorted(by_type.items())]

    blocking = [f"{v.get('severity')} {v.get('type')}: {v.get('description', '')}"
                for v in violations
                if v.get("severity") == "error" and v.get("type") not in allowed]
    if unconnected and "unconnected_items" not in allowed:
        blocking.append(f"{len(unconnected)} unconnected items (board not fully routed)")
    return lines, blocking


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("report", type=Path)
    ap.add_argument("--strict", action="store_true", help="also fail on unconnected items")
    ap.add_argument("--allow", action="append", default=[], metavar="TYPE",
                    help="additional violation type to tolerate (repeatable)")
    args = ap.parse_args(argv)
    report = json.loads(args.report.read_text())
    lines, blocking = summarize(report, args.strict,
                                PLACEMENT_PHASE_ALLOWED | frozenset(args.allow))
    print("\n".join(lines))
    if blocking:
        print(f"\nFAIL: {len(blocking)} blocking DRC problem(s)")
        for b in blocking[:200]:
            print("  - " + b)
        return 1
    print("\nPASS: no blocking DRC errors"
          + ("" if args.strict else " (unconnected items tolerated: board not routed yet)"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
