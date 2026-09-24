"""Gate a KiCad DRC JSON report (``kicad-cli pcb drc --format json``).

The board is currently *placed but not routed*, so every net still shows up
as "unconnected items". Those are reported and counted but do not fail the
build; any other violation of severity ``error`` does (courtyard overlaps,
clearance, holes, edge clearance, keep-out/rule-area hits ...).

Once the board is routed, run with ``--strict`` so unconnected items fail too.

``--routed`` is the gate for the autorouted prototype (``hardware/pcbnew/autoroute.py``):
strict, except that the signal-integrity rules of the .kicad_dru (differential-pair
gap/coupling, skew, length) are listed as *not met* instead of failing. An
autorouter does not route coupled pairs; those violations are real and are
published with the fabrication files, they are not manufacturing defects.

Usage::

    python -m tools.check_drc_report drc.json [--strict | --routed] [--allow TYPE ...]
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

# Violation types that are expected while the board is only placed.
PLACEMENT_PHASE_ALLOWED = frozenset({"unconnected_items"})
# Signal-integrity constraints (custom rules), reported but not blocking with --routed.
SI_RULE_TYPES = frozenset({
    "diff_pair_gap_out_of_range", "diff_pair_uncoupled_length_too_long",
    "skew_out_of_range", "length_out_of_range",
})


def si_summary(report: dict) -> list[str]:
    """One line per net group that misses a signal-integrity rule."""
    seen = Counter()
    for v in report.get("violations", []):
        if v.get("type") in SI_RULE_TYPES:
            nets = sorted({i.get("description", "").split("[")[-1].split("]")[0]
                           for i in v.get("items", []) if "[" in i.get("description", "")})
            seen[(v.get("type"), ", ".join(nets))] += 1
    return [f"  {typ}: {nets or '?'} ({n}x)" for (typ, nets), n in sorted(seen.items())]


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
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--strict", action="store_true", help="also fail on unconnected items")
    mode.add_argument("--routed", action="store_true",
                      help="strict, but list signal-integrity rule misses instead of failing")
    ap.add_argument("--allow", action="append", default=[], metavar="TYPE",
                    help="additional violation type to tolerate (repeatable)")
    args = ap.parse_args(argv)
    report = json.loads(args.report.read_text())
    if args.routed:
        lines, blocking = summarize(report, False, SI_RULE_TYPES | frozenset(args.allow))
    else:
        lines, blocking = summarize(report, args.strict,
                                    PLACEMENT_PHASE_ALLOWED | frozenset(args.allow))
    print("\n".join(lines))
    si = si_summary(report) if args.routed else []
    if si:
        print(f"\nSignal-integrity rules NOT met ({len(si)} net group/rule pairs):")
        print("\n".join(si))
    if blocking:
        print(f"\nFAIL: {len(blocking)} blocking DRC problem(s)")
        for b in blocking[:200]:
            print("  - " + b)
        return 1
    if args.routed:
        print("\nPASS: fully connected, no manufacturing/electrical DRC errors"
              + (" (signal-integrity rules above not met)" if si else ""))
    else:
        print("\nPASS: no blocking DRC errors"
              + ("" if args.strict else " (unconnected items tolerated: board not routed yet)"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
