"""Independent check of the SKiDL netlist against ``hardware.lib.board_spec``.

Parses ``hardware/output/esp32p4_extreme.net`` (KiCad s-expression) and
``hardware/output/esp32p4_extreme_nets.csv`` with a stand-alone reader (no
SKiDL import) and asserts that:

a) the set of component refs equals ``board_spec.COMPONENTS`` (no duplicates),
   and each component's footprint, value, MPN, Group and DNP flag match;
b) the set of (net, ref, pad) connections equals ``board_spec.nets()`` --
   nothing missing, nothing extra, no pad on two nets;
c) the pad count of every net matches;
d) every net's highest-priority class equals ``board_spec.netclass_of()``;
e) the CSV holds exactly the same connections and classes.

Run from the repository root, after the netlist script::

    python3 -m hardware.skidl.verify_netlist [--net FILE] [--csv FILE]

Exit status 0 = all checks passed.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if __package__ in (None, ""):
    sys.path.insert(0, str(REPO_ROOT))

from hardware.lib import board_spec as spec  # noqa: E402

OUT_DIR = REPO_ROOT / "hardware" / "output"

_TOKEN = re.compile(r'\s*(?:(\()|(\))|"((?:[^"\\]|\\.)*)"|([^\s()"]+))')


def parse_sexpr(text: str) -> list:
    """Parse an s-expression into nested lists of strings."""
    stack: list[list] = [[]]
    pos = 0
    while True:
        m = _TOKEN.match(text, pos)
        if not m or m.end() == pos:
            break
        pos = m.end()
        lpar, rpar, quoted, atom = m.groups()
        if lpar:
            stack.append([])
        elif rpar:
            done = stack.pop()
            stack[-1].append(done)
        elif quoted is not None:
            stack[-1].append(re.sub(r"\\(.)", r"\1", quoted))
        else:
            stack[-1].append(atom)
    if text[pos:].strip() or len(stack) != 1:
        raise ValueError(f"malformed s-expression near offset {pos}")
    return stack[0][0]


def children(node: list, key: str) -> list[list]:
    return [c for c in node[1:] if isinstance(c, list) and c and c[0] == key]


def child(node: list, key: str) -> list | None:
    found = children(node, key)
    return found[0] if found else None


def value(node: list, key: str, default: str = "") -> str:
    c = child(node, key)
    return c[1] if c is not None and len(c) > 1 else default


def read_netlist(path: Path):
    root = parse_sexpr(path.read_text())
    if root[0] != "export":
        raise ValueError(f"{path}: not a KiCad netlist (root is {root[0]!r})")
    comps = {}
    ref_counts = Counter()
    for comp in children(child(root, "components"), "comp"):
        ref = value(comp, "ref")
        ref_counts[ref] += 1
        fields = {value(f, "name"): (f[2] if len(f) > 2 else "")
                  for f in children(child(comp, "fields") or ["fields"], "field")}
        props = {value(p, "name") for p in children(comp, "property")}
        comps[ref] = {"value": value(comp, "value"), "footprint": value(comp, "footprint"),
                      "fields": fields, "props": props}
    nets = {}
    for net in children(child(root, "nets"), "net"):
        nodes = [(value(n, "ref"), value(n, "pin")) for n in children(net, "node")]
        nets[value(net, "name")] = {"class": value(net, "class", "Default"), "nodes": nodes}
    return comps, ref_counts, nets


def verify(net_path: Path, csv_path: Path) -> list[str]:
    errors: list[str] = []

    def check(ok: bool, msg: str) -> None:
        if not ok:
            errors.append(msg)

    comps, ref_counts, nets = read_netlist(net_path)
    spec_comps = {c.ref: c for c in spec.COMPONENTS}

    # (a) components
    check(not [r for r, n in ref_counts.items() if n > 1],
          f"duplicate refs in netlist: {[r for r, n in ref_counts.items() if n > 1]}")
    check(not set(spec_comps) - set(comps), f"missing refs: {sorted(set(spec_comps) - set(comps))}")
    check(not set(comps) - set(spec_comps), f"extra refs: {sorted(set(comps) - set(spec_comps))}")
    for ref, sc in spec_comps.items():
        nc = comps.get(ref)
        if nc is None:
            continue
        check(nc["footprint"] == sc.part.footprint,
              f"{ref}: footprint {nc['footprint']!r} != {sc.part.footprint!r}")
        check(nc["value"] == sc.value, f"{ref}: value {nc['value']!r} != {sc.value!r}")
        check(nc["fields"].get("MPN", "") == sc.part.mpn, f"{ref}: MPN mismatch")
        check(nc["fields"].get("Group", "") == sc.group, f"{ref}: Group mismatch")
        is_dnp = "dnp" in nc["props"] and nc["fields"].get("DNP") == "DNP"
        check(is_dnp == sc.dnp, f"{ref}: DNP flag {is_dnp} != spec {sc.dnp}")

    # (b) + (c) connections
    spec_nets = spec.nets()
    spec_conn = {(net, ref, pad) for net, members in spec_nets.items() for ref, pad in members}
    net_conn = {(net, ref, pad) for net, d in nets.items() for ref, pad in d["nodes"]}
    missing, extra = spec_conn - net_conn, net_conn - spec_conn
    check(not missing, f"{len(missing)} connections missing, e.g. {sorted(missing)[:10]}")
    check(not extra, f"{len(extra)} extra connections, e.g. {sorted(extra)[:10]}")
    pads = Counter((ref, pad) for d in nets.values() for ref, pad in d["nodes"])
    multi = sorted(k for k, n in pads.items() if n > 1)
    check(not multi, f"pads on more than one net / listed twice: {multi[:10]}")
    check(set(nets) == set(spec_nets),
          f"net names differ: missing {sorted(set(spec_nets) - set(nets))}, "
          f"extra {sorted(set(nets) - set(spec_nets))}")
    for name, members in spec_nets.items():
        if name in nets:
            check(len(nets[name]["nodes"]) == len(members),
                  f"net {name}: {len(nets[name]['nodes'])} pads in netlist, {len(members)} in spec")

    # (d) net classes (SKiDL lists the highest-priority class first)
    for name, d in nets.items():
        first = d["class"].split(",")[0]
        check(first == spec.netclass_of(name),
              f"net {name}: class {d['class']!r}, spec says {spec.netclass_of(name)!r}")

    # (e) CSV side file
    with csv_path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    csv_conn = {(r["net"], r["ref"], r["pad"]) for r in rows}
    check(len(rows) == len(csv_conn), "CSV has duplicate rows")
    check(csv_conn == spec_conn, f"CSV connections differ from spec "
          f"(missing {len(spec_conn - csv_conn)}, extra {len(csv_conn - spec_conn)})")
    bad_cls = sorted({r["net"] for r in rows if r["netclass"] != spec.netclass_of(r["net"])})
    check(not bad_cls, f"CSV netclass mismatch on {bad_cls[:10]}")

    print(f"netlist: {len(comps)} components, {len(nets)} nets, {len(net_conn)} connections")
    print(f"spec:    {len(spec_comps)} components, {len(spec_nets)} nets, {len(spec_conn)} connections")
    print(f"csv:     {len(rows)} rows")
    return errors


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Verify the SKiDL netlist against board_spec")
    ap.add_argument("--net", type=Path, default=OUT_DIR / "esp32p4_extreme.net")
    ap.add_argument("--csv", type=Path, default=OUT_DIR / "esp32p4_extreme_nets.csv")
    args = ap.parse_args(argv)
    errors = verify(args.net, args.csv)
    if errors:
        print(f"FAIL: {len(errors)} problem(s)")
        for e in errors:
            print("  - " + e)
        return 1
    print("PASS: netlist and CSV match board_spec exactly "
          "(refs, footprints, values, DNP, every net/pad, pad counts, netclasses)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
