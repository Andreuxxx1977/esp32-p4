"""TASK 3 -- SKiDL netlist of the ESP32-P4 "Extreme Performance" board.

The circuit is *not* re-typed here: every part, pin, net and connection is read
from the single source of truth, ``hardware.lib.board_spec``. This script only
translates that data model into SKiDL objects, runs SKiDL's electrical rules
check and writes the KiCad netlist. Edit the spec, re-run this script, and the
netlist follows.

How to run (from the repository root)::

    pip install skidl                     # tested with SKiDL 2.3.0
    python3 -m hardware.skidl.esp32p4_extreme_netlist
    # or: python3 hardware/skidl/esp32p4_extreme_netlist.py [--out DIR] [--allow-warnings]
    python3 -m hardware.skidl.verify_netlist      # independent cross-check

No KiCad installation or symbol library is needed: each ``PartType`` in the
spec becomes an in-memory SKiDL part template (tool=SKIDL) whose pins are the
spec's (pad number, pin name) pairs. SKiDL prints some "KICAD*_SYMBOL_DIR /
fp-lib-table not found" warnings when it is imported; they are harmless here
because footprints are plain "Library:Footprint" strings.

Outputs (in ``hardware/output/``):

* ``esp32p4_extreme.net``      KiCad s-expression netlist (pcbnew: File > Import > Netlist,
                               or kinet2pcb). DNP parts carry the ``dnp`` property and
                               a ``DNP`` field; mounting holes are ``exclude_from_bom``.
* ``esp32p4_extreme_nets.csv`` One row per connection: net, netclass, ref, pad, pin.
* ``esp32p4_extreme_erc.log``  The full SKiDL ERC report.

Modelling decisions (they keep ERC meaningful instead of noisy):

* Pads are connected by *pad number*, never by pin name: several parts repeat
  names (GND, VBUS, SW, PVIN, I/O1 ...).
* Pin electrical types: the ESP32-P4 uses the types from Espressif's KiCad
  symbol (power_in, power_out, input, output, bidirectional). Every other part
  only has pin names in the spec, so its pins are PASSIVE, except pins named
  ``NC`` which are NOCONNECT.
* Supply rails (``GND``, every net in the POWER netclass, and anything fed from
  them through a fitted 0R link, ferrite bead or fuse) get ``drive = POWER``.
  This is SKiDL's equivalent of a KiCad PWR_FLAG: the regulators and
  connectors that really drive those rails only have passive pins here.
  A power-input pin on any *other* net is still reported by ERC.
* A pad that the spec leaves open is tied to SKiDL's no-connect net (it then
  does not appear in the netlist) and is listed in the report, *unless* it
  looks like a supply/ground/shield/exposed pad or is a SoC power pin. Those
  are left floating on purpose so ERC flags them.
* Net classes come from ``board_spec.netclass_of()``; controlled-impedance
  classes also carry their geometry from ``hardware.lib.impedance``. SKiDL
  always adds ``Default`` as well, so a net's netlist ``class`` reads e.g.
  ``"USB_90,Default"`` (highest priority first). The CSV has the single
  effective class.

With these rules the current design passes ERC with 0 errors and 0 warnings,
so any ERC message is a real problem: the script then exits with status 1 and
does not write the netlist (``--allow-warnings`` writes it anyway). The spec's
own ``validate()`` must also be clean before anything is generated.
"""

from __future__ import annotations

import argparse
import builtins
import csv
import logging
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if __package__ in (None, ""):  # run as a plain script: make `hardware` importable
    sys.path.insert(0, str(REPO_ROOT))

from hardware.lib import board_spec as spec  # noqa: E402
from hardware.lib import esp32p4_pinout as p4  # noqa: E402
from hardware.lib.impedance import impedance_rules  # noqa: E402

import skidl  # noqa: E402
from skidl import (  # noqa: E402
    ERC, POWER, SKIDL, TEMPLATE, Net, NetClass, Part, Pin, erc_logger, generate_netlist,
)
from skidl.logger import active_logger  # noqa: E402

OUT_DIR = REPO_ROOT / "hardware" / "output"
NETLIST_NAME = "esp32p4_extreme.net"
CSV_NAME = "esp32p4_extreme_nets.csv"
ERC_LOG_NAME = "esp32p4_extreme_erc.log"

circuit = builtins.default_circuit   # SKiDL keeps its default Circuit in builtins
NO_CONNECT = builtins.NC             # ... and its no-connect net

T = Pin.types
# KiCad electrical type (Espressif symbol) -> SKiDL pin function.
KICAD_PIN_TYPE = {
    "input": T.INPUT, "output": T.OUTPUT, "bidirectional": T.BIDIR,
    "tri_state": T.TRISTATE, "passive": T.PASSIVE, "unspecified": T.UNSPEC,
    "power_in": T.PWRIN, "power_out": T.PWROUT, "open_collector": T.OPENCOLL,
    "open_emitter": T.OPENEMIT, "no_connect": T.NOCONNECT, "free": T.FREE,
}

# Pin names that must never float silently: leave them unconnected so ERC complains.
SUPPLY_PIN_NAME = re.compile(
    r"^(GND|AGND|PGND|DGND|VSS|EP|EPAD|SH|SHIELD|MP|VBUS|VCC|VDD\w*|AVDD|VIN|AVIN|PVIN|"
    r"VIO|VREGIN|VOS|3V3|\+\d+V\d*)$")


# ---------------------------------------------------------------------------
# 1. Part templates (one per PartType, built from the spec -- no symbol libs)
# ---------------------------------------------------------------------------

def pin_function(part: spec.PartType, pad: str, name: str) -> int:
    if part is spec.ESP32P4:
        return KICAD_PIN_TYPE[p4.PAD_TYPE[pad]]
    return T.NOCONNECT if name == "NC" else T.PASSIVE


def make_template(part: spec.PartType) -> Part:
    pads = [pad for pad, _ in part.pins]
    dup = [pad for pad, n in Counter(pads).items() if n > 1]
    if dup:
        raise ValueError(f"{part.key}: duplicate pad numbers {dup}")
    return Part(
        name=part.key, tool=SKIDL, dest=TEMPLATE, description=part.description,
        pins=[Pin(num=pad, name=name, func=pin_function(part, pad, name))
              for pad, name in part.pins])


# ---------------------------------------------------------------------------
# 2. Nets, net classes and supply rails
# ---------------------------------------------------------------------------

def make_netclasses() -> dict[str, NetClass]:
    """One SKiDL NetClass per spec class (priority above SKiDL's 'Default')."""
    geometry = {r.netclass: r for r in impedance_rules()}
    classes = {}
    for name, _pattern in spec.NETCLASS_PATTERNS:
        attrs = {}
        rule = geometry.get(name)
        if rule is not None:
            attrs["impedance_ohm"] = rule.target_ohm
            if rule.kind == "diff":
                attrs.update(diff_pair_width=rule.width_mm, diff_pair_gap=rule.gap_mm)
            else:
                attrs["trace_width"] = rule.width_mm
        classes[name] = NetClass(name, priority=1, **attrs)
    return classes


def supply_rails(net_names) -> set[str]:
    """GND + POWER-class nets, extended through fitted 0R links, ferrites and fuses."""
    rails = {n for n in net_names if n == spec.GND or spec.netclass_of(n) == "POWER"}
    links = [tuple(c.conns.values()) for c in spec.COMPONENTS
             if len(c.conns) == 2 and not c.dnp
             and (c.part.kind == "fb" or (c.part.kind == "res" and c.value == "0R"))]
    grown = True
    while grown:
        grown = False
        for a, b in links:
            if (a in rails) != (b in rails):
                rails |= {a, b}
                grown = True
    return rails


# ---------------------------------------------------------------------------
# 3. Build the circuit
# ---------------------------------------------------------------------------

def build() -> dict:
    """Instantiate every spec component and make every spec connection."""
    circuit.no_files = True      # SKiDL writes no log/backup files into the CWD
    circuit.track_src = False    # no per-part "SKiDL Line" field in the netlist

    templates = {key: make_template(pt) for key, pt in spec.PARTS.items()}

    spec_nets = spec.nets()
    nets = {}
    for name in sorted(spec_nets):
        nets[name] = Net(name)
        if nets[name].name != name:
            raise RuntimeError(f"SKiDL renamed net {name!r} to {nets[name].name!r}")

    classes = make_netclasses()
    for name, net in nets.items():
        cls = spec.netclass_of(name)
        if cls != "Default":
            net.netclasses = classes[cls]

    rails = supply_rails(nets)
    for name in rails:
        nets[name].drive = POWER

    no_connects: dict[str, list[str]] = defaultdict(list)   # ref -> ["pad name", ...]
    floating: list[str] = []
    nc_gpio_pads = {p4.GPIO_PAD[g] for g in spec.GPIO_NC}
    for comp in spec.COMPONENTS:
        pt = comp.part
        part = templates[pt.key](ref=comp.ref, value=comp.value, footprint=pt.footprint,
                                 tag=comp.ref)
        if part.ref != comp.ref:
            raise RuntimeError(f"SKiDL renamed {comp.ref} to {part.ref} (duplicate ref?)")
        part.fields.update({k: v for k, v in (
            ("MPN", pt.mpn), ("Manufacturer", pt.manufacturer), ("LCSC", pt.lcsc),
            ("DNP", "DNP" if comp.dnp else ""), ("Group", comp.group), ("Note", comp.note),
        ) if v})
        if comp.dnp:
            part.dnp = True                   # -> (property (name "dnp"))
        if pt.kind == "mech":
            part.exclude_from_bom = True      # -> (property (name "exclude_from_bom"))

        pins = {pin.num: pin for pin in part.pins}
        for pad, net_name in comp.conns.items():
            nets[net_name] += pins[pad]
        for pad, name in pt.pins:
            if pad in comp.conns:
                continue
            pin = pins[pad]
            if pin.func in (T.PWRIN, T.PWROUT) or SUPPLY_PIN_NAME.match(name):
                floating.append(f"{comp.ref}.{pad} ({name})")     # ERC will flag it
                continue
            NO_CONNECT.connect(pin)
            why = ("declared NC" if name == "NC" or (pt is spec.ESP32P4 and pad in nc_gpio_pads)
                   else "unused")
            no_connects[comp.ref].append(f"{pad} {name} [{why}]")

    return {"nets": nets, "rails": rails, "no_connects": no_connects, "floating": floating}


# ---------------------------------------------------------------------------
# 4. Reports
# ---------------------------------------------------------------------------

def _natural(text: str):
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", text)]


def effective_netclass(net) -> str:
    names = net.netclasses.by_priority()   # lowest priority first
    return names[-1] if names else "Default"


def write_csv(path: Path) -> int:
    rows = [(net.name, effective_netclass(net), pin.part.ref, pin.num, pin.name)
            for net in circuit.get_nets() for pin in net.pins]
    rows.sort(key=lambda r: (_natural(r[0]), _natural(r[2]), _natural(r[3])))
    with path.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["net", "netclass", "ref", "pad", "pin"])
        writer.writerows(rows)
    return len(rows)


def _show(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def run_erc(log_path: Path) -> tuple[int, int]:
    handler = logging.FileHandler(log_path, mode="w")
    handler.setFormatter(logging.Formatter("ERC %(levelname)s: %(message)s"))
    erc_logger.addHandler(handler)
    try:
        ERC()
    finally:
        erc_logger.removeHandler(handler)
        handler.close()
    return erc_logger.warning.count, erc_logger.error.count


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=OUT_DIR, help="output directory")
    ap.add_argument("--allow-warnings", action="store_true",
                    help="write the netlist even if ERC reports warnings")
    args = ap.parse_args(argv)

    problems = spec.validate()
    if problems:
        print("board_spec.validate() failed -- fix the spec first:\n  " + "\n  ".join(problems))
        return 1

    info = build()
    args.out.mkdir(parents=True, exist_ok=True)

    print(f"\n=== SKiDL {skidl.__version__}: ERC ===")
    print(f"PWR_FLAG-equivalent supply rails ({len(info['rails'])}): "
          + ", ".join(sorted(info["rails"], key=_natural)))
    n_nc = sum(len(v) for v in info["no_connects"].values())
    print(f"Pads tied to the no-connect net ({n_nc}):")
    for ref in sorted(info["no_connects"], key=_natural):
        print(f"  {ref:6s} " + "; ".join(info["no_connects"][ref]))
    if info["floating"]:
        print("Supply-type pads left open (ERC will warn): " + ", ".join(info["floating"]))
    warnings, errors = run_erc(args.out / ERC_LOG_NAME)
    print(f"ERC result: {errors} errors, {warnings} warnings "
          f"(log: {_show(args.out / ERC_LOG_NAME)})")
    if errors or (warnings and not args.allow_warnings):
        print("ERC failed -- netlist NOT written.")
        return 1

    print("\n=== Netlist ===")
    # Drop SKiDL's import-time library-path warnings from the netlist-phase count.
    # (SKiDL 2.3 also always warns once about a "Missing tag" on the root node.)
    active_logger.bare_warning.reset()
    netlist = generate_netlist(do_backup=False)
    net_path = args.out / NETLIST_NAME
    net_path.write_text(str(netlist))
    n_rows = write_csv(args.out / CSV_NAME)

    comps = spec.COMPONENTS
    by_class = Counter(effective_netclass(n) for n in circuit.get_nets())
    print(f"{len(comps)} components ({sum(c.dnp for c in comps)} DNP), "
          f"{len(circuit.get_nets())} nets, {n_rows} connections, {len(spec.PARTS)} part templates")
    print("Net classes: " + ", ".join(f"{k}={v}" for k, v in sorted(by_class.items())))
    for name in (NETLIST_NAME, CSV_NAME):
        print(f"wrote {_show(args.out / name)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
