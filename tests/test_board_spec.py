"""Design-rule tests for ``hardware/lib`` and the generated documents.

Run from the repository root:  python3 -m pytest -q
"""

from __future__ import annotations

import csv
import itertools
import math
import re
import subprocess
import sys
from collections import Counter
from pathlib import Path

import pytest

from hardware.lib import board_spec as bs
from hardware.lib import esp32p4_pinout as p4
from hardware.lib import impedance as imp

ROOT = Path(__file__).resolve().parents[1]
NETS = bs.nets()
GND = bs.GND
USE = {u.gpio: u for u in bs.GPIO_MAP}
MIN_TRACE_MM = 3.5 * 0.0254          # PCBWay HDI 3.5/3.5 mil

# Interfaces as GPIO_MAP block predicates (GPIO resources).
GPIO_IFACES = {
    "RMII": lambda u: "Ethernet" in u.block,
    "SDIO": lambda u: u.block == "microSD",
    "USB-JTAG": lambda u: u.block == "USB-JTAG",
    "UART0": lambda u: u.block == "Debug UART",
}
# IO_MUX uses that have no IOMUX_OPTIONS entry yet (reported to the spec owner: the LP UART
# pads are not in esp32p4_pinout.IOMUX_OPTIONS). Anything else unmatched fails the test.
KNOWN_UNLISTED_IOMUX: set[str] = set()


def _pads(spec: str) -> set[str]:
    a, _, b = spec.partition("-")
    return {str(n) for n in range(int(a), int(b or a) + 1)}


def _gpio_set(name: str) -> set[int]:
    return {u.gpio for u in bs.GPIO_MAP if GPIO_IFACES[name](u)}


def _iomux_signals(use: bs.GpioUse) -> list[str]:
    return [s for s in p4.IOMUX_OPTIONS
            if use.net in (s, s.replace("SD0_", "SD_"))
            or re.search(rf"(?<![A-Z0-9_]){re.escape(s)}(?![A-Z0-9_])", use.function)]


def _net_of_gpio(g: int) -> str:
    return USE[g].net


# ---------------------------------------------------------------------------
# Spec integrity
# ---------------------------------------------------------------------------

def test_validate_is_clean():
    assert bs.validate() == []


def test_no_gpio_used_twice():
    gpios = Counter(u.gpio for u in bs.GPIO_MAP)
    assert [g for g, n in gpios.items() if n > 1] == []
    nets = Counter(u.net for u in bs.GPIO_MAP)
    assert [n for n, k in nets.items() if k > 1] == []
    # no GPIO pad is also listed as a dedicated pad
    assert not {p4.GPIO_PAD[u.gpio] for u in bs.GPIO_MAP} & set(bs.DEDICATED_NETS)


def test_all_55_gpios_decided():
    used = {u.gpio for u in bs.GPIO_MAP}
    assert not used & set(bs.GPIO_NC)
    assert used | set(bs.GPIO_NC) == set(range(55))
    assert all(0 <= g <= 54 for g in used)


@pytest.mark.parametrize("name", sorted(GPIO_IFACES))
def test_interface_sets_are_populated(name):
    assert _gpio_set(name), f"no GPIO_MAP entries for {name} -- block names changed?"


@pytest.mark.parametrize("a,b", list(itertools.combinations(sorted(GPIO_IFACES), 2)))
def test_zero_overlap_between_gpio_interfaces(a, b):
    assert _gpio_set(a) & _gpio_set(b) == set()


def test_mipi_and_usb_hs_are_dedicated_pads_not_gpios():
    gpio_pads = set(p4.GPIO_PAD.values())
    groups = {k: _pads(v) for k, v in p4.DEDICATED_PADS.items() if "MIPI" in k or "USB 2.0 HS" in k}
    assert groups, "DEDICATED_PADS lost its MIPI / USB-HS entries"
    for name, pads in groups.items():
        assert not pads & gpio_pads, name
        for pad in pads:
            assert not p4.PAD_NAME[pad].startswith("GPIO"), (name, pad)
    # the MIPI / USB-HS nets live only on those dedicated pads ...
    for pad, net in bs.U1.conns.items():
        if re.match(r"^(DSI|CSI)_|^USBHS_", net):
            assert pad not in gpio_pads, (pad, net)
            assert any(pad in pads for pads in groups.values()), (pad, net)
    # ... and never appear in the GPIO map
    assert not [u.net for u in bs.GPIO_MAP if re.match(r"^(DSI|CSI|USBHS)_", u.net)]
    # and the dedicated pad sets do not overlap any GPIO interface set
    all_gpio_iface_pads = {p4.GPIO_PAD[g] for n in GPIO_IFACES for g in _gpio_set(n)}
    assert not all_gpio_iface_pads & set().union(*groups.values())


def test_every_iomux_fixed_signal_is_on_a_legal_pad():
    unmatched = []
    checked = 0
    for u in bs.GPIO_MAP:
        if u.mux != "IO_MUX":
            continue
        sigs = _iomux_signals(u)
        if not sigs:
            unmatched.append(u.net)
            continue
        assert len(sigs) == 1, (u.net, sigs)
        assert u.gpio in p4.IOMUX_OPTIONS[sigs[0]], f"{sigs[0]} on GPIO{u.gpio}, legal {p4.IOMUX_OPTIONS[sigs[0]]}"
        checked += 1
    assert checked >= 17
    assert set(unmatched) <= KNOWN_UNLISTED_IOMUX, f"IO_MUX uses with no IOMUX_OPTIONS entry: {unmatched}"


def test_rmii_signals_avoid_sdio_slot0_pads():
    sd_pads = {g for s, o in p4.IOMUX_OPTIONS.items() if s.startswith("SD0_") for g in o}
    rmii = [u for u in bs.GPIO_MAP if u.net.startswith("RMII_")]
    assert rmii
    assert not {u.gpio for u in rmii} & sd_pads


def test_every_u1_pad_has_a_net_or_explicit_nc():
    nc_pads = {p4.GPIO_PAD[g] for g in bs.GPIO_NC}
    problems = []
    for pad, name, _ in p4.PADS:
        has_net = pad in bs.U1.conns
        if has_net == (pad in nc_pads):          # neither, or both
            problems.append((pad, name, has_net))
    assert problems == []


def test_every_u1_supply_pad_is_decoupled():
    for pad, name, _ in p4.PADS:
        if not name.startswith("VDD"):
            continue
        caps = [c for c in bs.COMPONENTS if c.part.kind == "cap" and isinstance(c.place, bs.Near)
                and c.place.ref == "U1" and c.place.pad == pad]
        assert caps, f"U1 pad {pad} {name} has no decoupling capacitor"


# ---------------------------------------------------------------------------
# Strapping
# ---------------------------------------------------------------------------

def test_gpio35_boot_strap_has_a_pull_up():
    net = _net_of_gpio(p4.BOOT_MODE_GPIO)
    ups = [c for c in bs.COMPONENTS if c.part.kind == "res" and not c.dnp
           and set(c.conns.values()) == {net, "+3V3"}]
    assert ups, f"GPIO{p4.BOOT_MODE_GPIO} ({net}) needs a pull-up to +3V3"


def test_nothing_pulls_gpio36_low():
    net = _net_of_gpio(p4.DOWNLOAD_QUALIFIER_GPIO)
    offenders = []
    for c in bs.COMPONENTS:
        if c.dnp or c.ref == "U1" or net not in c.conns.values():
            continue
        others = set(c.conns.values()) - {net}
        if GND in others or c.part.kind in ("fet", "sw"):
            offenders.append(c.ref)
    assert offenders == []
    ups = [c for c in bs.COMPONENTS if c.part.kind == "res" and not c.dnp
           and set(c.conns.values()) == {net, "+3V3"}]
    assert ups, "GPIO36 must read 1 for joint-download boot"


def test_strapping_pins_listed():
    assert {p4.BOOT_MODE_GPIO, p4.DOWNLOAD_QUALIFIER_GPIO} <= p4.STRAPPING_GPIOS


# ---------------------------------------------------------------------------
# Nets / routing
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("p,n,cls", bs.DIFF_PAIRS, ids=[f"{p}/{n}" for p, n, _ in bs.DIFF_PAIRS])
def test_diff_pair_nets_exist_and_share_netclass(p, n, cls):
    assert p in NETS, p
    assert n in NETS, n
    assert bs.netclass_of(p) == bs.netclass_of(n) == cls
    assert len(NETS[p]) >= 2 and len(NETS[n]) >= 2


def test_decoupling_near_u1_is_0402():
    near = [c for c in bs.COMPONENTS if isinstance(c.place, bs.Near) and c.place.ref == "U1"]
    caps = [c for c in near if c.part.kind == "cap"]
    assert caps
    assert [c.ref for c in near if c.part.package != "0402" or c.part.kind not in ("res", "cap")] == []
    assert [c.ref for c in near if c.part.height_mm > bs.KEEPOUT_MAX_HEIGHT_MM] == []
    assert [c.ref for c in caps if c.group == "SoC decoupling" and c.place.max_mm > 2.0] == []


@pytest.mark.parametrize("rule", imp.impedance_rules(), ids=lambda r: r.netclass)
def test_impedance_rule_within_2_percent(rule):
    assert abs(rule.achieved_ohm - rule.target_ohm) / rule.target_ohm <= 0.02
    z = (imp.diff_microstrip_z(rule.width_mm, rule.gap_mm) if rule.kind == "diff"
         else imp.microstrip_z0(rule.width_mm))
    assert abs(z - rule.target_ohm) / rule.target_ohm <= 0.02
    assert rule.width_mm >= MIN_TRACE_MM
    if rule.gap_mm is not None:
        assert rule.gap_mm >= MIN_TRACE_MM


def test_impedance_rules_cover_every_controlled_netclass():
    classes = {bs.netclass_of(n) for n in NETS} - {"Default", "POWER"}
    assert classes <= {r.netclass for r in imp.impedance_rules()}


def test_stackup_thickness():
    assert math.isclose(imp.BOARD_THICKNESS_MM, 1.6, abs_tol=0.01)
    assert sum(1 for layer in imp.STACKUP if layer.kind == "copper") == 4


# ---------------------------------------------------------------------------
# BOM
# ---------------------------------------------------------------------------

def test_bom_total_equals_fitted_non_mechanical_components():
    lines = bs.bom_lines()
    fitted = [c for c in bs.COMPONENTS if c.part.kind != "mech" and not c.dnp]
    assert sum(line["qty"] for line in lines if not line["dnp"]) == len(fitted)
    refs = [r for line in lines for r in line["refs"]]
    assert len(refs) == len(set(refs)), "designator on two BOM lines"
    assert set(refs) == {c.ref for c in bs.COMPONENTS if c.part.kind != "mech"}
    assert all(line["part"].mpn for line in lines), "BOM line without an MPN"


def _md_rows(text: str, section: str) -> list[list[str]]:
    body = text.split(section, 1)[1].split("\n## ", 1)[0]
    rows = []
    for line in body.splitlines():
        if not line.startswith("| ") or set(line.replace("|", "").strip()) <= set("-: "):
            continue
        cells = [c.strip().replace("\\|", "|").replace("\\~", "~")
                 for c in re.split(r"(?<!\\)\|", line)[1:-1]]
        rows.append(cells)
    return rows


def test_bom_csv_matches_markdown_tables():
    from tools import gen_docs
    csv_rows = list(csv.reader((ROOT / "hardware/output/bom_pcbway.csv").open(encoding="utf-8")))
    header, body = csv_rows[0], csv_rows[1:]
    assert header[:6] == ["Item #", "Designator", "Quantity", "Value/Name", "Package/Footprint",
                          "Manufacturer Part Number (MPN)"]
    assert header[6:10] == ["Manufacturer", "Description", "Mount (SMD/THT)", "LCSC #"]
    md = (ROOT / "docs/TASK4_bom_pcbway.md").read_text(encoding="utf-8")
    fitted = _md_rows(md, "## 1. Fitted parts")
    dnp = _md_rows(md, "## 2. Do Not Populate")
    assert fitted[0] == header and dnp[0] == header
    assert fitted[1:] + dnp[1:] == body
    assert sum(int(r[2]) for r in fitted[1:]) == len(
        [c for c in bs.COMPONENTS if c.part.kind != "mech" and not c.dnp])
    assert [r[0] for r in body] == [str(i) for i in range(1, len(body) + 1)]
    assert all("DO NOT POPULATE" in r[-1] for r in dnp[1:])
    assert gen_docs.render_bom_csv() == (ROOT / "hardware/output/bom_pcbway.csv").read_text(encoding="utf-8")


# ---------------------------------------------------------------------------
# Generated documents
# ---------------------------------------------------------------------------

def test_gen_docs_check_passes():
    r = subprocess.run([sys.executable, "-m", "tools.gen_docs", "--check"], cwd=ROOT,
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stdout + r.stderr


def test_gen_docs_is_deterministic():
    from tools import gen_docs
    assert gen_docs.outputs() == gen_docs.outputs()


def test_pinout_table_lists_every_gpio_in_order():
    md = (ROOT / "docs/TASK1_pinout.md").read_text(encoding="utf-8")
    rows = [r for r in _md_rows(md, "### 2.1 GPIO0-GPIO54") if r and re.fullmatch(r"GPIO\d+", r[0])]
    assert [r[0] for r in rows] == [f"GPIO{g}" for g in range(55)]
    for r in rows:
        g = int(r[0][4:])
        assert r[1] == p4.GPIO_PAD[g]
        if g in bs.GPIO_NC:
            assert r[3] == "-" and r[6] == "NC"
        else:
            assert r[3] == f"`{USE[g].net}`"
            assert r[6] == {"IO_MUX": "IO_MUX fixed"}.get(USE[g].mux, USE[g].mux)


# ---------------------------------------------------------------------------
# Mechanical
# ---------------------------------------------------------------------------

def test_no_placed_part_inside_heatsink_hole_keepout():
    bad = []
    for c in bs.COMPONENTS:
        if not isinstance(c.place, bs.Place) or c.part.kind == "mech":
            continue
        for h, (hx, hy) in bs.HEATSINK_HOLES.items():
            if math.hypot(c.place.x - hx, c.place.y - hy) < bs.HEATSINK_HOLE_KEEPOUT_R:
                bad.append((c.ref, h))
    assert bad == []
