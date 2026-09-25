"""The three USB-C ports, checked end to end on the design data.

No board exists yet, so this is what can be proven before a bring-up: every USB-C
receptacle enumerates as a device in either plug orientation (Rd on both CC pins,
D+/D- paralleled across the A/B rows), keeps D+ and D- the right way round all the
way to the silicon, has ESD protection on its data pair, feeds VSYS through its own
Schottky, and its data pair is routed as a 90 ohm pair.
"""

import pytest

from hardware.lib import board_spec as bs
from hardware.lib import esp32p4_pinout as p4

COMPS = {c.ref: c for c in bs.COMPONENTS}
NETS = bs.nets()

# receptacle -> (D+ net at the connector, what the pair must reach, VBUS net)
PORTS = {
    "J1": ("USB1_D_P", "SoC USB 2.0 HS PHY", "VBUS1"),
    "J2": ("USB2_D_P", "CP2102N USB-UART", "VBUS2"),
    "J3": ("USB3_D_P", "SoC USB-Serial-JTAG", "VBUS3"),
}


def pins_of(ref: str) -> dict[str, str]:
    return dict(COMPS[ref].part.pins)


def through_series_r(net: str) -> str:
    """The net on the far side of the single 2-pad resistor in series with ``net``."""
    series = [r for r, _ in NETS[net] if COMPS[r].part.kind == "res"
              and bs.GND not in COMPS[r].conns.values()]
    assert len(series) == 1, (net, series)
    other = [n for n in COMPS[series[0]].conns.values() if n != net]
    return other[0]


@pytest.mark.parametrize("ref", PORTS)
def test_receptacle_works_in_both_orientations(ref):
    c = COMPS[ref].conns
    assert c["A6"] == c["B6"] and c["A7"] == c["B7"], "D+/D- must be paralleled across rows"
    assert c["A6"].endswith("_D_P") and c["A7"].endswith("_D_N")
    vbus = PORTS[ref][2]
    assert {c[p] for p in ("A4", "A9", "B4", "B9")} == {vbus}
    assert {c[p] for p in ("A1", "A12", "B1", "B12", "SH")} == {bs.GND}


@pytest.mark.parametrize("ref", PORTS)
def test_each_cc_pin_has_its_own_5k1_pull_down(ref):
    """Rd = 5.1 kOhm on CC1 and on CC2 separately: a USB-C host/charger then turns
    VBUS on and sees a device in either orientation (a shared resistor would not)."""
    rs = []
    for pin in ("A5", "B5"):
        net = COMPS[ref].conns[pin]
        parts = [r for r, _ in NETS[net] if r != ref]
        assert len(parts) == 1, (net, parts)
        r = COMPS[parts[0]]
        assert r.part.kind == "res" and r.value == "5.1k", r.ref
        assert set(r.conns.values()) == {net, bs.GND}
        rs.append(r.ref)
    assert rs[0] != rs[1]


def test_j1_reaches_the_high_speed_phy_with_polarity():
    dp = through_series_r("USB1_D_P")
    dn = through_series_r("USB1_D_N")
    u1 = COMPS["U1"].conns
    assert p4.PAD_NAME[next(p for p, n in u1.items() if n == dp)] == "USB-DP"
    assert p4.PAD_NAME[next(p for p, n in u1.items() if n == dn)] == "USB-DM"


def test_j3_reaches_usb_serial_jtag_with_polarity():
    """USB-Serial-JTAG is fixed in IO_MUX: D- = GPIO24, D+ = GPIO25 (22 ohm series)."""
    u1 = COMPS["U1"].conns
    for conn_net, gpio in (("USB3_D_P", 25), ("USB3_D_N", 24)):
        soc_net = through_series_r(conn_net)
        assert u1[p4.GPIO_PAD[gpio]] == soc_net
        r = next(r for r, _ in NETS[conn_net] if COMPS[r].part.kind == "res")
        assert COMPS[r].value == "22"


def test_j2_reaches_the_cp2102n_with_polarity_and_crossed_uart():
    cp = COMPS["U6"].conns
    names = pins_of("U6")
    by_name = {names[p]: n for p, n in cp.items()}
    assert by_name["D+"] == "USB2_D_P" and by_name["D-"] == "USB2_D_N"
    u1 = COMPS["U1"].conns
    assert by_name["TXD"] == u1[p4.GPIO_PAD[38]], "CP2102N TXD -> SoC U0RXD (GPIO38)"
    assert by_name["RXD"] == u1[p4.GPIO_PAD[37]], "SoC U0TXD (GPIO37) -> CP2102N RXD"
    # self-powered 3.3 V: VREGIN, VDD and VIO on +3V3, VBUS through the SiLabs divider
    assert by_name["VREGIN"] == by_name["VDD"] == by_name["VIO"] == "+3V3"
    sense = by_name["VBUS"]
    divider = [COMPS[r] for r, _ in NETS[sense] if COMPS[r].part.kind == "res"]
    top = next(r for r in divider if "VBUS2" in r.conns.values())
    bot = next(r for r in divider if bs.GND in r.conns.values())
    assert (top.value, bot.value) == ("22.1k", "47.5k")    # 5 V -> 3.4 V


def test_cp2102n_auto_program_circuit():
    """Espressif's cross-coupled pair: DTR/RTS drive EN (CHIP_PU) and the boot strap."""
    q = COMPS["Q1"].conns
    names = pins_of("Q1")
    by_name = {names[p]: n for p, n in q.items()}
    assert by_name["C1"] == "CHIP_PU" and by_name["E1"] == "CP_RTS"
    assert by_name["E2"] == "CP_DTR"
    boot = by_name["C2"]
    # the boot collector reaches the boot-mode strap GPIO35 (through 1k, shared with BOOT)
    strap_net = COMPS["U1"].conns[p4.GPIO_PAD[p4.BOOT_MODE_GPIO]]
    assert through_series_r(boot) == strap_net
    for base, drive in (("B1", "CP_DTR"), ("B2", "CP_RTS")):
        r = next(COMPS[x] for x, _ in NETS[by_name[base]] if x != "Q1")
        assert r.value == "10k" and drive in r.conns.values()


@pytest.mark.parametrize("ref", PORTS)
def test_data_pair_has_esd_protection(ref):
    dp = COMPS[ref].conns["A6"]
    dn = COMPS[ref].conns["A7"]
    esd = {r for r, _ in NETS[dp]} & {r for r, _ in NETS[dn]} - {ref, "U6"}
    assert esd, f"{ref}: no ESD part on both data lines"
    assert any(COMPS[r].part.mpn in ("TPD2EUSB30DRTR", "USBLC6-2SC6") for r in esd)
    for r in esd:
        assert bs.GND in COMPS[r].conns.values()


@pytest.mark.parametrize("ref", PORTS)
def test_vbus_feeds_vsys_through_its_own_schottky(ref):
    vbus = PORTS[ref][2]
    diodes = [r for r, _ in NETS[vbus] if [n for _, n in COMPS[r].part.pins] == ["K", "A"]]
    assert len(diodes) == 1, diodes
    d = COMPS[diodes[0]]
    names = dict(d.part.pins)
    by_name = {names[p]: n for p, n in d.conns.items()}
    assert by_name["A"] == vbus and by_name["K"] == "VSYS_OR"


@pytest.mark.parametrize("ref", PORTS)
def test_data_pair_is_a_90_ohm_differential_pair(ref):
    dp, dn = COMPS[ref].conns["A6"], COMPS[ref].conns["A7"]
    assert bs.netclass_of(dp) == bs.netclass_of(dn) == "USB_90"
    assert (dp, dn, "USB_90") in bs.DIFF_PAIRS
