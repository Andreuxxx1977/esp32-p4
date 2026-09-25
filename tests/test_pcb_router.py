"""The board router (tools/pcb_router.py) on small synthetic layouts: exact clearances,
octilinear tracks, coupled differential pairs, rip-up."""

import math
import shutil

import pytest

from tools import pcb_router as r

pytestmark = pytest.mark.skipif(shutil.which("cc") is None and shutil.which("gcc") is None,
                                reason="the router core needs a C compiler")

# a spot on the board away from the SoC breakout area and the heatsink holes
X0, Y0 = -40.0, -25.0


def pad(x, y, net, w=0.6, h=0.6, layers=r.TOP):
    return r.Item(0, (X0 + x, Y0 + y, w / 2, h / 2, 0.0, 0.0), layers, net, "smd")


def routed_tracks(router, net):
    return [it for k, it in enumerate(router.cu.items)
            if router.cu.alive[k] and not it.fixed and it.net == net and it.kind == 2]


def min_gap(router, net):
    """Smallest copper-to-copper distance from the routed tracks of ``net`` to any other net."""
    cu = router.cu
    worst = math.inf
    for t in routed_tracks(router, net):
        (xa, ya, xb, yb, rr) = t.par[:5]
        n = max(1, int(math.dist((xa, ya), (xb, yb)) / 0.01))
        for s in range(n + 1):
            x, y = xa + (xb - xa) * s / n, ya + (yb - ya) * s / n
            for k, it in enumerate(cu.items):
                if cu.alive[k] and it.net != net and it.what != "keepout" and (it.layers & t.layers):
                    worst = min(worst, it.dist(x, y) - rr)
    return worst


def test_offset_polyline_keeps_the_pair_spacing():
    line = [(0.0, 0.0), (2.0, 0.0), (3.0, 1.0), (3.0, 3.0)]      # two 45-degree bends
    left, right = r.offset_polyline(line, 0.17)
    # every vertex of one offset is at least 2h from the other offset line
    def seg_dist(p, a, b):
        vx, vy = b[0] - a[0], b[1] - a[1]
        u = max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / (vx * vx + vy * vy)))
        return math.dist(p, (a[0] + u * vx, a[1] + u * vy))
    for p in left:
        assert min(seg_dist(p, a, b) for a, b in zip(right, right[1:])) >= 2 * 0.17 - 1e-9
    assert r.offset_polyline([(0, 0), (1, 0), (0, 0.1)], 0.1) is None      # acute turn refused


def test_routes_around_an_obstacle_with_netclass_clearance():
    items = [pad(0, 0, "GPIO1"), pad(6, 0, "GPIO1"), pad(3, 0, "GPIO2", w=1.0, h=3.0)]
    router = r.Router(items, log=lambda *a: None)
    router.run()
    assert not router.failed
    tracks = routed_tracks(router, "GPIO1")
    assert tracks, "no track routed"
    for t in tracks:                      # octilinear: 0, 45 or 90 degrees
        dx, dy = t.par[2] - t.par[0], t.par[3] - t.par[1]
        assert abs(dx) < 1e-9 or abs(dy) < 1e-9 or abs(abs(dx) - abs(dy)) < 1e-9
    assert min_gap(router, "GPIO1") >= r.clearance("GPIO1") - 1e-6


def test_power_net_gets_the_widest_track_that_fits():
    items = [pad(0, 0, "VSYS_5V", 1.0, 1.0), pad(8, 0, "VSYS_5V", 1.0, 1.0)]
    router = r.Router(items, log=lambda *a: None)
    router.run()
    widths = {round(t.width, 3) for t in routed_tracks(router, "VSYS_5V")}
    assert widths == {max(r.widths("VSYS_5V"))}


def test_plane_pad_gets_its_own_via():
    items = [pad(0, 0, "GND", 0.6, 0.6), pad(1.2, 0, "GPIO3")]
    router = r.Router(items, log=lambda *a: None)
    router.run()
    vias = [it for k, it in enumerate(router.cu.items)
            if router.cu.alive[k] and not it.fixed and it.what == "micro" and it.net == "GND"]
    assert len(vias) == 1 and "1" in vias[0].planes          # laser microvia F.Cu -> In1.Cu (GND)


def test_differential_pair_is_routed_coupled_at_its_gap():
    p, n = "ETH_TX_P", "ETH_TX_N"
    w, gap = r.PAIR_CLASSES["ETH_100"]
    items = [pad(0, 0, p, 0.3, 0.3), pad(0, 0.8, n, 0.3, 0.3),
             pad(14, 5, p, 0.3, 0.3), pad(14, 5.8, n, 0.3, 0.3)]
    router = r.Router(items, log=lambda *a: None)
    router.run()
    assert not router.failed
    cp = next(c for c in router.conns if c.net == p)
    assert cp.note.startswith("coupled"), cp.note
    # the long straight runs of P and N are parallel at centre distance width + gap
    tp = max(routed_tracks(router, p), key=lambda t: math.dist(t.par[:2], t.par[2:4]))
    mid = ((tp.par[0] + tp.par[2]) / 2, (tp.par[1] + tp.par[3]) / 2)
    d = min(t.dist(*mid) for t in routed_tracks(router, n)) + w / 2
    assert abs(d - (w + r.pair_gap(p))) < 0.01
    assert gap <= r.pair_gap(p) <= gap + 0.01


def test_supply_uses_in2_when_the_outer_layers_are_walled_off():
    """A supply connection boxed in on F.Cu and B.Cu by other nets' copper goes down to
    In2.Cu (the power layer, Espressif's place for power traces) and back up."""
    wall = [r.Item(0, (X0 + 3, Y0, 0.3, 4.0, 0.0, 0.0), r.TOP | r.BOT, "GPIO9", "smd")]
    items = [pad(0, 0, "VDDO_FLASH", 0.6, 0.6, layers=r.TOP | r.BOT | r.IN2),
             pad(6, 0, "VDDO_FLASH", 0.6, 0.6, layers=r.TOP | r.BOT | r.IN2)] + wall
    items[0].what = items[1].what = "tht"
    router = r.Router(items, log=lambda *a: None)
    router.run()
    assert not router.failed
    layers = {t.layers for t in routed_tracks(router, "VDDO_FLASH")}
    assert r.IN2 in layers


def test_dangling_escape_end_is_cut_back_to_the_junction():
    """A route that joins a pre-routed escape track part-way along it leaves the rest of
    the escape as a dead stub; the trim cuts it back to the junction (KiCad's rule)."""
    def t(x0, y0, x1, y1, fixed):
        return r.Item(2, (X0 + x0, Y0 + y0, X0 + x1, Y0 + y1, 0.075), r.TOP, "GPIO5", "track",
                      fixed=fixed, width=0.15)
    escape = t(0, 0, 3, 0, True)                       # runs 1 mm past the junction at x = 2
    items = [pad(0, 0, "GPIO5"), pad(4, 2, "GPIO5"), escape, t(2, 0, 2, 2, False), t(2, 2, 4, 2, False)]
    cu = r.Copper(items)
    undo = r.trim_stubs(cu, set(), {})
    assert undo is not None and undo.count == 1
    tracks = [cu.items[k] for k in range(len(cu.items)) if cu.alive[k] and cu.items[k].kind == 2]
    cut = [it for it in tracks if it.rewrite]
    assert len(cut) == 1 and cut[0].par[:4] == pytest.approx((X0, Y0, X0 + 2, Y0))
    assert len(cu.dropped) == 1                        # the pre-routed original leaves the file
    assert r.trim_stubs(cu, set(), {}) is None         # nothing else dangles
    undo()
    assert not cu.dropped and cu.alive[items.index(escape)]
