"""Tests for the TASK 3 layout plan and the generated KiCad project.

* ``hardware/pcbnew/layout_plan.py`` -- the pure-Python placement plan
  (no KiCad needed): origin, holes, courtyards, keep-outs, Near distances,
  outline, thermal vias, zones, net classes, silkscreen, DRC rules.
* ``hardware/output/esp32p4_extreme.kicad_pcb`` -- the board written by
  ``write_kicad_pcb.py``, parsed back and checked with its *own* geometry.

Run from the repository root:  python3 -m pytest -q tests/test_layout_plan.py
"""

from __future__ import annotations

import json
import math
import re
import sys
from collections import Counter
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from hardware.lib import board_spec as bs  # noqa: E402
from hardware.lib import esp32p4_pinout as p4  # noqa: E402
from hardware.lib import impedance as imp  # noqa: E402
from hardware.pcbnew import layout_plan as lp  # noqa: E402

BOARD = ROOT / "hardware" / "output" / "esp32p4_extreme.kicad_pcb"
EPS = 1e-6


@pytest.fixture(scope="module")
def plan() -> lp.Plan:
    return lp.build_plan()


# ==========================================================================
# Conventions (checked against pcbnew 7.0.11 / 8.0.8 / 9.0.8 by the applier)
# ==========================================================================

def test_kicad_rotation_convention():
    # +90 deg turns local +X to screen-up (-Y) and local +Y to +X
    assert lp.rotate(1, 0, 90) == (0.0, -1.0)
    assert lp.rotate(0, 1, 90) == (1.0, 0.0)
    # bottom side = mirror local Y, then rotate
    assert lp.to_board(-3.2, -3.68, 0, 0, 0, "B") == (-3.2, 3.68)
    assert lp.to_board(-3.2, -3.68, 0, 0, 90, "B") == (3.68, 3.2)


def test_courtyard_json_covers_the_spec():
    geo = lp.load_geometry()
    missing = [f for f in lp.required_lib_ids() + [lp.U1_LIB_ID] if f not in geo]
    assert not missing, f"re-run `python3 -m hardware.pcbnew.layout_plan courtyards`: {missing}"


def test_every_spec_pad_exists_in_its_footprint():
    geo = lp.load_geometry()
    for comp in bs.COMPONENTS:
        mapping, notes = lp.pad_map(comp, geo[comp.part.footprint])
        lost = [n for n in notes if "dropped" in n]
        assert not lost, f"{comp.ref}: {lost}"
        assert set(comp.conns) <= set(mapping), comp.ref


def test_pad_remap_for_older_libraries():
    """KiCad 7/8/9 libraries call the USB-C shell 'S1' and the microSD shield '11'."""
    j1 = bs.component("J1")
    old = [n for n, _ in j1.part.pins if n != "SH"] + ["S1"]
    mapping, notes = lp.map_pads(j1, old)
    assert mapping["SH"] == ("S1",) and notes


# ==========================================================================
# Placement requirements
# ==========================================================================

def test_every_component_and_fiducial_is_placed_once(plan):
    want = {c.ref for c in bs.COMPONENTS} | set(bs.FIDUCIALS)
    assert set(plan.placements) == want


def test_u1_at_origin(plan):
    u1 = plan.placements["U1"]
    assert (u1.x, u1.y, u1.rot, u1.side) == (0.0, 0.0, 0.0, "F")
    assert u1.lib_id == "PCM_Espressif:ESP32-P4"


def test_heatsink_holes_exact(plan):
    want = {"H1": (-15.0, -15.0), "H2": (15.0, -15.0), "H3": (-15.0, 15.0), "H4": (15.0, 15.0)}
    assert bs.HEATSINK_HOLES == want
    for ref, (x, y) in {**want, **bs.BOARD_HOLES}.items():
        p = plan.placements[ref]
        assert (p.x, p.y, p.rot, p.side) == (x, y, 0.0, "F"), ref
        assert p.method == "fixed"


def test_no_courtyard_overlaps(plan):
    assert lp.check_overlaps(plan) == []


def test_heatsink_keepout_only_exempt_parts(plan):
    assert lp.check_keepout(plan) == []
    k = bs.KEEPOUT_HALF
    inside = [p.ref for p in plan.placements.values()
              if p.side == "F" and p.shape().overlaps_rect((-k, -k, k, k))]
    comps = {c.ref: c for c in bs.COMPONENTS}
    assert "U1" in inside
    assert all(r == "U1" or bs.is_keepout_exempt(comps[r]) for r in inside)
    for r in inside:        # the exempt parts are all low enough for the heatsink
        assert comps[r].part.height_mm <= bs.KEEPOUT_MAX_HEIGHT_MM or r == "U1"


def _near_distance(plan, comp) -> float:
    """Independent re-computation: target pad centre -> nearest pad centre of the part."""
    near = comp.place
    anchor = plan.placements[near.ref]
    acomp = bs.component(near.ref) if near.ref in {c.ref for c in bs.COMPONENTS} else None
    amap = lp.pad_map(acomp, anchor.geom)[0]
    if near.pad is not None:
        spec_pads = [near.pad]
    else:
        spec_pads = [n for n, net in acomp.conns.items() if net in set(comp.conns.values())]
    targets = [(x, y) for sp in spec_pads for fp in amap.get(sp, (sp,))
               for _, x, y in anchor.pad_positions(fp)]
    part = plan.placements[comp.ref]
    return min(math.hypot(px - tx, py - ty) for _, px, py in part.pad_positions() for tx, ty in targets)


def test_every_near_part_within_max_mm(plan):
    assert lp.check_near(plan) == []
    for comp in bs.COMPONENTS:
        if isinstance(comp.place, bs.Near):
            d = _near_distance(plan, comp)
            assert d <= comp.place.max_mm + EPS, f"{comp.ref}: {d:.3f} > {comp.place.max_mm}"
            assert abs(d - plan.placements[comp.ref].dist_mm) < 1e-3, comp.ref


def test_decoupling_caps_point_gnd_away_from_the_soc(plan):
    """Top-side SoC caps: the pad on the decoupled rail is the one nearest the SoC pad."""
    for comp in bs.COMPONENTS:
        p = plan.placements[comp.ref]
        if p.method != "ring-F" or comp.part.kind != "cap" or bs.GND not in comp.conns.values():
            continue
        u1 = plan.placements["U1"]
        tx, ty = u1.pad_positions(comp.place.pad)[0][1:]
        pads = {n: (x, y) for n, x, y in p.pad_positions()}
        gnd = next(n for n, net in comp.conns.items() if net == bs.GND)
        rail = next(n for n, net in comp.conns.items() if net != bs.GND)
        assert math.dist(pads[rail], (tx, ty)) < math.dist(pads[gnd], (tx, ty)), comp.ref


def test_nothing_in_the_hole_keepout_circles(plan):
    assert lp.check_holes(plan) == []


def test_everything_inside_the_board_outline(plan):
    assert lp.check_outline(plan) == []


def test_bottom_side_is_small_passives_off_the_via_field(plan):
    assert lp.check_bottom(plan) == []
    comps = {c.ref: c for c in bs.COMPONENTS}
    for p in plan.placements.values():
        if p.side == "B":
            part = comps[p.ref].part
            assert part.kind in ("res", "cap") and part.package in ("0402", "0603"), p.ref


def test_top_only_mode_never_violates_silently():
    """Without B.Cu the SoC ring cannot fit: every miss must be a logged WARNING."""
    top = lp.build_plan(allow_bottom=False)
    assert all(p.side == "F" for p in top.placements.values())
    missed = [c.ref for c in bs.COMPONENTS if isinstance(c.place, bs.Near)
              and top.placements[c.ref].dist_mm > c.place.max_mm + EPS]
    assert missed, "expected the top-only ring to overflow (see report)"
    for ref in missed:
        assert any(w.startswith(f"{ref}:") for w in top.warnings), ref
    assert lp.check_overlaps(top) == [] and lp.check_keepout(top) == []


# ==========================================================================
# Edge connectors
# ==========================================================================

KNOWN_MATING = {
    "Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal": "+Y",
    "Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5": "+Y",
    "Connector_FFC-FPC:Amphenol_F32Q-1A7x1-11022_1x22-1MP_P0.5mm_Horizontal": "+Y",
    "Connector_JST:JST_SH_SM02B-SRSS-TB_1x02-1MP_P1.00mm_Horizontal": "+Y",
    "Connector_RJ:RJ45_Hanrun_HR911105A_Horizontal": "+Y",
}


def test_connector_heuristic_on_known_footprints(plan):
    seen = {c.lib_id: c.direction for c in plan.connectors}
    for lib_id, want in KNOWN_MATING.items():
        if lib_id in seen:
            assert seen[lib_id] == want, lib_id


def test_edge_connectors_face_their_edge(plan):
    faces = {c.ref: c.place.faces for c in bs.COMPONENTS if isinstance(c.place, bs.Place) and c.place.faces}
    assert {c.ref for c in plan.connectors} == set(faces)
    for c in plan.connectors:
        p = plan.placements[c.ref]
        vx, vy = lp.rotate(*lp.DIRS[c.direction], p.rot)
        assert (round(vx), round(vy)) == lp.EDGE_VECTORS[faces[c.ref]], c.ref
        dev, _ = lp.Planner._edge_deviation(lp.Planner.__new__(lp.Planner), p, c.direction)
        assert -lp.EDGE_RECESS_MAX_MM - EPS <= dev <= lp.EDGE_OVERHANG_MAX_MM + EPS, c.ref


# ==========================================================================
# Vias, zones, rule areas, outline, net classes
# ==========================================================================

def test_49_thermal_vias_on_the_epad_grid(plan):
    tv = bs.THERMAL_VIA
    assert len(plan.vias) == tv["grid"] ** 2 == 49
    coords = sorted({v.x for v in plan.vias})
    assert coords == [(i - 3) * tv["pitch_mm"] for i in range(7)]
    assert sorted({(v.x, v.y) for v in plan.vias}) == sorted((x, y) for x in coords for y in coords)
    e = p4.EPAD_SIZE_MM / 2
    for v in plan.vias:
        assert (v.drill, v.diameter, v.net, v.kind) == (0.3, 0.6, "GND", "through")
        assert abs(v.x) + v.diameter / 2 <= e and abs(v.y) + v.diameter / 2 <= e


def test_zones(plan):
    z = {zone.name: zone for zone in plan.zones}
    assert z["GND_L2"].layers == ("In1.Cu",) and z["GND_L2"].net == "GND"
    assert z["3V3_L3"].layers == ("In2.Cu",) and z["3V3_L3"].net == "+3V3"
    island = z["VDD_HP_ISLAND"]
    assert island.net == "VDD_HP" and island.priority > z["3V3_L3"].priority
    assert z["GND_TOP"].layers == ("F.Cu",) and z["GND_BOTTOM"].layers == ("B.Cu",)
    spread = z["GND_HEAT_SPREADER"]
    xs = [p[0] for p in spread.polygon]
    assert max(xs) - min(xs) == pytest.approx(bs.BOTTOM_SPREADER_MM)
    assert spread.priority > z["GND_BOTTOM"].priority
    hs = z[lp.HEATSINK_AREA]
    assert hs.rule_area and hs.layers == ("F.Cu",) and not hs.keepout
    xs = [p[0] for p in hs.polygon]
    assert max(xs) - min(xs) == pytest.approx(2 * bs.KEEPOUT_HALF)
    for h, (hx, hy) in bs.HEATSINK_HOLES.items():
        ko = z[f"{h}_KEEPOUT"]
        assert ko.rule_area and set(ko.keepout) == {"tracks", "vias"} and len(ko.layers) == 4
        r = min(math.hypot(x - hx, y - hy) for x, y in ko.polygon)
        assert r >= bs.HEATSINK_HOLE_KEEPOUT_R      # the polygon contains the circle


def test_board_outline_and_mechanical_silhouettes(plan):
    edge = [g for g in plan.graphics if g.layer == "Edge.Cuts"]
    assert sum(g.kind == "arc" for g in edge) == 4 and sum(g.kind == "line" for g in edge) == 4
    for g in edge:
        for x, y in g.pts:
            assert lp.point_in_outline(x, y)
    layers = {g.layer for g in plan.graphics}
    assert {"User.1", "User.2", "B.Mask"} <= layers


def test_netclasses_follow_the_impedance_rules(plan):
    ncs = {n.name: n for n in plan.netclasses}
    assert set(ncs) == {"Default", "SE_50", "USB_90", "MIPI_100", "ETH_100", "POWER"}
    for rule in imp.impedance_rules():
        nc = ncs[rule.netclass]
        assert nc.track_width == rule.width_mm
        if rule.gap_mm:
            assert (nc.dp_width, nc.dp_gap) == (rule.width_mm, rule.gap_mm)
    assert (ncs["Default"].track_width, ncs["Default"].clearance) == (0.15, 0.10)
    assert ncs["POWER"].track_width == 0.5
    assert all(bs.netclass_of(n) == c for n, c in plan.net_assignments.items())
    r = lp.DESIGN_RULES
    assert (r["min_track"], r["min_clearance"], r["via_drill"], r["via_diameter"],
            r["uvia_drill"], r["uvia_diameter"]) == (0.09, 0.09, 0.2, 0.45, 0.1, 0.25)


# ==========================================================================
# Silkscreen
# ==========================================================================

def test_licence_text(plan):
    lic = [s for s in plan.silk if s.kind == "licence"]
    assert len(lic) == 1 and lic[0].text == lp.LICENSE_TEXT
    assert "CERN-OHL-P-2.0" in lic[0].text and 0.8 <= lic[0].size <= 1.0
    box = lp.Shape(lic[0].bbox)
    k = bs.KEEPOUT_HALF
    assert box.inside_outline() and not box.overlaps_rect((-k, -k, k, k))
    for p in plan.placements.values():
        if p.side == "F" or p.tht:
            assert not box.overlaps(p.shape()), p.ref


def test_j9_pin_labels(plan):
    labels = [s for s in plan.silk if s.kind == "legend:J9"]
    assert sorted(s.text for s in labels) == sorted(bs.LCD_HEADER_LABELS)
    assert all(s.size >= 0.8 for s in labels)
    j9 = plan.placements["J9"]
    for s in labels:     # outside the header, inside the board, clear of courtyards
        box = lp.Shape(s.bbox)
        assert box.inside_outline() and not box.overlaps(j9.shape())
    rows = Counter(round(s.y, 3) for s in labels)
    assert len(rows) == 2
    for y in rows:       # labels in one line keep >= 0.1 mm of air between them
        line = sorted((s.bbox[0], s.bbox[2]) for s in labels if round(s.y, 3) == y)
        assert all(b[0] - a[1] >= 0.1 for a, b in zip(line, line[1:]))


# ==========================================================================
# Custom DRC rules
# ==========================================================================

def test_dru_is_generated_and_up_to_date(plan):
    text = lp.dru_text(plan)
    assert text.startswith("(version 1)")
    body = "\n".join(line for line in text.splitlines() if not line.lstrip().startswith("#"))
    assert body.count("(") == body.count(")")
    names = re.findall(r'\(rule "([^"]+)"', text)
    assert len(names) == len(set(names))
    for must in ("heatsink_keepout_footprints", "thermal_via_size", "m25_hole_clearance",
                 "mipi_100_diff_pair", "skew_csi_d0_p", "sdio_card_side_matching"):
        assert must in names
    exempt = lp.keepout_exempt_refs()
    assert "U1" in exempt and all(f"A.Reference != '{r}'" in text for r in exempt)
    assert lp.DRU_FILE.read_text() == text, "run `python3 -m hardware.pcbnew.layout_plan dru`"


# Constraint forms verified with kicad-cli 9.0.8 / 10.0.6 (a single malformed constraint
# makes KiCad silently ignore *every* custom rule, e.g. "min_resolved_spokes (min 1)").
DRU_FORMS = {
    "disallow": r"disallow footprint",
    "clearance": r"clearance \(min [\d.]+mm\)",
    "hole_clearance": r"hole_clearance \(min [\d.]+mm\)",
    "hole_to_hole": r"hole_to_hole \(min [\d.]+mm\)",
    "hole_size": r"hole_size \(min [\d.]+mm\) \(max [\d.]+mm\)",
    "via_diameter": r"via_diameter \(min [\d.]+mm\)",
    "track_width": r"track_width \(min [\d.]+mm\)( \(opt [\d.]+mm\))?",
    "diff_pair_gap": r"diff_pair_gap \(min [\d.]+mm\) \(opt [\d.]+mm\) \(max [\d.]+mm\)",
    "diff_pair_uncoupled": r"diff_pair_uncoupled \(max [\d.]+mm\)",
    "skew": r"skew \(max [\d.]+mm\)",
    "min_resolved_spokes": r"min_resolved_spokes \d+",
    "zone_connection": r"zone_connection (solid|thermal_reliefs|none)",
}


def test_dru_constraints_use_verified_forms(plan):
    for c in re.findall(r"^\s*\(constraint (.*)\)\s*$", lp.dru_text(plan), re.M):
        kind = c.split()[0]
        assert kind in DRU_FORMS, f"unverified constraint: {c}"
        assert re.fullmatch(DRU_FORMS[kind], c), f"malformed constraint: {c}"
    for cond in re.findall(r'\(condition "([^"]*)"\)', lp.dru_text(plan)):
        assert cond.count("(") == cond.count(")") and "''" not in cond


# ==========================================================================
# The written KiCad project (parsed back, own geometry only)
# ==========================================================================

def _sexpr(text: str) -> list:
    return lp.parse_sexpr(text)


def _kids(node, key):
    return [c for c in node[1:] if isinstance(c, list) and c and c[0] == key]


def _kid(node, key):
    k = _kids(node, key)
    return k[0] if k else None


@pytest.fixture(scope="module")
def board_tree():
    if not BOARD.exists():
        pytest.skip("hardware/output/esp32p4_extreme.kicad_pcb not generated")
    text = BOARD.read_text()
    depth = 0
    for ch in re.sub(r'"(?:[^"\\]|\\.)*"', '""', text):
        depth += (ch == "(") - (ch == ")")
        assert depth >= 0
    assert depth == 0, "unbalanced s-expression"
    return _sexpr(text)


def _footprints(tree):
    out = {}
    for fp in _kids(tree, "footprint"):
        ref = next(p[2] for p in _kids(fp, "property") if p[1] == "Reference")
        assert ref not in out, f"duplicate {ref}"
        out[ref] = fp
    return out


def _fp_frame(fp):
    at = _kid(fp, "at")
    x, y = float(at[1]) - lp.PAGE_ORIGIN_MM[0], float(at[2]) - lp.PAGE_ORIGIN_MM[1]
    return x, y, float(at[3]) if len(at) > 3 else 0.0, _kid(fp, "layer")[1][0]


def test_board_header(board_tree):
    assert board_tree[0] == "kicad_pcb"
    assert _kid(board_tree, "version")[1] == "20241229"
    assert float(_kid(_kid(board_tree, "general"), "thickness")[1]) == 1.6
    layers = {row[1]: row[3] for row in _kid(board_tree, "layers")[1:] if len(row) > 3}
    assert [layers[k] for k in ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu")] == \
        ["Signal_Top", "GND", "VCC_3V3", "Signal_Bottom"]
    setup = _kid(board_tree, "setup")
    for key in ("aux_axis_origin", "grid_origin"):
        assert tuple(map(float, _kid(setup, key)[1:])) == lp.PAGE_ORIGIN_MM
    comments = " ".join(c[2] for c in _kids(_kid(board_tree, "title_block"), "comment"))
    assert "UNROUTED" in comments
    assert not _kids(board_tree, "segment") and not _kids(board_tree, "arc")


def test_board_footprints_at_plan(board_tree, plan):
    fps = _footprints(board_tree)
    assert set(fps) == set(plan.placements)
    for ref, fp in fps.items():
        p = plan.placements[ref]
        x, y, o, side = _fp_frame(fp)
        assert (round(x, 6), round(y, 6), side) == (round(p.x, 6), round(p.y, 6), p.side), ref
        assert round((o - p.rot) % 360, 6) in (0, 360), ref
        assert fp[1] == p.lib_id


def test_board_pads_carry_the_right_nets(board_tree):
    nets = {int(n[1]): n[2] for n in _kids(board_tree, "net")}
    fps = _footprints(board_tree)
    checked = 0
    for comp in bs.COMPONENTS:
        fp = fps[comp.ref]
        pads = _kids(fp, "pad")
        mapping, _ = lp.map_pads(comp, [p[1] for p in pads])
        want = {fpn: net for spec, net in comp.conns.items() for fpn in mapping.get(spec, ())}
        for pad in pads:
            net = _kid(pad, "net")
            if pad[1] in want:
                assert net is not None and net[2] == want[pad[1]], f"{comp.ref}.{pad[1]}"
                assert nets[int(net[1])] == net[2]
                checked += 1
            elif pad[1]:
                assert net is None, f"{comp.ref}.{pad[1]} unexpected net"
    assert checked >= sum(len(c.conns) for c in bs.COMPONENTS)


def _courtyard(fp):
    """Board-space bounding box of the footprint's own courtyard graphics."""
    x, y, o, side = _fp_frame(fp)
    pts = []
    for kind in ("fp_rect", "fp_line", "fp_circle", "fp_poly", "fp_arc"):
        for g in _kids(fp, kind):
            if _kid(g, "layer")[1] not in ("F.CrtYd", "B.CrtYd"):
                continue
            local = lp._graphic_points(g)
            if kind == "fp_circle":
                (cx, cy), (ex, ey) = lp._xy(_kid(g, "center")), lp._xy(_kid(g, "end"))
                r = math.hypot(ex - cx, ey - cy)
                local = [(cx + r * math.cos(a / 8 * math.pi), cy + r * math.sin(a / 8 * math.pi))
                         for a in range(16)]
            # children of a bottom-side footprint are stored already mirrored: rotate only
            pts += [lp.to_board(px, py, x, y, o, "F") for px, py in local]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys)), side, any(
        _kid(pad, "drill") for pad in _kids(fp, "pad") if pad[2] == "thru_hole")


def test_board_courtyards_do_not_overlap(board_tree):
    boxes = {ref: _courtyard(fp) for ref, fp in _footprints(board_tree).items()}
    refs = sorted(boxes)
    bad = []
    for i, a in enumerate(refs):
        ra, sa, ta = boxes[a]
        for b in refs[i + 1:]:
            rb, sb, tb = boxes[b]
            if (sa == sb or ta or tb) and lp.rects_overlap(ra, rb):
                # circle courtyards (holes, fiducials) are checked exactly by the plan tests
                if not (a.startswith(("H", "FID")) or b.startswith(("H", "FID"))):
                    bad.append((a, b))
    assert not bad


def test_board_copper_inside_the_outline(board_tree):
    edge = [g for g in board_tree if isinstance(g, list) and g[0] in ("gr_line", "gr_arc")
            and _kid(g, "layer")[1] == "Edge.Cuts"]
    xs = [float(_kid(g, k)[1]) for g in edge for k in ("start", "end")]
    ys = [float(_kid(g, k)[2]) for g in edge for k in ("start", "end")]
    ox, oy = lp.PAGE_ORIGIN_MM
    assert (min(xs) - ox, min(ys) - oy, max(xs) - ox, max(ys) - oy) == bs.BOARD_OUTLINE
    for ref, fp in _footprints(board_tree).items():
        x, y, o, _ = _fp_frame(fp)
        for pad in _kids(fp, "pad"):
            if pad[2] == "np_thru_hole" or not any(la.endswith("Cu") for la in _kid(pad, "layers")[1:]):
                continue
            px, py = lp._xy(_kid(pad, "at"))
            bx, by = lp.to_board(px, py, x, y, o, "F")
            assert lp.point_in_outline(bx, by), f"{ref}.{pad[1]}"


def test_board_vias_zones_and_silk(board_tree, plan):
    nets = {int(n[1]): n[2] for n in _kids(board_tree, "net")}
    vias = _kids(board_tree, "via")
    assert len(vias) == 49 and all(nets[int(_kid(v, "net")[1])] == "GND" for v in vias)
    assert all((_kid(v, "drill")[1], _kid(v, "size")[1]) == ("0.3", "0.6") for v in vias)
    names = {_kid(z, "name")[1] for z in _kids(board_tree, "zone")}
    assert {z.name for z in plan.zones} == names
    texts = [t[1] for t in _kids(board_tree, "gr_text")]
    assert lp.LICENSE_TEXT in texts and all(lab in texts for lab in bs.LCD_HEADER_LABELS)


def test_project_and_rules_next_to_the_board(board_tree, plan):
    pro = json.loads(BOARD.with_suffix(".kicad_pro").read_text())
    classes = {c["name"]: c for c in pro["net_settings"]["classes"]}
    assert set(classes) == {n.name for n in plan.netclasses}
    patterns = {p["pattern"]: p["netclass"] for p in pro["net_settings"]["netclass_patterns"]}
    assert patterns == plan.net_assignments
    assert pro["board"]["design_settings"]["rules"]["min_track_width"] == 0.09
    assert BOARD.with_suffix(".kicad_dru").read_text() == lp.dru_text(plan)


def test_board_file_is_up_to_date(tmp_path, plan):
    """Regenerate from the footprint cache (skipped when the cache is not populated)."""
    from hardware.pcbnew import write_kicad_pcb as w
    missing = [f for f in lp.required_lib_ids()
               if not lp.cached_footprint_path(lp.DEFAULT_CACHE, lp.LIBRARY_TAG, f).exists()]
    if missing or not BOARD.exists():
        pytest.skip(f"footprint cache {lp.DEFAULT_CACHE}/{lp.LIBRARY_TAG} incomplete")
    out = tmp_path / BOARD.name
    w.write(out, lp.DEFAULT_CACHE, offline=True)
    assert out.read_text() == BOARD.read_text(), "run `python3 -m hardware.pcbnew.write_kicad_pcb`"
