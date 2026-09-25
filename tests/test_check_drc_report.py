"""Tests for tools.check_drc_report (the CI gate on KiCad's DRC JSON)."""

import json

from tools import check_drc_report as drc

REPORT = {
    "kicad_version": "10.0.6",
    "source": "esp32p4_extreme.kicad_pcb",
    "violations": [
        {"type": "silk_overlap", "severity": "warning", "description": "silk over pad"},
    ],
    "unconnected_items": [{"type": "unconnected_items", "severity": "error"}] * 3,
    "schematic_parity": [],
}


def test_unrouted_board_passes_in_placement_phase():
    lines, blocking = drc.summarize(REPORT)
    assert blocking == []
    assert "unconnected items: 3" in lines[1]


def test_strict_mode_fails_on_unconnected_items():
    _, blocking = drc.summarize(REPORT, strict=True)
    assert blocking == ["3 unconnected items (board not fully routed)"]


def test_real_error_fails_even_in_placement_phase():
    bad = dict(REPORT, violations=REPORT["violations"] + [
        {"type": "courtyards_overlap", "severity": "error", "description": "U5 / Y2"}])
    _, blocking = drc.summarize(bad)
    assert blocking == ["error courtyards_overlap: U5 / Y2"]


def test_cli_exit_codes(tmp_path):
    path = tmp_path / "drc.json"
    path.write_text(json.dumps(REPORT))
    assert drc.main([str(path)]) == 0
    assert drc.main([str(path), "--strict"]) == 1


ROUTED = {
    "kicad_version": "10.0.6",
    "source": "routed.kicad_pcb",
    "violations": [
        {"type": "diff_pair_uncoupled_length_too_long", "severity": "error",
         "description": "uncoupled 4.1 mm",
         "items": [{"description": "Track [DSI_D0_P] on F.Cu"},
                   {"description": "Track [DSI_D0_N] on F.Cu"}]},
        {"type": "skew_out_of_range", "severity": "error", "description": "skew 1.2 mm",
         "items": [{"description": "Track [USB1_D_P] on B.Cu"}]},
    ],
    "unconnected_items": [],
    "schematic_parity": [],
}


def test_routed_mode_lists_si_rule_misses_without_failing(capsys, tmp_path):
    path = tmp_path / "drc.json"
    path.write_text(json.dumps(ROUTED))
    assert drc.main([str(path), "--routed"]) == 0
    out = capsys.readouterr().out
    assert "diff_pair_uncoupled_length_too_long: DSI_D0_N, DSI_D0_P (1x)" in out
    assert "skew_out_of_range: USB1_D_P (1x)" in out
    assert drc.main([str(path), "--strict"]) == 1


def test_routed_mode_still_fails_on_unconnected_and_clearance(tmp_path):
    bad = dict(ROUTED, unconnected_items=[{"type": "unconnected_items"}],
               violations=ROUTED["violations"] + [
                   {"type": "clearance", "severity": "error", "description": "0.05 mm"}])
    path = tmp_path / "drc.json"
    path.write_text(json.dumps(bad))
    assert drc.main([str(path), "--routed"]) == 1
    _, blocking = drc.summarize(bad, False, drc.SI_RULE_TYPES)
    assert blocking == ["error clearance: 0.05 mm", "1 unconnected items (board not fully routed)"]
