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
