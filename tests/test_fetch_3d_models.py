"""Tests for tools.fetch_3d_models (offline: parsing and stand-in bookkeeping)."""

from pathlib import Path

from tools import fetch_3d_models as f3d

BOARD = Path(__file__).resolve().parents[1] / "hardware" / "output" / "esp32p4_extreme.kicad_pcb"


def test_model_paths_parses_kicad_env_var_paths():
    text = ('(model "${KICAD10_3DMODEL_DIR}/Resistor_SMD.3dshapes/R_0402_1005Metric.step"\n'
            '(model "${KICAD9_3DMODEL_DIR}/Crystal.3dshapes/X.step"\n'
            '(model "${KICAD10_3DMODEL_DIR}/Resistor_SMD.3dshapes/R_0402_1005Metric.step"')
    assert f3d.model_paths(text) == ["Crystal.3dshapes/X.step",
                                     "Resistor_SMD.3dshapes/R_0402_1005Metric.step"]


def test_every_board_model_is_fetchable_or_explicitly_modelless():
    paths = f3d.model_paths(BOARD.read_text())
    assert len(paths) >= 30
    # stand-ins and model-less entries must refer to models the board actually uses
    assert set(f3d.STAND_INS) <= set(paths)
    assert set(f3d.NO_MODEL) <= set(paths)
    assert not set(f3d.STAND_INS) & set(f3d.NO_MODEL)


def test_soc_has_a_representative_body():
    assert any("ArtInChip_QFN-88-1EP_10x10mm" in p for p in f3d.model_paths(BOARD.read_text()))
