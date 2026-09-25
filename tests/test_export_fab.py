"""Tests for tools.export_fab's reproducibility check (no KiCad needed)."""

from tools import export_fab as fab

GERBER = """%TF.GenerationSoftware,KiCad,Pcbnew,{ver}*%
%TF.CreationDate,{date}*%
%TF.FileFunction,Copper,L1,Top*%
G04 Created by KiCad (PCBNEW {ver}) date {date}*
X1000Y2000D02*
X{x}Y2000D01*
"""


def write(root, name, **kw):
    path = root / "gerbers" / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(GERBER.format(**kw))


def test_dates_and_build_strings_are_ignored(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    write(a, "x-F_Cu.gtl", ver="10.0.6+dfsg-1", date="2026-09-24T14:04:47", x=3000)
    write(b, "x-F_Cu.gtl", ver="10.0.6", date="2027-01-01T00:00:00", x=3000)
    assert fab.compare(a, b) == []


def test_changed_copper_is_reported(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    write(a, "x-F_Cu.gtl", ver="10.0.6", date="d", x=3000)
    write(b, "x-F_Cu.gtl", ver="10.0.6", date="d", x=3100)
    problems = fab.compare(a, b)
    assert len(problems) == 1 and problems[0].startswith("gerbers/x-F_Cu.gtl")
    assert "X3100Y2000D01" in problems[0]


def test_missing_file_is_reported(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    write(a, "x-F_Cu.gtl", ver="v", date="d", x=1)
    write(a, "x-B_Cu.gbl", ver="v", date="d", x=1)
    write(b, "x-F_Cu.gtl", ver="v", date="d", x=1)
    assert fab.compare(a, b) == ["gerbers/x-B_Cu.gbl: only in committed package"]
