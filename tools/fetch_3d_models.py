"""Download the 3D models a KiCad board references, so renders show the parts.

KiCad's 3D models live in a separate library (kicad-packages3D) that most CI
images and minimal installs don't ship. Without them, ``kicad-cli pcb render``
shows only copper and silkscreen. This script reads every
``(model "${KICAD10_3DMODEL_DIR}/<Lib>.3dshapes/<name>.step")`` in the board,
downloads exactly those files from the official library at a pinned tag, and
prints the directory to export as ``KICAD10_3DMODEL_DIR``.

Usage::

    python -m tools.fetch_3d_models [board.kicad_pcb] [--dest DIR] [--ref 10.0.6]
    KICAD10_3DMODEL_DIR=<printed dir> kicad-cli pcb render ...
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

DEFAULT_BOARD = Path("hardware/output/esp32p4_extreme.kicad_pcb")
DEFAULT_REF = os.environ.get("KICAD_LIB_REF", "10.0.6")
DEFAULT_DEST = Path(os.environ.get("KICAD_3D_CACHE", Path.home() / ".cache" / "esp32p4-3dmodels"))
RAW = "https://gitlab.com/kicad/libraries/kicad-packages3D/-/raw/{ref}/{path}"
_MODEL = re.compile(r'\(model\s+"\$\{KICAD\d+_3DMODEL_DIR\}/([^"]+)"')

# Footprints whose referenced model is not in kicad-packages3D. Where a model of the
# same, centred package body exists it is downloaded and stored under the expected
# name, so renders show a body. Stand-ins are cosmetic only.
STAND_INS = {
    "Package_DFN_QFN.3dshapes/VQFN-16-1EP_3x3mm_P0.5mm_EP1.68x1.68mm.step":
        "Package_DFN_QFN.3dshapes/QFN-16-1EP_3x3mm_P0.5mm_EP1.7x1.7mm.step",
    "Package_DFN_QFN.3dshapes/VQFN-24-1EP_4x4mm_P0.5mm_EP2.5x2.5mm.step":
        "Package_DFN_QFN.3dshapes/QFN-24-1EP_4x4mm_P0.5mm_EP2.6x2.6mm.step",
    "Package_SON.3dshapes/WSON-8-1EP_8x6mm_P1.27mm_EP3.4x4.3mm.step":
        "Package_SON.3dshapes/WSON-8-1EP_6x5mm_P1.27mm_EP3.4x4.3mm.step",
    "Fuse.3dshapes/Fuse_1812_4532Metric.step":
        "Resistor_SMD.3dshapes/R_1812_4532Metric.step",
}
# No official model and no body-compatible stand-in (a different connector's model
# would sit at the wrong origin): rendered as footprint only.
NO_MODEL = frozenset({
    "Connector_RJ.3dshapes/RJ45_Hanrun_HR911105A_Horizontal.step",
    "Connector_FFC-FPC.3dshapes/Amphenol_F32Q-1A7x1-11022_1x22-1MP_P0.5mm_Horizontal.step",
})


def model_paths(board_text: str) -> list[str]:
    """Library-relative model paths referenced by a board (unique, sorted)."""
    return sorted(set(_MODEL.findall(board_text)))


def fetch(paths: list[str], dest: Path, ref: str) -> tuple[list[str], list[str]]:
    """Download missing models into dest; returns (fetched, failed)."""
    fetched, failed = [], []
    for rel in paths:
        out = dest / rel
        if rel in NO_MODEL or (out.exists() and out.stat().st_size > 0):
            continue
        url = RAW.format(ref=ref, path=urllib.parse.quote(STAND_INS.get(rel, rel)))
        try:
            with urllib.request.urlopen(urllib.request.Request(
                    url, headers={"User-Agent": "esp32p4-3d-fetch"}), timeout=120) as resp:
                data = resp.read()
        except urllib.error.URLError as exc:
            failed.append(f"{rel}: {exc}")
            continue
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(data)
        fetched.append(rel)
    return fetched, failed


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("board", nargs="?", type=Path, default=DEFAULT_BOARD)
    ap.add_argument("--dest", type=Path, default=DEFAULT_DEST, help="model cache root")
    ap.add_argument("--ref", default=DEFAULT_REF, help="kicad-packages3D tag (default %(default)s)")
    args = ap.parse_args(argv)
    paths = model_paths(args.board.read_text())
    dest = args.dest / args.ref
    fetched, failed = fetch(paths, dest, args.ref)
    skipped = [p for p in paths if p in NO_MODEL]
    print(f"{len(paths)} models referenced, {len(fetched)} downloaded, {len(failed)} failed, "
          f"{len(skipped)} without an official model (footprint only), "
          f"{sum(1 for p in paths if p in STAND_INS)} via a same-body stand-in", file=sys.stderr)
    for f in failed:
        print("  FAILED " + f, file=sys.stderr)
    print(dest)                       # stdout: the directory for KICAD10_3DMODEL_DIR
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
