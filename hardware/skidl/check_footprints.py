"""Cross-check every spec connection against the *real* KiCad footprint pads.

For each distinct footprint in ``board_spec.PARTS`` this downloads the
``.kicad_mod`` file (cached), then checks:

* the footprint exists -- if not, the ``.pretty`` directory is listed and the
  closest existing names are suggested (and the pad check runs against the
  best suggestion so the rest of the report is still useful);
* every pad number in the spec's pin list, and every pad used in a
  ``Component.conns``, exists in the footprint                         [ERROR];
* footprint pads that the spec's pin list does not model at all (they can
  never be connected, e.g. shield tabs or mounting pads)                [WARN];
* footprint pads a component leaves unconnected, excluding intentional NC
  (pins named "NC" and the SoC's ``GPIO_NC`` pads)                      [INFO];
* SMD/THT mismatch between ``PartType.tht`` and the footprint           [WARN].

Sources: KiCad official footprints (gitlab.com/kicad/libraries/kicad-footprints)
and, for ``PCM_Espressif:*``, github.com/espressif/kicad-libraries.

Run from the repository root::

    python3 -m hardware.skidl.check_footprints [--cache DIR] [--offline]

The cache directory defaults to ``$KICAD_FP_CACHE`` or ``~/.cache/esp32p4-footprints``.
Exit status is 1 when any ERROR is found.
"""

from __future__ import annotations

import argparse
import difflib
import json
import os
import re
import sys
import urllib.error
import urllib.parse
import urllib.request
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if __package__ in (None, ""):
    sys.path.insert(0, str(REPO_ROOT))

from hardware.lib import board_spec as spec  # noqa: E402
from hardware.lib import esp32p4_pinout as p4  # noqa: E402

KICAD_RAW = ("https://gitlab.com/kicad/libraries/kicad-footprints/-/raw/master/"
             "{lib}.pretty/{name}.kicad_mod")
KICAD_TREE = ("https://gitlab.com/api/v4/projects/kicad%2Flibraries%2Fkicad-footprints/"
              "repository/tree?ref=master&path={lib}.pretty&per_page=100&page={page}")
ESPRESSIF_RAW = ("https://raw.githubusercontent.com/espressif/kicad-libraries/main/"
                 "footprints/Espressif.pretty/{name}.kicad_mod")
DEFAULT_CACHE = Path(os.environ.get("KICAD_FP_CACHE", Path.home() / ".cache" / "esp32p4-footprints"))

_PAD = re.compile(r'\(pad\s+(?:"((?:[^"\\]|\\.)*)"|([^\s()"]+))\s+(\w+)\s+(\w+)')
_ATTR = re.compile(r"\(attr\s+([^)]*)\)")


# ---------------------------------------------------------------------------
# Download + cache
# ---------------------------------------------------------------------------

class Fetcher:
    def __init__(self, cache: Path, offline: bool = False) -> None:
        self.cache = cache
        self.offline = offline

    def _get(self, url: str) -> str | None:
        if self.offline:
            return None
        req = urllib.request.Request(url, headers={"User-Agent": "esp32p4-fp-check"})
        try:
            with urllib.request.urlopen(req, timeout=60) as resp:
                return resp.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            if exc.code == 404:
                return None
            raise

    def footprint(self, fp: str) -> str | None:
        lib, name = fp.split(":", 1)
        path = self.cache / f"{lib}.pretty" / f"{name}.kicad_mod"
        if path.exists():
            return path.read_text()
        if lib == "PCM_Espressif":
            url = ESPRESSIF_RAW.format(name=urllib.parse.quote(name))
        else:
            url = KICAD_RAW.format(lib=urllib.parse.quote(lib), name=urllib.parse.quote(name))
        text = self._get(url)
        if text is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
        return text

    def listing(self, lib: str) -> list[str]:
        """Footprint names in a KiCad official .pretty library (cached)."""
        path = self.cache / f"{lib}.pretty" / "_index.json"
        if path.exists():
            return json.loads(path.read_text())
        names: list[str] = []
        page = 1
        while not self.offline:
            text = self._get(KICAD_TREE.format(lib=urllib.parse.quote(lib), page=page))
            batch = json.loads(text) if text else []
            names += [e["name"][:-len(".kicad_mod")] for e in batch
                      if e["name"].endswith(".kicad_mod")]
            if len(batch) < 100:
                break
            page += 1
        if names:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(sorted(names), indent=0))
        return sorted(names)


# ---------------------------------------------------------------------------
# Footprint model
# ---------------------------------------------------------------------------

@dataclass
class Footprint:
    name: str
    pads: Counter = field(default_factory=Counter)        # number -> copies
    pad_types: dict = field(default_factory=dict)         # number -> {smd, thru_hole, ...}
    unnumbered: int = 0                                   # e.g. NPTH locating holes
    attrs: str = ""

    @classmethod
    def parse(cls, name: str, text: str) -> "Footprint":
        fp = cls(name)
        for m in _PAD.finditer(text):
            number = m.group(1) if m.group(1) is not None else m.group(2)
            if not number:
                fp.unnumbered += 1
                continue
            fp.pads[number] += 1
            fp.pad_types.setdefault(number, set()).add(m.group(3))
        attr = _ATTR.search(text)
        fp.attrs = attr.group(1) if attr else ""
        return fp

    @property
    def is_tht(self) -> bool:
        words = self.attrs.split()
        if "through_hole" in words or "smd" in words:
            return "through_hole" in words
        return any("thru_hole" in t for t in self.pad_types.values())   # e.g. mounting holes

    def plated_legs(self) -> set[str]:
        """Pad numbers that are *only* plated through-holes in an SMD footprint.

        Thermal vias share their number with an SMD exposed pad and are ignored.
        """
        if self.is_tht:
            return set()
        return {n for n, t in self.pad_types.items() if t == {"thru_hole"}}


def _natural(text: str):
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", text)]


def _fmt(pads) -> str:
    return ", ".join(sorted(pads, key=_natural))


def intentional_nc(comp: spec.Component) -> set[str]:
    nc = {pad for pad, name in comp.part.pins if name == "NC"}
    if comp.part is spec.ESP32P4:
        nc |= {p4.GPIO_PAD[g] for g in spec.GPIO_NC}
    return nc


def suggest(name: str, candidates: list[str]) -> list[str]:
    """Closest existing names, preferring ones with the same size/series digits."""
    close = difflib.get_close_matches(name, candidates, n=6, cutoff=0.5)
    digits = re.findall(r"\d{4,}", name)
    digits += [d[:4] for d in digits if len(d) > 4]      # "252012" (LxWxH) -> "2520" (LxW)
    same_size = [c for c in candidates if any(d in c for d in digits)]
    ranked = [c for c in close if c in same_size] + [c for c in same_size if c not in close]
    ranked += [c for c in close if c not in ranked]
    return ranked[:8]


# ---------------------------------------------------------------------------
# Check
# ---------------------------------------------------------------------------

class Report:
    def __init__(self) -> None:
        self.lines: list[str] = []
        self.errors: list[str] = []      # -> exit status 1
        self.fixes: list[str] = []       # actionable spec edits

    def add(self, text: str) -> None:
        self.lines.append(text)

    def error(self, text: str, fix: str = "") -> None:
        self.errors.append(text)
        if fix:
            self.fixes.append(fix)


def check_part(rep: Report, fp: Footprint, pt: spec.PartType, comps: list[spec.Component]) -> None:
    names = dict(pt.pins)
    fp_pads = set(fp.pads)
    missing = pt.pad_numbers - fp_pads
    unmodelled = fp_pads - pt.pad_numbers
    rep.add(f"   part {pt.key}:")
    if missing:
        users = sorted({c.ref for c in comps if set(c.conns) & missing}, key=_natural)
        text = (f"{pt.key}: spec pad(s) not in {fp.name}: "
                + ", ".join(f"{p} ({names[p]})" for p in sorted(missing, key=_natural))
                + (f" -- connected on {_fmt(users)}" if users else ""))
        fix = ""
        if len(missing) == 1 and len(unmodelled) == 1:
            old, new = next(iter(missing)), next(iter(unmodelled))
            fix = (f"{pt.key}: rename pad '{old}' ({names[old]}) to '{new}' "
                   f"(footprint has {fp.pads[new]} pad(s) numbered '{new}')")
        rep.error(text, fix)
        rep.add(f"     [ERROR] {text}" + (f"\n     [FIX]   {fix}" if fix else ""))
    if unmodelled and not (len(missing) == 1 and len(unmodelled) == 1):
        rep.add(f"     [WARN]  footprint pads the spec pin list does not model "
                f"(can never be connected): {_fmt(unmodelled)}")
    if pt.kind != "mech" and pt.tht != fp.is_tht:
        rep.add(f"     [WARN]  PartType.tht={pt.tht} but the footprint is "
                f"{'THT' if fp.is_tht else 'SMD'} (attr: {fp.attrs or '-'})")
    legs = fp.plated_legs() if pt.kind != "mech" else set()
    if legs:
        rep.add(f"     [INFO]  SMD footprint with plated through-hole pad(s) {_fmt(legs)} "
                f"({sum(fp.pads[n] for n in legs)} holes): needs THT / pin-in-paste soldering")

    open_groups: dict[tuple, list[str]] = defaultdict(list)
    for comp in comps:
        left_open = (fp_pads & pt.pad_numbers) - set(comp.conns) - intentional_nc(comp)
        open_groups[tuple(sorted(left_open, key=_natural))].append(comp.ref)
    for left_open, refs in sorted(open_groups.items()):
        refs_s = _fmt(refs) if len(refs) <= 8 else f"{len(refs)} parts"
        if left_open:
            rep.add(f"     [INFO]  {refs_s}: unconnected pads "
                    + ", ".join(f"{p} ({names[p]})" for p in left_open))
        else:
            rep.add(f"     [OK]    {refs_s}: every footprint pad connected (or intentional NC)")


def check(fetcher: Fetcher) -> Report:
    rep = Report()
    by_fp: dict[str, list[spec.PartType]] = defaultdict(list)
    for pt in spec.PARTS.values():
        by_fp[pt.footprint].append(pt)
    comps_by_part: dict[str, list[spec.Component]] = defaultdict(list)
    for comp in spec.COMPONENTS:
        comps_by_part[comp.part.key].append(comp)

    for spec_fp in sorted(by_fp):
        fp_name = spec_fp            # may be replaced by a suggested existing footprint
        lib, name = spec_fp.split(":", 1)
        text = fetcher.footprint(spec_fp)
        header = spec_fp
        if text is None:
            candidates = suggest(name, [] if lib.startswith("PCM_") else fetcher.listing(lib))
            keys = ", ".join(pt.key for pt in by_fp[spec_fp])
            msg = f"{spec_fp} ({keys}): footprint does NOT exist in {lib}.pretty"
            fix = (f"{keys}: footprint '{spec_fp}' does not exist; candidates in {lib}.pretty: "
                   + ", ".join(candidates)) if candidates else ""
            rep.error(msg, fix)
            rep.add(f"\n[ERROR] {msg}" + (f"\n        candidates: {', '.join(candidates)}"
                                          if candidates else ""))
            if not candidates:
                continue
            fp_name = f"{lib}:{candidates[0]}"
            text = fetcher.footprint(fp_name)
            header = f"{fp_name}  (pads checked against the first candidate)"
        fp = Footprint.parse(fp_name, text)
        multi = {n: c for n, c in fp.pads.items() if c > 1}
        rep.add(f"\n== {header}   [{'THT' if fp.is_tht else 'SMD'}]")
        rep.add(f"   pads: {_fmt(fp.pads)}"
                + ("; repeated: " + ", ".join(f"{n} x{multi[n]}" for n in sorted(multi, key=_natural))
                   if multi else "")
                + (f"; {fp.unnumbered} unnumbered (NPTH)" if fp.unnumbered else ""))
        for pt in by_fp[spec_fp]:
            check_part(rep, fp, pt, comps_by_part[pt.key])
    return rep


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Check spec pads against real KiCad footprints")
    ap.add_argument("--cache", type=Path, default=DEFAULT_CACHE, help="download cache directory")
    ap.add_argument("--offline", action="store_true", help="use the cache only")
    args = ap.parse_args(argv)
    rep = check(Fetcher(args.cache, args.offline))
    print("\n".join(rep.lines))
    print(f"\n==== {len(rep.errors)} error(s)")
    for e in rep.errors:
        print("  - " + e)
    if rep.fixes:
        print("\n==== Suggested spec changes")
        for f in rep.fixes:
            print("  - " + f)
    return 1 if rep.errors else 0


if __name__ == "__main__":
    sys.exit(main())
