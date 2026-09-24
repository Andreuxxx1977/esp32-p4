#!/usr/bin/env python3
"""TASK 3 (pcbnew) -- ESP32-P4 "Extreme Performance" board: placement applier.

Layer (B) of the placement flow. All decisions -- every coordinate, rotation,
side, via, zone, rule area, net class and DRC rule -- are made by the pure-
Python planner ``hardware/pcbnew/layout_plan.py`` (testable without KiCad).
This script only copies that plan onto a KiCad board through the pcbnew API,
then reads the result back and asserts it.

Tested with the pcbnew Python module of KiCad 7.0.11, 8.0.8, 9.0.8 and 10.0.6
(headless; the CI job runs it inside the official ``kicad/kicad:10.0`` image with
``--fp-dir /usr/share/kicad/footprints``).

Usage (a Python that can ``import pcbnew``, e.g. KiCad's bundled one)::

    python3 hardware/pcbnew/esp32p4_extreme_place.py                   # new board
    python3 hardware/pcbnew/esp32p4_extreme_place.py --board in.kicad_pcb  # update refs in place
        [-o hardware/output/esp32p4_extreme.kicad_pcb] [--fp-dir DIR ...]
        [--fill] [--drc] [--top-only] [--origin 148.5 105]

Inside the pcbnew scripting console::

    import sys; sys.argv = ['x', '-o', '/path/out.kicad_pcb']
    exec(open('/path/to/esp32p4_extreme_place.py').read())

Footprint libraries are searched in: ``--fp-dir``, then ``$KICAD9_FOOTPRINT_DIR``,
``$KICAD8_FOOTPRINT_DIR``, ``$KICAD7_FOOTPRINT_DIR`` (the running version's
variable first), then the usual install locations. U1 (PCM_Espressif:ESP32-P4)
is always generated from ``esp32p4_pinout``. A footprint that cannot be found
is built from ``footprint_courtyards.json`` (pads + courtyard) with a WARNING,
unless ``--strict-libs`` is given.

Outputs next to the board: ``<name>.kicad_pro`` (net classes, design rules),
``<name>.kicad_dru`` (custom rules), ``<name>.placement.txt`` (report).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import pcbnew  # noqa: E402  (KiCad)

from hardware.lib import board_spec as bs  # noqa: E402
from hardware.lib import esp32p4_pinout as p4  # noqa: E402
from hardware.pcbnew import layout_plan as lp  # noqa: E402

KICAD_MAJOR = int(pcbnew.Version().strip("()").split(".")[0])
GROUP_NAME = "esp32p4_plan"      # everything this script creates (except footprints)
DEFAULT_OUT = REPO / "hardware" / "output" / "esp32p4_extreme.kicad_pcb"


# ==========================================================================
# Version shims (KiCad 7 / 8 / 9)
# ==========================================================================

def nm(mm: float) -> int:
    """mm -> KiCad internal units (nm), exact for values on a 1 nm grid."""
    return int(round(mm * 1_000_000))


class Frame:
    """Design coordinates (SoC-centred mm) -> board coordinates (nm)."""

    def __init__(self, origin_mm: tuple[float, float]):
        self.ox, self.oy = nm(origin_mm[0]), nm(origin_mm[1])

    def pt(self, x: float, y: float) -> "pcbnew.VECTOR2I":
        return pcbnew.VECTOR2I(self.ox + nm(x), self.oy + nm(y))

    def back(self, v) -> tuple[int, int]:
        """Board position -> design position in nm (exact integers)."""
        return v.x - self.ox, v.y - self.oy


LAYER_IDS = {
    "F.Cu": pcbnew.F_Cu, "In1.Cu": pcbnew.In1_Cu, "In2.Cu": pcbnew.In2_Cu, "B.Cu": pcbnew.B_Cu,
    "F.SilkS": pcbnew.F_SilkS, "B.SilkS": pcbnew.B_SilkS, "F.Mask": pcbnew.F_Mask,
    "B.Mask": pcbnew.B_Mask, "F.Paste": pcbnew.F_Paste, "F.Fab": pcbnew.F_Fab,
    "F.CrtYd": pcbnew.F_CrtYd, "Edge.Cuts": pcbnew.Edge_Cuts, "User.1": pcbnew.User_1,
    "User.2": pcbnew.User_2, "Dwgs.User": pcbnew.Dwgs_User,
}


def lset(*layers: int) -> "pcbnew.LSET":
    s = pcbnew.LSET()
    for layer in layers:
        s.AddLayer(layer)
    return s


def set_field(fp, name: str, value: str) -> None:
    """A hidden, Fab-layer footprint field (KiCad 8+) or a footprint property (KiCad 7)."""
    if hasattr(fp, "SetField"):              # KiCad 8+: fields are text items -> keep them
        fp.SetField(name, value)             # off the silkscreen
        for field in fp.GetFields():         # (GetFieldByName is gone in 10, GetField(int) in 8/9)
            if field.GetName() == name:
                field.SetVisible(False)
                field.SetLayer(pcbnew.B_Fab if fp.GetLayer() == pcbnew.B_Cu else pcbnew.F_Fab)
    else:                                    # KiCad 7: footprint properties (no text)
        fp.SetProperty(name, value)


def set_dnp(fp) -> None:
    if hasattr(fp, "SetDNP"):                # KiCad 8+
        fp.SetDNP(True)
    else:                                    # KiCad 7 has no DNP flag
        fp.SetAttributes(fp.GetAttributes() | pcbnew.FP_EXCLUDE_FROM_POS_FILES)
        set_field(fp, "DNP", "DNP")


def set_pad_solid(pad) -> None:
    if hasattr(pad, "SetLocalZoneConnection"):   # KiCad 9
        pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
    else:                                        # KiCad 7/8
        pad.SetZoneConnection(pcbnew.ZONE_CONNECTION_FULL)


def set_pad_geometry(pad, shape: int, w: float, h: float) -> None:
    size = pcbnew.VECTOR2I(nm(w), nm(h))
    try:
        pad.SetShape(shape)
        pad.SetSize(size)
    except TypeError:                            # padstack API: (layer, value)
        pad.SetShape(pcbnew.F_Cu, shape)
        pad.SetSize(pcbnew.F_Cu, size)


def fp_graphic(fp, kind: int):
    """A graphic item owned by a footprint (FP_SHAPE in KiCad 7, PCB_SHAPE in 8+)."""
    cls = getattr(pcbnew, "FP_SHAPE", None)
    return cls(fp, kind) if cls else pcbnew.PCB_SHAPE(fp, kind)


def finish_fp_item(item) -> None:
    """KiCad 7 stores local copies of child coordinates; refresh them."""
    if hasattr(item, "SetLocalCoord"):
        item.SetLocalCoord()


def set_priority(zone, priority: int) -> None:
    (zone.SetAssignedPriority if hasattr(zone, "SetAssignedPriority") else zone.SetPriority)(priority)


def mm_units():
    return getattr(pcbnew, "EDA_UNITS_MM", getattr(pcbnew, "EDA_UNITS_MILLIMETRES", None))


def run_drc(board_path: Path) -> str:
    """DRC with kicad-cli >= 8 when available ($KICAD_CLI or PATH), else pcbnew's report.

    ``pcbnew.WriteDRCReport`` aborts in KiCad 10's standalone Python (it needs the GUI
    program instance), so KiCad 10 always uses kicad-cli.
    """
    cli = os.environ.get("KICAD_CLI") or shutil.which("kicad-cli")
    if cli:
        ver = subprocess.run([cli, "version"], capture_output=True, text=True).stdout.strip()
        if ver[:1].isdigit() and int(ver.split(".")[0]) >= 8:
            rpt = board_path.with_suffix(".drc.json")
            res = subprocess.run([cli, "pcb", "drc", "--format", "json", "--severity-all",
                                  "-o", str(rpt), str(board_path)], capture_output=True, text=True)
            tail = (res.stdout.strip().splitlines() or [""])[-1]
            return f"kicad-cli {ver} DRC -> {rpt} ({tail})"
    if KICAD_MAJOR >= 10:
        return "DRC skipped: no kicad-cli >= 8 found (set $KICAD_CLI)"
    rpt = board_path.with_suffix(".drc.rpt")
    board = pcbnew.LoadBoard(str(board_path))      # reload so the project + rules are used
    ok = pcbnew.WriteDRCReport(board, str(rpt), mm_units(), True)
    return f"DRC report {rpt} ({'written' if ok else 'FAILED'})"


# ==========================================================================
# Footprints
# ==========================================================================

class Library:
    """Finds ``Lib:Name`` in the footprint library directories."""

    def __init__(self, dirs: list[str]):
        env_order = [f"KICAD{KICAD_MAJOR}_FOOTPRINT_DIR"] + [
            f"KICAD{v}_FOOTPRINT_DIR" for v in (9, 8, 7) if v != KICAD_MAJOR]
        cands = list(dirs) + [os.environ[e] for e in env_order if os.environ.get(e)] + [
            "/usr/share/kicad/footprints", "/usr/local/share/kicad/footprints",
            "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints",
            f"C:/Program Files/KiCad/{KICAD_MAJOR}.0/share/kicad/footprints"]
        self.dirs = [Path(d) for d in dict.fromkeys(cands) if d and Path(d).is_dir()]

    def load(self, lib_id: str):
        lib, name = lib_id.split(":", 1)
        for d in self.dirs:
            pretty = d / f"{lib}.pretty"
            if (pretty / f"{name}.kicad_mod").exists():
                fp = pcbnew.FootprintLoad(str(pretty), name)
                if fp is not None:
                    fp.SetFPID(pcbnew.LIB_ID(lib, name))
                    return fp, str(pretty)
        return None, None


def _add_pad(fp, number: str, x: float, y: float, w: float, h: float, shape: int,
             layers, attrib=None, paste_margin: float | None = None, drill: float | None = None):
    pad = pcbnew.PAD(fp)
    pad.SetNumber(number)
    pad.SetAttribute(attrib if attrib is not None else pcbnew.PAD_ATTRIB_SMD)
    set_pad_geometry(pad, shape, w, h)
    if drill:
        pad.SetDrillSize(pcbnew.VECTOR2I(nm(drill), nm(drill)))
    pad.SetLayerSet(layers)
    if paste_margin is not None:
        pad.SetLocalSolderPasteMargin(nm(paste_margin))
    fp.Add(pad)
    pad.SetPosition(pcbnew.VECTOR2I(nm(x), nm(y)))      # footprint still at the origin
    finish_fp_item(pad)
    if hasattr(pad, "SetFPRelativePosition"):
        pad.SetFPRelativePosition(pcbnew.VECTOR2I(nm(x), nm(y)))
    return pad


def _add_line(fp, layer: int, x0, y0, x1, y1, width: float = 0.12) -> None:
    g = fp_graphic(fp, pcbnew.SHAPE_T_SEGMENT)
    g.SetLayer(layer)
    g.SetWidth(nm(width))
    fp.Add(g)
    g.SetStart(pcbnew.VECTOR2I(nm(x0), nm(y0)))
    g.SetEnd(pcbnew.VECTOR2I(nm(x1), nm(y1)))
    finish_fp_item(g)


def _add_rect(fp, layer: int, r, width: float = 0.05) -> None:
    x0, y0, x1, y1 = r
    for a, b in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
        _add_line(fp, layer, *a, *b, width)


def _add_circle(fp, layer: int, cx: float, cy: float, r: float, width: float = 0.12,
                filled: bool = False) -> None:
    g = fp_graphic(fp, pcbnew.SHAPE_T_CIRCLE)
    g.SetLayer(layer)
    g.SetWidth(nm(width))
    fp.Add(g)
    g.SetCenter(pcbnew.VECTOR2I(nm(cx), nm(cy)))
    g.SetEnd(pcbnew.VECTOR2I(nm(cx + r), nm(cy)))
    if filled:
        g.SetFilled(True)
    finish_fp_item(g)


def build_u1(board):
    """ESP32-P4 QFN-104 from esp32p4_pinout: 104 pads, solid 7.5 mm EPAD + 3x3 paste windows."""
    fp = pcbnew.FOOTPRINT(board)
    fp.SetFPID(pcbnew.LIB_ID("PCM_Espressif", "ESP32-P4"))
    fp.SetAttributes(pcbnew.FP_SMD)
    cu_mask_paste = lset(pcbnew.F_Cu, pcbnew.F_Mask, pcbnew.F_Paste)
    geom = lp.u1_geometry()
    for pad in geom.pads:
        if pad.number == "105":
            continue
        _add_pad(fp, pad.number, pad.x, pad.y, pad.sx, pad.sy, pcbnew.PAD_SHAPE_OVAL, cu_mask_paste)
    e = p4.EPAD_SIZE_MM
    epad = _add_pad(fp, "105", 0.0, 0.0, e, e, pcbnew.PAD_SHAPE_RECT, lset(pcbnew.F_Cu, pcbnew.F_Mask))
    set_pad_solid(epad)
    t, pitch = p4.EPAD_TILE_MM, p4.EPAD_TILE_PITCH_MM
    for i in (-1, 0, 1):          # paste-only aperture pads: 2.1 mm tiles, -0.2 mm margin
        for j in (-1, 0, 1):
            _add_pad(fp, "", i * pitch, j * pitch, t, t, pcbnew.PAD_SHAPE_RECT,
                     lset(pcbnew.F_Paste), paste_margin=-0.2)
    c, f = lp.U1_COURTYARD_HALF_MM, lp.U1_FAB_HALF_MM
    _add_rect(fp, pcbnew.F_CrtYd, (-c, -c, c, c), 0.05)
    ch = 1.0                                        # fab outline with pin-1 chamfer
    fab = [(-f + ch, -f), (f, -f), (f, f), (-f, f), (-f, -f + ch), (-f + ch, -f)]
    for a, b in zip(fab, fab[1:]):
        _add_line(fp, pcbnew.F_Fab, *a, *b, 0.1)
    s, k = 5.15, 4.70                               # silk corner marks clear of the pad rows
    for sx in (-1, 1):
        for sy in (-1, 1):
            _add_line(fp, pcbnew.F_SilkS, sx * s, sy * s, sx * k, sy * s)
            _add_line(fp, pcbnew.F_SilkS, sx * s, sy * s, sx * s, sy * k)
    _add_circle(fp, pcbnew.F_SilkS, -5.45, -5.45, 0.12, 0.12, filled=True)   # pin-1 dot
    fp.Reference().SetPosition(pcbnew.VECTOR2I(0, nm(-6.4)))
    fp.Value().SetPosition(pcbnew.VECTOR2I(0, nm(6.4)))
    fp.Value().SetLayer(pcbnew.F_Fab)
    for item in (fp.Reference(), fp.Value()):
        finish_fp_item(item)
    set_field(fp, "Height", f"{p4.HEIGHT_MAX_MM:.2f} mm")
    try:                                            # representative 10 x 10 mm QFN body
        model = pcbnew.FP_3DMODEL()
        model.m_Filename = lp.U1_MODEL
        fp.Models().push_back(model)
    except (AttributeError, TypeError):             # very old API: renders without a body
        pass
    return fp


def build_placeholder(board, geom: lp.FootprintGeom):
    """Pads + courtyard from footprint_courtyards.json when a library is missing."""
    fp = pcbnew.FOOTPRINT(board)
    lib, name = geom.lib_id.split(":", 1)
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    shapes = {"circle": pcbnew.PAD_SHAPE_CIRCLE, "oval": pcbnew.PAD_SHAPE_OVAL,
              "roundrect": pcbnew.PAD_SHAPE_ROUNDRECT}
    for p in geom.pads:
        shape = shapes.get(p.shape, pcbnew.PAD_SHAPE_RECT)
        if p.kind == "tht":
            _add_pad(fp, p.number, p.x, p.y, p.sx, p.sy, shape, pcbnew.PAD.PTHMask(),
                     pcbnew.PAD_ATTRIB_PTH, drill=min(p.sx, p.sy) * 0.6)
        elif p.kind == "npth":
            _add_pad(fp, p.number, p.x, p.y, p.sx, p.sy, pcbnew.PAD_SHAPE_CIRCLE,
                     pcbnew.PAD.UnplatedHoleMask(), pcbnew.PAD_ATTRIB_NPTH, drill=p.sx)
        elif p.kind == "smd":
            _add_pad(fp, p.number, p.x, p.y, p.sx, p.sy, shape, pcbnew.PAD.SMDMask())
    if geom.courtyard_r:
        _add_circle(fp, pcbnew.F_CrtYd, 0, 0, geom.courtyard_r, 0.05)
    else:
        _add_rect(fp, pcbnew.F_CrtYd, geom.courtyard)
    fp.SetAttributes(pcbnew.FP_THROUGH_HOLE if geom.tht else pcbnew.FP_SMD)
    return fp


# ==========================================================================
# Board set-up
# ==========================================================================

def setup_board(board, frame: Frame) -> None:
    board.SetCopperLayerCount(4)
    for layer, name in lp.LAYERS.items():
        board.SetLayerName(LAYER_IDS[layer], name)
    board.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_POWER)
    board.SetLayerType(pcbnew.In2_Cu, pcbnew.LT_POWER)
    enabled = board.GetEnabledLayers()
    for layer, name in lp.USER_LAYERS.items():
        enabled.AddLayer(LAYER_IDS[layer])
    board.SetEnabledLayers(enabled)
    for layer, name in lp.USER_LAYERS.items():
        board.SetLayerName(LAYER_IDS[layer], name)
    bds = board.GetDesignSettings()
    r = lp.DESIGN_RULES
    bds.SetBoardThickness(nm(r["board_thickness"]))
    origin = frame.pt(0.0, 0.0)
    bds.SetAuxOrigin(origin)
    bds.SetGridOrigin(origin)
    bds.m_TrackMinWidth = nm(r["min_track"])
    bds.m_MinClearance = nm(r["min_clearance"])
    bds.m_ViasMinSize = nm(r["via_diameter"])
    bds.m_MinThroughDrill = nm(r["via_drill"])
    bds.m_ViasMinAnnularWidth = nm(r["min_annular"])
    bds.m_MicroViasMinSize = nm(r["uvia_diameter"])
    bds.m_MicroViasMinDrill = nm(r["uvia_drill"])
    bds.m_HoleToHoleMin = nm(r["hole_to_hole"])
    bds.m_HoleClearance = nm(r["hole_clearance"])
    bds.m_CopperEdgeClearance = nm(r["copper_edge"])
    for flag in ("m_MicroViasAllowed", "m_BlindBuriedViaAllowed"):   # KiCad 6 only
        if hasattr(bds, flag):
            setattr(bds, flag, True)
    try:
        bds.m_ViasDimensionsList.append(pcbnew.VIA_DIMENSION(nm(r["via_diameter"]), nm(r["via_drill"])))
        bds.m_ViasDimensionsList.append(pcbnew.VIA_DIMENSION(nm(r["uvia_diameter"]), nm(r["uvia_drill"])))
    except Exception as exc:  # pragma: no cover - optional convenience
        print(f"note: via size list not set ({exc})")


def ensure_nets(board) -> dict[str, object]:
    nets = {}
    for name in sorted(bs.nets()):
        net = board.FindNet(name)
        if net is None:
            net = pcbnew.NETINFO_ITEM(board, name)
            board.Add(net)
        nets[name] = net
    return nets


def apply_netclasses(board, plan: lp.Plan) -> None:
    """Create the net classes in memory (the .kicad_pro patch below makes them persistent)."""
    ns = board.GetDesignSettings().m_NetSettings
    for ncp in plan.netclasses:
        nc = pcbnew.NETCLASS(ncp.name)
        nc.SetTrackWidth(nm(ncp.track_width))
        nc.SetClearance(nm(ncp.clearance))
        nc.SetViaDiameter(nm(ncp.via_diameter))
        nc.SetViaDrill(nm(ncp.via_drill))
        nc.SetuViaDiameter(nm(ncp.uvia_diameter))
        nc.SetuViaDrill(nm(ncp.uvia_drill))
        if ncp.dp_width:
            nc.SetDiffPairWidth(nm(ncp.dp_width))
            nc.SetDiffPairGap(nm(ncp.dp_gap))
        nc.SetDescription(ncp.description)
        try:
            if ncp.name == "Default":
                target = ns.GetDefaultNetclass() if hasattr(ns, "GetDefaultNetclass") else ns.m_DefaultNetClass
                for setter in ("SetTrackWidth", "SetClearance", "SetViaDiameter", "SetViaDrill",
                               "SetuViaDiameter", "SetuViaDrill"):
                    getter = "Get" + setter[3:]
                    getattr(target, setter)(getattr(nc, getter)())
            elif hasattr(ns, "SetNetclass"):          # KiCad 9
                ns.SetNetclass(ncp.name, nc)
            else:                                     # KiCad 7/8
                ns.m_NetClasses[ncp.name] = nc
        except Exception as exc:
            print(f"note: net class {ncp.name} kept for the .kicad_pro patch only ({exc})")
    if hasattr(ns, "SetNetclassPatternAssignment"):   # KiCad 9
        for net, cls in plan.net_assignments.items():
            ns.SetNetclassPatternAssignment(net, cls)


def patch_project(pro: Path, plan: lp.Plan) -> None:
    """Make net classes, net->class patterns and the HDI rules persistent in the .kicad_pro.

    KiCad 7/8 expose no Python API for pattern assignments; editing the JSON that
    this very KiCad version just wrote (reusing its own 'Default' entry as the
    template) is version-proof.
    """
    data = json.loads(pro.read_text()) if pro.exists() else {}
    ns = data.setdefault("net_settings", {})
    classes = ns.setdefault("classes", [])
    template = next((c for c in classes if c.get("name") == "Default"), {})
    by_name = {c.get("name"): c for c in classes}
    for ncp in plan.netclasses:
        entry = by_name.get(ncp.name)
        if entry is None:
            entry = dict(template)
            classes.append(entry)
        entry.update({"name": ncp.name, "track_width": ncp.track_width, "clearance": ncp.clearance,
                      "via_diameter": ncp.via_diameter, "via_drill": ncp.via_drill,
                      "microvia_diameter": ncp.uvia_diameter, "microvia_drill": ncp.uvia_drill})
        if ncp.dp_width:
            entry.update({"diff_pair_width": ncp.dp_width, "diff_pair_gap": ncp.dp_gap})
        if "priority" in template and ncp.name != "Default":
            entry["priority"] = [c.name for c in plan.netclasses].index(ncp.name)
    patterns = [p for p in (ns.get("netclass_patterns") or []) if p.get("pattern") not in plan.net_assignments]
    patterns += [{"netclass": cls, "pattern": net} for net, cls in plan.net_assignments.items()]
    ns["netclass_patterns"] = patterns
    rules = data.setdefault("board", {}).setdefault("design_settings", {}).setdefault("rules", {})
    r = lp.DESIGN_RULES
    rules.update({"min_track_width": r["min_track"], "min_clearance": r["min_clearance"],
                  "min_via_diameter": r["via_diameter"], "min_through_hole_diameter": r["via_drill"],
                  "min_via_annular_width": r["min_annular"], "min_microvia_diameter": r["uvia_diameter"],
                  "min_microvia_drill": r["uvia_drill"], "min_hole_to_hole": r["hole_to_hole"],
                  "min_hole_clearance": r["hole_clearance"], "min_copper_edge_clearance": r["copper_edge"],
                  "allow_microvias": True, "allow_blind_buried_vias": True})
    pro.write_text(json.dumps(data, indent=2) + "\n")


# ==========================================================================
# Placement
# ==========================================================================

def find_footprint(board, ref: str):
    for fp in board.GetFootprints():
        if fp.GetReference() == ref:
            return fp
    return None


def place_footprints(board, frame: Frame, plan: lp.Plan, lib: Library, nets, strict: bool,
                     log: list[str]) -> None:
    comps = {c.ref: c for c in bs.COMPONENTS}
    for ref in sorted(plan.placements, key=lp.comp_sort_key):
        pl = plan.placements[ref]
        comp = comps.get(ref)
        fp = find_footprint(board, ref)
        if fp is None:
            if pl.lib_id == lp.U1_LIB_ID:
                fp = build_u1(board)
                source = "generated from esp32p4_pinout"
            else:
                fp, source = lib.load(pl.lib_id)
                if fp is None:
                    if strict:
                        raise SystemExit(f"{ref}: footprint {pl.lib_id} not found in {lib.dirs}")
                    fp = build_placeholder(board, pl.geom)
                    source = "PLACEHOLDER from footprint_courtyards.json"
                    log.append(f"WARNING {ref}: {pl.lib_id} not found -> placeholder footprint")
            fp.SetReference(ref)
            board.Add(fp)
            log.append(f"{ref}: {pl.lib_id} ({source})")
        if comp:
            fp.SetValue(comp.value)
            if comp.part.mpn:
                set_field(fp, "MPN", comp.part.mpn)
                set_field(fp, "Manufacturer", comp.part.manufacturer)
            if comp.part.lcsc:
                set_field(fp, "LCSC", comp.part.lcsc)
            if comp.dnp:
                set_dnp(fp)
            numbers = [p.GetNumber() for p in fp.Pads()]
            mapping, notes = lp.map_pads(comp, numbers)
            for note in notes:
                log.append(f"{ref}: {note}")
            for spec_pad, net in comp.conns.items():
                for fp_pad in mapping.get(spec_pad, ()):
                    for pad in fp.Pads():
                        if pad.GetNumber() == fp_pad:
                            pad.SetNet(nets[net])
        elif ref.startswith("FID"):
            fp.SetValue("Fiducial")
            fp.SetAttributes(fp.GetAttributes() | pcbnew.FP_EXCLUDE_FROM_BOM)
        if ref == "U1":
            for pad in fp.Pads():
                if pad.GetNumber() == "105":
                    set_pad_solid(pad)
        if not pl.ref_on_silk:
            fp.Reference().SetLayer(pcbnew.B_Fab if fp.GetLayer() == pcbnew.B_Cu else pcbnew.F_Fab)
        # side last (flip about its own position mirrors every child, fields included),
        # then position and orientation
        want_layer = pcbnew.F_Cu if pl.side == "F" else pcbnew.B_Cu
        if fp.GetLayer() != want_layer:
            fp.SetLayerAndFlip(want_layer)
        fp.SetPosition(frame.pt(pl.x, pl.y))
        fp.SetOrientationDegrees(pl.rot)
        if pl.ref_on_silk and pl.ref_xy:                 # planned, pad-free, upright
            ref_text = fp.Reference()
            ref_text.SetPosition(frame.pt(*pl.ref_xy))
            ref_text.SetTextAngleDegrees(0.0)
            ref_text.SetTextSize(pcbnew.VECTOR2I(nm(pl.ref_size), nm(pl.ref_size)))
            ref_text.SetTextThickness(nm(0.15 * pl.ref_size))
            finish_fp_item(ref_text)


def verify(board, frame: Frame, plan: lp.Plan, log: list[str]) -> None:
    """Read positions back in nm and assert the plan (hard asserts for U1 and H1..H4)."""
    exact = {"U1": (0.0, 0.0), **bs.HEATSINK_HOLES}
    for ref, (x, y) in exact.items():
        fp = find_footprint(board, ref)
        got = frame.back(fp.GetPosition())
        assert got == (nm(x), nm(y)), f"{ref} at {got} nm, expected {(nm(x), nm(y))}"
        assert fp.GetLayer() == pcbnew.F_Cu, f"{ref} not on F.Cu"
    assert find_footprint(board, "U1").GetOrientationDegrees() == 0.0
    worst = (0.0, "")
    for ref, pl in plan.placements.items():
        fp = find_footprint(board, ref)
        assert frame.back(fp.GetPosition()) == (nm(pl.x), nm(pl.y)), f"{ref} position"
        rot = fp.GetOrientationDegrees() % 360.0
        assert abs(rot - pl.rot % 360.0) < 1e-6, f"{ref} rotation {rot} != {pl.rot}"
        assert (fp.GetLayer() == pcbnew.B_Cu) == (pl.side == "B"), f"{ref} side"
        planned = {}
        for number, px, py in pl.pad_positions():
            planned.setdefault(number, []).append((px, py))
        for pad in fp.Pads():
            spots = planned.get(pad.GetNumber())
            if not spots:
                continue
            gx, gy = frame.back(pad.GetPosition())
            d = min(math.hypot(gx / 1e6 - px, gy / 1e6 - py) for px, py in spots)
            if d > worst[0]:
                worst = (d, f"{ref}.{pad.GetNumber()}")
    log.append(f"verify: U1 and H1..H4 exact (nm); all {len(plan.placements)} footprints at plan "
               f"position/rotation/side; worst pad offset vs planning geometry "
               f"{worst[0]:.3f} mm ({worst[1] or '-'})")


# ==========================================================================
# Vias, zones, rule areas, graphics, silkscreen
# ==========================================================================

def clear_previous(board) -> None:
    for grp in list(board.Groups()):
        if grp.GetName() == GROUP_NAME:
            for item in list(grp.GetItems()):
                board.Remove(item)
            board.Remove(grp)


def add_items(board, frame: Frame, plan: lp.Plan, nets) -> int:
    grp = pcbnew.PCB_GROUP(board)
    grp.SetName(GROUP_NAME)
    board.Add(grp)
    count = 0

    def keep(item):
        nonlocal count
        board.Add(item)
        grp.AddItem(item)
        count += 1

    for v in plan.vias:
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(frame.pt(v.x, v.y))
        via.SetViaType(pcbnew.VIATYPE_THROUGH)
        via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        via.SetDrill(nm(v.drill))
        via.SetWidth(nm(v.diameter))
        via.SetNet(nets[v.net])
        keep(via)

    for z in plan.zones:
        zone = pcbnew.ZONE(board)
        layers = [LAYER_IDS[layer] for layer in z.layers]
        if len(layers) == 1:
            zone.SetLayer(layers[0])
        else:
            zone.SetLayerSet(lset(*layers))
        outline = zone.Outline()
        outline.NewOutline()
        for x, y in z.polygon:
            p = frame.pt(x, y)
            outline.Append(p.x, p.y)
        zone.SetZoneName(z.name)
        if z.rule_area:
            zone.SetIsRuleArea(True)
            zone.SetDoNotAllowTracks("tracks" in z.keepout)
            zone.SetDoNotAllowVias("vias" in z.keepout)
            zone.SetDoNotAllowPads("pads" in z.keepout)
            zone.SetDoNotAllowFootprints("footprints" in z.keepout)
            set_no_zone_fills = getattr(zone, "SetDoNotAllowZoneFills", None) or \
                zone.SetDoNotAllowCopperPour                 # renamed in KiCad 10
            set_no_zone_fills("pours" in z.keepout)
        else:
            zone.SetNet(nets[z.net])
            set_priority(zone, z.priority)
            zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if z.connection == "solid"
                                  else pcbnew.ZONE_CONNECTION_THERMAL)
            zone.SetMinThickness(nm(lp.ZONE_MIN_WIDTH_MM))
            zone.SetLocalClearance(nm(lp.ZONE_CLEARANCE_MM))
            zone.SetThermalReliefGap(nm(0.2))
            zone.SetThermalReliefSpokeWidth(nm(0.25))
            zone.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        keep(zone)

    kinds = {"line": pcbnew.SHAPE_T_SEGMENT, "arc": pcbnew.SHAPE_T_ARC, "rect": pcbnew.SHAPE_T_RECT,
             "circle": pcbnew.SHAPE_T_CIRCLE}
    for g in plan.graphics:
        if g.kind == "text":
            t = pcbnew.PCB_TEXT(board)
            t.SetText(g.text)
            t.SetLayer(LAYER_IDS[g.layer])
            t.SetPosition(frame.pt(*g.pts[0]))
            t.SetTextSize(pcbnew.VECTOR2I(nm(1.0), nm(1.0)))
            t.SetTextThickness(nm(0.15))
            keep(t)
            continue
        s = pcbnew.PCB_SHAPE(board, kinds[g.kind])
        s.SetLayer(LAYER_IDS[g.layer])
        s.SetWidth(nm(g.width))
        if g.kind == "arc":
            s.SetArcGeometry(*(frame.pt(*p) for p in g.pts))
        elif g.kind == "circle":
            s.SetCenter(frame.pt(*g.pts[0]))
            s.SetEnd(frame.pt(*g.pts[1]))
        else:
            s.SetStart(frame.pt(*g.pts[0]))
            s.SetEnd(frame.pt(*g.pts[1]))
        if g.filled:
            s.SetFilled(True)
        keep(s)

    for st in plan.silk:
        t = pcbnew.PCB_TEXT(board)
        t.SetText(st.text)
        t.SetLayer(LAYER_IDS[st.layer])
        t.SetTextSize(pcbnew.VECTOR2I(nm(st.glyph_width), nm(st.size)))
        t.SetTextThickness(nm(st.thickness))
        t.SetPosition(frame.pt(st.x, st.y))
        t.SetTextAngleDegrees(st.angle)
        keep(t)
    return count


# ==========================================================================
# Driver
# ==========================================================================

def open_board(path: str | None, out: Path):
    if path:
        board = pcbnew.LoadBoard(str(Path(path).resolve()))
        board.SetFileName(str(out))
        return board
    out.parent.mkdir(parents=True, exist_ok=True)
    return pcbnew.NewBoard(str(out))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Place the ESP32-P4 Extreme board from layout_plan.")
    ap.add_argument("--board", help="existing .kicad_pcb to update (footprints matched by reference)")
    ap.add_argument("-o", "--output", "--out", default=str(DEFAULT_OUT), help="board file to write")
    ap.add_argument("--fp-dir", action="append", default=[], help="footprint library root (repeatable)")
    ap.add_argument("--origin", nargs=2, type=float, default=lp.PAGE_ORIGIN_MM, metavar=("X", "Y"),
                    help="sheet position of the design origin in mm (default: A4 centre)")
    ap.add_argument("--fill", action="store_true", help="fill the zones before saving")
    ap.add_argument("--drc", action="store_true",
                    help="run DRC (kicad-cli >= 8 -> <name>.drc.json, else <name>.drc.rpt)")
    ap.add_argument("--top-only", action="store_true", help="never place SoC-ring parts on B.Cu")
    ap.add_argument("--strict-libs", action="store_true", help="fail if a footprint is missing")
    args = ap.parse_args(argv)

    plan = lp.build_plan(allow_bottom=not args.top_only)
    problems = lp.validate(plan)
    if problems:
        print("layout plan has problems:\n  " + "\n  ".join(problems))
        return 2
    out = Path(args.output).resolve()
    frame = Frame(tuple(args.origin))
    log = [f"KiCad {pcbnew.Version()} ({'new board' if not args.board else args.board})"]

    board = open_board(args.board, out)
    clear_previous(board)
    setup_board(board, frame)
    nets = ensure_nets(board)
    lib = Library(args.fp_dir)
    log.append(f"footprint libraries: {[str(d) for d in lib.dirs] or 'none (placeholders)'}")
    place_footprints(board, frame, plan, lib, nets, args.strict_libs, log)
    n_items = add_items(board, frame, plan, nets)
    apply_netclasses(board, plan)
    verify(board, frame, plan, log)
    board.BuildConnectivity()
    pcbnew.SaveBoard(str(out), board)
    pro = out.with_suffix(".kicad_pro")
    patch_project(pro, plan)
    out.with_suffix(".kicad_dru").write_text(lp.dru_text(plan))
    if args.fill:
        # Fill only after reloading: LoadBoard() initialises the DRC engine from the
        # .kicad_pro/.kicad_dru just written, so the filler honours hole clearances and
        # the custom rules (a board from NewBoard() has no rules loaded yet).
        board = pcbnew.LoadBoard(str(out))
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        pcbnew.SaveBoard(str(out), board)
        patch_project(pro, plan)
        log.append("zones filled (after reloading the project rules)")
    log.append(f"saved {out} (+ {pro.name}, {out.with_suffix('.kicad_dru').name}); "
               f"{len(plan.placements)} footprints, {n_items} planned items (group '{GROUP_NAME}')")
    if args.drc:
        log.append(run_drc(out))
    text = lp.report(plan) + "\n\nApplier log\n" + "\n".join("  " + x for x in log)
    out.with_suffix(".placement.txt").write_text(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
