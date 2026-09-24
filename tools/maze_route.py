"""Finish a routed board: route the connections KiCad's DRC still reports as unconnected.

A grid router for what the autorouter left behind (usually long connections it kept
deferring). It works on the two signal layers (F.Cu, B.Cu) of the routed board:

* obstacles are the exact copper shapes -- pads from the layout plan, tracks and vias
  parsed from the board file -- and the distance from every grid-cell centre to the
  nearest copper of *other* nets is computed exactly (not rasterised), so a cell is free
  for a net when that distance is at least ``width / 2 + clearance`` (+ a small margin);
* each unconnected pair from ``kicad-cli pcb drc`` is routed from the copper island of
  one item to the island of the other with Dijkstra (8-neighbour moves, 45-degree
  tracks; a through via where both layers are free for the via pad and its hole);
* new tracks and vias are appended to the board file; refill the zones and re-run DRC
  afterwards (KiCad is the judge -- nothing here is trusted on its own).

Usage (needs numpy and scipy; no pcbnew)::

    python -m tools.maze_route ROUTED.kicad_pcb DRC.json -o OUT.kicad_pcb [--grid 0.05]
    kicad-cli pcb drc --refill-zones --save-board ... OUT.kicad_pcb
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from hardware.lib import board_spec as bs  # noqa: E402
from hardware.pcbnew import layout_plan as lp  # noqa: E402

ORIGIN = lp.PAGE_ORIGIN_MM
LAYERS = ("F.Cu", "B.Cu")
LAYER_OF = {"Signal_Top": 0, "F.Cu": 0, "Signal_Bottom": 1, "B.Cu": 1}
HI_CLEARANCE = 0.20            # the largest netclass clearance (MIPI / USB / ETH)
EDGE_MM = lp.DESIGN_RULES["copper_edge"]
VIA_D, VIA_DRILL = lp.DESIGN_RULES["via_diameter"], lp.DESIGN_RULES["via_drill"]
HOLE_TO_HOLE = lp.DESIGN_RULES["hole_to_hole"]
VIA_COST = 12.0                # in grid steps
WINDOW_MM = 6.0                # search window around the pair's bounding box
UUID_NS = uuid.UUID("5a1f7e0e-8d3b-4c55-9d0e-2f7b1d3a9c11")


@dataclass
class Shape:
    kind: str                   # rect | circle | capsule
    layers: tuple[int, ...]
    net: str
    a: tuple                    # rect: (x0, y0, x1, y1); circle: (cx, cy, r); capsule: (x0, y0, x1, y1, r)
    hole: float = 0.0           # drill diameter (vias, NPTH)
    via: bool = False

    def bbox(self) -> tuple[float, float, float, float]:
        if self.kind == "rect":
            return self.a
        if self.kind == "circle":
            cx, cy, r = self.a
            return cx - r, cy - r, cx + r, cy + r
        x0, y0, x1, y1, r = self.a
        return min(x0, x1) - r, min(y0, y1) - r, max(x0, x1) + r, max(y0, y1) + r

    def dist(self, X: np.ndarray, Y: np.ndarray) -> np.ndarray:
        """Exact distance from points to the shape (0 inside)."""
        if self.kind == "rect":
            x0, y0, x1, y1 = self.a
            dx = np.maximum(np.maximum(x0 - X, 0), X - x1)
            dy = np.maximum(np.maximum(y0 - Y, 0), Y - y1)
            return np.hypot(dx, dy)
        if self.kind == "circle":
            cx, cy, r = self.a
            return np.maximum(np.hypot(X - cx, Y - cy) - r, 0)
        x0, y0, x1, y1, r = self.a
        vx, vy = x1 - x0, y1 - y0
        L = vx * vx + vy * vy
        u = np.zeros_like(X) if L == 0 else np.clip(((X - x0) * vx + (Y - y0) * vy) / L, 0, 1)
        return np.maximum(np.hypot(x0 + u * vx - X, y0 + u * vy - Y) - r, 0)


# ==========================================================================
# Board model
# ==========================================================================

def net_rules(net: str) -> tuple[float, float]:
    """(track width, clearance) for routing a net."""
    cls = bs.netclass_of(net)
    if cls == "POWER":
        return 0.30, 0.15       # 0.3 mm fits where the finisher routes; the rails' main paths exist
    return lp.ESCAPE_TRACKS.get(cls, lp.ESCAPE_TRACKS["Default"])


def clearance_of(net: str) -> float:
    return net_rules(net)[1]


def plan_pads(plan) -> list[Shape]:
    comps = {c.ref: c for c in bs.COMPONENTS}
    out = []
    for ref, pl in plan.placements.items():
        comp = comps.get(ref)
        mapping = plan.pad_maps.get(ref, {})
        nets = {fp: n for spec, n in (comp.conns.items() if comp else ()) for fp in mapping.get(spec, (spec,))}
        side = 0 if pl.side == "F" else 1
        for p, sh in zip(pl.geom.copper_pads(), pl.pad_shapes()):
            layers = (0, 1) if p.kind == "tht" else (side,)
            net = nets.get(p.number, "")
            if sh.circle:
                out.append(Shape("circle", layers, net, sh.circle))
            else:
                out.append(Shape("rect", layers, net, sh.rect))
        for p in pl.geom.pads:
            if p.kind == "npth":
                cx, cy = lp.to_board(p.x, p.y, pl.x, pl.y, pl.rot, pl.side)
                out.append(Shape("circle", (0, 1), "#npth", (cx, cy, p.sx / 2), hole=p.sx))
    return out


_SEG = re.compile(r"\(segment\s+\(start ([-\d.]+) ([-\d.]+)\)\s+\(end ([-\d.]+) ([-\d.]+)\)\s+"
                  r"\(width ([\d.]+)\)\s+\(layer \"([^\"]+)\"\)\s+\(net \"([^\"]*)\"\)", re.S)
_VIA = re.compile(r"\(via( micro| blind| buried)?\s+\(at ([-\d.]+) ([-\d.]+)\)\s+\(size ([\d.]+)\)\s+"
                  r"\(drill ([\d.]+)\)\s+\(layers \"([^\"]+)\" \"([^\"]+)\"\)(?:\s+\([^()]*\))*?\s+"
                  r"\(net \"([^\"]*)\"\)", re.S)


def board_copper(text: str) -> list[Shape]:
    ox, oy = ORIGIN
    out = []
    for x0, y0, x1, y1, w, layer, net in _SEG.findall(text):
        if layer in LAYER_OF:
            out.append(Shape("capsule", (LAYER_OF[layer],), net,
                             (float(x0) - ox, float(y0) - oy, float(x1) - ox, float(y1) - oy, float(w) / 2)))
    for kind, x, y, size, drill, la, lb, net in _VIA.findall(text):
        layers = tuple(sorted({LAYER_OF[l] for l in (la, lb) if l in LAYER_OF}))
        if kind.strip() == "" or not layers:
            layers = (0, 1) if not kind.strip() else layers
        out.append(Shape("circle", layers, net, (float(x) - ox, float(y) - oy, float(size) / 2),
                         hole=float(drill), via=True))
    return out


# ==========================================================================
# Router
# ==========================================================================

class Grid:
    def __init__(self, shapes: list[Shape], g: float):
        self.shapes = shapes
        self.g = g
        x0, y0, x1, y1 = bs.BOARD_OUTLINE
        self.x0, self.y0 = x0, y0
        self.nx, self.ny = int(round((x1 - x0) / g)) + 1, int(round((y1 - y0) / g)) + 1

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return int(round((x - self.x0) / self.g)), int(round((y - self.y0) / self.g))

    def xy(self, i: int, j: int) -> tuple[float, float]:
        return self.x0 + i * self.g, self.y0 + j * self.g

    def fields(self, win, net: str, reach: float):
        """Per layer: distance to other-net copper with high / low clearance classes, to
        own-net copper, to holes; restricted to the window ``win`` = (i0, j0, i1, j1)."""
        i0, j0, i1, j1 = win
        xs = self.x0 + np.arange(i0, i1) * self.g
        ys = self.y0 + np.arange(j0, j1) * self.g
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        inf = np.full(X.shape, 1e9)
        hi = [inf.copy(), inf.copy()]
        lo = [inf.copy(), inf.copy()]
        own = [inf.copy(), inf.copy()]
        holes = inf.copy()
        wx0, wy0, wx1, wy1 = xs[0] - reach, ys[0] - reach, xs[-1] + reach, ys[-1] + reach
        for s in self.shapes:
            bx0, by0, bx1, by1 = s.bbox()
            if bx1 < wx0 or bx0 > wx1 or by1 < wy0 or by0 > wy1:
                continue
            a0 = max(0, int((bx0 - reach - xs[0]) / self.g))
            a1 = min(len(xs), int((bx1 + reach - xs[0]) / self.g) + 2)
            b0 = max(0, int((by0 - reach - ys[0]) / self.g))
            b1 = min(len(ys), int((by1 + reach - ys[0]) / self.g) + 2)
            if a0 >= a1 or b0 >= b1:
                continue
            d = s.dist(X[a0:a1, b0:b1], Y[a0:a1, b0:b1])
            if s.hole:
                cx, cy = (s.a[0], s.a[1])
                hd = np.hypot(X[a0:a1, b0:b1] - cx, Y[a0:a1, b0:b1] - cy) - s.hole / 2
                np.minimum(holes[a0:a1, b0:b1], hd, out=holes[a0:a1, b0:b1])
            for L in s.layers:
                if s.net == net:
                    np.minimum(own[L][a0:a1, b0:b1], d, out=own[L][a0:a1, b0:b1])
                else:
                    tgt = hi if (s.net.startswith("#") or clearance_of(s.net) >= HI_CLEARANCE) else lo
                    np.minimum(tgt[L][a0:a1, b0:b1], d, out=tgt[L][a0:a1, b0:b1])
        return X, Y, hi, lo, own, holes

    def keepout(self, X, Y, net: str, layer: int) -> np.ndarray:
        x0, y0, x1, y1 = bs.BOARD_OUTLINE
        k = np.zeros(X.shape, bool)
        r = bs.BOARD_CORNER_R
        # outline with rounded corners: distance to the inside of the rounded rectangle
        cx = np.clip(X, x0 + r, x1 - r)
        cy = np.clip(Y, y0 + r, y1 - r)
        inside = np.hypot(X - cx, Y - cy) <= r
        k |= ~inside
        for hx, hy in bs.HEATSINK_HOLES.values():            # H1..H4 keep-out rule areas
            k |= np.hypot(X - hx, Y - hy) < bs.HEATSINK_HOLE_KEEPOUT_R
        if layer == 1 and net != bs.GND:                     # bottom thermal-pad mask window
            tv = bs.THERMAL_VIA
            h = (tv["grid"] - 1) / 2 * tv["pitch_mm"] + tv["pad_mm"] / 2
            k |= (np.abs(X) < h) & (np.abs(Y) < h)
        return k, (np.minimum(np.minimum(X - x0, x1 - X), np.minimum(Y - y0, y1 - Y)))

    def route(self, net: str, a: tuple, b: tuple, margin: float) -> tuple[list, list] | None:
        """Route net from the copper island at ``a`` to the one at ``b`` ((x, y, layers))."""
        w, c = net_rules(net)
        pad = WINDOW_MM
        ia, ja = self.cell(a[0], a[1])
        ib, jb = self.cell(b[0], b[1])
        win = (max(0, min(ia, ib) - int(pad / self.g)), max(0, min(ja, jb) - int(pad / self.g)),
               min(self.nx, max(ia, ib) + int(pad / self.g) + 1), min(self.ny, max(ja, jb) + int(pad / self.g) + 1))
        reach = w / 2 + HI_CLEARANCE + VIA_D + 0.5
        X, Y, hi, lo, own, holes = self.fields(win, net, reach)
        shape = X.shape
        free, via_free, own_m = [], [], []
        vr = VIA_D / 2
        for L in (0, 1):
            k, edge = self.keepout(X, Y, net, L)
            f = (hi[L] >= w / 2 + max(c, HI_CLEARANCE) + margin) & (lo[L] >= w / 2 + max(c, 0.15) + margin)
            f &= ~k & (edge >= EDGE_MM + w / 2 + margin)
            v = (hi[L] >= vr + max(c, HI_CLEARANCE) + margin) & (lo[L] >= vr + max(c, 0.15) + margin)
            v &= ~k & (edge >= EDGE_MM + vr + margin)
            o = own[L] <= 0
            free.append(f | o)
            via_free.append(v)
            own_m.append(o)
        via_ok = via_free[0] & via_free[1] & (holes >= VIA_DRILL / 2 + HOLE_TO_HOLE + margin)
        # own-net islands (layers joined where an own via / THT pad covers both)
        lab = [ndimage.label(own_m[L], structure=np.ones((3, 3)))[0] for L in (0, 1)]
        n0 = lab[0].max()
        comp = [lab[0], np.where(lab[1] > 0, lab[1] + n0, 0)]
        parent = list(range(n0 + lab[1].max() + 1))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        both = (comp[0] > 0) & (comp[1] > 0)
        for p, q in zip(comp[0][both], comp[1][both]):
            rp, rq = find(p), find(q)
            if rp != rq:
                parent[rq] = rp

        def island(pt):
            i, j = self.cell(pt[0], pt[1])
            i, j = i - win[0], j - win[1]
            if not (0 <= i < shape[0] and 0 <= j < shape[1]):
                return None
            for L in pt[2]:
                for di in range(0, 4):              # the item's own cell, else the nearest own cell
                    sl = comp[L][max(0, i - di):i + di + 1, max(0, j - di):j + di + 1]
                    ids = sl[sl > 0]
                    if ids.size:
                        return find(int(ids[0]))
            return None
        ra, rb = island(a), island(b)
        if ra is None or rb is None or ra == rb:
            return None
        root_of = np.array([find(i) for i in range(len(parent))])
        root_of[0] = 0
        R = [root_of[comp[0]], root_of[comp[1]]]
        src = [(R[L] == ra) for L in (0, 1)]
        dst = [(R[L] == rb) for L in (0, 1)]
        # graph over the window: node id = L * N + i * ny + j
        nxw, nyw = shape
        N = nxw * nyw
        ids = np.arange(N).reshape(shape)
        rows, cols, wts = [], [], []
        for L in (0, 1):
            fr = free[L]
            for di, dj, cost in ((1, 0, 1.0), (0, 1, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2))):
                a_sl = (slice(max(0, -di), nxw - max(0, di)), slice(max(0, -dj), nyw - max(0, dj)))
                b_sl = (slice(a_sl[0].start + di, a_sl[0].stop + di), slice(a_sl[1].start + dj, a_sl[1].stop + dj))
                m = fr[a_sl] & fr[b_sl]
                if di and dj:       # no corner cutting: both orthogonal cells free too
                    m &= fr[a_sl[0], b_sl[1]] & fr[b_sl[0], a_sl[1]]
                u = ids[a_sl][m] + L * N
                v = ids[b_sl][m] + L * N
                # moving inside own copper is almost free (existing tracks, pads)
                inside = own_m[L][a_sl][m] & own_m[L][b_sl][m]
                wgt = np.where(inside, 0.01, cost)
                rows += [u, v]
                cols += [v, u]
                wts += [wgt, wgt]
        vm = via_ok | (own_m[0] & own_m[1])
        vi = ids[vm]
        vw = np.where((own_m[0] & own_m[1])[vm], 0.01, VIA_COST)
        rows += [vi, vi + N]
        cols += [vi + N, vi]
        wts += [vw, vw]
        S = 2 * N                                              # super source
        s_ids = np.concatenate([ids[src[0]], ids[src[1]] + N])
        rows.append(np.full(s_ids.size, S))
        cols.append(s_ids)
        wts.append(np.full(s_ids.size, 1e-6))
        G = csr_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))), shape=(S + 1, S + 1))
        dist, pred = dijkstra(G, directed=True, indices=S, return_predecessors=True)
        t_ids = np.concatenate([ids[dst[0]], ids[dst[1]] + N])
        if t_ids.size == 0:
            return None
        best = t_ids[np.argmin(dist[t_ids])]
        if not np.isfinite(dist[best]):
            return None
        path = []
        node = best
        while node != S and node >= 0:
            path.append(node)
            node = pred[node]
        path.reverse()
        # to geometry: runs of straight steps per layer, vias where the layer changes
        pts = []
        for node in path:
            L, rem = divmod(int(node), N)
            i, j = divmod(rem, nyw)
            pts.append((L, i, j, bool(own_m[L][i, j])))
        segs, vias = [], []
        k = 0
        while k < len(pts) - 1:
            L, i, j, o = pts[k]
            L2, i2, j2, o2 = pts[k + 1]
            if L2 != L:
                if not (own_m[0][i, j] and own_m[1][i, j]):
                    vias.append(self.xy(i + win[0], j + win[1]))
                k += 1
                continue
            if o and o2:                    # travelling along existing own copper
                k += 1
                continue
            di, dj = i2 - i, j2 - j
            m = k + 1
            while m + 1 < len(pts) and pts[m + 1][0] == L and (pts[m + 1][1] - pts[m][1], pts[m + 1][2] - pts[m][2]) == (di, dj) \
                    and not (pts[m][3] and pts[m + 1][3]):
                m += 1
            segs.append((L, self.xy(i + win[0], j + win[1]), self.xy(pts[m][1] + win[0], pts[m][2] + win[1]), w))
            k = m
        return segs, vias

    def commit(self, net: str, segs: list, vias: list) -> None:
        for L, (x0, y0), (x1, y1), w in segs:
            self.shapes.append(Shape("capsule", (L,), net, (x0, y0, x1, y1, w / 2)))
        for x, y in vias:
            self.shapes.append(Shape("circle", (0, 1), net, (x, y, VIA_D / 2), hole=VIA_DRILL, via=True))


# ==========================================================================
# DRC pairs, board file output
# ==========================================================================

_ITEM = re.compile(r"\[([^\]]*)\]")


def item_endpoint(item: dict) -> tuple[str, tuple] | None:
    desc = item.get("description", "")
    m = _ITEM.search(desc)
    if not m:
        return None
    net = m.group(1)
    x, y = item["pos"]["x"] - ORIGIN[0], item["pos"]["y"] - ORIGIN[1]
    if desc.startswith(("PTH pad", "Via")) and "Micro" not in desc:
        layers = (0, 1)
    elif "Signal_Bottom" in desc and "Signal_Top" not in desc:
        layers = (1,)
    elif "Signal_Top" in desc:
        layers = (0,) if "Signal_Bottom" not in desc else (0, 1)
    else:
        layers = (0, 1)
    return net, (x, y, layers)


def sexpr_items(segs: list, vias: list, net: str) -> list[str]:
    ox, oy = ORIGIN
    out = []
    for L, (x0, y0), (x1, y1), w in segs:
        key = f"{net}:{L}:{x0:.4f},{y0:.4f}:{x1:.4f},{y1:.4f}"
        out.append(f'\t(segment\n\t\t(start {x0 + ox:.4f} {y0 + oy:.4f})\n\t\t(end {x1 + ox:.4f} {y1 + oy:.4f})\n'
                   f'\t\t(width {w:g})\n\t\t(layer "{LAYERS[L]}")\n\t\t(net "{net}")\n'
                   f'\t\t(uuid "{uuid.uuid5(UUID_NS, key)}")\n\t)')
    for x, y in vias:
        key = f"{net}:via:{x:.4f},{y:.4f}"
        out.append(f'\t(via\n\t\t(at {x + ox:.4f} {y + oy:.4f})\n\t\t(size {VIA_D:g})\n\t\t(drill {VIA_DRILL:g})\n'
                   f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "{net}")\n\t\t(uuid "{uuid.uuid5(UUID_NS, key)}")\n\t)')
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("board", type=Path)
    ap.add_argument("drc", type=Path)
    ap.add_argument("-o", "--out", type=Path, required=True)
    ap.add_argument("--grid", type=float, default=0.05)
    ap.add_argument("--margin", type=float, default=0.012)
    args = ap.parse_args(argv)
    text = args.board.read_text()
    plan = lp.build_plan()
    grid = Grid(plan_pads(plan) + board_copper(text), args.grid)
    pairs = []
    for u in json.loads(args.drc.read_text()).get("unconnected_items", []):
        ends = [item_endpoint(i) for i in u.get("items", [])]
        if len(ends) == 2 and all(ends) and ends[0][0] == ends[1][0]:
            pairs.append((ends[0][0], ends[0][1], ends[1][1]))
    pairs.sort(key=lambda p: math.dist(p[1][:2], p[2][:2]))       # short ones first
    new, done, failed = [], 0, []
    for net, a, b in pairs:
        res = grid.route(net, a, b, args.margin)
        if res is None:
            failed.append(f"{net} ({a[0]:.2f},{a[1]:.2f})-({b[0]:.2f},{b[1]:.2f})")
            continue
        segs, vias = res
        grid.commit(net, segs, vias)
        new += sexpr_items(segs, vias, net)
        done += 1
        print(f"  routed {net}: {len(segs)} segments, {len(vias)} vias", flush=True)
    body = text.rstrip()
    assert body.endswith(")")
    args.out.write_text(body[:-1].rstrip() + "\n" + "\n".join(new) + "\n)\n")
    print(f"{done} of {len(pairs)} connections routed, {len(failed)} failed")
    for f in failed:
        print("  FAILED " + f)
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
