/* Search core of tools/pcb_router.py (compiled on first use; plain C99, no deps).
 *
 * fields(): exact distance from every cell centre of a window to the copper shapes
 *           passed in (the caller has already left out the net being routed), one
 *           field per (group, clearance class, layer), plus the distance to holes.
 * astar():  A* over a two-layer grid with a direction per state, so a path pays for
 *           its bends: 45-degree turns cost a little, 90-degree turns more (or are
 *           forbidden), sharper turns are never taken. Layer changes are vias.
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ---------------------------------------------------------------- distances */

/* shape kinds: 0 = (rotated) rounded rectangle  p = cx cy hx hy angle(rad) corner_r
 *              1 = circle                        p = cx cy r
 *              2 = capsule (track)               p = x0 y0 x1 y1 r                     */
static double shape_dist(int kind, const double *p, double x, double y)
{
    if (kind == 0) {
        double dx = x - p[0], dy = y - p[1];
        if (p[4] != 0.0) {
            double c = cos(p[4]), s = sin(p[4]);
            double u = dx * c + dy * s, v = -dx * s + dy * c;
            dx = u;
            dy = v;
        }
        double r = p[5];
        double ex = fabs(dx) - (p[2] - r), ey = fabs(dy) - (p[3] - r);
        double ox = ex > 0 ? ex : 0, oy = ey > 0 ? ey : 0;
        double d = sqrt(ox * ox + oy * oy) - r;
        return d > 0 ? d : 0;
    }
    if (kind == 1) {
        double d = hypot(x - p[0], y - p[1]) - p[2];
        return d > 0 ? d : 0;
    }
    double vx = p[2] - p[0], vy = p[3] - p[1];
    double L = vx * vx + vy * vy, u = 0;
    if (L > 0) {
        u = ((x - p[0]) * vx + (y - p[1]) * vy) / L;
        u = u < 0 ? 0 : (u > 1 ? 1 : u);
    }
    double d = hypot(p[0] + u * vx - x, p[1] + u * vy - y) - p[4];
    return d > 0 ? d : 0;
}

/* out: ngrp * ncls * 2 layers * nx * ny floats (caller fills with a large value)
 * hole_out: nx * ny floats, distance to the nearest hole edge (all nets)
 * bbox: 4 doubles per shape (x0 y0 x1 y1) */
void fields(int ns, const int *kind, const int *layers, const int *cls, const int *grp,
            const double *par, const double *bbox, const double *hole,
            double wx0, double wy0, double g, int nx, int ny, double reach,
            int ncls, float *out, float *hole_out)
{
    size_t N = (size_t)nx * ny;
    for (int s = 0; s < ns; s++) {
        const double *b = bbox + 4 * s, *p = par + 6 * s;
        int i0 = (int)floor((b[0] - reach - wx0) / g), i1 = (int)ceil((b[2] + reach - wx0) / g);
        int j0 = (int)floor((b[1] - reach - wy0) / g), j1 = (int)ceil((b[3] + reach - wy0) / g);
        if (i0 < 0) i0 = 0;
        if (j0 < 0) j0 = 0;
        if (i1 > nx - 1) i1 = nx - 1;
        if (j1 > ny - 1) j1 = ny - 1;
        if (i0 > i1 || j0 > j1) continue;
        const double *h = hole + 3 * s;
        for (int i = i0; i <= i1; i++) {
            double x = wx0 + i * g;
            for (int j = j0; j <= j1; j++) {
                double y = wy0 + j * g;
                size_t c = (size_t)i * ny + j;
                if (h[2] > 0) {
                    double hd = hypot(x - h[0], y - h[1]) - h[2];
                    if (hd < hole_out[c]) hole_out[c] = (float)hd;
                }
                if (layers[s] == 0 || cls[s] < 0) continue;
                float d = (float)shape_dist(kind[s], p, x, y);
                for (int L = 0; L < 2; L++) {
                    if (!(layers[s] & (1 << L))) continue;
                    float *f = out + (((size_t)grp[s] * ncls + cls[s]) * 2 + L) * N;
                    if (d < f[c]) f[c] = d;
                }
            }
        }
    }
}

/* ---------------------------------------------------------------- A* search */

static const int DI[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int DJ[8] = {0, 1, 1, 1, 0, -1, -1, -1};

typedef struct { float f; int32_t s; } node;

typedef struct { node *a; size_t n, cap; } heap;

static int hpush(heap *h, float f, int32_t s)
{
    if (h->n == h->cap) {
        size_t cap = h->cap ? h->cap * 2 : 1 << 16;
        node *a = realloc(h->a, cap * sizeof(node));
        if (!a) return -1;
        h->a = a;
        h->cap = cap;
    }
    size_t k = h->n++;
    while (k > 0) {
        size_t p = (k - 1) / 2;
        if (h->a[p].f <= f) break;
        h->a[k] = h->a[p];
        k = p;
    }
    h->a[k].f = f;
    h->a[k].s = s;
    return 0;
}

static node hpop(heap *h)
{
    node top = h->a[0], last = h->a[--h->n];
    size_t k = 0;
    for (;;) {
        size_t c = 2 * k + 1;
        if (c >= h->n) break;
        if (c + 1 < h->n && h->a[c + 1].f < h->a[c].f) c++;
        if (last.f <= h->a[c].f) break;
        h->a[k] = h->a[c];
        k = c;
    }
    if (h->n) h->a[k] = last;
    return top;
}

/* cost: 2 * nx * ny, extra cost per unit length entering a cell (< 0 = blocked)
 * via: nx * ny, 1 = a via may stand here (both layers must be passable too)
 * src / dst: 2 * nx * ny flags; tbox: target bounding box i0 j0 i1 j1 (heuristic)
 * turn45 / turn90: bend costs (turn90 < 0 forbids right angles)
 * path: out, (layer, i, j) triples from source to target; returns their count,
 * 0 = no path (the search reached the window border), -3 = no path and the search never
 * reached the border (a bigger window cannot help), -1 = out of memory, -2 = path longer
 * than max_path */
int astar(int nx, int ny, const float *cost, const uint8_t *via, float via_cost,
          const uint8_t *src, const uint8_t *dst, const int *tbox,
          float turn45, float turn90, long max_expand, int *path, int max_path)
{
    size_t N = (size_t)nx * ny, S = 2 * N * 9;
    float *gs = malloc(S * sizeof(float));
    int32_t *par = malloc(S * sizeof(int32_t));
    heap h = {0};
    int result = 0;
    if (!gs || !par) { free(gs); free(par); return -1; }
    for (size_t k = 0; k < S; k++) gs[k] = INFINITY;
    const float R2 = 1.41421356f;
#define HEUR(i, j) ({ int dx_ = (i) < tbox[0] ? tbox[0] - (i) : ((i) > tbox[2] ? (i) - tbox[2] : 0); \
                      int dy_ = (j) < tbox[1] ? tbox[1] - (j) : ((j) > tbox[3] ? (j) - tbox[3] : 0); \
                      int mn_ = dx_ < dy_ ? dx_ : dy_, mx_ = dx_ < dy_ ? dy_ : dx_; \
                      (float)(mx_ - mn_) + R2 * (float)mn_; })
    for (int L = 0; L < 2; L++)
        for (size_t c = 0; c < N; c++)
            if (src[L * N + c] && cost[L * N + c] >= 0) {
                int32_t s = (int32_t)(((L * N + c) * 9) + 8);
                gs[s] = 0;
                par[s] = -1;
                int i = (int)(c / ny), j = (int)(c % ny);
                if (hpush(&h, HEUR(i, j), s)) { result = -1; goto done; }
            }
    long expanded = 0;
    int32_t goal = -1;
    int border = 0;
    while (h.n) {
        node nd = hpop(&h);
        int32_t s = nd.s;
        int d = s % 9;
        size_t lc = (size_t)s / 9;
        int L = (int)(lc / N);
        size_t c = lc % N;
        int i = (int)(c / ny), j = (int)(c % ny);
        float g0 = gs[s];
        if (nd.f > g0 + HEUR(i, j) + 1e-4f) continue;          /* stale entry */
        if (dst[lc]) { goal = s; break; }
        if (++expanded > max_expand) { border = 1; break; }
        if (i == 0 || j == 0 || i == nx - 1 || j == ny - 1) border = 1;
        for (int nd8 = 0; nd8 < 8; nd8++) {
            float bend = 0;
            if (d != 8) {
                int t = abs(nd8 - d);
                if (t > 4) t = 8 - t;
                if (t >= 3) continue;
                if (t == 2) { if (turn90 < 0) continue; bend = turn90; }
                else if (t == 1) bend = turn45;
            }
            int i2 = i + DI[nd8], j2 = j + DJ[nd8];
            if (i2 < 0 || j2 < 0 || i2 >= nx || j2 >= ny) continue;
            size_t c2 = (size_t)i2 * ny + j2;
            float cc = cost[L * N + c2];
            if (cc < 0) continue;
            float len = 1.0f;
            if (DI[nd8] && DJ[nd8]) {                           /* no corner cutting */
                if (cost[L * N + (size_t)i2 * ny + j] < 0 || cost[L * N + (size_t)i * ny + j2] < 0) continue;
                len = R2;
            }
            float g2 = g0 + len * (1.0f + cc) + bend;
            int32_t s2 = (int32_t)((((size_t)L * N + c2) * 9) + nd8);
            if (g2 < gs[s2]) {
                gs[s2] = g2;
                par[s2] = s;
                if (hpush(&h, g2 + HEUR(i2, j2), s2)) { result = -1; goto done; }
            }
        }
        if (via[c]) {
            int L2 = 1 - L;
            if (cost[L2 * N + c] >= 0) {
                float g2 = g0 + via_cost;
                int32_t s2 = (int32_t)((((size_t)L2 * N + c) * 9) + 8);
                if (g2 < gs[s2]) {
                    gs[s2] = g2;
                    par[s2] = s;
                    if (hpush(&h, g2 + HEUR(i, j), s2)) { result = -1; goto done; }
                }
            }
        }
    }
    if (goal >= 0) {
        int n = 0;
        for (int32_t s = goal; s >= 0; s = par[s]) n++;
        if (n > max_path) { result = -2; goto done; }
        int k = n;
        for (int32_t s = goal; s >= 0; s = par[s]) {
            size_t lc = (size_t)s / 9;
            k--;
            path[3 * k] = (int)(lc / N);
            path[3 * k + 1] = (int)((lc % N) / ny);
            path[3 * k + 2] = (int)((lc % N) % ny);
        }
        result = n;
    } else if (!border) {
        result = -3;
    }
done:
    free(gs);
    free(par);
    free(h.a);
    return result;
}
