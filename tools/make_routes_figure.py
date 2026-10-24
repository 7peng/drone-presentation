"""Small-multiple route figure for the generalisation slide (inline SVG, deck fonts and tokens).

One representative route per shape (the first of each in eval_cl/missions_complex.json), drawn in ink with the home
square and the GPS-jam point. Shape names are labels, not colours. Writes between the routes-figure markers in
index.html.

    python3 tools/make_routes_figure.py
"""
import json
import math
import os
import re

SRC = "/scratch/users/wgpeng/drone-work/eval_cl/missions_complex.json"
INDEX = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "index.html")
NAMES = {"zigzag": "Zig-zag", "lawnmower": "Survey", "orbit": "Orbit", "crossing": "Self-crossing"}

M = json.load(open(SRC))


def pts(home, legs):
    p = [tuple(home)]
    for hd, ln, *_ in legs:
        h = math.radians(hd)
        p.append((p[-1][0] + ln * math.cos(h), p[-1][1] + ln * math.sin(h)))
    return p   # (north, east)


def jam_point(P, frac):
    seg = [0.0]
    for a, b in zip(P, P[1:]):
        seg.append(seg[-1] + math.dist(a, b))
    s = frac * seg[-1]
    for i in range(len(P) - 1):
        if seg[i + 1] >= s:
            t = (s - seg[i]) / max(seg[i + 1] - seg[i], 1e-9)
            return (P[i][0] + t * (P[i + 1][0] - P[i][0]), P[i][1] + t * (P[i + 1][1] - P[i][1]))
    return P[-1]


CELL, PAD, LAB = 200, 18, 34
out = [f'<svg class="routes" viewBox="0 0 {2 * CELL + 30} {2 * (CELL + LAB) + 10}" role="img" '
       f'aria-label="Four example complex routes: zig-zag, survey, orbit, self-crossing">']
for k, shape in enumerate(NAMES):
    m = next(x for x in M if x["shape"] == shape)
    P = pts(m["home_ne"], m["legs"])
    ns, es = [p[0] for p in P], [p[1] for p in P]
    span = max(max(ns) - min(ns), max(es) - min(es))
    sc = (CELL - 2 * PAD) / span
    ox, oy = (k % 2) * (CELL + 30), (k // 2) * (CELL + LAB + 10)
    cx, cy = (min(es) + max(es)) / 2, (min(ns) + max(ns)) / 2

    def xy(p):
        return ox + CELL / 2 + (p[1] - cx) * sc, oy + CELL / 2 - (p[0] - cy) * sc

    out.append(f'<rect x="{ox}" y="{oy}" width="{CELL}" height="{CELL}" class="cell"/>')
    out.append('<polyline class="route" points="' + " ".join(f"{a:.1f},{b:.1f}" for a, b in map(xy, P)) + '"/>')
    hx, hy = xy(P[0])
    out.append(f'<rect x="{hx - 7:.1f}" y="{hy - 7:.1f}" width="14" height="14" class="home"/>')
    if m.get("jam_frac", 1.0) < 1.0:
        jx, jy = xy(jam_point(P, m["jam_frac"]))
    else:
        jx, jy = xy(P[-1])
    out.append(f'<path d="M{jx - 8:.1f},{jy - 8:.1f}L{jx + 8:.1f},{jy + 8:.1f}M{jx + 8:.1f},{jy - 8:.1f}L{jx - 8:.1f},{jy + 8:.1f}" class="jam"/>')
    out.append(f'<text x="{ox}" y="{oy + CELL + 26}" class="name">{NAMES[shape]}</text>')
out.append("</svg>")
svg = "\n".join(out)

html = open(INDEX).read()
pat = re.compile(r"(<!-- routes-figure:start -->)(.*?)(<!-- routes-figure:end -->)", re.S)
assert pat.search(html), "markers not found"
html = pat.sub(lambda mm: mm.group(1) + "\n" + svg + "\n" + mm.group(3), html)
open(INDEX, "w").write(html)
km = [x["out_m"] / 1000 for x in M]
print(f"{len(M)} routes, {min(km):.1f}-{max(km):.1f} km, mid-route jams: {sum(x.get('jam_frac', 1) < 1 for x in M)}")
