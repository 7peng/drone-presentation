"""Slide 3: 2D plot of the cx_011 closed-loop run (v3): whole route + breadcrumbs, and a zoom on one return frame with
the model's predicted vector (fix -> breadcrumb), its 1-sigma circle, and the true drone position. Writes inline SVG
between the SLIDE3-PLOT markers in index.html."""
import json
import math
import re
import sys

import numpy as np

RUN = "/scratch/users/wgpeng/drone-work/eval_cl/sweep_cx_v3/cx_011/"
T_PICK = float(sys.argv[1]) if len(sys.argv) > 1 else None
INK, MUTED, HAIR, BLUE, GREEN = "#202630", "#626b76", "#c9d1d8", "#315c91", "#2e8b57"

z = np.load(RUN + "log.npz")
fr = json.load(open(RUN + "frames.json"))
tr = z["traj"]
ph = tr[:, 11]
out = tr[ph == 1][:, [1, 2]]                    # (n, e)
ret = tr[ph >= 2][:, [1, 2]]
est = tr[ph >= 2][:, [4, 5]]
bcs = z["bc_p_true"][:, :2]
home = z["home"]
jam = out[-1]

cand = [f for f in fr if f["status"] == "fused" and 17 < math.hypot(*f["true_off"]) < 30 and f["fix_err"] < 3]
f = min(cand, key=lambda f: abs(f["t"] - T_PICK)) if T_PICK else cand[len(cand) // 2]
k, P, fix, off = f["k"], np.array(f["p_true"]), np.array(f["fix"]), np.array(f["pred_off"])
bc = fix + off                                   # where the model says breadcrumb k is, relative to its own fix
sig = f["sigma"]


def mapper(lo_n, hi_n, lo_e, hi_e, x0, y0, w, h):
    s = min(w / (hi_e - lo_e), h / (hi_n - lo_n))
    cx, cy = x0 + w / 2, y0 + h / 2
    mn, me = (lo_n + hi_n) / 2, (lo_e + hi_e) / 2
    return (lambda n, e: (cx + (e - me) * s, cy - (n - mn) * s)), s


def poly(pts, M, step=1):
    return " ".join(f"{x:.1f},{y:.1f}" for x, y in (M(n, e) for n, e in pts[::step]))


svg = []
# ---- panel A: whole route
allp = np.vstack([out, ret])
pad = 120
A, sA = mapper(allp[:, 0].min() - pad, allp[:, 0].max() + pad, allp[:, 1].min() - pad, allp[:, 1].max() + pad, 40, 40, 600, 560)
svg.append(f'<polyline points="{poly(out, A, 5)}" fill="none" stroke="{HAIR}" stroke-width="3"/>')
for n, e in bcs[::2]:
    x, y = A(n, e)
    svg.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2.6" fill="{BLUE}" opacity=".55"/>')
svg.append(f'<polyline points="{poly(ret, A, 5)}" fill="none" stroke="{INK}" stroke-width="2.4"/>')
hx, hy = A(*home)
svg.append(f'<rect x="{hx - 8:.1f}" y="{hy - 8:.1f}" width="16" height="16" fill="{INK}"/>')
svg.append(f'<text x="{hx + 16:.1f}" y="{hy + 7:.1f}" class="pl">home</text>')
jx, jy = A(*jam)
svg.append(f'<path d="M{jx - 9:.1f},{jy - 9:.1f} l18,18 m0,-18 l-18,18" stroke="#c0392b" stroke-width="4"/>')
svg.append(f'<text x="{jx:.1f}" y="{jy - 20:.1f}" class="pl h" text-anchor="start">GPS jammed</text>')
ZW = 24                                          # zoom half-width (m)
zx0, zy0 = A(P[0] + ZW, P[1] - ZW)
zx1, zy1 = A(P[0] - ZW, P[1] + ZW)
zc = ((zx0 + zx1) / 2, (zy0 + zy1) / 2)
svg.append(f'<circle cx="{zc[0]:.1f}" cy="{zc[1]:.1f}" r="14" fill="none" stroke="{GREEN}" stroke-width="3"/>')
X0 = 700 + (900 - 560) / 2
svg.append(f'<path d="M{zc[0] + 10:.1f},{zc[1] - 10:.1f} L{X0:.1f},40 M{zc[0] + 10:.1f},{zc[1] + 10:.1f} L{X0:.1f},600" stroke="{GREEN}" stroke-width="1.5" stroke-dasharray="4 6"/>')
bx = 40 + 0
svg.append(f'<path d="M{bx},590 h{500 * sA:.1f}" stroke="{MUTED}" stroke-width="2"/><text x="{bx}" y="616" class="pl m">500 m</text>')

# ---- panel B: zoom
B, sB = mapper(P[0] - ZW, P[0] + ZW, P[1] - ZW, P[1] + ZW, 700, 40, 900, 560)
X0 = 700 + (900 - 560) / 2
svg.append(f'<rect x="{X0:.1f}" y="40" width="560" height="560" fill="#f6f8fa" stroke="{GREEN}" stroke-width="3" rx="6"/>')
svg.append(f'<clipPath id="zc"><rect x="{X0:.1f}" y="40" width="560" height="560" rx="6"/></clipPath><g clip-path="url(#zc)">')
near = lambda p: np.all(np.abs(p - P) < ZW * 1.6, axis=1)
o2 = out[near(out)]
svg.append(f'<polyline points="{poly(o2, B)}" fill="none" stroke="{HAIR}" stroke-width="5"/>')
r2 = ret[near(ret)]
svg.append(f'<polyline points="{poly(r2, B)}" fill="none" stroke="{INK}" stroke-width="4"/>')
for i, (n, e) in enumerate(bcs):
    if np.all(np.abs(np.array([n, e]) - P) < ZW * 1.2):
        x, y = B(n, e)
        svg.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{14 if i == k else 9}" fill="{BLUE}" opacity="{1 if i == k else .45}"/>')
# model fix with 1-sigma circle, and the predicted vector fix -> breadcrumb k
fx, fy = B(*fix)
svg.append(f'<circle cx="{fx:.1f}" cy="{fy:.1f}" r="{sig * sB:.1f}" fill="{GREEN}" fill-opacity=".12" stroke="{GREEN}" stroke-width="2" stroke-dasharray="6 5"/>')
kx, ky = B(*bc)
ang = math.atan2(ky - fy, kx - fx)
L = math.hypot(kx - fx, ky - fy) - 14
ex, ey = fx + L * math.cos(ang), fy + L * math.sin(ang)
svg.append(f'<line x1="{fx:.1f}" y1="{fy:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" stroke="{GREEN}" stroke-width="6"/>')
a1, a2 = ang + 2.6, ang - 2.6
svg.append(f'<path d="M{ex + 16 * math.cos(ang):.1f},{ey + 16 * math.sin(ang):.1f} L{ex + 18 * math.cos(a1):.1f},{ey + 18 * math.sin(a1):.1f} L{ex + 18 * math.cos(a2):.1f},{ey + 18 * math.sin(a2):.1f} z" fill="{GREEN}"/>')
svg.append(f'<circle cx="{fx:.1f}" cy="{fy:.1f}" r="8" fill="{GREEN}"/>')
px, py = B(*P)
svg.append(f'<g transform="translate({px:.1f},{py:.1f})" stroke="{INK}" stroke-width="3" fill="#fff">'
           '<line x1="-11" y1="-11" x2="11" y2="11"/><line x1="-11" y1="11" x2="11" y2="-11"/>'
           '<circle cx="-11" cy="-11" r="6"/><circle cx="11" cy="11" r="6"/><circle cx="-11" cy="11" r="6"/><circle cx="11" cy="-11" r="6"/></g>')
svg.append("</g>")
mid = ((fx + kx) / 2, (fy + ky) / 2)
nx, ny = -math.sin(ang), math.cos(ang)
svg.append(f'<text x="{X0 + 24:.1f}" y="84" class="pl g">model → breadcrumb #{k}: {math.hypot(*off):.0f} m</text>')
svg.append(f'<path d="M{X0 + 20:.1f},580 h{10 * sB:.1f}" stroke="{MUTED}" stroke-width="2"/><text x="{X0 + 20:.1f}" y="572" class="pl m">10 m</text>')

# ---- legend (right column)
LX = X0 + 560 + 40
items = [("line", HAIR, 5, "outbound (GPS)"), ("dot", BLUE, 0, "breadcrumb, every 20 m"), ("line", INK, 3, "return (GPS jammed)"),
         ("arrow", GREEN, 6, "model's vector"), ("ring", GREEN, 2, "fix ± 1σ"), ("drone", INK, 3, "true drone")]
for i, (kind, col, w, lab) in enumerate(items):
    y = 70 + i * 52
    if kind == "line":
        svg.append(f'<line x1="{LX}" y1="{y}" x2="{LX + 34}" y2="{y}" stroke="{col}" stroke-width="{w}"/>')
    elif kind == "dot":
        svg.append(f'<circle cx="{LX + 17}" cy="{y}" r="8" fill="{col}"/>')
    elif kind == "arrow":
        svg.append(f'<line x1="{LX}" y1="{y}" x2="{LX + 26}" y2="{y}" stroke="{col}" stroke-width="5"/><path d="M{LX + 38},{y} l-14,-8 v16 z" fill="{col}"/>')
    elif kind == "ring":
        svg.append(f'<circle cx="{LX + 17}" cy="{y}" r="13" fill="{col}" fill-opacity=".12" stroke="{col}" stroke-width="2" stroke-dasharray="5 4"/><circle cx="{LX + 17}" cy="{y}" r="5" fill="{col}"/>')
    else:
        svg.append(f'<g transform="translate({LX + 17},{y}) scale(.7)" stroke="{col}" stroke-width="3" fill="#fff"><line x1="-11" y1="-11" x2="11" y2="11"/><line x1="-11" y1="11" x2="11" y2="-11"/><circle cx="-11" cy="-11" r="6"/><circle cx="11" cy="11" r="6"/><circle cx="-11" cy="11" r="6"/><circle cx="11" cy="-11" r="6"/></g>')
    svg.append(f'<text x="{LX + 52}" y="{y + 8}" class="pl">{lab}</text>')

body = "\n      ".join(svg)
out_svg = f'''<svg class="plot3" viewBox="0 0 1920 640" role="img" aria-label="Route cx_011 with breadcrumbs, and a close-up of one return frame showing the model's vector from its position fix to the matched breadcrumb">
      <style>.pl{{font:500 26px 'Google Sans','Noto Sans',sans-serif;fill:{INK}}} .pl.m{{fill:{MUTED};font-size:22px}} .pl.g{{fill:{GREEN};font-weight:600;paint-order:stroke;stroke:#f6f8fa;stroke-width:10px}} .pl.h{{paint-order:stroke;stroke:#fff;stroke-width:10px}}</style>
      {body}
    </svg>'''
p = "/scratch/users/wgpeng/drone-presentation/index.html"
s = open(p).read()
s = re.sub(r"<!-- SLIDE3-PLOT -->.*?<!-- /SLIDE3-PLOT -->", lambda m: "<!-- SLIDE3-PLOT -->\n    " + out_svg + "\n    <!-- /SLIDE3-PLOT -->", s, flags=re.S)
open(p, "w").write(s)
print("frame t", round(f["t"]), "k", k, "pred", np.round(off, 1), "true", np.round(f["true_off"], 1), "fix_err", round(f["fix_err"], 1), "sigma", round(sig, 1))
