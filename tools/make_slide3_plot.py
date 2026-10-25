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
A, sA = mapper(allp[:, 0].min() - pad, allp[:, 0].max() + pad, allp[:, 1].min() - pad, allp[:, 1].max() + pad, 20, 30, 640, 580)
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

bx = 40 + 0
svg.append(f'<path d="M{bx},590 h{500 * sA:.1f}" stroke="{MUTED}" stroke-width="2"/><text x="{bx}" y="616" class="pl m">500 m</text>')

# ---- right: model architecture, fed with the real live frame + breadcrumb from the circled moment
from PIL import Image
q = np.load(RUN + "rect_query.npy", mmap_mode="r")[f["rect_i"]]
bimg = np.load(RUN + "rect_bc.npy", mmap_mode="r")[k]
IMG = "/scratch/users/wgpeng/drone-presentation/assets/img/"
Image.fromarray(np.ascontiguousarray(q)).save(IMG + "s3_live.jpg", quality=85)
Image.fromarray(np.ascontiguousarray(bimg)).save(IMG + "s3_bc.jpg", quality=85)
X0 = 760
svg.append(f'<path d="M{zc[0] + 12:.1f},{zc[1] - 8:.1f} L{X0 - 6},{60} M{zc[0] + 12:.1f},{zc[1] + 8:.1f} L{X0 - 6},{580}" stroke="{GREEN}" stroke-width="1.5" stroke-dasharray="4 6"/>')


def box(x, y, w, h, title, sub, cls="ours"):
    fill, stroke = ("#eef3f9", BLUE) if cls == "ours" else ("#fff", HAIR)
    svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" fill="{fill}" stroke="{stroke}" stroke-width="2"/>')
    svg.append(f'<text x="{x + w / 2}" y="{y + h / 2 - (8 if sub else -9)}" class="pl b" text-anchor="middle">{title}</text>')
    for i, line in enumerate(sub):
        svg.append(f'<text x="{x + w / 2}" y="{y + h / 2 + 24 + 26 * i}" class="pl m" text-anchor="middle">{line}</text>')


def arrow(x1, y1, x2, y2, col=MUTED):
    svg.append(f'<path d="M{x1},{y1} L{x2 - 10},{y2}" stroke="{col}" stroke-width="2.5" fill="none"/><path d="M{x2},{y2} l-14,-7 v14 z" fill="{col}" transform="rotate({math.degrees(math.atan2(y2 - y1, x2 - x1)):.1f} {x2} {y2})"/>')


S = 168                                            # thumbnail size
rows = [(60, "s3_live.jpg", "live frame", "now, rectified"), (412, "s3_bc.jpg", f"breadcrumb #{k}", "saved on the way out")]
for y, img, t1, t2 in rows:
    svg.append(f'<image href="assets/img/{img}" x="{X0}" y="{y}" width="{S}" height="{S}"/>')
    svg.append(f'<rect x="{X0}" y="{y}" width="{S}" height="{S}" fill="none" stroke="{HAIR}" stroke-width="2"/>')
    svg.append(f'<text x="{X0 + S / 2}" y="{y - 14 if y < 300 else y + S + 34}" class="pl m" text-anchor="middle"><tspan class="b0">{t1}</tspan> · {t2}</text>')
    box(X0 + S + 60, y + 4, 250, S - 8, "Encoder", ["MobileNetV3-S", "frozen + small head"])
    arrow(X0 + S + 6, y + S / 2, X0 + S + 56, y + S / 2)
    gx = X0 + S + 60 + 250 + 50
    for j in range(3):                               # stacked grid glyph = 14x14x16 descriptor
        svg.append(f'<rect x="{gx + 10 - 5 * j}" y="{y + 40 + 5 * j}" width="80" height="80" fill="#fff" stroke="{BLUE}" stroke-width="2"/>')
    for g in range(1, 4):
        svg.append(f'<path d="M{gx + g * 20},{y + 50} v80 M{gx},{y + 50 + g * 20} h80" stroke="{BLUE}" stroke-width="1" opacity=".5"/>')
    svg.append(f'<text x="{gx + 40}" y="{y + 160}" class="pl m" text-anchor="middle">14×14×16</text>')
    arrow(X0 + S + 60 + 250 + 4, y + S / 2, gx - 8, y + S / 2)
MX = X0 + S + 60 + 250 + 50 + 130
box(MX, 236, 240, 168, "Match", ["slide ±7 cells", "softmax over 225"])
arrow(MX - 40, 60 + S / 2, MX - 4, 290)
arrow(MX - 40, 412 + S / 2, MX - 4, 350)
OX = MX + 240 + 60
outs = [("offset (N, E)", "metres"), ("uncertainty", "2×2 covariance"), ("arrived?", "probability"), ("no match?", "probability")]
for i, (o1, o2) in enumerate(outs):
    y = 150 + i * 92
    svg.append(f'<text x="{OX + 30}" y="{y + 8}" class="pl b">{o1}</text><text x="{OX + 30}" y="{y + 38}" class="pl m">{o2}</text>')
    svg.append(f'<path d="M{MX + 244},320 C{OX - 20},320 {OX - 30},{y} {OX + 16},{y}" stroke="{MUTED}" stroke-width="2" fill="none"/>')
svg.append(f'<text x="{OX + 30}" y="{150 + 4 * 92 + 18}" class="pl g">→ position fix for PX4</text>')
# left legend
for i, (kind, col, lab) in enumerate([("line", HAIR, "outbound (GPS)"), ("dot", BLUE, "breadcrumb (20 m)"), ("line", INK, "return (jammed)")]):
    lx = 40 + i * 270
    if kind == "line":
        svg.append(f'<line x1="{lx}" y1="650" x2="{lx + 30}" y2="650" stroke="{col}" stroke-width="5"/>')
    else:
        svg.append(f'<circle cx="{lx + 15}" cy="650" r="7" fill="{col}"/>')
    svg.append(f'<text x="{lx + 42}" y="658" class="pl m">{lab}</text>')

body = "\n      ".join(svg)
out_svg = f'''<svg class="plot3" viewBox="0 0 1920 680" role="img" aria-label="Route cx_011 with breadcrumbs, and a close-up of one return frame showing the model's vector from its position fix to the matched breadcrumb">
      <style>.pl{{font:500 26px 'Google Sans','Noto Sans',sans-serif;fill:{INK}}} .pl.m{{fill:{MUTED};font-size:22px}} .pl.b{{font-weight:600;font-size:28px}} .b0{{fill:{INK};font-weight:600}} .pl.g{{fill:{GREEN};font-weight:600;paint-order:stroke;stroke:#f6f8fa;stroke-width:10px}} .pl.h{{paint-order:stroke;stroke:#fff;stroke-width:10px}}</style>
      {body}
    </svg>'''
p = "/scratch/users/wgpeng/drone-presentation/index.html"
s = open(p).read()
s = re.sub(r"<!-- SLIDE3-PLOT -->.*?<!-- /SLIDE3-PLOT -->", lambda m: "<!-- SLIDE3-PLOT -->\n    " + out_svg + "\n    <!-- /SLIDE3-PLOT -->", s, flags=re.S)
open(p, "w").write(s)
print("frame t", round(f["t"]), "k", k, "pred", np.round(off, 1), "true", np.round(f["true_off"], 1), "fix_err", round(f["fix_err"], 1), "sigma", round(sig, 1))
