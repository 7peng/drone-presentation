"""Per-mission strip chart for the results slide (inline SVG, so it uses the deck's fonts and tokens).

Final distance from home after the return, log scale, one dot per mission:
  ours  = eval_cl/sweep_v2/mission_*/summary.json  final_dist_m   (model v2, closed loop)
  stock = eval_cl/sweep_table.csv px4_stock_end   (PX4 EKF2 inertial-only replay of the same flight;
          its error when the flown return reached home)
Writes the SVG between the results-chart markers in index.html.

    python3 tools/make_results_chart.py
"""
import csv
import glob
import json
import math
import os
import re

EVAL = "/scratch/users/wgpeng/drone-work/eval_cl"
HERE = os.path.dirname(os.path.abspath(__file__))
INDEX = os.path.join(HERE, "..", "index.html")

ours = {}
for f in sorted(glob.glob(f"{EVAL}/sweep_v2/mission_*/summary.json")):
    s = json.load(open(f))
    ours[s["id"]] = s["final_dist_m"]
stock = {r["id"]: float(r["px4_stock_end"]) for r in csv.DictReader(open(f"{EVAL}/sweep_table.csv")) if r["px4_stock_end"]}
ids = sorted(set(ours) & set(stock))
assert len(ids) == 32, len(ids)


def med(v):
    v = sorted(v)
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


W, H = 1600, 300
X0, X1 = 250, 1560
LO, HI = math.log10(0.5), math.log10(20000)
ROWS = [("Our system", "#315c91", [ours[i] for i in ids], 95), ("Stock PX4", "#8a929b", [stock[i] for i in ids], 200)]


def x(v):
    return X0 + (math.log10(max(v, 0.5)) - LO) / (HI - LO) * (X1 - X0)


out = [f'<svg class="chart" viewBox="0 0 {W} {H}" role="img" aria-label="Final distance from home per mission, log scale: ours all under 10 m, stock PX4 106 m to 8 km">']
# grid + axis (recessive)
for v, lab in [(1, "1 m"), (10, "10 m"), (100, "100 m"), (1000, "1 km"), (10000, "10 km")]:
    out.append(f'<line x1="{x(v):.1f}" x2="{x(v):.1f}" y1="40" y2="250" class="grid"/>')
    out.append(f'<text x="{x(v):.1f}" y="288" class="tick" text-anchor="middle">{lab}</text>')
# home threshold
out.append(f'<line x1="{x(10):.1f}" x2="{x(10):.1f}" y1="32" y2="250" class="thr"/>')
out.append(f'<text x="{x(10) + 12:.1f}" y="28" class="thr-l">home: within 10 m</text>')
for name, col, vals, y in ROWS:
    out.append(f'<text x="0" y="{y + 8}" class="row">{name}</text>')
    order = sorted(range(len(vals)), key=lambda k: vals[k])
    for rank, k in enumerate(order):
        jy = y + ((rank % 3) - 1) * 13
        out.append(f'<circle cx="{x(vals[k]):.1f}" cy="{jy}" r="9" fill="{col}" class="dot"><title>{ids[k]}: {vals[k]:,.1f} m</title></circle>')
    m = med(vals)
    mx = x(m)
    out.append(f'<line x1="{mx:.1f}" x2="{mx:.1f}" y1="{y - 34}" y2="{y + 34}" class="med"/>')
    lab = f"median {m:.1f} m" if m < 100 else f"median {m:,.0f} m"
    ly = y + 60 if name == "Our system" else y - 44      # ours below its dots: clear of the threshold label
    out.append(f'<text x="{mx:.1f}" y="{ly}" class="med-l" text-anchor="middle">{lab}</text>')
out.append("</svg>")
svg = "\n".join(out)

html = open(INDEX).read()
pat = re.compile(r"(<!-- results-chart:start -->)(.*?)(<!-- results-chart:end -->)", re.S)
assert pat.search(html), "markers not found"
html = pat.sub(lambda mm: mm.group(1) + "\n" + svg + "\n" + mm.group(3), html)
open(INDEX, "w").write(html)
print(f"ours median {med([ours[i] for i in ids]):.2f} m, ok {sum(ours[i] <= 10 for i in ids)}/32; "
      f"stock median {med([stock[i] for i in ids]):.0f} m, ok {sum(stock[i] <= 10 for i in ids)}/32")
