"""Plot tracker logs. Run from the repo folder.

python plot.py results/lqr.csv results/pure_pursuit.csv     ROS 2 runs  -> tracking.png
python plot.py results/bench_traj.csv                        benchmark   -> tracking.png
"""
import csv
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

files = sys.argv[1:] or ["results/lqr.csv", "results/pure_pursuit.csv"]
runs = {}
for f in files:
    rows = list(csv.DictReader(open(f)))
    if "controller" in rows[0]:            # benchmark file: several runs in one CSV
        groups = defaultdict(list)
        for r in rows:
            groups[r["controller"]].append(r)
        for k, g in groups.items():
            runs[k] = g
    else:
        runs[f.split("/")[-1].replace(".csv", "")] = rows

fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5))
colors = ["tab:green", "tab:red", "tab:blue", "tab:orange"]
for (name, rows), c in zip(runs.items(), colors):
    t = [float(r["t"]) for r in rows]
    cte_key = "true_cte" if "true_cte" in rows[0] else "cte"
    a1.plot([float(r["x"]) for r in rows], [float(r["y"]) for r in rows], color=c, lw=1.2, label=name)
    a2.plot(t, [100 * float(r[cte_key]) for r in rows], color=c, lw=1.0, label=name)
a1.set_aspect("equal"); a1.set_title("Robot path (starts 20 cm off the reference)"); a1.legend(fontsize=8)
a2.axhline(0, color="k", lw=0.5); a2.set_ylim(-5, 22)
a2.set_xlabel("time (s)"); a2.set_ylabel("cross-track error (cm)"); a2.set_title("Cross-track error"); a2.legend(fontsize=8)
fig.tight_layout(); fig.savefig("tracking.png", dpi=120)
print("saved tracking.png")
