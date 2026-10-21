"""Summarize tracker CSV logs (from csv_path:=...). Standard library only.

python3 scripts/summarize.py results/*.csv
python3 scripts/summarize.py results/lqr.csv --max-rms-cm 3     (exit code 1 if any run is worse)
"""
import argparse
import csv
import math
import sys


def summarize(path, skip_s=3.0):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise SystemExit(f"{path}: empty log")
    t = [float(r["t"]) for r in rows]
    cte = [float(r["true_cte"]) for r in rows]
    w = [float(r["w"]) for r in rows]
    scored = [c for ti, c in zip(t, cte) if ti > skip_s]
    rms = math.sqrt(sum(c * c for c in scored) / len(scored)) if scored else float("nan")
    worst = max((abs(c) for c in scored), default=float("nan"))
    settle = next((ti for ti, c in zip(t, cte) if abs(c) < 0.03), float("nan"))
    dt = (t[-1] - t[0]) / max(1, len(t) - 1)
    w_rate = sum(abs(b - a) for a, b in zip(w, w[1:])) / max(1, len(w) - 1) / dt if dt > 0 else float("nan")
    return dict(duration=t[-1], rms=rms * 100, max=worst * 100, settle=settle, w_rate=w_rate)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--max-rms-cm", type=float, default=None)
    a = ap.parse_args()
    print(f"{'log':<32}{'duration':>10}{'settle':>9}{'RMS CTE':>10}{'max CTE':>10}{'w rate':>8}")
    bad = False
    for p in a.csv:
        s = summarize(p)
        print(f"{p:<32}{s['duration']:>8.1f} s{s['settle']:>7.2f} s{s['rms']:>7.1f} cm{s['max']:>7.1f} cm{s['w_rate']:>8.2f}")
        if a.max_rms_cm is not None and not s["rms"] <= a.max_rms_cm:
            print(f"  FAIL: RMS {s['rms']:.1f} cm > {a.max_rms_cm} cm")
            bad = True
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
