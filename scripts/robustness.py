"""Perception-noise robustness: sweep position/velocity noise and dropout."""

import argparse
import _bootstrap  # noqa: F401
import copy
import csv
import os

from radwa.config import load_config
from radwa.metrics import summarize
from radwa.planners import PLANNERS
from radwa.simulator import run_episode
from radwa.stats import wilson

ROOT = _bootstrap.ROOT

LEVELS = [
    ("clean", {"pos_std": 0.0, "vel_std": 0.0, "dropout": 0.0}),
    ("low", {"pos_std": 0.2, "vel_std": 0.3, "dropout": 0.05}),
    ("medium", {"pos_std": 0.5, "vel_std": 0.8, "dropout": 0.10}),
    ("high", {"pos_std": 1.0, "vel_std": 1.5, "dropout": 0.20}),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--planners", default="DWA,APF,VO,MPPI,RA-DWA")
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--out", default=os.path.join(ROOT, "results", "robustness.csv"))
    ap.add_argument("--set", action="append", default=[],
                    help="dotted config override, e.g. vo.margin=0.7")
    args = ap.parse_args()
    base = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    for item in args.set:
        path, _, raw = item.partition("=")
        node = base
        keys = path.split(".")
        for k in keys[:-1]:
            node = node[k]
        val = raw
        for conv in (int, float):
            try:
                val = conv(raw)
                break
            except ValueError:
                pass
        if raw in ("true", "false"):
            val = raw == "true"
        node[keys[-1]] = val
    scenarios = list(base["scenarios"].keys())
    planners = args.planners.split(",")
    seed0 = base["benchmark"]["seed0"]

    print(f"\nRobustness: {len(scenarios)} scenarios x {args.seeds} seeds "
          f"x {planners}")
    print(f"{'level':8s} {'planner':8s} {'succ':>6s} {'coll':>6s} {'tmo':>6s} "
          f"{'mclear':>7s} {'mTTC':>6s} {'jerk':>6s}")
    saved = []
    for name, sensor in LEVELS:
        cfg = copy.deepcopy(base)
        cfg["sensor"] = sensor
        for pname in planners:
            rows = []
            for sc in scenarios:
                for k in range(args.seeds):
                    res = run_episode(PLANNERS[pname](cfg), sc, seed0 + k, cfg)
                    res.update(planner=pname, scenario=sc, seed=seed0 + k)
                    rows.append(res)
            _, overall = summarize(rows)
            r = overall[0]
            n = len(rows)
            slo, shi = wilson(sum(x["success"] for x in rows), n)
            clo, chi = wilson(sum(x["collision"] for x in rows), n)
            saved.append({"level": name, "planner": pname, "n": n,
                          "success": r["success"], "succ_ci_lo": slo,
                          "succ_ci_hi": shi, "collision": r["collision"],
                          "coll_ci_lo": clo, "coll_ci_hi": chi,
                          "timeout": r["timeout"],
                          "min_clearance": r["min_clearance"],
                          "min_ttc": r["min_ttc"],
                          "mean_abs_jerk": r["mean_abs_jerk"]})
            print(f"{name:8s} {pname:8s} {r['success']:6.2f} {r['collision']:6.2f} "
                  f"{r['timeout']:6.2f} {r['min_clearance']:7.2f} "
                  f"{r['min_ttc']:6.2f} {r['mean_abs_jerk']:6.2f}")

    out_csv = args.out
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(saved[0].keys()))
        w.writeheader()
        for row in saved:
            w.writerow(row)
    print("wrote", out_csv)


if __name__ == "__main__":
    main()
