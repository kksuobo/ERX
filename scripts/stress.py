"""Actuation stress: command delay and command jitter (perception clean)."""

import argparse
import _bootstrap  # noqa: F401
import copy
import csv
import os

from radwa.config import load_config
from radwa.metrics import summarize
from radwa.planners import PLANNERS
from radwa.simulator import run_episode

ROOT = _bootstrap.ROOT

LEVELS = [
    ("ideal", {"actuation_delay": 0, "command_noise": 0.0}),
    ("delay 1 cycle", {"actuation_delay": 1, "command_noise": 0.0}),
    ("delay 2 cycles", {"actuation_delay": 2, "command_noise": 0.0}),
    ("jitter 0.2", {"actuation_delay": 0, "command_noise": 0.2}),
    ("delay 2 + jitter", {"actuation_delay": 2, "command_noise": 0.2}),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--planners", default="DWA,VO,RA-DWA")
    ap.add_argument("--seeds", type=int, default=8)
    args = ap.parse_args()
    base = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    scenarios = list(base["scenarios"].keys())
    planners = args.planners.split(",")
    seed0 = base["benchmark"]["seed0"]

    print(f"\nActuation stress: {len(scenarios)} scenarios x {args.seeds} seeds "
          f"x {planners}")
    print(f"{'level':16s} {'planner':8s} {'succ':>6s} {'coll':>6s} {'tmo':>6s} "
          f"{'mclear':>7s} {'jerk':>6s}")
    saved = []
    for name, over in LEVELS:
        cfg = copy.deepcopy(base)
        cfg["sim"].update(over)
        for pname in planners:
            rows = []
            for sc in scenarios:
                for k in range(args.seeds):
                    res = run_episode(PLANNERS[pname](cfg), sc, seed0 + k, cfg)
                    res.update(planner=pname, scenario=sc, seed=seed0 + k)
                    rows.append(res)
            _, overall = summarize(rows)
            r = overall[0]
            saved.append({"level": name, "planner": pname,
                          "n": len(rows), "success": r["success"],
                          "collision": r["collision"], "timeout": r["timeout"],
                          "min_clearance": r["min_clearance"],
                          "mean_abs_jerk": r["mean_abs_jerk"]})
            print(f"{name:16s} {pname:8s} {r['success']:6.2f} {r['collision']:6.2f} "
                  f"{r['timeout']:6.2f} {r['min_clearance']:7.2f} "
                  f"{r['mean_abs_jerk']:6.2f}")

    out = os.path.join(ROOT, "results", "stress.csv")
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(saved[0].keys()))
        w.writeheader()
        for row in saved:
            w.writerow(row)
    print("wrote", out)


if __name__ == "__main__":
    main()
