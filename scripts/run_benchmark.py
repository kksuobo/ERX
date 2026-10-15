"""Run the full benchmark and write results CSV + summary tables."""

import argparse
import _bootstrap  # noqa: F401
import json
import os

from radwa.config import load_config
from radwa.metrics import summarize
from radwa.runner import run_benchmark

ROOT = _bootstrap.ROOT


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(ROOT, "configs", "experiment.json"))
    ap.add_argument("--seeds", type=int, default=50)
    ap.add_argument("--scenarios", default="all")
    ap.add_argument("--planners", default="DWA,APF,VO,MPPI,RA-DWA")
    ap.add_argument("--out", default=os.path.join(ROOT, "results", "episodes.csv"))
    args = ap.parse_args()

    cfg = load_config(args.config)
    scenarios = (list(cfg["scenarios"].keys()) if args.scenarios == "all"
                 else args.scenarios.split(","))
    planners = args.planners.split(",")

    print(f"Benchmark: {len(scenarios)} scenarios x {args.seeds} seeds "
          f"x {len(planners)} planners")
    rows, _ = run_benchmark(cfg, scenarios, args.seeds, planners, out_csv=args.out)

    _, overall = summarize(rows)
    out_summary = os.path.splitext(args.out)[0] + "_overall.json"
    with open(out_summary, "w", encoding="utf-8") as f:
        json.dump(overall, f, indent=2)

    print(f"\nWrote {args.out} ({len(rows)} episodes) and {out_summary}")
    print(f"\n{'planner':8s} {'succ':>6s} {'coll':>6s} {'mclear':>7s} "
          f"{'mTTC':>7s} {'jerk':>8s} {'p95ms':>7s}")
    for rec in sorted(overall, key=lambda r: r["planner"]):
        print(f"{rec['planner']:8s} {rec['success']:6.2f} {rec['collision']:6.2f} "
              f"{rec['min_clearance']:7.2f} {rec['min_ttc']:7.2f} "
              f"{rec['mean_abs_jerk']:8.3f} {rec['lat_p95_ms']:7.2f}")


if __name__ == "__main__":
    main()
