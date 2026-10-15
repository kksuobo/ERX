"""Per-(planner, scenario) diagnostic table."""

import argparse
import _bootstrap  # noqa: F401
import os

from radwa.config import load_config
from radwa.metrics import summarize
from radwa.runner import run_benchmark

ROOT = _bootstrap.ROOT


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--planners", default="DWA,APF,RA-DWA")
    args = ap.parse_args()
    cfg = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    scenarios = list(cfg["scenarios"].keys())
    rows, _ = run_benchmark(cfg, scenarios, args.seeds,
                            args.planners.split(","), progress=False)
    s, _ = summarize(rows)
    print(f"\n{'planner':7s} {'scenario':16s} {'succ':>5s} {'coll':>5s} "
          f"{'tmo':>5s} {'mclear':>7s} {'mTTC':>6s} {'jerk':>7s} {'wvar':>6s}")
    for r in sorted(s, key=lambda x: (x["scenario"], x["planner"])):
        print(f"{r['planner']:7s} {r['scenario']:16s} {r['success']:5.2f} "
              f"{r['collision']:5.2f} {r['timeout']:5.2f} "
              f"{r['min_clearance']:7.2f} {r['min_ttc']:6.2f} "
              f"{r['mean_abs_jerk']:7.2f} {r['omega_var']:6.3f}")


if __name__ == "__main__":
    main()
