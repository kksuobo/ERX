"""Trace a single episode (per-cycle state) for debugging."""

import argparse
import _bootstrap  # noqa: F401
import os

from radwa.config import load_config
from radwa.planners import PLANNERS
from radwa.simulator import run_episode

ROOT = _bootstrap.ROOT


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--planner", default="RA-DWA")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--upto", type=float, default=None)
    args = ap.parse_args()

    cfg = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    planner = PLANNERS[args.planner](cfg)
    res = run_episode(planner, args.scenario, args.seed, cfg, record=True)
    log = res["_log"]
    print(f"{args.planner} / {args.scenario} / seed={args.seed} -> {res['status']} "
          f"(n={res['n_cycles']}, mean_speed={res['mean_speed']:.2f})")
    print(f"{'t':>5s} {'x':>6s} {'y':>6s} {'th':>6s} {'v':>5s} {'w':>6s} "
          f"{'risk':>5s} {'clear':>6s} {'ttc':>6s} state")
    for i in range(0, len(log["t"]), args.every):
        if args.upto is not None and log["t"][i] > args.upto:
            break
        print(f"{log['t'][i]:5.1f} {log['x'][i]:6.2f} {log['y'][i]:6.2f} "
              f"{log['theta'][i]:6.2f} {log['v'][i]:5.2f} {log['omega'][i]:6.2f} "
              f"{log['risk'][i]:5.2f} {log['clearance'][i]:6.2f} "
              f"{(log['ttc'][i] if log['ttc'][i] < 1e8 else 99):6.2f} "
              f"{log['state'][i]}")


if __name__ == "__main__":
    main()
