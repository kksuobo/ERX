"""Print per-seed outcome for one (scenario, planner) pair."""

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
    ap.add_argument("--seeds", type=int, default=10)
    args = ap.parse_args()
    cfg = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    seed0 = cfg["benchmark"]["seed0"]
    line = []
    for k in range(args.seeds):
        seed = seed0 + k
        res = run_episode(PLANNERS[args.planner](cfg), args.scenario, seed, cfg)
        line.append(f"{seed}:{res['status'][:4]}")
    print(f"{args.planner} {args.scenario}: " + " ".join(line))


if __name__ == "__main__":
    main()
