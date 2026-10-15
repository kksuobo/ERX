"""Sweep a dotted config parameter and report aggregate metrics (Fig. 8)."""

import argparse
import _bootstrap  # noqa: F401
import copy
import os

from radwa.config import load_config
from radwa.metrics import summarize
from radwa.planners import PLANNERS
from radwa.simulator import run_episode

ROOT = _bootstrap.ROOT


def _set(cfg, path, value):
    keys = path.split(".")
    node = cfg
    for k in keys[:-1]:
        node = node[k]
    node[keys[-1]] = value


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--param", required=True, help="dotted path, e.g. radwa.T0")
    ap.add_argument("--values", required=True, help="comma separated")
    ap.add_argument("--planners", default="RA-DWA")
    ap.add_argument("--scenarios", default="all")
    ap.add_argument("--seeds", type=int, default=8)
    args = ap.parse_args()

    base = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    scenarios = (list(base["scenarios"].keys()) if args.scenarios == "all"
                 else args.scenarios.split(","))
    planners = args.planners.split(",")
    seed0 = base["benchmark"]["seed0"]

    print(f"\nSweep {args.param} over {args.values} "
          f"({len(scenarios)} scenarios x {args.seeds} seeds x {planners})")
    print(f"{'value':>8s} {'planner':8s} {'succ':>6s} {'coll':>6s} {'tmo':>6s} "
          f"{'mclear':>7s} {'mTTC':>6s} {'jerk':>6s} {'p95ms':>7s}")
    for raw in args.values.split(","):
        val = float(raw) if raw.replace(".", "").replace("-", "").isdigit() else raw
        cfg = copy.deepcopy(base)
        _set(cfg, args.param, val)
        for pname in planners:
            rows = []
            for sc in scenarios:
                for k in range(args.seeds):
                    res = run_episode(PLANNERS[pname](cfg), sc, seed0 + k, cfg)
                    res.update(planner=pname, scenario=sc, seed=seed0 + k)
                    rows.append(res)
            _, overall = summarize(rows)
            r = overall[0]
            print(f"{raw:>8s} {pname:8s} {r['success']:6.2f} {r['collision']:6.2f} "
                  f"{r['timeout']:6.2f} {r['min_clearance']:7.2f} "
                  f"{r['min_ttc']:6.2f} {r['mean_abs_jerk']:6.2f} "
                  f"{r['lat_p95_ms']:7.2f}")


if __name__ == "__main__":
    main()
