"""Replay an episode and dump candidate costs at the first near-stall cycle."""

import argparse
import _bootstrap  # noqa: F401
import math
import os

from radwa.config import load_config
from radwa.geometry import dist
from radwa.planners.radwa import RADWA
from radwa.scenarios import build_scenario
from radwa.simulator import step_plant
from radwa.state import EgoState

ROOT = _bootstrap.ROOT


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="static_field")
    ap.add_argument("--seed", type=int, default=2)
    args = ap.parse_args()
    cfg = load_config(os.path.join(ROOT, "configs", "experiment.json"))

    start, goal, obstacles = build_scenario(args.scenario, args.seed, cfg)
    dt = cfg["sim"]["dt"]
    r_ego = cfg["ego"]["r"]
    theta0 = math.atan2(goal[1] - start[1], goal[0] - start[0])
    ego = EgoState(start[0], start[1], theta0, 0.0, 0.0)
    planner = RADWA(cfg)
    planner.reset(goal)
    planner.debug = True

    t = 0.0
    print("replay start")
    while t < 10.0:
        ctrl = planner.plan(ego, obstacles, t)
        step_plant(ego, ctrl, cfg)
        t += dt
        for o in obstacles:
            o.step(dt, t)
        if abs(t - 2.0) < 1e-9 or abs(t - 2.1) < 1e-9 or abs(t - 2.2) < 1e-9:
            print(f"  t={t:.1f} x={ego.x:.2f} v={ego.v:.2f} state={ctrl.state} "
                  f"rho={ctrl.risk:.3f}")
        if ego.v < 0.05 and t > 1.0:
            print(f"near-stall at t={t:.1f} pos=({ego.x:.2f},{ego.y:.2f}) "
                  f"theta={ego.theta:.2f} state={ctrl.state} rho={ctrl.risk:.3f}")
            print(f"nearest obstacles: "
                  + ", ".join(f"({o.x:.2f},{o.y:.2f},r={o.r:.2f})"
                              for o in sorted(obstacles,
                                              key=lambda o: dist(ego.x, ego.y, o.x, o.y))[:4]))
            print(f"{'cost':>9s} {'v':>6s} {'w':>6s} {'prog':>7s} {'head':>6s} "
                  f"{'risk':>6s} {'clear':>6s} {'viol':>7s} {'minc':>6s}")
            for c in sorted(planner._last_candidates)[:12]:
                print(f"{c[0]:9.3f} {c[1]:6.2f} {c[2]:6.2f} {c[3]:7.2f} "
                      f"{c[4]:6.2f} {c[5]:6.3f} {c[6]:6.3f} {c[7]:7.2f} {c[8]:6.2f}")
            break
    print(f"replay end at t={t:.1f} x={ego.x:.2f} v={ego.v:.2f}")


if __name__ == "__main__":
    main()
