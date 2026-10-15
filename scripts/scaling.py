"""Runtime scaling: per-cycle latency vs obstacle count and vs candidate count.

Isolated timing of one plan() call, averaged over repeated calls, so the
complexity claim of the method is measured rather than asserted.
"""

import _bootstrap  # noqa: F401
import csv
import math
import os
import random
from time import perf_counter

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from radwa.config import load_config
from radwa.planners import PLANNERS
from radwa.state import EgoState

ROOT = _bootstrap.ROOT
COLORS = {"DWA": "#1f77b4", "VO": "#2ca02c", "RA-DWA": "#d62728"}


def _scene(cfg, k, rng):
    obstacles = []
    for _ in range(k):
        x = rng.uniform(6, 22)
        y = rng.uniform(-6, 6)
        o = type("O", (), {})()
        o.x, o.y, o.r = x, y, 1.0
        o.vx, o.vy = rng.uniform(-2, 2), rng.uniform(-2, 2)
        obstacles.append(o)
    return obstacles


def time_planner(planner, cfg, k, seed, calls=200):
    rng = random.Random(seed)
    obstacles = _scene(cfg, k, rng)
    ego = EgoState(6.0, 0.0, 0.0, 4.0, 0.0)
    planner.reset((40.0, 0.0), random.Random(seed))
    # warm-up
    for _ in range(20):
        planner.plan(ego, obstacles, 0.0)
    t0 = perf_counter()
    for i in range(calls):
        planner.plan(ego, obstacles, i * 0.1)
    return (perf_counter() - t0) / calls * 1e3


def main():
    cfg = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    ks = [1, 2, 4, 8, 16, 32]
    rows = []
    for name in ("DWA", "VO", "RA-DWA"):
        for k in ks:
            ms = np.mean([time_planner(PLANNERS[name](cfg), cfg, k, s)
                          for s in (1, 2, 3)])
            rows.append({"study": "obstacles", "planner": name,
                         "value": k, "lat_ms": round(ms, 3)})
            print(f"{name:7s} k={k:3d}  {ms:8.3f} ms")

    # candidate-count scaling for the sampling planners
    cfg_keys = {"DWA": "dwa", "RA-DWA": "radwa"}
    grid = [3, 5, 7, 9, 11, 13]
    for name in ("DWA", "RA-DWA"):
        for g in grid:
            c = load_config(os.path.join(ROOT, "configs", "experiment.json"))
            c[cfg_keys[name]]["n_v"] = g
            c[cfg_keys[name]]["n_w"] = 2 * g - 1
            ms = np.mean([time_planner(PLANNERS[name](c), c, 8, s)
                          for s in (1, 2, 3)])
            rows.append({"study": "candidates", "planner": name,
                         "value": g * (2 * g - 1), "lat_ms": round(ms, 3)})

    with open(os.path.join(ROOT, "results", "scaling.csv"), "w", newline="",
              encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.0))
    for name in ("DWA", "VO", "RA-DWA"):
        r = [x for x in rows if x["study"] == "obstacles" and x["planner"] == name]
        axes[0].plot([x["value"] for x in r], [x["lat_ms"] for x in r], "o-",
                     color=COLORS[name], label=name)
    axes[0].set_xlabel("number of obstacles"), axes[0].set_ylabel("latency [ms]")
    axes[0].set_title("Latency vs obstacle count"), axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    for name in ("DWA", "RA-DWA"):
        r = [x for x in rows if x["study"] == "candidates" and x["planner"] == name]
        axes[1].plot([x["value"] for x in r], [x["lat_ms"] for x in r], "s-",
                     color=COLORS[name], label=name)
    axes[1].set_xlabel("roll-outs per cycle"), axes[1].set_ylabel("latency [ms]")
    axes[1].set_title("Latency vs candidate count"), axes[1].grid(alpha=0.3)
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    out = os.path.join(ROOT, "figures", "fig11_scaling.png")
    fig.savefig(out, dpi=180)
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    main()
