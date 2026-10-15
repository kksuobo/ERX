"""Figure 8: parameter sensitivity + real-time latency headroom."""

import _bootstrap  # noqa: F401
import copy
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from radwa.config import load_config
from radwa.planners.radwa import RADWA
from radwa.simulator import run_episode

ROOT = _bootstrap.ROOT


def _set(cfg, path, value):
    keys = path.split(".")
    node = cfg
    for k in keys[:-1]:
        node = node[k]
    node[keys[-1]] = value


def sweep(base, path, values, scenarios, seeds):
    seed0 = base["benchmark"]["seed0"]
    out = []
    for val in values:
        cfg = copy.deepcopy(base)
        _set(cfg, path, val)
        succ = coll = jerk = mttc = 0.0
        n = 0
        for sc in scenarios:
            for k in range(seeds):
                res = run_episode(RADWA(cfg), sc, seed0 + k, cfg)
                succ += res["success"]
                coll += res["collision"]
                jerk += res["mean_abs_jerk"]
                mttc += (res["min_ttc"] if res["min_ttc"] == res["min_ttc"] else 0.0)
                n += 1
        out.append((val, succ / n, coll / n, jerk / n, mttc / n))
    return out


def main():
    base = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    scenarios = list(base["scenarios"].keys())
    seeds = 6

    t0 = sweep(base, "radwa.T0", [2.0, 3.0, 4.0, 5.0, 6.0], scenarios, seeds)
    sm = sweep(base, "radwa.safety_margin", [0.0, 0.2, 0.4, 0.6], scenarios, seeds)

    fig, axes = plt.subplots(1, 3, figsize=(13, 3.8))
    for name, data, xlabel in [("$T_0$", t0, "$T_0$ [s]"),
                               ("safety margin", sm, "safety margin [m]")]:
        xs = [d[0] for d in data]
        axes[0].plot(xs, [d[1] for d in data], "o-", label=f"success ({name})")
        axes[0].plot(xs, [d[2] for d in data], "s--", label=f"collision ({name})")
        axes[1].plot(xs, [d[3] for d in data], "o-", label=f"jerk ({name})")
        axes[1].plot(xs, [d[4] for d in data], "s--", label=f"min TTC ({name})")
    axes[0].set_xlabel("parameter"), axes[0].set_ylabel("rate")
    axes[0].set_title("Success / collision"), axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=7)
    axes[1].set_xlabel("parameter"), axes[1].set_title("Comfort / margin")
    axes[1].grid(alpha=0.3), axes[1].legend(fontsize=7)

    import csv
    per = {}
    with open(os.path.join(ROOT, "results", "episodes.csv"), newline="") as f:
        for r in csv.DictReader(f):
            per.setdefault(r["planner"], []).append(float(r["lat_p95_ms"]))
    budgets = [p for p in ["DWA", "VO", "TEB", "MPPI", "RA-DWA"] if p in per]
    colors = {"DWA": "#1f77b4", "VO": "#2ca02c", "TEB": "#8c564b",
              "MPPI": "#9467bd", "RA-DWA": "#d62728"}
    axes[2].bar(budgets, [max(per[b]) for b in budgets],
                color=[colors[b] for b in budgets])
    axes[2].axhline(100.0, color="k", ls="--", lw=1, label="100 ms budget")
    axes[2].set_ylabel("worst-scenario p95 [ms]")
    axes[2].set_title("Real-time headroom"), axes[2].legend(fontsize=7)
    axes[2].grid(alpha=0.3, axis="y")
    fig.tight_layout()
    out = os.path.join(ROOT, "figures", "fig8_sensitivity_realtime.png")
    fig.savefig(out, dpi=180)
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    main()
