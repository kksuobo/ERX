"""Figure 9: success/collision vs perception-noise level (from robustness.csv)."""

import _bootstrap  # noqa: F401
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = _bootstrap.ROOT
LEVELS = ["clean", "low", "medium", "high"]
COLORS = {"DWA": "#1f77b4", "VO": "#2ca02c", "RA-DWA": "#d62728"}


def main():
    path = os.path.join(ROOT, "results", "robustness.csv")
    data = {}
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            data.setdefault(r["planner"], {})[r["level"]] = r

    fig, axes = plt.subplots(1, 2, figsize=(9, 3.6))
    for planner, by_level in data.items():
        c = COLORS.get(planner, "#888")
        succ = [float(by_level[lv]["success"]) for lv in LEVELS]
        coll = [float(by_level[lv]["collision"]) for lv in LEVELS]
        axes[0].plot(LEVELS, succ, "o-", color=c, label=planner)
        axes[1].plot(LEVELS, coll, "s--", color=c, label=planner)
    axes[0].set_ylabel("success rate"), axes[0].set_ylim(-0.02, 1.02)
    axes[0].set_title("Success vs perception noise")
    axes[1].set_ylabel("collision rate"), axes[1].set_ylim(-0.02, 0.75)
    axes[1].set_title("Collision vs perception noise")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    out = os.path.join(ROOT, "figures", "fig9_robustness.png")
    fig.savefig(out, dpi=180)
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    main()
