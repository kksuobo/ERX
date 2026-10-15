"""Figure: success/collision under actuation delay and command jitter."""

import _bootstrap  # noqa: F401
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = _bootstrap.ROOT
LEVELS = ["ideal", "delay 1 cycle", "delay 2 cycles", "jitter 0.2",
          "delay 2 + jitter"]
COLORS = {"DWA": "#1f77b4", "VO": "#2ca02c", "RA-DWA": "#d62728"}


def main():
    path = os.path.join(ROOT, "results", "stress.csv")
    data = {}
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            data.setdefault(r["planner"], {})[r["level"]] = r

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    for planner, by_level in data.items():
        c = COLORS.get(planner, "#888")
        succ = [float(by_level[lv]["success"]) for lv in LEVELS]
        coll = [float(by_level[lv]["collision"]) for lv in LEVELS]
        axes[0].plot(LEVELS, succ, "o-", color=c, label=planner)
        axes[1].plot(LEVELS, coll, "s--", color=c, label=planner)
    axes[0].set_ylabel("success rate"), axes[0].set_ylim(-0.02, 1.02)
    axes[0].set_title("Success vs actuation stress")
    axes[1].set_ylabel("collision rate"), axes[1].set_ylim(-0.02, 0.5)
    axes[1].set_title("Collision vs actuation stress")
    for ax in axes:
        ax.grid(alpha=0.3), ax.legend(fontsize=8)
        for lbl in ax.get_xticklabels():
            lbl.set_rotation(20), lbl.set_ha("right")
    fig.tight_layout()
    out = os.path.join(ROOT, "figures", "fig12_stress.png")
    fig.savefig(out, dpi=180)
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    main()
