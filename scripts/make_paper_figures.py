"""Remaining paper figures.

Manuscript numbering produced by this script:
  Figure 1 -> figures/fig1_pipeline.png       (system overview schematic)
  Figure 4 -> figures/fig4_scenarios.png      (scenario suite, top-down)
  Figure 9 -> figures/fig9_ablation.png       (ablation summary bars)
"""

import _bootstrap  # noqa: F401
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, Rectangle

from radwa.config import load_config
from radwa.scenarios import build_scenario

ROOT = _bootstrap.ROOT


# --------------------------------------------------------------------------- Fig 1
def _box(ax, x, y, w, h, text, fc="#e8f0fb", ec="#33507a", fs=9):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle="round,pad=0.06,rounding_size=0.12",
                                fc=fc, ec=ec, lw=1.2))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs)


def _arrow(ax, p0, p1, style="->", color="#33507a", lw=1.3):
    ax.annotate("", xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle=style, color=color, lw=lw))


def fig_pipeline(out):
    fig, ax = plt.subplots(figsize=(13, 5.2))
    ax.set_xlim(0, 15.4)
    ax.set_ylim(0, 6.6)
    ax.axis("off")

    y, h = 3.4, 1.3
    b1 = (0.2, 2.05)
    b2 = (2.65, 1.95)
    b3 = (4.95, 1.85)
    b4 = (7.15, 2.05)
    b5 = (9.6, 2.05)
    b6 = (12.0, 1.6)
    _box(ax, b1[0], y, b1[1], h, "Ego state $(x,y,\\theta,v,\\omega)$\nGoal  ·  Obstacles")
    _box(ax, b2[0], y, b2[1], h, "Dynamic window\n$V_d$  (Eq. 2)")
    _box(ax, b3[0], y, b3[1], h, "Sample $(v,\\omega)$\n$n_v\\!\\times\\!n_w$")
    _box(ax, b4[0], y, b4[1], h, "Roll-out  +  normalized\nTTC risk field\n$R=e^{-TTC/T_0}$",
         fc="#fde9e0")
    _box(ax, b5[0], y, b5[1], h, "Coupled cost $C$\n(Eq. 5)  +\ncollision penalty",
         fc="#e7f4e7")
    _box(ax, b6[0], y, b6[1], h, "arg min\n$(v,\\omega)$", fc="#fdf3d0")

    for (a, wa), (b, _wb) in [(b1, b2), (b2, b3), (b3, b4), (b4, b5), (b5, b6)]:
        _arrow(ax, (a + wa, y + h / 2), (b, y + h / 2))
    # while-loop marker
    ax.text(2.1, y + h + 0.08, "for all candidates", fontsize=8, color="#666")

    _box(ax, 4.4, 5.1, 6.2, 1.1,
         "Risk-adaptive scheduler:  $\\rho$  →  {free, follow, conflict, emergency}\n"
         "weights $w_g,w_c,w_r,w_s,w_j,w_d$  (hysteresis + EMA)",
         fc="#efe6f7", ec="#5a3f7a", fs=8.5)
    _arrow(ax, (7.0, 5.1), (10.2, y + h))
    ax.text(8.9, 4.78, "weights", fontsize=8, color="#5a3f7a")
    _arrow(ax, (5.0, 5.1), (3.4, y + h))
    ax.text(3.2, 4.78, "$v_{max,eff}$ (Eq. 6)", fontsize=8, color="#5a3f7a")

    _box(ax, 5.6, 1.4, 3.1, 0.95,
         "Plant: unicycle + actuator limits (Eq. 1)", fc="#f2f2f2", ec="#444")
    _arrow(ax, (b6[0] + b6[1] / 2, y), (7.15, 2.35))
    _arrow(ax, (5.6, 1.4), (1.2, 2.05), style="->", color="#888")
    ax.text(2.4, 1.05, "next cycle (feedback)", fontsize=8, color="#666")
    ax.set_title("RA-DWA control cycle: one risk field couples the steering and speed decisions",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=170)
    plt.close(fig)
    print("wrote", out)


# --------------------------------------------------------------------------- Fig 4
def fig_scenarios(cfg, out):
    names = list(cfg["scenarios"].keys())
    fig, axes = plt.subplots(2, 3, figsize=(15, 7.2))
    for ax, name in zip(axes.ravel(), names):
        start, goal, obs = build_scenario(name, cfg["benchmark"]["seed0"], cfg)
        for o in obs:
            ax.add_patch(plt.Circle((o.x, o.y), o.r, color="0.72", ec="k", lw=0.6))
        ax.plot(start[0], start[1], "o", color="#2ca02c", ms=9, mec="k")
        bearing = math.atan2(goal[1] - start[1], goal[0] - start[0])
        ax.arrow(start[0], start[1], 3 * math.cos(bearing), 3 * math.sin(bearing),
                 color="#2ca02c", width=0.12, head_width=0.7)
        ax.plot(goal[0], goal[1], "*", color="#d62728", ms=16, mec="k")
        ax.set_title(name, fontsize=10)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.25)
        ax.set_xlabel("x [m]"), ax.set_ylabel("y [m]")
    axes.ravel()[0].plot([], [], "o", color="#2ca02c", label="ego start")
    axes.ravel()[0].plot([], [], "*", color="#d62728", label="goal")
    axes.ravel()[0].legend(fontsize=8, loc="lower right")
    fig.suptitle("Scenario suite: obstacles (grey), ego start (green), goal (red star)",
                 fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out, dpi=170)
    plt.close(fig)
    print("wrote", out)


# --------------------------------------------------------------------------- Fig 9
def fig_ablation(out):
    path = os.path.join(ROOT, "results", "ablation.csv")
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    by = {}
    for r in rows:
        by.setdefault(r["planner"], []).append(r)
    order = ["RA-DWA", "no risk field", "distance risk", "fixed weights",
             "no hysteresis", "no smoothing", "no motion pred", "no speed mod"]
    order = [o for o in order if o in by]

    def mean(name, key):
        v = [float(r[key]) for r in by[name]]
        v = [x for x in v if x == x]
        return float(np.mean(v))

    labels = order
    succ = [mean(n, "success") for n in labels]
    mttc = [mean(n, "min_ttc") for n in labels]
    jerk = [mean(n, "mean_abs_jerk") for n in labels]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    for ax, vals, title, fmt in [
            (axes[0], succ, "Success rate", "{:.2f}"),
            (axes[1], mttc, "Minimum TTC [s]", "{:.1f}"),
            (axes[2], jerk, "Mean |jerk|", "{:.2f}")]:
        colors = ["#d62728"] + ["#7f7f7f"] * (len(labels) - 1)
        ax.bar(range(len(labels)), vals, color=colors)
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, rotation=40, ha="right", fontsize=8)
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3, axis="y")
        for i, v in enumerate(vals):
            ax.text(i, v, fmt.format(v), ha="center", va="bottom", fontsize=8)
    fig.suptitle("Ablation (6 scenarios x 20 seeds): full method in red",
                 fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out, dpi=170)
    plt.close(fig)
    print("wrote", out)


def main():
    cfg = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    figdir = os.path.join(ROOT, "figures")
    fig_pipeline(os.path.join(figdir, "fig1_pipeline.png"))
    fig_scenarios(cfg, os.path.join(figdir, "fig4_scenarios.png"))
    fig_ablation(os.path.join(figdir, "fig9_ablation.png"))


if __name__ == "__main__":
    main()
