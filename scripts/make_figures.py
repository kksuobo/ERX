"""Generate paper figures (Fig. 5 qualitative, Fig. 6 quantitative)."""

import argparse
import _bootstrap  # noqa: F401
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from radwa.config import load_config
from radwa.planners import PLANNERS
from radwa.stats import wilson
from radwa.runner import load_rows
from radwa.scenarios import build_scenario
from radwa.simulator import run_episode

ROOT = _bootstrap.ROOT
COLORS = {"DWA": "#1f77b4", "DWA-P": "#17becf", "APF": "#ff7f0e",
          "VO": "#2ca02c", "TEB": "#8c564b", "MPPI": "#9467bd",
          "RA-DWA": "#d62728"}


def fig_qualitative(cfg, scenario, seed, planners, out):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    _, goal, obstacles = build_scenario(scenario, seed, cfg)
    for o in obstacles:
        axes[0].add_patch(plt.Circle((o.x, o.y), o.r, color="0.6", alpha=0.7))
    axes[0].plot(goal[0], goal[1], "k*", ms=15, label="goal")
    for pname in planners:
        res = run_episode(PLANNERS[pname](cfg), scenario, seed, cfg, record=True)
        log = res["_log"]
        axes[0].plot(log["x"], log["y"], color=COLORS.get(pname, None),
                     lw=2, label=f"{pname} ({res['status']})")
        axes[1].plot(log["t"], log["v"], color=COLORS.get(pname, None),
                     lw=1.8, label=pname)
    axes[0].set_title(f"{scenario} (seed {seed}) - path")
    axes[0].set_xlabel("x [m]"), axes[0].set_ylabel("y [m]")
    axes[0].axis("equal"), axes[0].grid(alpha=0.3), axes[0].legend(fontsize=8)
    axes[1].set_title("Speed profile")
    axes[1].set_xlabel("t [s]"), axes[1].set_ylabel("v [m/s]")
    axes[1].grid(alpha=0.3), axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)
    print("wrote", out)


def fig_quantitative(rows, out):
    import numpy as np
    by = {}
    for r in rows:
        by.setdefault(r["planner"], []).append(r)
    order = [p for p in ["DWA", "DWA-P", "APF", "VO", "TEB", "MPPI", "RA-DWA"]
             if p in by]
    names = order

    def rate(p, key):
        rs = by[p]
        k = sum(r[key] for r in rs)
        n = len(rs)
        lo, hi = wilson(k, n)
        return k / n, k / n - lo, hi - k / n

    def mean_sem(p, key):
        v = np.array([r[key] for r in by[p]], dtype=float)
        v = v[np.isfinite(v)]
        return float(np.mean(v)), float(1.96 * np.std(v) / math.sqrt(len(v)))

    metrics = [("success", "Success rate", 1, "rate"),
               ("collision", "Collision rate", 1, "rate"),
               ("mean_abs_jerk", "Mean |jerk|", 1, "mean"),
               ("omega_var", "Var(omega)", 1e3, "mean"),
               ("lat_p95_ms", "p95 latency [ms]", 1, "mean")]
    fig, axes = plt.subplots(1, len(metrics), figsize=(16, 3.6))
    for ax, (k, title, scale, kind) in zip(axes, metrics):
        vals, errs = [], []
        for p in names:
            if kind == "rate":
                v, lo, hi = rate(p, k)
                vals.append(v * scale)
                errs.append([lo * scale, hi * scale])
            else:
                m, e = mean_sem(p, k)
                vals.append(m * scale)
                errs.append(e * scale)
        errs = np.array(errs).T
        ax.bar(names, vals, color=[COLORS.get(n, "#888") for n in names],
               yerr=errs, capsize=3, error_kw=dict(lw=0.8, ecolor="#333"))
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3, axis="y")
        for i, v in enumerate(vals):
            ax.text(i, v, f"{v:.2f}" if scale == 1 else f"{v:.1f}",
                    ha="center", va="bottom", fontsize=8)
    for ax in axes:
        for lbl in ax.get_xticklabels():
            lbl.set_rotation(25)
            lbl.set_ha("right")
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)
    print("wrote", out)


def fig_pareto(rows, out):
    """Success vs safety margin, with comfort as marker size (Figure 10)."""
    import numpy as np
    by = {}
    for r in rows:
        by.setdefault(r["planner"], []).append(r)
    fig, ax = plt.subplots(figsize=(6.6, 5.0))
    for p, rs in by.items():
        succ = np.mean([r["success"] for r in rs])
        tt = [r["min_ttc"] for r in rs if r["min_ttc"] == r["min_ttc"]]
        mttc = np.mean(tt) if tt else 0.0
        jerk = np.mean([r["mean_abs_jerk"] for r in rs])
        ax.scatter(mttc, succ, s=90 + 45 * jerk, color=COLORS.get(p, "#888"),
                   alpha=0.78, edgecolor="k", zorder=3)
        ax.annotate(p, (mttc, succ), textcoords="offset points", xytext=(7, 6),
                    fontsize=9)
    ax.axvline(2.0, color="0.6", ls=":", lw=1)
    ax.set_xlabel("mean minimum TTC margin [s]")
    ax.set_ylabel("success rate")
    ax.set_title("Safety-margin / success trade-off (marker size = mean |jerk|)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)
    print("wrote", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.join(ROOT, "results", "episodes.csv"))
    ap.add_argument("--scenario", default="merge")
    ap.add_argument("--seed", type=int, default=3)
    args = ap.parse_args()
    cfg = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    figdir = os.path.join(ROOT, "figures")

    fig_qualitative(cfg, args.scenario, args.seed, ["DWA", "VO", "RA-DWA"],
                    os.path.join(figdir, f"fig5_qualitative_{args.scenario}.png"))
    if os.path.exists(args.csv):
        rows = load_rows(args.csv)
        fig_quantitative(rows, os.path.join(figdir, "fig6_quantitative.png"))
        fig_pareto(rows, os.path.join(figdir, "fig10_pareto.png"))


if __name__ == "__main__":
    main()
