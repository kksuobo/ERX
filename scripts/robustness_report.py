"""Render results/robustness.md from results/robustness.csv (with CIs)."""

import _bootstrap  # noqa: F401
import csv
import os

ROOT = _bootstrap.ROOT
ORDER = ["DWA", "DWA-P", "APF", "VO", "MPPI", "RA-DWA"]


def main():
    src = os.path.join(ROOT, "results", "robustness.csv")
    rows = list(csv.DictReader(open(src, newline="", encoding="utf-8")))
    levels = []
    for r in rows:
        if r["level"] not in levels:
            levels.append(r["level"])

    lines = ["# Robustness to perception noise (6 scenarios x 15 seeds = 90 "
             "episodes per cell)\n",
             "Noise adds Gaussian position/velocity error and random obstacle "
             "dropout to what the planner perceives; ground truth drives motion "
             "and metrics. Reproduce with `python scripts\\robustness.py "
             "--seeds 15` (TEB is excluded: its per-cycle cost precludes a "
             "real-time robustness comparison).\n",
             "| Level | Method | Success (95% CI) | Collision (95% CI) | "
             "Min clear. | Min TTC | Mean \\|jerk\\| |",
             "|---|---|---|---|---|---|---|"]
    for lv in levels:
        for p in ORDER:
            r = next((x for x in rows if x["level"] == lv and x["planner"] == p),
                     None)
            if not r:
                continue
            lines.append(
                f"| {lv} | {p} | {float(r['success']):.2f} "
                f"[{float(r['succ_ci_lo']):.2f}, {float(r['succ_ci_hi']):.2f}] | "
                f"{float(r['collision']):.2f} "
                f"[{float(r['coll_ci_lo']):.2f}, {float(r['coll_ci_hi']):.2f}] | "
                f"{float(r['min_clearance']):.2f} | {float(r['min_ttc']):.2f} | "
                f"{float(r['mean_abs_jerk']):.2f} |")
    lines.append("\nKey finding: DWA-P (prediction, no risk field) is the most "
                 "robust planner at medium noise, and DWA-P and RA-DWA are the "
                 "best at high noise. VO's apparent collapse under noise is "
                 "largely a safety-margin effect: with its margin matched to "
                 "RA-DWA's it holds 0.92 at medium and 0.68 at high noise "
                 "(results/robustness_vo_m07.csv, results/robustness_vo_m12.csv). "
                 "RA-DWA's residual advantage under noise is the larger minimum "
                 "TTC, not task success.\n")

    out = os.path.join(ROOT, "results", "robustness.md")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("wrote", out)


if __name__ == "__main__":
    main()
