"""Paired significance testing of RA-DWA against each baseline (Table 4 support).

Episodes are matched by (scenario, seed). Binary metrics use an exact McNemar
test on discordant pairs; continuous metrics use the Wilcoxon signed-rank test.
Reference method: RA-DWA.
"""

import argparse
import _bootstrap  # noqa: F401
import os

import numpy as np

from radwa.runner import load_rows
from radwa.stats import mcnemar, star, wilcoxon

ROOT = _bootstrap.ROOT
REFERENCE = "RA-DWA"
BIN_METRICS = ["success", "collision"]
CONT_METRICS = ["min_clearance", "min_ttc", "mean_abs_jerk", "omega_var",
                "lat_p95_ms"]
ORDER = ["DWA", "DWA-P", "APF", "VO", "TEB", "MPPI", "RA-DWA"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.join(ROOT, "results", "episodes.csv"))
    args = ap.parse_args()
    rows = load_rows(args.csv)

    by_planner = {}
    for r in rows:
        by_planner.setdefault(r["planner"], {})[(r["scenario"], int(r["seed"]))] = r

    ref = by_planner[REFERENCE]
    planners = [p for p in ORDER if p in by_planner]

    def paired(pname, metric):
        a, b = [], []
        for key, rref in ref.items():
            other = by_planner[pname].get(key)
            if other is None:
                continue
            a.append(rref[metric])
            b.append(other[metric])
        return np.asarray(a), np.asarray(b)

    def ms(vals):
        vals = np.asarray(vals, dtype=float)
        vals = vals[np.isfinite(vals)]
        return (np.mean(vals) if len(vals) else float("nan"),
                np.std(vals) if len(vals) else float("nan"))

    lines = []
    lines.append(f"# Table 4 support -- paired comparison vs {REFERENCE} "
                 f"(matched by scenario+seed; reference n={len(ref)})\n")
    lines.append("| Method | n | Success | Collision | Min clear. | Min TTC | "
                 "Mean |jerk| | Var(omega) | p95 (ms) |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for p in planners:
        d = by_planner[p]
        n_match = sum(1 for k in d if k in ref)
        cells = []
        for m in ["success", "collision", "min_clearance", "min_ttc",
                  "mean_abs_jerk", "omega_var", "lat_p95_ms"]:
            mu, sd = ms([r[m] for r in d.values()])
            cells.append(f"{mu:.2f}+-{sd:.2f}")
        lines.append(f"| {p} | {n_match} | " + " | ".join(cells) + " |")

    lines.append(f"\nSignificance vs {REFERENCE} "
                 f"(McNemar for rates, Wilcoxon for the rest):\n")
    lines.append("| Baseline | succ p | coll p | clear p | TTC p | jerk p | "
                 "var(w) p | p95 p |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for p in planners:
        if p == REFERENCE:
            lines.append(f"| {p} (ref) | - | - | - | - | - | - | - |")
            continue
        cells = []
        for m in BIN_METRICS:
            a, b = paired(p, m)
            pv = mcnemar(a, b)[0]
            cells.append(f"{pv:.1e} {star(pv)}")
        for m in CONT_METRICS:
            a, b = paired(p, m)
            pv = wilcoxon(a, b)[0]
            cells.append(f"{pv:.1e} {star(pv)}")
        lines.append(f"| {p} | " + " | ".join(cells) + " |")

    scenarios = sorted({k[0] for k in ref})
    lines.append(f"\nPer-scenario success/collision McNemar p (vs {REFERENCE}):\n")
    lines.append("| Scenario | Baseline | #succ RA/base | #coll RA/base | "
                 "succ p | coll p |")
    lines.append("|---|---|---|---|---|---|")
    for sc in scenarios:
        for p in planners:
            if p == REFERENCE:
                continue
            keys = [k for k in ref if k[0] == sc and k in by_planner[p]]
            if not keys:
                continue
            ka = [ref[k] for k in keys]
            kb = [by_planner[p][k] for k in keys]
            succ_a = sum(r["success"] for r in ka)
            coll_a = sum(r["collision"] for r in ka)
            succ_b = sum(r["success"] for r in kb)
            coll_b = sum(r["collision"] for r in kb)
            ps = mcnemar([r["success"] for r in ka],
                         [r["success"] for r in kb])[0]
            pc = mcnemar([r["collision"] for r in ka],
                         [r["collision"] for r in kb])[0]
            lines.append(f"| {sc} | {p} | {succ_a}/{succ_b} | {coll_a}/{coll_b} "
                         f"| {ps:.1e} {star(ps)} | {pc:.1e} {star(pc)} |")

    text = "\n".join(lines) + "\n"
    print(text)
    out = os.path.join(os.path.dirname(args.csv), "table_significance.md")
    with open(out, "w", encoding="utf-8") as f:
        f.write(text)
    print("wrote", out)


if __name__ == "__main__":
    main()
