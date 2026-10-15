"""RA-DWA ablation study (Table 5): saved rows + paired tests vs the full method."""

import argparse
import _bootstrap  # noqa: F401
import copy
import csv
import os

import numpy as np

from radwa.config import load_config
from radwa.planners.radwa import RADWA
from radwa.simulator import run_episode
from radwa.stats import mcnemar, star, wilcoxon

ROOT = _bootstrap.ROOT

VARIANTS = {
    "RA-DWA": {},
    "fixed weights": {"use_scheduler": False},
    "no hysteresis": {"hysteresis_enabled": False},
    "no smoothing": {"hysteresis_enabled": False, "risk_ema": 1.0, "weight_ema": 1.0},
    "no motion pred": {"motion_prediction": False},
    "no risk field": {"risk_scale": 0.0},
    "distance risk": {"risk_mode": "distance"},
    "no speed mod": {"speed_modulation": False},
}
FULL = "RA-DWA"
METRICS = ["success", "collision", "min_clearance", "min_ttc", "mean_abs_jerk",
           "omega_var"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--scenarios", default="all")
    ap.add_argument("--out", default=os.path.join(ROOT, "results", "ablation.csv"))
    ap.add_argument("--table", default=os.path.join(ROOT, "results",
                                                    "table5_ablation.md"))
    ap.add_argument("--from-csv", default=None,
                    help="report-only: build the table from an existing CSV")
    args = ap.parse_args()
    base = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    scenarios = (list(base["scenarios"].keys()) if args.scenarios == "all"
                 else args.scenarios.split(","))
    seed0 = base["benchmark"]["seed0"]

    print(f"\nAblation: {len(VARIANTS)} variants x {len(scenarios)} scenarios "
          f"x {args.seeds} seeds")
    rows = []
    if args.from_csv:
        from radwa.runner import load_rows
        rows = load_rows(args.from_csv)
        print(f"report-only from {args.from_csv}: {len(rows)} episodes")
    else:
        for name, over in VARIANTS.items():
            cfg = copy.deepcopy(base)
            cfg["radwa"].update(over)
            for sc in scenarios:
                for k in range(args.seeds):
                    res = run_episode(RADWA(cfg), sc, seed0 + k, cfg)
                    res.update(planner=name, scenario=sc, seed=seed0 + k)
                    rows.append(res)

    by = {}
    for r in rows:
        by.setdefault(r["planner"], []).append(r)

    def col(name, key):
        return [r[key] for r in by[name]]

    def ms(name, key):
        v = np.asarray(col(name, key), float)
        v = v[np.isfinite(v)]
        return np.mean(v), np.std(v)

    lines = [f"# Table 5 -- ablation ({len(scenarios)} scenarios x {args.seeds} "
             f"seeds = {len(by[FULL])} episodes/variant)\n",
             "| Variant | Succ. | Coll. | Min clear. | Min TTC | Mean \\|jerk\\| | "
             "Var(omega) | succ p | coll p | TTC p | jerk p |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for name in VARIANTS:
        s = col(name, "success")
        c = col(name, "collision")
        cells = [f"{np.mean(s):.2f}", f"{np.mean(c):.2f}"]
        for key in ("min_clearance", "min_ttc", "mean_abs_jerk", "omega_var"):
            mu, sd = ms(name, key)
            cells.append(f"{mu:.2f}")
        if name == FULL:
            cells += ["-", "-", "-", "-"]
        else:
            bin_p = [mcnemar(col(FULL, "success"), s)[0],
                     mcnemar(col(FULL, "collision"), c)[0]]
            tp = wilcoxon(col(FULL, "min_ttc"), col(name, "min_ttc"))[0]
            jp = wilcoxon(col(FULL, "mean_abs_jerk"),
                          col(name, "mean_abs_jerk"))[0]
            cells += [f"{bin_p[0]:.1e} {star(bin_p[0])}",
                      f"{bin_p[1]:.1e} {star(bin_p[1])}",
                      f"{tp:.1e} {star(tp)}",
                      f"{jp:.1e} {star(jp)}"]
        lines.append(f"| {name} | " + " | ".join(cells) + " |")

    text = "\n".join(lines) + "\n"
    print(text)
    if args.table.upper() != "NONE":
        with open(args.table, "w", encoding="utf-8") as f:
            f.write(text)

    if not args.from_csv:
        with open(args.out, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            for r in rows:
                w.writerow(r)
    print(f"wrote {args.table}"
          + ("" if args.from_csv else f" and {args.out}"))


if __name__ == "__main__":
    main()
