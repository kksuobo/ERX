"""Paper tables: Table 4 with Wilson CIs + noise, Table 6 (latency), Table 7."""

import _bootstrap  # noqa: F401
import csv
import os

import numpy as np

from radwa.config import config_fingerprint, load_config
from radwa.runner import load_rows
from radwa.stats import wilson

ROOT = _bootstrap.ROOT
ORDER = ["DWA", "DWA-P", "APF", "VO", "TEB", "MPPI", "RA-DWA"]


def main():
    rows = load_rows(os.path.join(ROOT, "results", "episodes.csv"))
    cfg = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    fp = config_fingerprint(cfg)
    by = {}
    for r in rows:
        by.setdefault(r["planner"], []).append(r)

    # success under medium perception noise, for the headline table
    noise = {}
    rb = os.path.join(ROOT, "results", "robustness.csv")
    if os.path.exists(rb):
        with open(rb, newline="", encoding="utf-8") as f:
            for r in csv.DictReader(f):
                if r["level"] == "medium":
                    noise[r["planner"]] = float(r["success"])

    lines = ["# Table 4 -- aggregate metrics with 95% Wilson CI on the rates, "
             "plus success under medium perception noise\n",
             "| Method | n | Success (95% CI) | Collision (95% CI) | Min clear. | "
             "Min TTC | Mean |jerk| | Var(omega) | Succ. (noise) |",
             "|---|---|---|---|---|---|---|---|---|"]
    for p in [x for x in ORDER if x in by]:
        rs = by[p]
        n = len(rs)
        ks = sum(r["success"] for r in rs)
        kc = sum(r["collision"] for r in rs)
        sl, sh = wilson(ks, n)
        cl, ch = wilson(kc, n)

        def ms(key, p=rs):
            v = np.array([r[key] for r in p], dtype=float)
            v = v[np.isfinite(v)]
            return float(np.mean(v)) if len(v) else float("nan")

        nz = f"{noise[p]:.2f}" if p in noise else "n/a"
        lines.append(
            f"| {p} | {n} | {ks/n:.2f} [{sl:.2f}, {sh:.2f}] | "
            f"{kc/n:.2f} [{cl:.2f}, {ch:.2f}] | {ms('min_clearance'):.2f} | "
            f"{ms('min_ttc'):.2f} | {ms('mean_abs_jerk'):.2f} | "
            f"{ms('omega_var'):.3f} | {nz} |")
    out4 = os.path.join(ROOT, "results", "table4_ci.md")
    with open(out4, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))

    def candidates_per_cycle(p):
        if p in ("DWA", "DWA-P"):
            return str(cfg["dwa"]["n_v"] * cfg["dwa"]["n_w"])
        if p == "VO":
            return str(cfg["vo"]["n_v"] * cfg["vo"]["n_w"])
        if p == "RA-DWA":
            return str(cfg["radwa"]["n_v"] * cfg["radwa"]["n_w"])
        if p == "MPPI":
            return str(cfg["mppi"]["n_samples"])
        if p == "TEB":
            return "3 x L-BFGS"
        return "1"

    lines6 = ["# Table 6 -- real-time cost per control cycle (100 ms budget)\n",
              "| Method | mean lat. [ms] | p95 lat. [ms] | max lat. [ms] | "
              "roll-outs/cycle | trained model |",
              "|---|---|---|---|---|---|"]
    for p in [x for x in ORDER if x in by]:
        rs = by[p]
        lat_mean = np.mean([r["lat_mean_ms"] for r in rs])
        lat_p95 = np.max([r["lat_p95_ms"] for r in rs])
        lat_max = np.max([r["lat_max_ms"] for r in rs])
        lines6.append(f"| {p} | {lat_mean:.2f} | {lat_p95:.2f} | {lat_max:.2f} | "
                      f"{candidates_per_cycle(p)} | no |")
    out6 = os.path.join(ROOT, "results", "table6_latency.md")
    with open(out6, "w", encoding="utf-8") as f:
        f.write("\n".join(lines6) + "\n")
    print("\n" + "\n".join(lines6))

    scenarios = sorted({r["scenario"] for r in rows})
    lines7 = ["# Table 7 -- per-scenario success/collision with 95% Wilson CI\n",
              f"Config fingerprint (SHA-256 of frozen config): `{fp}`\n",
              "| Scenario | Method | Success (95% CI) | Collision (95% CI) | "
              "Min TTC | Mean |jerk| |",
              "|---|---|---|---|---|---|"]
    for sc in scenarios:
        for p in [x for x in ORDER if x in by]:
            rs = [r for r in by[p] if r["scenario"] == sc]
            if not rs:
                continue
            n = len(rs)
            sl, sh = wilson(sum(r["success"] for r in rs), n)
            cl, ch = wilson(sum(r["collision"] for r in rs), n)
            kt = [r["min_ttc"] for r in rs if r["min_ttc"] == r["min_ttc"]]
            kj = [r["mean_abs_jerk"] for r in rs]
            lines7.append(
                f"| {sc} | {p} | {sum(r['success'] for r in rs)/n:.2f} "
                f"[{sl:.2f}, {sh:.2f}] | {sum(r['collision'] for r in rs)/n:.2f} "
                f"[{cl:.2f}, {ch:.2f}] | "
                f"{np.mean(kt) if kt else float('nan'):.2f} | {np.mean(kj):.2f} |")
    out7 = os.path.join(ROOT, "results", "table7_per_scenario.md")
    with open(out7, "w", encoding="utf-8") as f:
        f.write("\n".join(lines7) + "\n")

    print(f"\nwrote {out4}, {out6} and {out7}")
    print(f"config fingerprint: {fp}")


if __name__ == "__main__":
    main()
