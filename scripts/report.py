"""Build paper tables (Table 4 per-scenario, overall) from episodes.csv."""

import argparse
import _bootstrap  # noqa: F401
import os

from radwa.metrics import summarize
from radwa.runner import load_rows

ROOT = _bootstrap.ROOT
ORDER = ["DWA", "DWA-P", "APF", "VO", "TEB", "MPPI", "RA-DWA"]


def _fmt(x, nd=2):
    return "  n/a" if x != x else f"{x:.{nd}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.join(ROOT, "results", "episodes.csv"))
    args = ap.parse_args()
    rows = load_rows(args.csv)
    per, overall = summarize(rows)

    def key(rec):
        return (ORDER.index(rec["planner"]) if rec["planner"] in ORDER else 99,
                rec["planner"])

    print(f"\n=== Overall ({len(rows)} episodes) ===")
    print(f"{'planner':8s} {'succ':>6s} {'coll':>6s} {'tmo':>6s} {'mclear':>7s} "
          f"{'mTTC':>6s} {'jerk':>6s} {'wvar':>6s} {'p95ms':>7s}")
    for r in sorted(overall, key=key):
        print(f"{r['planner']:8s} {r['success']:6.2f} {r['collision']:6.2f} "
              f"{r['timeout']:6.2f} {_fmt(r['min_clearance']):>7s} "
              f"{_fmt(r['min_ttc']):>6s} {_fmt(r['mean_abs_jerk']):>6s} "
              f"{_fmt(r['omega_var'], 3):>6s} {r['lat_p95_ms']:7.2f}")

    print(f"\n=== Per scenario ===")
    print(f"{'scenario':16s} {'planner':8s} {'succ':>6s} {'coll':>6s} {'tmo':>6s} "
          f"{'mclear':>7s} {'jerk':>6s} {'wvar':>6s}")
    for r in sorted(per, key=lambda x: (x["scenario"], key(x))):
        print(f"{r['scenario']:16s} {r['planner']:8s} {r['success']:6.2f} "
              f"{r['collision']:6.2f} {r['timeout']:6.2f} "
              f"{_fmt(r['min_clearance']):>7s} {_fmt(r['mean_abs_jerk']):>6s} "
              f"{_fmt(r['omega_var'], 3):>6s}")

    # Markdown tables for direct pasting into the manuscript.
    md = ["| Method | Succ. | Coll. | Timeout | Min clear. | Min TTC | Mean |jerk| | Var(omega) | p95 (ms) |",
          "|---|---|---|---|---|---|---|---|---|"]
    for r in sorted(overall, key=key):
        md.append(f"| {r['planner']} | {r['success']:.2f} | {r['collision']:.2f} | "
                  f"{r['timeout']:.2f} | {_fmt(r['min_clearance'])} | {_fmt(r['min_ttc'])} | "
                  f"{_fmt(r['mean_abs_jerk'])} | {_fmt(r['omega_var'], 3)} | {r['lat_p95_ms']:.1f} |")
    out = os.path.join(os.path.dirname(args.csv), "table_overall.md")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
