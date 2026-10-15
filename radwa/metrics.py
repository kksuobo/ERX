"""Aggregation of episode metrics into paper tables (Table 4/5/6)."""

import math

import numpy as np

RATE_KEYS = ["success", "collision", "timeout", "deadlock"]
SCALAR_KEYS = ["time_to_goal", "path_length", "min_clearance", "min_ttc",
               "mean_abs_jerk", "omega_var", "mean_speed",
               "lat_mean_ms", "lat_p95_ms", "lat_max_ms"]


def _nanmean(vals):
    vals = [v for v in vals if not (isinstance(v, float) and math.isnan(v))]
    return float(np.mean(vals)) if vals else float("nan")


def _nanstd(vals):
    vals = [v for v in vals if not (isinstance(v, float) and math.isnan(v))]
    return float(np.std(vals)) if vals else float("nan")


def summarize(rows):
    """Group episode rows by (planner, scenario) and by planner only."""
    groups = {}
    for r in rows:
        groups.setdefault((r["planner"], r["scenario"]), []).append(r)

    summary = []
    for (planner, scenario), rs in groups.items():
        rec = {"planner": planner, "scenario": scenario, "n": len(rs)}
        for k in RATE_KEYS:
            rec[k] = _nanmean([r[k] for r in rs])
        for k in SCALAR_KEYS:
            rec[k] = _nanmean([r[k] for r in rs])
            rec[k + "_std"] = _nanstd([r[k] for r in rs])
        summary.append(rec)

    overall = {}
    for r in rows:
        overall.setdefault(r["planner"], []).append(r)
    overall_rows = []
    for planner, rs in overall.items():
        rec = {"planner": planner, "n": len(rs)}
        for k in RATE_KEYS:
            rec[k] = _nanmean([r[k] for r in rs])
        for k in SCALAR_KEYS:
            rec[k] = _nanmean([r[k] for r in rs])
            rec[k + "_std"] = _nanstd([r[k] for r in rs])
        overall_rows.append(rec)
    return summary, overall_rows
