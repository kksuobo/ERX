"""Batch benchmark driver."""

import csv
import os

from .planners import PLANNERS
from .simulator import run_episode


def run_benchmark(cfg, scenarios, seeds, planner_names, out_csv=None,
                  record_planners=(), progress=True):
    seed0 = cfg["benchmark"]["seed0"]
    rows, records = [], {}
    total = len(scenarios) * seeds * len(planner_names)
    done = 0
    for scenario in scenarios:
        for k in range(seeds):
            seed = seed0 + k
            for pname in planner_names:
                planner = PLANNERS[pname](cfg)
                rec = pname in record_planners
                res = run_episode(planner, scenario, seed, cfg, record=rec)
                if rec:
                    records[(pname, scenario, seed)] = res.get("_log")
                    res = {k2: v for k2, v in res.items() if not k2.startswith("_")}
                res.update(planner=pname, scenario=scenario, seed=seed)
                rows.append(res)
                done += 1
                if progress and (done % 20 == 0 or done == total):
                    print(f"  [{done}/{total}] {pname:7s} {scenario:16s} "
                          f"seed={seed} {res['status']}")

    if out_csv:
        os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)
        with open(out_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            for r in rows:
                writer.writerow(r)
    return rows, records


def load_rows(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = []
        for r in csv.DictReader(f):
            for k, v in list(r.items()):
                if k in ("planner", "scenario", "status"):
                    continue
                try:
                    r[k] = float(v)
                except (TypeError, ValueError):
                    pass
            rows.append(r)
    return rows
