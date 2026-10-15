"""Fast sanity check: one seed on every scenario with every planner."""

import _bootstrap  # noqa: F401
import os

from radwa.config import load_config
from radwa.planners import PLANNERS
from radwa.runner import run_benchmark
from radwa.metrics import summarize

ROOT = _bootstrap.ROOT


def main():
    cfg = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    scenarios = list(cfg["scenarios"].keys())
    planners = list(PLANNERS.keys())
    rows, _ = run_benchmark(cfg, scenarios, seeds=1, planner_names=planners,
                            progress=False)
    _, overall = summarize(rows)
    print("\n=== smoke test (1 seed) ===")
    print(f"{'planner':8s} {'succ':>6s} {'coll':>6s} {'p95ms':>7s} {'jerk':>8s}")
    for rec in sorted(overall, key=lambda r: r["planner"]):
        print(f"{rec['planner']:8s} {rec['success']:6.2f} {rec['collision']:6.2f} "
              f"{rec['lat_p95_ms']:7.2f} {rec['mean_abs_jerk']:8.3f}")

    # Invariant checks.
    assert all(r["n_cycles"] >= 1 for r in rows), "planner produced no output"
    assert all(0.0 <= r["success"] <= 1 for r in rows)
    print("\nsmoke test OK:", len(rows), "episodes")


if __name__ == "__main__":
    main()
