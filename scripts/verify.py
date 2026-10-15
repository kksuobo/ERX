"""Dependency-free regression / invariant tests (run: python scripts\\verify.py).

Covers geometry, TTC/risk math, scenario determinism, full-episode determinism,
metric bookkeeping and headline performance band. Exits non-zero on failure.
"""

import _bootstrap  # noqa: F401
import math
import os
import subprocess
import sys

from radwa.config import config_fingerprint, load_config
from radwa.geometry import angle_diff, dist, unicycle_step, wrap_angle
from radwa.planners import PLANNERS
from radwa.risk import INF, risk_from_clearance, risk_from_ttc, time_to_collision
from radwa.scenarios import build_scenario
from radwa.simulator import run_episode

ROOT = _bootstrap.ROOT
_checks = [0, 0]


def check(cond, msg):
    _checks[1] += 1
    if cond:
        _checks[0] += 1
    else:
        print("FAIL:", msg)


def approx(a, b, tol=1e-9):
    return abs(a - b) <= tol


def main():
    cfg = load_config(os.path.join(ROOT, "configs", "experiment.json"))

    # --- geometry ---------------------------------------------------------
    check(wrap_angle(math.pi + 0.1) < -math.pi + 0.2, "wrap_angle upper")
    check(abs(angle_diff(3.0, -3.0) - (3.0 + 3.0 - 2 * math.pi)) < 1e-9,
          "angle_diff wrap")
    check(approx(dist(0, 0, 3, 4), 5.0), "dist")
    x, y, th = unicycle_step(0, 0, 0, 1.0, 0.0, 0.1)
    check(approx(x, 0.1) and approx(y, 0.0) and approx(th, 0.0), "unicycle straight")
    x, y, th = unicycle_step(0, 0, 0, 0.0, 1.0, 0.1)
    check(approx(th, 0.1) and approx(x, 0.0) and approx(y, 0.0), "unicycle turn-in-place")

    # --- TTC / risk -------------------------------------------------------
    check(approx(time_to_collision(0, 0, 1, 0, 10, 0, -1, 0, 2.0), 4.0),
          "TTC head-on")
    check(time_to_collision(0, 0, 1, 0, -10, 0, -1, 0, 2.0) == INF,
          "TTC receding is INF")
    check(time_to_collision(0, 0, 0, 0, 1, 0, 0, 0, 2.0) == 0.0, "TTC inside")
    r_lo = risk_from_ttc(0.5, 2.0)
    r_hi = risk_from_ttc(5.0, 2.0)
    check(0.0 <= r_hi < r_lo <= 1.0, "risk monotone in TTC")
    check(risk_from_ttc(INF, 2.0) == 0.0, "risk at INF")
    check(risk_from_ttc(100.0, 2.0, cutoff=10.0) == 0.0, "risk cutoff")
    check(risk_from_clearance(0.0, 4.0) == 1.0, "clearance risk at 0")
    check(risk_from_clearance(5.0, 4.0) == 0.0, "clearance risk beyond range")

    # --- scenario determinism --------------------------------------------
    s1 = build_scenario("crossing", 7, cfg)
    s2 = build_scenario("crossing", 7, cfg)
    check(s1[1] == s2[1], "same goal for same seed")
    check([(o.x, o.y, o.vx, o.vy) for o in s1[2]]
          == [(o.x, o.y, o.vx, o.vy) for o in s2[2]], "same obstacles for same seed")

    # --- episode determinism ---------------------------------------------
    ra = PLANNERS["RA-DWA"](cfg)
    a = run_episode(ra, "merge", 3, cfg)
    b = run_episode(PLANNERS["RA-DWA"](cfg), "merge", 3, cfg)
    for k in ("status", "min_clearance", "min_ttc", "path_length", "n_cycles"):
        check(a[k] == b[k], f"episode determinism: {k}")

    # --- metric bookkeeping ----------------------------------------------
    check(a["success"] + a["collision"] + a["timeout"] == 1,
          "statuses mutually exclusive")
    check(0.0 <= a["success"] <= 1 and 0.0 <= a["collision"] <= 1, "rates in [0,1]")

    # --- noise-off leaves the benchmark unchanged -------------------------
    cfg0 = dict(cfg)
    cfg0["sensor"] = {"pos_std": 0.0, "vel_std": 0.0, "dropout": 0.0}
    c = run_episode(PLANNERS["RA-DWA"](cfg), "merge", 3, cfg0)
    check(c["status"] == a["status"] and c["n_cycles"] == a["n_cycles"],
          "zero-noise sensor matches clean run")

    # --- headline performance band (regression guard) ---------------------
    collisions = 0
    successes = 0
    n = 0
    for sc in cfg["scenarios"]:
        for seed in range(1, 11):
            res = run_episode(PLANNERS["RA-DWA"](cfg), sc, seed, cfg)
            collisions += res["collision"]
            successes += res["success"]
            n += 1
    # Documented residual: one roundabout seed collides within seeds 1..10.
    check(collisions <= 1, f"RA-DWA collisions over {n} episodes = {collisions}")
    check(successes >= n - 2, f"RA-DWA success {successes}/{n} below band")

    check(set(PLANNERS) == {"DWA", "DWA-P", "APF", "VO", "MPPI", "TEB", "RA-DWA"},
          "planner registry")

    # --- statistics module ------------------------------------------------
    from radwa.stats import mcnemar, star, wilcoxon
    p, n01, n10 = mcnemar([1, 1, 0, 0], [1, 0, 0, 0])
    check(0.0 <= p <= 1.0 and n01 == 1 and n10 == 0, "mcnemar counts/p")
    check(mcnemar([1, 1, 1], [1, 1, 1])[0] == 1.0, "mcnemar no discordant -> p=1")
    pw, used = wilcoxon([1, 2, 3, 4, 5, 6], [1, 2, 3, 4, 5, 7])
    check(0.0 <= pw <= 1.0 and used == 6, "wilcoxon p/used")
    check(star(1e-4) == "***" and star(0.02) == "*" and star(0.5) == "n.s.",
          "star thresholds")

    # --- configuration fingerprint ---------------------------------------
    fp = config_fingerprint(cfg)
    check(len(fp) == 64 and fp == config_fingerprint(cfg), "fingerprint stable")
    tweaked = dict(cfg)
    tweaked["benchmark"] = dict(cfg["benchmark"], seed0=2)
    check(config_fingerprint(tweaked) != fp, "fingerprint changes with config")

    # --- cross-process determinism (guards the salted-hash bug class) ------
    code = ("import json;from radwa.scenarios import build_scenario;"
            "from radwa.config import load_config;"
            "c=load_config('configs/experiment.json');"
            "s=build_scenario('roundabout',1,c);"
            "print(json.dumps([[round(o.x,9),round(o.y,9),round(o.vx,9)] "
            "for o in s[2]]))")
    outs = []
    for _ in range(2):
        r = subprocess.run([sys.executable, "-c", code], cwd=ROOT,
                           capture_output=True, text=True)
        outs.append(r.stdout.strip())
    check(bool(outs[0]) and outs[0] == outs[1],
          "cross-process scenario determinism")

    print(f"\n{_checks[0]}/{_checks[1]} checks passed "
          f"(RA-DWA 10-seed: {successes}/{n} success, {collisions} collisions)")
    return 0 if _checks[0] == _checks[1] else 1


if __name__ == "__main__":
    raise SystemExit(main())
