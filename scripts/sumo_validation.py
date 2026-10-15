"""External validation: run the planners against SUMO-generated traffic.

SUMO drives the surrounding vehicles (car-following and lane-changing) and the
recorded trajectories are replayed as moving obstacles for our plant and
planners. The ego's local planner and plant are unchanged, which isolates the
local-planning comparison; the traffic is open-loop with respect to the ego, the
standard way to hold the environment fixed across planners.

Requires SUMO (sumo.exe, netconvert.exe) on PATH.
"""

import argparse
import _bootstrap  # noqa: F401
import copy
import math
import os
import random
import subprocess
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from radwa.config import load_config
from radwa.obstacles import Obstacle
from radwa.planners import PLANNERS
from radwa.simulator import run_episode_custom
from radwa.stats import mcnemar, star, wilson

ROOT = _bootstrap.ROOT
SUMO_DIR = os.path.join(ROOT, "sumo")
NET = os.path.join(SUMO_DIR, "net.net.xml")
R_OBST = 1.5          # SUMO passenger car approximated as a disc
COLORS = {"DWA": "#1f77b4", "DWA-P": "#17becf", "VO": "#2ca02c",
          "MPPI": "#9467bd", "RA-DWA": "#d62728"}


def build_routes(seed, path):
    rng = random.Random(seed)
    j = lambda a, b: rng.uniform(a, b)
    veh = []
    # (id, type, depart, lane, pos, speed, route)
    spec = [
        ("lead1", "slow", 0.0, 1, 60 + j(-8, 8), 3.5 + j(-0.5, 0.5), "E0a E0b"),
        ("lead2", "slow", 0.0, 0, 90 + j(-8, 8), 3.0 + j(-0.5, 0.5), "E0a E0b"),
        ("lead0", "slow", 0.0, 1, 120 + j(-8, 8), 3.0 + j(-0.5, 0.5), "E0a E0b"),
        ("mrg0", "car", 1.0, 0, 5, 6.0 + j(-1, 1), "Em E0b"),
        ("onc0", "car", 2.0, 0, 10, 10.0 + j(-1, 1), "E1a E1b"),
        ("mrg1", "car", 6.0, 0, 5, 6.0 + j(-1, 1), "Em E0b"),
        ("onc1", "car", 8.0, 1, 10, 10.0 + j(-1, 1), "E1a E1b"),
    ]
    # ego is present only to mark the corridor; excluded from the obstacle set,
    # and emitted first so the file is departure-sorted.
    veh.append('  <vehicle id="ego" type="car" depart="0" departLane="1" '
               'departPos="10" departSpeed="0.0"><route edges="E0a E0b"/></vehicle>')
    for vid, typ, dep, lane, pos, spd, route in sorted(spec, key=lambda s: s[2]):
        veh.append(
            f'  <vehicle id="{vid}" type="{typ}" depart="{dep:.2f}" '
            f'departLane="{lane}" departPos="{max(1.0, pos):.1f}" '
            f'departSpeed="{max(0.5, spd):.2f}"><route edges="{route}"/></vehicle>')
    txt = ("<routes>\n"
           '  <vType id="car" length="5" accel="2.6" decel="4.5" sigma="0.5" '
           'maxSpeed="13.9"/>\n'
           '  <vType id="slow" length="5" accel="2.0" decel="4.5" sigma="0.5" '
           'maxSpeed="4.0"/>\n' + "\n".join(veh) + "\n</routes>\n")
    with open(path, "w", encoding="utf-8") as f:
        f.write(txt)


def run_sumo(seed):
    routes = os.path.join(SUMO_DIR, f"routes_{seed}.rou.xml")
    fcd = os.path.join(SUMO_DIR, f"fcd_{seed}.xml")
    build_routes(seed, routes)
    # With --step-length 0.1 the FCD trace is written every 0.1 s.
    cmd = ["sumo", "-n", NET, "-r", routes, "--begin", "0", "--end", "60",
           "--step-length", "0.1", "--fcd-output", fcd,
           "--no-step-log", "--collision.action", "none", "--seed", str(seed),
           "--error-log", os.path.join(SUMO_DIR, "sumo.err")]
    subprocess.run(cmd, check=True, cwd=SUMO_DIR, capture_output=True)
    return fcd


def parse_traces(fcd):
    """Return {vehicle id: {'y0':..., 'trace': [(x,y,vx,vy), ...]}} at 0.1 s."""
    tree = ET.parse(fcd)
    traces, times = {}, []
    for step in tree.getroot().iter("timestep"):
        times.append(float(step.get("time")))
        for v in step.iter("vehicle"):
            a = math.radians(float(v.get("angle")))
            spd = float(v.get("speed"))
            traces.setdefault(v.get("id"), []).append(
                (float(v.get("x")), float(v.get("y")),
                 spd * math.sin(a), spd * math.cos(a)))
    return traces, len(times)


def make_obstacles(traces, skip="ego"):
    obs = []
    for vid, tr in traces.items():
        if vid == skip or len(tr) < 5:
            continue
        x, y, _, _ = tr[0]
        obs.append(Obstacle(x, y, R_OBST, "trace", trace=tr, trace_dt=0.1))
    return obs


def make_planner(name, cfg):
    """DWA-P is DWA with motion-aware obstacle prediction (control baseline)."""
    if name == "DWA-P":
        c = copy.deepcopy(cfg)
        c["dwa"] = dict(c["dwa"], motion_prediction=True)
        return PLANNERS["DWA"](c)
    return PLANNERS[name](cfg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    # DWA-P is DWA with motion-aware obstacle prediction, the control that
    # separates prediction from the decision rule in this study.
    ap.add_argument("--planners", default="DWA,DWA-P,VO,RA-DWA,MPPI")
    args = ap.parse_args()

    cfg = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    cfg["sim"] = dict(cfg["sim"], max_time=60.0, goal_tol=2.0)
    planners = args.planners.split(",")
    seed0 = cfg["benchmark"]["seed0"]

    results = {p: [] for p in planners}
    sample = None
    for k in range(args.seeds):
        seed = seed0 + k
        fcd = run_sumo(seed)
        traces, _ = parse_traces(fcd)
        y0 = traces["ego"][0][1] if "ego" in traces else 0.0
        start, goal = (10.0, y0), (295.0, y0)
        for p in planners:
            obs = make_obstacles(traces)
            rec = p in ("DWA", "VO", "RA-DWA") and k == 0
            res = run_episode_custom(make_planner(p, cfg), start, goal, obs, cfg,
                                     seed=seed, record=rec)
            res.update(planner=p, seed=seed)
            results[p].append(res)
            if rec:
                sample = (p, res["_log"], obs)
        print(f"seed {seed}: " + "  ".join(
            f"{p}={results[p][-1]['status']}" for p in planners))

    # summarise
    lines = ["# External validation on SUMO-generated traffic\n",
             f"SUMO 1.27 [22] corridor (3-lane main road with oncoming and "
             f"merging traffic), {args.seeds} seeds. The ego plant and planners "
             f"are unchanged; SUMO traffic is replayed as moving obstacles "
             f"(open-loop with respect to the ego). Goal tolerance 2.0 m and a "
             f"60 s episode limit (larger than the main benchmark, the corridor "
             f"being longer); obstacles are modelled as 1.5 m discs. DWA-P is "
             f"DWA with motion-aware obstacle prediction, the control that "
             f"separates prediction from the decision rule.\n",
             "| Method | n | Success (95% CI) | Collision (95% CI) | Min clear. | "
             "Min TTC | Mean \\|jerk\\| | succ p vs RA-DWA |",
             "|---|---|---|---|---|---|---|---|"]
    summary = {}
    ref = results.get("RA-DWA")
    for p in planners:
        rs = results[p]
        n = len(rs)
        ks = sum(r["success"] for r in rs)
        kc = sum(r["collision"] for r in rs)
        sl, sh = wilson(ks, n)
        cl, ch = wilson(kc, n)
        mclear = np.mean([r["min_clearance"] for r in rs])
        tt = [r["min_ttc"] for r in rs if r["min_ttc"] == r["min_ttc"]]
        mttc = np.mean(tt) if tt else float("nan")
        jerk = np.mean([r["mean_abs_jerk"] for r in rs])
        summary[p] = (ks / n, kc / n)
        if ref and p != "RA-DWA":
            pv, _, _ = mcnemar([r["success"] for r in ref],
                               [r["success"] for r in rs])
            pcell = f"{pv:.2e} {star(pv)}"
        else:
            pcell = "ref"
        lines.append(f"| {p} | {n} | {ks/n:.2f} [{sl:.2f}, {sh:.2f}] | "
                     f"{kc/n:.2f} [{cl:.2f}, {ch:.2f}] | {mclear:.2f} | "
                     f"{mttc:.2f} | {jerk:.2f} | {pcell} |")
    out = os.path.join(ROOT, "results", "sumo_validation.md")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n" + "\n".join(lines))

    # figure: bars + one sample trajectory
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))
    names = [p for p in planners]
    axes[0].bar([f"{p}\nsucc" for p in names], [summary[p][0] for p in names],
                color=[COLORS.get(p, "#888") for p in names])
    axes[0].bar([f"{p}\ncoll" for p in names], [summary[p][1] for p in names],
                color=[COLORS.get(p, "#888") for p in names], alpha=0.45)
    axes[0].set_ylim(0, 1.0), axes[0].grid(alpha=0.3, axis="y")
    axes[0].set_title("SUMO validation: success (solid) / collision (faded)")
    if sample:
        p, log, obs = sample
        for o in obs:
            xs = [s[0] for s in o.trace]
            ys = [s[1] for s in o.trace]
            axes[1].plot(xs, ys, color="0.75", lw=0.6)
        axes[1].plot(log["x"], log["y"], color=COLORS.get(p, "#d62728"), lw=2,
                     label=f"{p} (sample)")
    axes[1].set_xlabel("x [m]"), axes[1].set_ylabel("y [m]")
    axes[1].set_title("Sample trajectory on SUMO traffic"), axes[1].grid(alpha=0.3)
    axes[1].legend(fontsize=8)
    axes[1].set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    figout = os.path.join(ROOT, "figures", "fig13_sumo.png")
    fig.savefig(figout, dpi=170)
    plt.close(fig)
    print("wrote", out, "and", figout)


if __name__ == "__main__":
    main()
