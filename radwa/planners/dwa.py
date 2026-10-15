"""Baseline: classic Dynamic Window Approach with fixed cost weights."""

import copy
import math

from ..geometry import angle_diff, clamp, dist, linspace
from ..state import Control
from .base import Planner
from .common import (DEFAULT_SENSOR_RANGE, obstacle_penalty, roll_trajectory)


def dynamic_window(ego, ego_cfg, dt):
    v_lo = max(ego_cfg["v_min"], ego.v + ego_cfg["a_dec"] * dt)
    v_hi = min(ego_cfg["v_max"], ego.v + ego_cfg["a_acc"] * dt)
    w_lo = max(-ego_cfg["omega_max"], ego.omega - ego_cfg["alpha"] * dt)
    w_hi = min(ego_cfg["omega_max"], ego.omega + ego_cfg["alpha"] * dt)
    return v_lo, v_hi, w_lo, w_hi


class DWA(Planner):
    name = "DWA"

    def __init__(self, cfg):
        super().__init__(cfg)
        self.p = cfg["dwa"]
        self.ego_cfg = cfg["ego"]
        self.dt = cfg["sim"]["dt"]
        self.r_ego = cfg["ego"]["r"]
        self.motion_pred = self.p.get("motion_prediction", False)

    def plan(self, ego, obstacles, t):
        gx, gy = self.goal
        v_lo, v_hi, w_lo, w_hi = dynamic_window(ego, self.ego_cfg, self.dt)
        v_lo = max(v_lo, 0.0)
        vs = linspace(v_lo, v_hi, self.p["n_v"])
        ws = linspace(w_lo, w_hi, self.p["n_w"])
        W = self.p["weights"]
        r_pen = self.p["obstacle_radius"]
        horizon = self.p["horizon"]

        bearing = math.atan2(gy - ego.y, gx - ego.x)
        best, best_cost = Control(0.0, 0.0), float("inf")
        for v in vs:
            for w in ws:
                traj = roll_trajectory(ego.x, ego.y, ego.theta, v, w, horizon, self.dt)
                _, ex, ey, eth, _, _ = traj[-1]
                c_prog = dist(ex, ey, gx, gy)
                c_head = abs(angle_diff(eth, bearing))
                c_obs = obstacle_penalty(traj, obstacles, r_pen, self.r_ego,
                                         moving=self.motion_pred)
                c_smooth = abs(v - ego.v) + abs(w - ego.omega)
                c_vel = self.ego_cfg["v_max"] - v
                cost = (W["progress"] * c_prog + W["heading"] * c_head
                        + W["obstacle"] * c_obs + W["smooth"] * c_smooth
                        + W["velocity"] * c_vel)
                if cost < best_cost:
                    best_cost = cost
                    best = Control(v, w, state="dwa")
        return best


class DWAP(DWA):
    """DWA with motion-aware obstacle prediction (control baseline).

    Isolates the effect of the prediction step from the decision rule: it is the
    classical fixed-weight DWA cost with obstacles propagated along their
    measured velocity, matching RA-DWA's prediction but not its risk field.
    """

    name = "DWA-P"

    def __init__(self, cfg):
        c = copy.deepcopy(cfg)
        c["dwa"] = dict(c["dwa"], motion_prediction=True)
        super().__init__(c)
