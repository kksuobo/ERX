"""Baseline: Artificial Potential Field local planner."""

import math

from ..geometry import angle_diff, clamp
from ..state import Control
from .base import Planner


class APF(Planner):
    name = "APF"

    def __init__(self, cfg):
        super().__init__(cfg)
        self.p = cfg["apf"]
        self.ego_cfg = cfg["ego"]
        self.r_ego = cfg["ego"]["r"]

    def plan(self, ego, obstacles, t):
        p = self.p
        gx, gy = self.goal

        fx, fy = gx - ego.x, gy - ego.y
        n = math.hypot(fx, fy)
        if n > 1e-9:
            fx, fy = p["k_att"] * fx / n, p["k_att"] * fy / n

        for o in obstacles:
            dx, dy = ego.x - o.x, ego.y - o.y
            d = math.hypot(dx, dy)
            if 1e-6 < d < p["d0"]:
                mag = p["k_rep"] * (1.0 / d - 1.0 / p["d0"]) / (d * d)
                fx += mag * dx / d
                fy += mag * dy / d

        desired = math.atan2(fy, fx)
        err = angle_diff(desired, ego.theta)
        omega = clamp(p["k_omega"] * err, -self.ego_cfg["omega_max"],
                      self.ego_cfg["omega_max"])

        near = min((math.hypot(ego.x - o.x, ego.y - o.y) - o.r - self.r_ego
                    for o in obstacles), default=float("inf"))
        if near < p["slow_radius"]:
            v = p["v_ref"] * max(0.0, near / p["slow_radius"])
        else:
            v = p["v_ref"]
        v *= max(0.3, 1.0 - abs(err) / math.pi)
        v = clamp(v, self.ego_cfg["v_min"], self.ego_cfg["v_max"])
        return Control(v, omega, state="apf")
