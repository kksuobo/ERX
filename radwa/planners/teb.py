"""Baseline: Timed Elastic Band (TEB) local planner.

A receding-horizon formulation of TEB: a band of poses plus time intervals is
optimised with L-BFGS-B against obstacle clearance, kinematic feasibility and
elapsed time, warm-independent (re-seeded to a straight band each cycle). The
first (v, omega) of the optimised band drives the plant. Obstacle positions are
propagated with their measured velocity during the band, matching the
motion-aware assumption used by RA-DWA.
"""

import math

import numpy as np
from scipy.optimize import minimize

from ..geometry import angle_diff, clamp
from ..state import Control
from .base import Planner


class TEB(Planner):
    name = "TEB"

    def __init__(self, cfg):
        super().__init__(cfg)
        self.p = cfg["teb"]
        self.ego_cfg = cfg["ego"]
        self.r_ego = cfg["ego"]["r"]
        self.n = int(self.p["n_poses"])

    def reset(self, goal, rng=None):
        super().reset(goal, rng)

    # ------------------------------------------------------------- band helpers
    def _init_z(self, ego, goal, lateral=0.0):
        n = self.n
        bearing = math.atan2(goal[1] - ego.y, goal[0] - ego.x)
        px, py = math.cos(bearing + math.pi / 2), math.sin(bearing + math.pi / 2)
        dt, v = self.p["dt_init"], self.p["v_init"]
        z = []
        for i in range(1, n):
            s = i / (n - 1)
            off = lateral * 4.0 * s * (1.0 - s)   # parabola peaking mid-band
            d = v * dt * i
            z += [ego.x + d * math.cos(bearing) + off * px,
                  ego.y + d * math.sin(bearing) + off * py, bearing]
        z += [dt] * (n - 1)
        return np.asarray(z, dtype=float)

    def _unpack(self, ego, z):
        n = self.n
        xs = [ego.x]
        ys = [ego.y]
        ths = [ego.theta]
        for i in range(1, n):
            xs.append(z[3 * (i - 1)])
            ys.append(z[3 * (i - 1) + 1])
            ths.append(z[3 * (i - 1) + 2])
        base = 3 * (n - 1)
        dts = [z[base + i] for i in range(n - 1)]
        return xs, ys, ths, dts

    def _cost(self, z, ego, obstacles):
        p = self.p
        n = self.n
        r_ego = self.r_ego
        xs, ys, ths, dts = self._unpack(ego, z)
        w_obs, margin = p["w_obstacle"], p["obstacle_margin"]
        w_kin, w_sm, w_t = p["w_kinematics"], p["w_smooth"], p["w_time"]
        vmax, vmin, omax = (self.ego_cfg["v_max"], self.ego_cfg["v_min"],
                            self.ego_cfg["omega_max"])

        J = 0.0
        times = np.cumsum([0.0] + dts)
        # Obstacle clearance at each band pose (motion-aware).
        for i in range(1, n):
            ti = times[i]
            for o in obstacles:
                ox = o.x + o.vx * ti
                oy = o.y + o.vy * ti
                d = math.hypot(xs[i] - ox, ys[i] - oy) - o.r - r_ego
                if d < margin:
                    J += w_obs * (margin - d) ** 2

        prev_v = prev_w = None
        for i in range(n - 1):
            dx, dy = xs[i + 1] - xs[i], ys[i + 1] - ys[i]
            ds = math.hypot(dx, dy)
            dt = max(dts[i], 1e-2)
            v = ds / dt
            hd = math.atan2(dy, dx) if ds > 1e-6 else ths[i]
            J += w_kin * (angle_diff(ths[i], hd) ** 2
                          + angle_diff(ths[i + 1], hd) ** 2)
            if v > vmax:
                J += w_kin * (v - vmax) ** 2
            elif v < vmin:
                J += w_kin * (vmin - v) ** 2
            w = angle_diff(ths[i + 1], ths[i]) / dt
            if abs(w) > omax:
                J += w_kin * (abs(w) - omax) ** 2
            if prev_v is not None:
                J += w_sm * ((v - prev_v) ** 2 + (w - prev_w) ** 2)
            prev_v, prev_w = v, w

        J += w_t * float(np.sum(dts))
        J += p["w_goal"] * math.hypot(xs[-1] - self.goal[0], ys[-1] - self.goal[1])
        return J

    # ------------------------------------------------------------------- plan
    def plan(self, ego, obstacles, t):
        n = self.n
        bounds = []
        for _ in range(n - 1):
            bounds += [(None, None), (None, None), (None, None)]
        bounds += [(self.p["dt_min"], self.p["dt_max"])] * (n - 1)
        # Multi-start over lateral biases breaks the head-on symmetry that traps
        # a straight-band initialisation in a spurious local minimum.
        best_z, best_c = None, float("inf")
        for lateral in (0.0, self.p["restart_lateral"],
                        -self.p["restart_lateral"]):
            z0 = self._init_z(ego, self.goal, lateral)
            try:
                res = minimize(self._cost, z0, args=(ego, obstacles),
                               method="L-BFGS-B", bounds=bounds,
                               options={"maxiter": self.p["maxiter"],
                                        "ftol": 1e-8})
                z, c = res.x, float(res.fun)
            except Exception:
                z, c = z0, self._cost(z0, ego, obstacles)
            if c < best_c:
                best_c, best_z = c, z
        z = best_z
        xs, ys, ths, dts = self._unpack(ego, z)
        dt0 = max(dts[0], 1e-2)
        v = math.hypot(xs[1] - xs[0], ys[1] - ys[0]) / dt0
        w = angle_diff(ths[1], ths[0]) / dt0
        # Respect the actuator envelope (same as the other planners).
        dt = self.cfg["sim"]["dt"]
        v = clamp(v, self.ego_cfg["v_min"], self.ego_cfg["v_max"])
        v = clamp(v, ego.v + self.ego_cfg["a_dec"] * dt,
                  ego.v + self.ego_cfg["a_acc"] * dt)
        w = clamp(w, -self.ego_cfg["omega_max"], self.ego_cfg["omega_max"])
        w = clamp(w, ego.omega - self.ego_cfg["alpha"] * dt,
                  ego.omega + self.ego_cfg["alpha"] * dt)
        return Control(v, w, state="teb")
