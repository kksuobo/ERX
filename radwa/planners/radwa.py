"""RA-DWA: Risk-Adaptive Dynamic Window Approach.

One normalized TTC risk field drives two decisions at once:
  * it is integrated into the dynamic-window cost (lateral avoidance), and
  * it reshapes the admissible speed set through the dynamic window
    (longitudinal modulation),
so path and speed are chosen in a single arg-min over (v, omega).

The cost weights are selected by a four-state scheduler
(free / follow / conflict / emergency) with hysteresis to suppress chattering.
"""

import math

from ..geometry import angle_diff, clamp, dist, linspace
from ..risk import INF, risk_from_clearance, risk_from_ttc
from ..state import Control
from .base import Planner
from .common import (min_ttc_at, roll_trajectory, trajectory_min_clearance,
                     trajectory_risk, trajectory_risk_distance)
from .dwa import dynamic_window

_STATE_RANK = {"free": 0, "follow": 1, "conflict": 2, "emergency": 3}


class RADWA(Planner):
    name = "RA-DWA"

    def __init__(self, cfg):
        super().__init__(cfg)
        self.p = cfg["radwa"]
        self.ego_cfg = cfg["ego"]
        self.dt = cfg["sim"]["dt"]
        self.r_ego = cfg["ego"]["r"]
        self.state = "free"
        self.rho_f = 0.0
        self.collision_penalty = self.p.get("collision_penalty", 100.0)
        self.debug = False
        self._last_candidates = None
        self.v_cmd_prev = 0.0
        self.w_cmd_prev = 0.0
        self.a_cmd_prev = 0.0
        # Ablation switches.
        self.use_scheduler = cfg["radwa"].get("use_scheduler", True)
        self.use_hysteresis = cfg["radwa"].get("hysteresis_enabled", True)
        self.use_speed_mod = cfg["radwa"].get("speed_modulation", True)
        self.risk_mode = cfg["radwa"].get("risk_mode", "ttc")
        self.motion_pred = cfg["radwa"].get("motion_prediction", True)
        self.turn_pred = cfg["radwa"].get("turn_rate_prediction", False)
        self.weight_ema = cfg["radwa"].get("weight_ema", 0.25)
        self.risk_scale = cfg["radwa"].get("risk_scale", 1.0)
        self.W_f = dict(cfg["radwa"]["weights"]["free"])
        self._prev_vel = []
        self._omega_f = []

    def reset(self, goal, rng=None):
        super().reset(goal, rng)
        self.state = "free"
        self.rho_f = 0.0
        self.v_cmd_prev = 0.0
        self.w_cmd_prev = 0.0
        self.a_cmd_prev = 0.0
        self.W_f = dict(self.p["weights"]["free"])
        self._prev_vel = []
        self._omega_f = []

    def _turn_rates(self, obstacles):
        """Per-obstacle turn-rate estimate from consecutive velocity headings.

        The raw estimate is EMA-smoothed and gated: with perception noise the
        per-cycle heading difference is dominated by measurement jitter, so an
        unsmoothed estimate would inject spurious curvature. A gate keeps the
        prediction at constant velocity unless a genuine turn is sustained.
        """
        if not self.turn_pred:
            return None
        if len(self._prev_vel) != len(obstacles):
            self._prev_vel = [(o.vx, o.vy) for o in obstacles]
            self._omega_f = [0.0] * len(obstacles)
            return None
        a = self.p.get("turn_rate_ema", 0.4)
        raw = []
        for o, (pvx, pvy) in zip(obstacles, self._prev_vel):
            if math.hypot(o.vx, o.vy) < 1e-6 or math.hypot(pvx, pvy) < 1e-6:
                raw.append(0.0)
            else:
                raw.append(angle_diff(math.atan2(o.vy, o.vx),
                                      math.atan2(pvy, pvx)) / self.dt)
        self._omega_f = [(1.0 - a) * p + a * r
                         for p, r in zip(self._omega_f, raw)]
        self._prev_vel = [(o.vx, o.vy) for o in obstacles]
        gate = self.p.get("turn_rate_gate", 0.08)
        return [w if abs(w) > gate else 0.0 for w in self._omega_f]

    # ------------------------------------------------------------------ risk
    def _current_risk(self, ego, obstacles):
        if self.risk_mode == "distance":
            m = min((dist(ego.x, ego.y, o.x, o.y) - o.r - self.r_ego
                     for o in obstacles), default=INF)
            return risk_from_clearance(m, self.p.get("d_risk", 4.0))
        vx = ego.v * math.cos(ego.theta)
        vy = ego.v * math.sin(ego.theta)
        ttc = min_ttc_at(ego.x, ego.y, vx, vy, obstacles,
                         self.p["d_safe_margin"], self.r_ego,
                         self.p["sensor_range"])
        return risk_from_ttc(ttc, self.p["T0"], self.p.get("risk_cutoff", INF))

    def _is_follow(self, ego, obstacles):
        f = self.p["follow"]
        ct, st = math.cos(ego.theta), math.sin(ego.theta)
        for o in obstacles:
            dx, dy = o.x - ego.x, o.y - ego.y
            if math.hypot(dx, dy) > f["dist"] + o.r:
                continue
            if dx * ct + dy * st <= 0.0:          # not ahead
                continue
            osp = math.hypot(o.vx, o.vy)
            if osp < 1e-6:
                continue
            cosang = (o.vx * ct + o.vy * st) / osp
            if cosang < math.cos(f["heading_tol"]):
                continue
            if osp <= f["speed_ratio"] * ego.v:
                return True
        return False

    def _transition(self, prev, rho, ego, obstacles):
        if not self.use_scheduler:
            return "free"
        th = self.p["thresholds"]
        t1, t2, t3, h = (th["theta1"], th["theta2"], th["theta3"],
                         th["hysteresis"])
        follow = self._is_follow(ego, obstacles)
        if rho >= t3:
            cand = "emergency"
        elif rho >= t2:
            cand = "conflict"
        elif follow:
            cand = "follow"
        else:
            cand = "free"

        if not self.use_hysteresis:
            return cand
        if _STATE_RANK[cand] >= _STATE_RANK[prev]:
            return cand
        # Downgrade only once the risk has dropped below the lowered threshold.
        if prev == "emergency" and rho < t3 - h:
            return cand
        if prev == "conflict" and rho < t2 - h:
            return cand
        if prev == "follow" and rho < t1 - h and not follow:
            return cand
        return prev

    # ------------------------------------------------------------------ plan
    def plan(self, ego, obstacles, t):
        gx, gy = self.goal
        omegas = self._turn_rates(obstacles)
        rho = self._current_risk(ego, obstacles)
        beta = self.p.get("risk_ema", 0.3)
        self.rho_f = (1.0 - beta) * self.rho_f + beta * rho
        rho = self.rho_f
        self.state = self._transition(self.state, rho, ego, obstacles)
        target = self.p["weights"][self.state]
        g = self.weight_ema
        self.W_f = {k: (1.0 - g) * self.W_f[k] + g * target[k] for k in target}
        W = self.W_f

        # Risk reshapes the admissible speed set only once the scheduler has
        # entered a risky context; capping speed in benign states only adds
        # chattering without a safety benefit.
        v_max_eff = self.ego_cfg["v_max"]
        if self.use_speed_mod and self.state in ("conflict", "emergency"):
            gain = self.p["speed_risk_gain"]
            v_max_eff = clamp(self.ego_cfg["v_max"] * (1.0 - gain * rho),
                              0.5, self.ego_cfg["v_max"])

        v_lo, v_hi, w_lo, w_hi = dynamic_window(ego, self.ego_cfg, self.dt)
        v_hi = min(v_hi, v_max_eff)
        v_lo = min(max(v_lo, 0.0), v_hi)
        vs = linspace(v_lo, v_hi, self.p["n_v"])
        ws = linspace(w_lo, w_hi, self.p["n_w"])

        bearing = math.atan2(gy - ego.y, gx - ego.x)
        clear_target = self.p["clear_target"]
        safety_margin = self.p.get("safety_margin", 0.0)
        best, best_cost = Control(0.0, 0.0), float("inf")
        cands = [] if self.debug else None
        for v in vs:
            for w in ws:
                traj = roll_trajectory(ego.x, ego.y, ego.theta, v, w,
                                       self.p["horizon"], self.dt)
                _, ex, ey, eth, _, _ = traj[-1]
                c_prog = dist(ex, ey, gx, gy)
                c_head = abs(angle_diff(eth, bearing))
                if self.risk_mode == "distance":
                    c_risk = trajectory_risk_distance(
                        traj, obstacles, self.p.get("d_risk", 4.0), self.r_ego,
                        omegas=omegas)
                else:
                    c_risk = trajectory_risk(traj, obstacles, self.p["T0"],
                                             self.p["d_safe_margin"], self.r_ego,
                                             self.p["sensor_range"],
                                             self.p.get("risk_cutoff", INF),
                                             omegas=omegas)
                min_clear = trajectory_min_clearance(traj, obstacles, self.r_ego,
                                                     moving=self.motion_pred,
                                                     omegas=omegas)
                c_clear = max(0.0, clear_target - min_clear)
                dv = v - self.v_cmd_prev
                c_smooth = abs(dv) + abs(w - self.w_cmd_prev)
                # Second difference of the longitudinal command ~ jerk; this
                # directly damps the acceleration-change spikes that dominate
                # the comfort metric.
                c_jerk = abs(dv - self.a_cmd_prev)
                # Predicted collisions are inadmissible; keep a small buffer so
                # the planner never trades the last few centimetres for speed.
                c_viol = self.collision_penalty * max(0.0, safety_margin - min_clear)
                cost = (W["progress"] * c_prog + W["heading"] * c_head
                        + W["risk"] * self.risk_scale * c_risk + W["clear"] * c_clear
                        + W["smooth"] * c_smooth + W.get("jerk", 0.0) * c_jerk
                        + c_viol)
                if cands is not None:
                    cands.append((cost, v, w, c_prog, c_head, c_risk, c_clear,
                                  c_viol, min_clear))
                if cost < best_cost:
                    best_cost = cost
                    best = Control(v, w, state=self.state, risk=rho)
        self._last_candidates = cands
        self.a_cmd_prev = best.v - self.v_cmd_prev
        self.v_cmd_prev = best.v
        self.w_cmd_prev = best.omega
        return best
