"""Obstacle agents with behaviour used by the scenario suite."""

import math


class Obstacle:
    """A circular obstacle.

    behaviour:
        ``static``   -- never moves.
        ``linear``   -- constant velocity (vx, vy).
        ``circular`` -- moves on a circle of ``radius`` around (cx, cy) at
                        angular rate ``omega`` starting from ``phase``.
        ``trace``    -- replays a recorded trajectory (used to ingest
                        SUMO-generated traffic); ``trace`` is a list of
                        (x, y, vx, vy) sampled every ``trace_dt`` seconds.
    """

    def __init__(self, x, y, r, behavior="static", vx=0.0, vy=0.0, **kw):
        self.x = float(x)
        self.y = float(y)
        self.r = float(r)
        self.behavior = behavior
        self.vx = float(vx)
        self.vy = float(vy)
        self.cx = kw.get("cx")
        self.cy = kw.get("cy")
        self.radius = kw.get("radius")
        self.omega = float(kw.get("omega", 0.0))
        self.phase = float(kw.get("phase", 0.0))
        self.trace = kw.get("trace")
        self.trace_dt = float(kw.get("trace_dt", 0.1))
        if self.behavior == "circular" and self.cx is not None:
            # Place the agent at its phase position now, instead of leaving every
            # ring agent stacked at the seed point until the first step.
            ang = self.phase
            self.x = self.cx + self.radius * math.cos(ang)
            self.y = self.cy + self.radius * math.sin(ang)
            self.vx = -self.radius * self.omega * math.sin(ang)
            self.vy = self.radius * self.omega * math.cos(ang)

    def step(self, dt, t):
        if self.behavior == "linear":
            self.x += self.vx * dt
            self.y += self.vy * dt
        elif self.behavior == "circular":
            ang = self.phase + self.omega * (t + dt)
            self.x = self.cx + self.radius * math.cos(ang)
            self.y = self.cy + self.radius * math.sin(ang)
            self.vx = -self.radius * self.omega * math.sin(ang)
            self.vy = self.radius * self.omega * math.cos(ang)
        elif self.behavior == "trace" and self.trace:
            i = int(round(t / self.trace_dt))
            i = max(0, min(len(self.trace) - 1, i))
            self.x, self.y, self.vx, self.vy = self.trace[i]

    def active(self, limit=120.0):
        return abs(self.x) < limit and abs(self.y) < limit
