"""Scenario suite (Table 2 of the paper).

Every builder is deterministic given ``(name, seed)`` so the whole benchmark is
reproducible.  Six scenarios cover static fields, dynamic crossing, platooning,
oncoming traffic, intersections and a roundabout.
"""

import math
import random
import zlib

from .geometry import clamp
from .obstacles import Obstacle


def _rng(seed, name):
    # zlib.crc32 is process-stable; Python's str hash is salted per run and
    # would break reproducibility across invocations.
    name_key = zlib.crc32(name.encode("utf-8"))
    return random.Random((seed + 1) * 7919 + name_key)


def build_scenario(name, seed, cfg):
    spec = cfg["scenarios"][name]
    rng = _rng(seed, name)
    return _BUILDERS[spec["type"]](spec, rng, cfg)


def _static_field(sc, rng, cfg):
    start, goal = (0.0, 0.0), (40.0, 0.0)
    n = int(sc.get("n_obstacles", 8))
    obs = []
    for i in range(n):
        xi = 7.0 + i * (26.0 / (n - 1)) + rng.uniform(-1.5, 1.5)
        yi = rng.uniform(-2.4, 2.4)
        ri = rng.uniform(0.5, 1.0)
        obs.append(Obstacle(xi, yi, ri, "static"))
    return start, goal, obs


def _crossing(sc, rng, cfg):
    start, goal = (0.0, 0.0), (40.0, 0.0)
    n = int(sc.get("n_cross", 1))
    obs = []
    for i in range(n):
        xc = 15.0 + i * 5.0 + rng.uniform(-1.0, 1.0)
        d0 = rng.uniform(6.0, 9.0)
        speed = clamp(d0 / (xc / 6.0), 1.0, 3.5)
        direction = 1.0 if i % 2 == 0 else -1.0
        obs.append(Obstacle(xc, -direction * d0, 0.9, "linear",
                            vx=0.0, vy=direction * speed))
    return start, goal, obs


def _merge(sc, rng, cfg):
    start, goal = (0.0, 0.0), (50.0, 0.0)
    n = int(sc.get("n_obstacles", 5))
    obs = []
    x = 9.0
    for _ in range(n):
        obs.append(Obstacle(x, rng.uniform(-0.4, 0.4), 0.9, "linear",
                            vx=rng.uniform(2.0, 3.5), vy=0.0))
        x += rng.uniform(2.5, 3.8)
    return start, goal, obs


def _narrow_opposite(sc, rng, cfg):
    start, goal = (0.0, -0.6), (50.0, -0.6)
    n = int(sc.get("n_onside", 2))
    obs = []
    for i in range(n):
        xi = 24.0 + i * rng.uniform(12.0, 16.0)
        obs.append(Obstacle(xi, rng.uniform(-0.6, 0.6), 0.95, "linear",
                            vx=-3.0, vy=0.0))
    return start, goal, obs


def _intersection(sc, rng, cfg):
    start, goal = (0.0, 0.0), (40.0, 0.0)
    d0 = rng.uniform(6.0, 9.0)
    s1 = clamp(d0 / (18.0 / 6.0), 1.0, 3.0)
    d2 = rng.uniform(6.0, 9.0)
    s2 = clamp(d2 / (27.0 / 6.0), 1.0, 3.0)
    obs = [Obstacle(18.0, -d0, 0.9, "linear", vx=0.0, vy=s1),
           Obstacle(27.0, d2, 0.9, "linear", vx=0.0, vy=-s2)]
    return start, goal, obs


def _roundabout(sc, rng, cfg):
    start, goal = (0.0, 0.0), (48.0, 0.0)
    cx, cy, ring = 25.0, 0.0, 9.0
    obs = [Obstacle(cx, cy, 3.0, "static")]
    n = int(sc.get("n_vehicles", 4))
    omega = float(sc.get("omega", 0.30))
    for _ in range(n):
        obs.append(Obstacle(cx + ring, cy, 0.8, "circular",
                            cx=cx, cy=cy, radius=ring, omega=omega,
                            phase=rng.uniform(0.0, 2.0 * math.pi)))
    return start, goal, obs


_BUILDERS = {
    "static_field": _static_field,
    "crossing": _crossing,
    "merge": _merge,
    "narrow_opposite": _narrow_opposite,
    "intersection": _intersection,
    "roundabout": _roundabout,
}
