"""Normalized time-to-collision (TTC) risk field.

This is the single quantity that RA-DWA couples across the lateral and
longitudinal decision.  ``R = 1 / (1 + TTC / T0)`` is bounded in (0, 1]:
R -> 1 as the predicted impact time goes to zero, R -> 0 when no impact is
predicted on the current relative-velocity ray.
"""

import math

INF = float("inf")


def time_to_collision(px, py, vx, vy, ox, oy, ovx, ovy, d_safe):
    """Earliest time at which two point agents with constant velocities come
    within ``d_safe`` metres of each other.  Returns ``INF`` if they never do.
    """
    dpx = ox - px
    dpy = oy - py
    dvx = ovx - vx
    dvy = ovy - vy

    c = dpx * dpx + dpy * dpy - d_safe * d_safe
    if c <= 0.0:
        return 0.0

    a = dvx * dvx + dvy * dvy
    if a < 1e-12:
        return INF

    b = 2.0 * (dpx * dvx + dpy * dvy)
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return INF

    sq = math.sqrt(disc)
    t1 = (-b - sq) / (2.0 * a)
    t2 = (-b + sq) / (2.0 * a)
    if t1 >= 0.0:
        return t1
    if t2 >= 0.0:
        return t2
    return INF


_INF_CUTOFF = 1e9


def risk_from_clearance(clear, d_risk):
    """Distance-based alternative to the TTC risk (ablation baseline)."""
    if clear >= d_risk:
        return 0.0
    if clear <= 0.0:
        return 1.0
    return 1.0 - clear / d_risk


def risk_from_ttc(ttc, T0, cutoff=float("inf")):
    """Map a TTC value to a bounded risk in [0, 1].

    Uses the exponential form ``R = exp(-TTC / T0)``, which (unlike the
    hyperbolic ``1 / (1 + TTC/T0)``) decays quickly.  ``cutoff`` truncates the
    residual tail to exactly zero beyond ``cutoff`` seconds: the hyperbolic tail
    kept taxing safe forward motion and, once progress is down-weighted in risky
    states, made stopping locally optimal (a deadlock).
    """
    if ttc >= _INF_CUTOFF or ttc > cutoff:
        return 0.0
    return math.exp(-ttc / T0)
