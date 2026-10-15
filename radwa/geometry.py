"""Small 2-D geometry / kinematics helpers shared across the package."""

import math

TWO_PI = 2.0 * math.pi


def wrap_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % TWO_PI - math.pi


def angle_diff(a, b):
    """Signed smallest difference a - b wrapped to (-pi, pi]."""
    return wrap_angle(a - b)


def dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def linspace(a, b, n):
    if n <= 1:
        return [0.5 * (a + b)]
    step = (b - a) / (n - 1)
    return [a + step * i for i in range(n)]


def unicycle_step(x, y, theta, v, omega, dt):
    """Advance a unicycle/bicycle-like state by one control period.

    Heading is integrated first, then position, so the sampled roll-out in the
    planners is bit-identical to the simulator update.
    """
    theta2 = theta + omega * dt
    x2 = x + v * math.cos(theta2) * dt
    y2 = y + v * math.sin(theta2) * dt
    return x2, y2, theta2
