"""Planner interface."""

from abc import ABC, abstractmethod


class Planner(ABC):
    name = "planner"

    def __init__(self, cfg):
        self.cfg = cfg
        self.goal = (0.0, 0.0)

    def reset(self, goal, rng=None):
        self.goal = goal
        self.rng = rng

    @abstractmethod
    def plan(self, ego, obstacles, t):
        """Return a :class:`radwa.state.Control` for the current cycle."""
        raise NotImplementedError
