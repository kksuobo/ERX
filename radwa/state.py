"""Lightweight state containers."""

from dataclasses import dataclass


@dataclass
class EgoState:
    x: float
    y: float
    theta: float
    v: float
    omega: float = 0.0


@dataclass
class Control:
    v: float
    omega: float
    state: str = ""
    risk: float = 0.0
