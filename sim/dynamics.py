"""Dinamika robot omnidirectional dengan batas kecepatan dan percepatan."""
from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import wrap_angle
from .types import Pose, Velocity


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class RobotLimits:
    max_speed: float  # m/s, norma (vx, vy)
    max_accel: float  # m/s^2, norma perubahan (vx, vy)
    max_omega: float  # rad/s
    max_alpha: float  # rad/s^2

    def clamp(self, cmd: Velocity, current: Velocity, dt: float) -> Velocity:
        """Perintah kecepatan yang sanggup dicapai dalam dt dari kecepatan sekarang."""
        vx, vy = cmd.vx, cmd.vy
        speed = math.hypot(vx, vy)
        if speed > self.max_speed:
            vx, vy = vx * self.max_speed / speed, vy * self.max_speed / speed
        omega = _clip(cmd.omega, self.max_omega)

        dvx, dvy = vx - current.vx, vy - current.vy
        dv = math.hypot(dvx, dvy)
        allowed = self.max_accel * dt
        if dv > allowed:
            dvx, dvy = dvx * allowed / dv, dvy * allowed / dv
        domega = _clip(omega - current.omega, self.max_alpha * dt)
        return Velocity(current.vx + dvx, current.vy + dvy, current.omega + domega)


class Robot:
    """Robot lingkaran. Keadaannya hanya berubah lewat step() dan teleport()."""

    def __init__(self, pose: Pose, radius: float, limits: RobotLimits):
        self._pose = pose
        self._velocity = Velocity()
        self.radius = radius
        self.limits = limits

    @property
    def pose(self) -> Pose:
        return self._pose

    @property
    def velocity(self) -> Velocity:
        return self._velocity

    def teleport(self, pose: Pose) -> None:
        """Pindah seketika; kecepatan dinolkan."""
        self._pose = pose
        self._velocity = Velocity()

    def step(self, cmd: Velocity, dt: float, width: float, height: float) -> None:
        v = self.limits.clamp(cmd, self._velocity, dt)
        p = self._pose
        c, s = math.cos(p.theta), math.sin(p.theta)
        x = p.x + (c * v.vx - s * v.vy) * dt
        y = p.y + (s * v.vx + c * v.vy) * dt
        x = max(self.radius, min(width - self.radius, x))
        y = max(self.radius, min(height - self.radius, y))
        self._pose = Pose(x, y, wrap_angle(p.theta + v.omega * dt))
        self._velocity = v
