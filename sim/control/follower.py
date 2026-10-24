"""Pengendali pengikut sederhana + class dasar Controller."""
from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Union

from ..types import Pose, Velocity, TrackResult, Vec2, Obstacle
from ..geometry import wrap_angle


@dataclass
class TangentPlane:
    """Bidang singgung di sekitar obstacle (untuk visualisasi & constraint)."""
    obstacle_id: int
    point: Vec2
    normal: Vec2
    p1: Vec2
    p2: Vec2
    safe_radius: float
    slack: float = 0.0


class Controller(ABC):
    """Kelas dasar semua pengendali."""

    def __init__(self, cfg):
        self.cfg = cfg

    @property
    @abstractmethod
    def name(self) -> str:
        pass

    @abstractmethod
    def track(self, state, trajectory, obstacles) -> TrackResult:
        pass

    def reset(self):
        pass


class FollowerController(Controller):
    """Pengendali sederhana: mengikuti jalur dengan lookahead"""

    @property
    def name(self) -> str:
        return "Pengikut sederhana"

    def track(self, state, trajectory, obstacles) -> TrackResult:
        t0 = time.perf_counter()
        pose = state.pose if hasattr(state, "pose") else state

        # Tidak ada jalur → diam
        if not trajectory:
            return TrackResult(
                command=Velocity(0.0, 0.0, 0.0),
                horizon=[pose],
                tangent_planes=[],
                slack=0.0,
                solve_time_ms=(time.perf_counter() - t0) * 1000.0,
            )

        pos = pose.position

        # 1. Cari titik terdekat di jalur
        best_idx = 0
        min_dist = float("inf")
        for i, tp in enumerate(trajectory):
            d = pos.dist(tp.pos)
            if d < min_dist:
                min_dist = d
                best_idx = i

        # 2. Lookahead sekitar 0.4 meter
        target_idx = best_idx
        jarak = 0.0
        while target_idx < len(trajectory) - 1 and jarak < 0.4:
            jarak += trajectory[target_idx].pos.dist(trajectory[target_idx + 1].pos)
            target_idx += 1

        target = trajectory[target_idx]
        diff = target.pos - pos
        dist = diff.norm()

        # 3. Hitung kecepatan
        if dist < 1e-4:
            vx, vy = 0.0, 0.0
        else:
            speed = min(self.cfg.max_speed, getattr(target, "speed", 1.0))
            # Perlambat saat mendekati tujuan
            dist_end = pos.dist(trajectory[-1].pos)
            if dist_end < 0.6:
                speed = max(0.1, speed * (dist_end / 0.6))

            arah = math.atan2(diff.y, diff.x)
            vx_w = speed * math.cos(arah)
            vy_w = speed * math.sin(arah)

            # Ubah ke kerangka tubuh robot
            c, s = math.cos(pose.theta), math.sin(pose.theta)
            vx =  c * vx_w + s * vy_w
            vy = -s * vx_w + c * vy_w

        # 4. Kontrol arah hadap
        err_theta = wrap_angle(getattr(target, "theta", pose.theta) - pose.theta)
        omega = max(-self.cfg.max_omega, min(self.cfg.max_omega, 3.0 * err_theta))

        command = Velocity(vx, vy, omega)

        # 5. Prediksi horizon (untuk visualisasi)
        horizon = []
        p = pose
        dt = getattr(self.cfg, "dt_mpc", 0.1)
        N = getattr(self.cfg, "horizon", 10)
        for _ in range(N):
            c, s = math.cos(p.theta), math.sin(p.theta)
            dx = (c * vx - s * vy) * dt
            dy = (s * vx + c * vy) * dt
            p = Pose(p.x + dx, p.y + dy, wrap_angle(p.theta + omega * dt))
            horizon.append(p)

        return TrackResult(
            command=command,
            horizon=horizon,
            tangent_planes=[],
            slack=0.0,
            solve_time_ms=(time.perf_counter() - t0) * 1000.0,
        )