"""Pengendali pengikut sederhana + class dasar Controller (PRD §4.2, §4.3)."""
from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from ..geometry import wrap_angle
from ..types import Obstacle, Pose, TrackResult, TrajectoryPoint, Vec2, Velocity


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
    """Pengendali sederhana: mengikuti trajektori dengan lookahead (PRD §4.2)."""

    @property
    def name(self) -> str:
        return "Pengikut sederhana"

    def track(self, state, trajectory, obstacles) -> TrackResult:
        t0 = time.perf_counter()
        pose: Pose = state.pose if hasattr(state, "pose") else state
        dt = getattr(self.cfg, "dt_mpc", 0.1)
        N = getattr(self.cfg, "horizon", 10)
        max_speed = getattr(self.cfg, "max_speed", 2.0)
        max_omega = getattr(self.cfg, "max_omega", 2.0)

        # 0. Trajektori kosong -> robot diam
        if not trajectory:
            return TrackResult(
                command=Velocity(0.0, 0.0, 0.0),
                horizon=[pose] * N,
                tangent_planes=[],
                slack=0.0,
                solve_time_ms=(time.perf_counter() - t0) * 1000.0,
            )

        pos = pose.position

        # Helper ekstraksi posisi Vec2 dari TrajectoryPoint atau Vec2
        def get_pos(tp) -> Vec2:
            return tp.pos if hasattr(tp, "pos") else tp

        # 1. Cari titik terdekat di trajektori
        best_idx = 0
        min_dist = float("inf")
        for i, tp in enumerate(trajectory):
            d = pos.dist(get_pos(tp))
            if d < min_dist:
                min_dist = d
                best_idx = i

        # 2. Lookahead sepanjang jalur (~0.4 meter)
        target_idx = best_idx
        jarak = 0.0
        while target_idx < len(trajectory) - 1 and jarak < 0.4:
            p_curr = get_pos(trajectory[target_idx])
            p_next = get_pos(trajectory[target_idx + 1])
            jarak += p_curr.dist(p_next)
            target_idx += 1

        target = trajectory[target_idx]
        target_pos = get_pos(target)
        diff = target_pos - pos
        dist = diff.norm()

        # 3. Hitung kecepatan linear
        goal_pos = get_pos(trajectory[-1])
        dist_end = pos.dist(goal_pos)

        if dist < 1e-4 or dist_end < 1e-4:
            vx, vy = 0.0, 0.0
        else:
            speed = min(max_speed, getattr(target, "speed", max_speed))
            # Perlambat saat mendekati titik akhir (deceleration zone 0.6 m)
            if dist_end < 0.6:
                speed = max(0.1, speed * (dist_end / 0.6))

            arah = math.atan2(diff.y, diff.x)
            vx_w = speed * math.cos(arah)
            vy_w = speed * math.sin(arah)

            # Transformasi ke kerangka tubuh robot (x maju, y kiri)
            c, s = math.cos(pose.theta), math.sin(pose.theta)
            vx = c * vx_w + s * vy_w
            vy = -s * vx_w + c * vy_w

            # Potong ke batas kecepatan jika melebihi akibat pembulatan
            v_norm = math.hypot(vx, vy)
            if v_norm > max_speed and v_norm > 1e-9:
                scale = max_speed / v_norm
                vx *= scale
                vy *= scale

        # 4. Kontrol arah hadap
        target_theta = getattr(target, "theta", pose.theta)
        err_theta = wrap_angle(target_theta - pose.theta)
        if abs(err_theta) < 1e-4:
            omega = 0.0
        else:
            omega = max(-max_omega, min(max_omega, 3.0 * err_theta))

        command = Velocity(vx, vy, omega)

        # 5. Prediksi horizon N-langkah (untuk visualisasi Lapisan 8)
        horizon: list[Pose] = []
        p = pose
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
