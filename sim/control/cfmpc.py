"""Pengendali cf-MPC (convex-feasible MPC) dengan slack variable.

Ref: Hou et al., ICRA 2025, arXiv 2504.02184, Bagian 3.
Lihat docs/cfmpc_algorithm.md untuk detail formulasi QP.

Robot dimodelkan sebagai titik berkecepatan yang dikendalikan lewat percepatan
(linear, jadi hanya batas tabrakan yang perlu dilinearkan). Tiap obstacle dekat
memberi satu bidang singgung pada zona amannya di tiap langkah horizon:

    n_k . p_k  >=  n_k . c_k + R - delta_k,    delta_k >= 0

dengan n_k diarahkan dari obstacle ke posisi horizon hasil solve sebelumnya
(titik acuan bila belum ada), dan c_k posisi obstacle yang diprediksi bergerak
lurus. Berpatokan pada solusi sebelumnya menjaga robot tetap di sisi obstacle
yang sama; berpatokan pada acuan membuat arah bidang berbalik-balik begitu
acuan jatuh di dalam zona obstacle yang bergerak. QP dikompilasi sekali dengan jumlah
slot obstacle yang tetap, jadi tidak ada kompilasi ulang saat jumlah obstacle
berubah.
"""
from __future__ import annotations

import logging
import math
import time
import warnings
from typing import List, Tuple

import cvxpy as cp
import numpy as np

from ..geometry import wrap_angle
from ..types import Pose, TrackResult, TrajectoryPoint, Vec2, Velocity
from .follower import Controller, FollowerController, TangentPlane

log = logging.getLogger(__name__)

SLOTS = 6  # obstacle terdekat yang diberi bidang singgung; sisanya diabaikan
ANCHOR_MAX_DRIFT = 0.5  # solusi sebelumnya dipakai sebagai patokan bila posisinya masih sedekat ini ke robot (m)
RELEVANT_MARGIN = 1.0  # bidang hanya dilaporkan bila acuan lebih dekat dari ini ke zona aman (m)
SLACK_EPS = 1e-4  # slack di bawah ini dianggap nol (galat numerik solver)
TURN_GAIN = 3.0  # sama dengan FollowerController
SQRT2 = math.sqrt(2.0)


class CfMpcController(Controller):
    """Pengendali cf-MPC dengan slack variable untuk obstacle avoidance."""

    # Bobot QP (mengikuti parameter referensi paper)
    _W_POS: float = 10.0   # bobot posisi pada setiap langkah horizon
    _W_VEL: float = 1.0    # bobot kecepatan terhadap kecepatan acuan
    _W_ACC: float = 0.1    # regularisasi percepatan
    _W_TERM: float = 40.0  # bobot terminal (posisi akhir -> referensi)

    def __init__(self, cfg):
        super().__init__(cfg)
        self._follower = FollowerController(cfg)
        self._prev_positions = None  # horizon (2, N+1) hasil solve sebelumnya
        self._build()
        self._set_idle_parameters()
        # kompilasi di muka, bukan saat AUTO pertama atau saat jumlah obstacle berubah
        self._problem.solve(solver=cp.OSQP, warm_start=True)

    @property
    def name(self) -> str:
        return "cf-MPC"

    def reset(self) -> None:
        """Reset state internal pengendali."""
        self._follower.reset()
        self._prev_positions = None

    # ------------------------------------------------------------------
    # Masalah QP
    # ------------------------------------------------------------------

    def _build(self) -> None:
        cfg = self.cfg
        n, dt = cfg.horizon, cfg.dt_mpc
        self.p0, self.v0 = cp.Parameter(2), cp.Parameter(2)
        self.p_ref, self.v_ref = cp.Parameter((2, n)), cp.Parameter((2, n))
        self.nx = [cp.Parameter(n) for _ in range(SLOTS)]
        self.ny = [cp.Parameter(n) for _ in range(SLOTS)]
        self.rhs = [cp.Parameter(n) for _ in range(SLOTS)]

        P, V, A = cp.Variable((2, n + 1)), cp.Variable((2, n + 1)), cp.Variable((2, n))
        D = cp.Variable((SLOTS, n), nonneg=True)
        self._P, self._V, self._A, self._D = P, V, A, D

        constraints = [
            P[:, 0] == self.p0,
            V[:, 0] == self.v0,
            P[:, 1:] == P[:, :-1] + V[:, :-1] * dt + A * (0.5 * dt * dt),
            V[:, 1:] == V[:, :-1] + A * dt,
        ]
        constraints += self._octagon(V[:, 1:], cfg.max_speed)
        constraints += self._octagon(A, cfg.max_accel)
        r = cfg.robot_radius
        constraints += [
            P[0, 1:] >= r, P[0, 1:] <= cfg.field_width - r,
            P[1, 1:] >= r, P[1, 1:] <= cfg.field_height - r,
        ]
        for j in range(SLOTS):
            constraints.append(
                cp.multiply(self.nx[j], P[0, 1:]) + cp.multiply(self.ny[j], P[1, 1:]) >= self.rhs[j] - D[j]
            )

        cost = (
            self._W_POS * cp.sum_squares(P[:, 1:-1] - self.p_ref[:, :-1])
            + self._W_TERM * cp.sum_squares(P[:, -1] - self.p_ref[:, -1])
            + self._W_VEL * cp.sum_squares(V[:, 1:] - self.v_ref)
            + self._W_ACC * cp.sum_squares(A)
            + cfg.rho_slack * cp.sum_squares(D)  # PRD §4.2: rho * sum delta^2
        )
        self._problem = cp.Problem(cp.Minimize(cost), constraints)
        assert self._problem.is_dpp()

    @staticmethod
    def _octagon(x, limit: float) -> list:
        """Aproksimasi linear norma <= limit, supaya masalahnya tetap QP."""
        return [
            x <= limit, x >= -limit,
            cp.sum(x, axis=0) <= SQRT2 * limit, cp.sum(x, axis=0) >= -SQRT2 * limit,
            x[0] - x[1] <= SQRT2 * limit, x[1] - x[0] <= SQRT2 * limit,
        ]

    def _set_idle_parameters(self) -> None:
        n = self.cfg.horizon
        self.p0.value, self.v0.value = np.array([1.0, 1.0]), np.zeros(2)
        self.p_ref.value, self.v_ref.value = np.ones((2, n)), np.zeros((2, n))
        self._clear_planes(0)

    def _clear_planes(self, first_free: int) -> None:
        n = self.cfg.horizon
        for j in range(first_free, SLOTS):
            self.nx[j].value = np.zeros(n)
            self.ny[j].value = np.zeros(n)
            self.rhs[j].value = -np.ones(n)  # 0 >= -1: selalu terpenuhi

    # ------------------------------------------------------------------
    # Antarmuka publik
    # ------------------------------------------------------------------

    def track(self, state, trajectory: List[TrajectoryPoint], obstacles) -> TrackResult:
        """Hitung perintah kecepatan optimal menggunakan cf-MPC."""
        t0 = time.perf_counter()
        pose: Pose = state.pose if hasattr(state, "pose") else state
        n, dt = self.cfg.horizon, self.cfg.dt_mpc

        # Tidak ada trajektori -> diam
        if not trajectory:
            return TrackResult(
                command=Velocity(0.0, 0.0, 0.0),
                horizon=[pose] * (n + 1),
                tangent_planes=[],
                slack=0.0,
                solve_time_ms=(time.perf_counter() - t0) * 1000.0,
            )

        refs, ref_prev, ref_theta = self._slice_reference(pose, trajectory, n)
        self._set_state(pose, getattr(state, "velocity", None), refs, ref_prev, dt)
        anchors = self._anchors(pose, refs)
        planes = self._fill_planes(pose, anchors, refs, obstacles, n, dt)

        try:
            self._solve()
        except Exception as exc:  # solver gagal: pengikut sederhana, bukan perintah yang tak dapat dipercaya
            self._prev_positions = None
            log.warning("cf-MPC gagal (%s); memakai pengikut sederhana", exc)
            fallback = self._follower.track(state, trajectory, obstacles)
            return TrackResult(
                command=fallback.command,
                horizon=fallback.horizon,
                tangent_planes=[tp for tp, _ in planes],
                slack=0.0,
                solve_time_ms=(time.perf_counter() - t0) * 1000.0,
            )

        self._prev_positions = np.array(self._P.value)
        slacks = self._D.value.max(axis=1)
        for tp, slot in planes:
            tp.slack = float(slacks[slot]) if slacks[slot] > SLACK_EPS else 0.0
        slack = float(max((slacks[slot] for _, slot in planes), default=0.0))

        return TrackResult(
            command=self._command(pose, self._V.value[:, 1], ref_theta[0]),
            horizon=self._horizon(pose, ref_theta),
            tangent_planes=[tp for tp, _ in planes],
            slack=slack if slack > SLACK_EPS else 0.0,
            solve_time_ms=(time.perf_counter() - t0) * 1000.0,
        )

    # ------------------------------------------------------------------
    # Pembantu privat
    # ------------------------------------------------------------------

    def _slice_reference(
        self, pose: Pose, trajectory: List[TrajectoryPoint], n: int
    ) -> Tuple[List[Vec2], Vec2, List[float]]:
        """N titik acuan setelah titik terdekat, titik terdekatnya, dan sudut hadap acuan."""
        pos = Vec2(pose.x, pose.y)
        best = min(range(len(trajectory)), key=lambda i: pos.dist(trajectory[i].pos))
        picked = [trajectory[min(best + k + 1, len(trajectory) - 1)] for k in range(n)]
        return [tp.pos for tp in picked], trajectory[best].pos, [tp.theta for tp in picked]

    def _set_state(self, pose: Pose, velocity, refs: List[Vec2], ref_prev: Vec2, dt: float) -> None:
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        vx, vy = (velocity.vx, velocity.vy) if velocity is not None else (0.0, 0.0)
        self.p0.value = np.array([pose.x, pose.y])
        self.v0.value = np.array([c * vx - s * vy, s * vx + c * vy])  # kerangka tubuh -> dunia
        chain = [ref_prev] + refs
        self.p_ref.value = np.array([[p.x for p in refs], [p.y for p in refs]])
        self.v_ref.value = np.array([
            [(b.x - a.x) / dt for a, b in zip(chain, chain[1:])],
            [(b.y - a.y) / dt for a, b in zip(chain, chain[1:])],
        ])

    def _anchors(self, pose: Pose, refs: List[Vec2]) -> List[Vec2]:
        """Posisi patokan bidang singgung per langkah: solusi sebelumnya bila masih dekat robot, kalau tidak acuan."""
        prev = self._prev_positions
        if prev is None or math.hypot(prev[0, 0] - pose.x, prev[1, 0] - pose.y) > ANCHOR_MAX_DRIFT:
            return refs
        return [Vec2(float(prev[0, k]), float(prev[1, k])) for k in range(1, len(refs) + 1)]

    def _fill_planes(self, pose: Pose, anchors: List[Vec2], refs: List[Vec2], obstacles, n: int, dt: float):
        """Mengisi parameter bidang singgung; mengembalikan [(TangentPlane, slot)] yang relevan untuk lapisan 9."""
        pos = Vec2(pose.x, pose.y)
        nearest = sorted(obstacles, key=lambda o: pos.dist(o.center) - self.cfg.safe_radius(o))[:SLOTS]
        reported = []
        for j, obs in enumerate(nearest):
            radius = self.cfg.safe_radius(obs)
            nx, ny, rhs = np.zeros(n), np.zeros(n), np.zeros(n)
            best_gap, best_plane = math.inf, None
            for k in range(1, n + 1):
                center = obs.center + obs.velocity * (k * dt)  # obstacle diprediksi bergerak lurus
                away = anchors[k - 1] - center
                if away.norm() < 1e-6:
                    away = pos - center
                if away.norm() < 1e-6:
                    away = Vec2(1.0, 0.0)
                normal = away * (1.0 / away.norm())
                nx[k - 1], ny[k - 1] = normal.x, normal.y
                rhs[k - 1] = normal.x * center.x + normal.y * center.y + radius
                gap = (refs[k - 1] - center).norm() - radius
                if gap < best_gap:
                    best_gap, best_plane = gap, (center, normal)
            self.nx[j].value, self.ny[j].value, self.rhs[j].value = nx, ny, rhs
            if best_gap < RELEVANT_MARGIN and best_plane is not None:
                center, normal = best_plane
                contact = center + normal * radius
                reported.append((TangentPlane(
                    obstacle_id=obs.id,
                    point=contact,
                    normal=normal,
                    p1=Vec2(contact.x - normal.y, contact.y + normal.x),
                    p2=Vec2(contact.x + normal.y, contact.y - normal.x),
                    safe_radius=radius,
                    slack=0.0,
                ), j))
        self._clear_planes(len(nearest))
        return reported

    def _solve(self) -> None:
        with warnings.catch_warnings():
            # OPTIMAL_INACCURATE diterima di bawah; peringatan cvxpy-nya hanya mengotori keluaran
            warnings.simplefilter("ignore", UserWarning)
            self._problem.solve(
                solver=cp.OSQP, warm_start=True, verbose=False, eps_abs=1e-4, eps_rel=1e-4, max_iter=4000
            )
        if self._problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) or self._V.value is None:
            raise RuntimeError(f"QP tidak terpecahkan: status={self._problem.status}")

    def _command(self, pose: Pose, v_world: np.ndarray, ref_theta: float) -> Velocity:
        """Kecepatan dunia pada langkah pertama menjadi perintah kerangka tubuh."""
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        wx, wy = float(v_world[0]), float(v_world[1])
        omega = max(-self.cfg.max_omega, min(self.cfg.max_omega, TURN_GAIN * wrap_angle(ref_theta - pose.theta)))
        return Velocity(c * wx + s * wy, -s * wx + c * wy, omega)

    def _horizon(self, pose: Pose, ref_theta: List[float]) -> List[Pose]:
        """Pose saat ini diikuti prediksi N langkah (N + 1 pose)."""
        return [pose] + [
            Pose(float(x), float(y), float(theta))
            for (x, y), theta in zip(self._P.value[:, 1:].T, ref_theta)
        ]
