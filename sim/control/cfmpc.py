"""Pengendali cf-MPC (convex-feasible MPC) dengan slack variable.

Ref: Hou et al., ICRA 2025, arXiv 2504.02184, Bagian 3.
Lihat docs/cfmpc_algorithm.md untuk detail formulasi QP.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cvxpy as cp
import numpy as np

from ..geometry import wrap_angle
from ..types import Pose, TrackResult, TrajectoryPoint, Vec2, Velocity
from .follower import Controller, FollowerController, TangentPlane


@dataclass
class _QPModel:
    """Model QP terkompilasi dengan cvxpy.Parameter untuk satu jumlah obstacle."""
    problem: cp.Problem
    x0: cp.Parameter
    c0: cp.Parameter
    s0: cp.Parameter
    ref: cp.Parameter
    ref_N: cp.Parameter
    U: cp.Variable
    X: cp.Variable
    Sigma: Optional[cp.Variable] = None
    normals: Optional[cp.Parameter] = None
    distances: Optional[cp.Parameter] = None


class CfMpcController(Controller):
    """Pengendali cf-MPC dengan slack variable untuk obstacle avoidance."""

    # Bobot QP (mengikuti parameter referensi paper)
    _W_POS: float = 10.0   # bobot posisi pada setiap langkah horizon
    _W_VEL: float = 1.0    # bobot kontrol (regularisasi u)
    _W_TERM: float = 50.0  # bobot terminal (posisi akhir -> referensi)

    def __init__(self, cfg):
        super().__init__(cfg)
        self._models: Dict[int, _QPModel] = {}
        self._follower = FollowerController(cfg)

    @property
    def name(self) -> str:
        return "cf-MPC"

    # ------------------------------------------------------------------
    # Antarmuka publik
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Reset state internal pengendali."""
        self._follower.reset()

    def track(self, state, trajectory: List[TrajectoryPoint], obstacles) -> TrackResult:
        """Hitung perintah kecepatan optimal menggunakan cf-MPC."""
        t0 = time.perf_counter()
        pose: Pose = state.pose if hasattr(state, "pose") else state

        # Tidak ada trajektori -> diam
        if not trajectory:
            return TrackResult(
                command=Velocity(0.0, 0.0, 0.0),
                horizon=[pose] * (self.cfg.horizon + 1),
                tangent_planes=[],
                slack=0.0,
                solve_time_ms=(time.perf_counter() - t0) * 1000.0,
            )

        N = self.cfg.horizon
        dt = self.cfg.dt_mpc
        rho = self.cfg.rho_slack

        # 1. Kumpulkan trajektori referensi sejauh horizon
        ref = self._slice_reference(pose, trajectory, N)

        # 2. Bangun bidang singgung dari obstacle aktif
        tangent_planes = self._build_tangent_planes(pose, obstacles)

        # 3. Formulasikan dan pecahkan QP (dengan kompilasi Parameter)
        try:
            u_opt, x_pred, slacks = self._solve_qp(
                pose, ref, tangent_planes, N, dt, rho
            )
        except Exception:
            # Fallback ke pengikut sederhana jika solver mengalami galat tak terduga
            fallback = self._follower.track(state, trajectory, obstacles)
            return TrackResult(
                command=fallback.command,
                horizon=fallback.horizon,
                tangent_planes=tangent_planes,
                slack=0.0,
                solve_time_ms=(time.perf_counter() - t0) * 1000.0,
            )

        # Simpan nilai slack pada masing-masing bidang singgung
        for j, tp in enumerate(tangent_planes):
            if j < len(slacks):
                tp.slack = float(slacks[j])

        # 4. Kemas hasil
        command = self._to_velocity(u_opt)
        horizon = self._to_pose_list(x_pred)
        total_slack = float(np.sum(slacks))

        return TrackResult(
            command=command,
            horizon=horizon,
            tangent_planes=tangent_planes,
            slack=total_slack,
            solve_time_ms=(time.perf_counter() - t0) * 1000.0,
        )

    # ------------------------------------------------------------------
    # Pembantu privat
    # ------------------------------------------------------------------

    def _slice_reference(self, pose: Pose, trajectory: List[TrajectoryPoint], N: int) -> np.ndarray:
        """Ambil N titik referensi terdekat. Mengembalikan array (N, 3)."""
        pos = Vec2(pose.x, pose.y)
        best = min(range(len(trajectory)), key=lambda i: pos.dist(trajectory[i].pos))
        ref = []
        for k in range(N):
            idx = min(best + k + 1, len(trajectory) - 1)
            tp = trajectory[idx]
            ref.append([tp.pos.x, tp.pos.y, tp.theta])
        return np.array(ref, dtype=float)

    def _build_tangent_planes(self, pose: Pose, obstacles) -> List[TangentPlane]:
        """Buat bidang singgung dari obstacle di sekitar robot."""
        robot_pos = Vec2(pose.x, pose.y)
        planes: List[TangentPlane] = []

        for obs in obstacles:
            diff = robot_pos - obs.center
            dist = diff.norm()
            safe_r = self.cfg.safe_radius(obs)

            if dist < 1e-6:
                # Robot tepat di pusat obstacle -- normal ke arah x+
                nx, ny = 1.0, 0.0
            else:
                nx, ny = diff.x / dist, diff.y / dist

            # Titik kontak pada permukaan zona aman
            contact = Vec2(obs.center.x + nx * safe_r, obs.center.y + ny * safe_r)

            # Dua titik pada garis singgung (tegak lurus normal)
            p1 = Vec2(contact.x - ny, contact.y + nx)
            p2 = Vec2(contact.x + ny, contact.y - nx)

            planes.append(TangentPlane(
                obstacle_id=obs.id,
                point=contact,
                normal=Vec2(nx, ny),
                p1=p1,
                p2=p2,
                safe_radius=safe_r,
                slack=0.0,
            ))

        return planes

    def _get_or_create_model(self, n_obs: int, N: int, dt: float, rho: float) -> _QPModel:
        """Kompilasi problem CVXPY sekali menggunakan Parameter untuk jumlah obstacle tertentu."""
        if n_obs in self._models:
            return self._models[n_obs]

        v_max = self.cfg.max_speed
        w_max = self.cfg.max_omega

        x0 = cp.Parameter(3)
        c0 = cp.Parameter()
        s0 = cp.Parameter()
        ref = cp.Parameter((N, 2))
        ref_N = cp.Parameter(2)

        U = cp.Variable((N, 3))
        X = cp.Variable((N + 1, 3))

        cost = 0.0
        # Biaya pelacakan trajektori posisi
        for k in range(N):
            cost += self._W_POS * cp.sum_squares(X[k + 1, :2] - ref[k, :2])
        # Biaya terminal
        cost += self._W_TERM * cp.sum_squares(X[N, :2] - ref_N)
        # Regularisasi kontrol
        cost += self._W_VEL * cp.sum_squares(U)

        constraints = [X[0, :] == x0]

        # Dinamika omnidirectional linear
        for k in range(N):
            vx_w = c0 * U[k, 0] - s0 * U[k, 1]
            vy_w = s0 * U[k, 0] + c0 * U[k, 1]
            constraints += [
                X[k + 1, 0] == X[k, 0] + dt * vx_w,
                X[k + 1, 1] == X[k, 1] + dt * vy_w,
                X[k + 1, 2] == X[k, 2] + dt * U[k, 2],
                U[k, 0] <= v_max, U[k, 0] >= -v_max,
                U[k, 1] <= v_max, U[k, 1] >= -v_max,
                U[k, 2] <= w_max, U[k, 2] >= -w_max,
            ]

        if n_obs > 0:
            Sigma = cp.Variable((N, n_obs), nonneg=True)
            normals = cp.Parameter((n_obs, 2))
            distances = cp.Parameter(n_obs)
            # Penalti kuadratik slack (PRD §4.2: rho * sum delta^2)
            cost += rho * cp.sum_squares(Sigma)

            for j in range(n_obs):
                for k in range(N):
                    constraints.append(
                        normals[j] @ X[k + 1, :2] >= distances[j] - Sigma[k, j]
                    )
        else:
            Sigma = None
            normals = None
            distances = None

        prob = cp.Problem(cp.Minimize(cost), constraints)
        model = _QPModel(
            problem=prob,
            x0=x0,
            c0=c0,
            s0=s0,
            ref=ref,
            ref_N=ref_N,
            U=U,
            X=X,
            Sigma=Sigma,
            normals=normals,
            distances=distances,
        )
        self._models[n_obs] = model
        return model

    def _solve_qp(
        self,
        pose: Pose,
        ref: np.ndarray,
        tangent_planes: List[TangentPlane],
        N: int,
        dt: float,
        rho: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Pecahkan QP menggunakan model Parameter terkompilasi."""
        n_obs = len(tangent_planes)
        model = self._get_or_create_model(n_obs, N, dt, rho)

        # Berikan nilai ke parameter
        model.x0.value = np.array([pose.x, pose.y, pose.theta], dtype=float)
        model.c0.value = float(math.cos(pose.theta))
        model.s0.value = float(math.sin(pose.theta))
        model.ref.value = ref[:, :2]
        model.ref_N.value = ref[-1, :2]

        if n_obs > 0:
            norm_arr = np.zeros((n_obs, 2), dtype=float)
            dist_arr = np.zeros(n_obs, dtype=float)
            for j, tp in enumerate(tangent_planes):
                nj = np.array([tp.normal.x, tp.normal.y], dtype=float)
                cj = np.array([tp.point.x, tp.point.y], dtype=float)
                norm_arr[j, :] = nj
                dist_arr[j] = float(nj @ cj)
            model.normals.value = norm_arr
            model.distances.value = dist_arr

        # Selesaikan QP menggunakan OSQP
        model.problem.solve(
            solver=cp.OSQP,
            warm_starting=True,
            verbose=False,
            eps_abs=1e-4,
            eps_rel=1e-4,
            max_iter=4000,
        )

        status = model.problem.status
        if status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) or model.U.value is None:
            raise RuntimeError(f"QP tidak terpecahkan: status={status}")

        u_opt = np.array(model.U.value[0], dtype=float)
        x_pred = np.array(model.X.value, dtype=float)

        if n_obs > 0 and model.Sigma is not None and model.Sigma.value is not None:
            slacks = np.array(model.Sigma.value.sum(axis=0), dtype=float)
        else:
            slacks = np.zeros(max(1, n_obs), dtype=float)

        return u_opt, x_pred, slacks

    def _to_velocity(self, u: np.ndarray) -> Velocity:
        """Ubah array kontrol [vx_body, vy_body, omega] menjadi Velocity dengan pemotongan kelajuan."""
        v_max = self.cfg.max_speed
        w_max = self.cfg.max_omega
        vx, vy = float(u[0]), float(u[1])
        speed = math.hypot(vx, vy)
        if speed > v_max > 0.0:
            vx, vy = vx * v_max / speed, vy * v_max / speed
        omega = float(np.clip(u[2], -w_max, w_max))
        return Velocity(vx=vx, vy=vy, omega=omega)

    def _to_pose_list(self, x_pred: np.ndarray) -> List[Pose]:
        """Ubah array prediksi state (N+1, 3) menjadi daftar Pose."""
        return [
            Pose(float(row[0]), float(row[1]), float(wrap_angle(row[2])))
            for row in x_pred
        ]
