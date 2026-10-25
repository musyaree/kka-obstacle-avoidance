"""Pencarian rute dan interpolasi trajektori untuk DAVG (PRD §4.2).

Mengimplementasikan:
- AugmentedSearch: A* dengan bobot sudut belok (jarak + λ·|sudut belok|).
- TrajectoryInterpolator: interpolasi linear berwaktu menghasilkan TrajectoryPoint.
- Planner: orkestrator yang menghasilkan PlanResult.
"""
from __future__ import annotations

import heapq
import math
import time
from typing import Optional

from sim.geometry import wrap_angle
from sim.types import PlanResult, PlanStatus, Scene, TrajectoryPoint, Vec2
from sim.planning.graph import VisibilityGraph


def get_turn_angle(prev: Vec2, curr: Vec2, nxt: Vec2) -> float:
    """Menghitung sudut belok (radian) dari segmen prev->curr ke curr->nxt."""
    angle1 = math.atan2(curr.y - prev.y, curr.x - prev.x)
    angle2 = math.atan2(nxt.y - curr.y, nxt.x - curr.x)
    return wrap_angle(angle2 - angle1)


class AugmentedSearch:
    """A* dengan state yang diaugmentasi (curr, prev) untuk menghitung penalti sudut belok.

    Bobot edge: jarak + lambda_turn * |sudut belok| (PRD §4.2.3).
    Heuristik: jarak Euclidean ke goal (admissible karena bobot sudut >= 0).
    """

    def __init__(
        self,
        start: Vec2,
        goal: Vec2,
        edges: dict[Vec2, list[tuple[Vec2, float]]],
        lambda_turn: float = 1.0,
        initial_heading: Optional[float] = None,
    ):
        self.start = start
        self.goal = goal
        self.edges = edges
        self.lambda_turn = lambda_turn
        self.initial_heading = initial_heading

    def search(self) -> Optional[list[Vec2]]:
        if self.start.dist(self.goal) < 1e-6:
            return [self.start]

        counter = 0
        h_start = self.start.dist(self.goal)
        # Entry di heap: (f, g, counter, curr, prev)
        queue: list[tuple[float, float, int, Vec2, Optional[Vec2]]] = [
            (h_start, 0.0, counter, self.start, None)
        ]
        best_g: dict[tuple[Vec2, Optional[Vec2]], float] = {(self.start, None): 0.0}
        came_from: dict[tuple[Vec2, Optional[Vec2]], tuple[Vec2, Optional[Vec2]]] = {}

        while queue:
            f, g, _, curr, prev = heapq.heappop(queue)

            if g > best_g.get((curr, prev), float("inf")):
                continue

            if curr == self.goal or curr.dist(self.goal) < 1e-6:
                path = []
                state: Optional[tuple[Vec2, Optional[Vec2]]] = (curr, prev)
                while state is not None:
                    c_node, _ = state
                    path.append(c_node)
                    state = came_from.get(state)
                path.reverse()
                return path

            for nxt, dist in self.edges.get(curr, []):
                if prev is not None and nxt == prev:
                    continue

                if prev is not None:
                    turn_angle = get_turn_angle(prev, curr, nxt)
                    turn_penalty = self.lambda_turn * abs(turn_angle)
                elif self.initial_heading is not None:
                    curr_heading = math.atan2(nxt.y - curr.y, nxt.x - curr.x)
                    turn_angle = wrap_angle(curr_heading - self.initial_heading)
                    turn_penalty = self.lambda_turn * abs(turn_angle)
                else:
                    turn_penalty = 0.0

                new_g = g + dist + turn_penalty
                next_state = (nxt, curr)
                if new_g < best_g.get(next_state, float("inf")):
                    best_g[next_state] = new_g
                    came_from[next_state] = (curr, prev)
                    h = nxt.dist(self.goal)
                    new_f = new_g + h
                    counter += 1
                    heapq.heappush(queue, (new_f, new_g, counter, nxt, curr))

        return None


# Alias sesuai nama awal Ben
AugmentedAStar = AugmentedSearch


class TrajectoryInterpolator:
    """Menginterpolasi titik-titik jalur secara linear dan memberi timestamp (PRD §4.2.6)."""

    @staticmethod
    def interpolate(path: list[Vec2], max_speed: float, dt: float) -> list[TrajectoryPoint]:
        if not path or max_speed <= 0.0 or dt <= 0.0:
            return []
        if len(path) == 1:
            return [TrajectoryPoint(pos=path[0], theta=0.0, speed=0.0, t=0.0)]

        segments: list[tuple[Vec2, Vec2, float, float]] = []
        total_dist = 0.0
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            seg_len = p1.dist(p2)
            if seg_len > 1e-9:
                theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
                segments.append((p1, p2, seg_len, theta))
                total_dist += seg_len

        if not segments:
            return [TrajectoryPoint(pos=path[0], theta=0.0, speed=0.0, t=0.0)]

        total_time = total_dist / max_speed

        def sample_at_s(s: float) -> tuple[Vec2, float]:
            s = max(0.0, min(total_dist, s))
            accum = 0.0
            for p1, p2, seg_len, theta in segments:
                if accum + seg_len >= s or abs(accum + seg_len - s) < 1e-9:
                    ratio = (s - accum) / seg_len if seg_len > 1e-9 else 0.0
                    pos = Vec2(p1.x + ratio * (p2.x - p1.x), p1.y + ratio * (p2.y - p1.y))
                    return pos, theta
                accum += seg_len
            last = segments[-1]
            return last[1], last[3]

        trajectory: list[TrajectoryPoint] = []
        t = 0.0
        while t < total_time - 1e-5:
            s = t * max_speed
            pos, theta = sample_at_s(s)
            trajectory.append(TrajectoryPoint(pos=pos, theta=theta, speed=max_speed, t=t))
            t += dt

        goal_pos, goal_theta = segments[-1][1], segments[-1][3]
        trajectory.append(TrajectoryPoint(pos=goal_pos, theta=goal_theta, speed=0.0, t=total_time))
        return trajectory


# Alias fungsi untuk kompatibilitas
interpolate_trajectory = TrajectoryInterpolator.interpolate


class Planner:
    """Perencana rute DAVG (PRD §4.3)."""

    def __init__(self, cfg=None):
        self.cfg = cfg

    def plan(self, scene: Scene, cfg=None) -> PlanResult:
        t0 = time.perf_counter()
        config = cfg if cfg is not None else self.cfg

        lambda_turn = getattr(config, "lambda_turn", 1.0) if config else 1.0
        max_speed = getattr(config, "max_speed", getattr(config, "v_max", 2.0)) if config else 2.0
        dt_mpc = getattr(config, "dt_mpc", getattr(config, "dt", 0.25)) if config else 0.25

        if hasattr(scene.robot, "pose"):
            start = scene.robot.pose.position
            theta = scene.robot.pose.theta
        elif hasattr(scene.robot, "position"):
            start = scene.robot.position
            theta = getattr(scene.robot, "theta", 0.0)
        else:
            start = Vec2(0, 0)
            theta = 0.0

        vg = VisibilityGraph(scene, config, start_theta=theta)
        adj, _ = vg.build()

        searcher = AugmentedSearch(
            start=start,
            goal=scene.goal,
            edges=adj,
            lambda_turn=lambda_turn,
            initial_heading=theta,
        )
        path = searcher.search()

        if path:
            status = PlanStatus.OK
            trajectory = TrajectoryInterpolator.interpolate(path, max_speed, dt_mpc)
        else:
            status = PlanStatus.NO_PATH
            path = []
            trajectory = []

        dt_ms = (time.perf_counter() - t0) * 1000.0

        return PlanResult(
            status=status,
            path=path,
            trajectory=trajectory,
            region=vg.active_region,
            edges=vg.edge_list,
            solve_time_ms=dt_ms,
        )
