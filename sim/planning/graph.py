"""Visibility graph untuk DAVG (PRD §4.2).

Membangun graf keterlihatan dari start, goal, dan sudut-sudut obstacle aktif.
Wilayah aktif dipilih dari obstacle yang memotong garis S-G lalu diperluas
berulang sampai tidak ada obstacle baru yang memotong bounding box (titik tetap).
Jika start atau goal berada di dalam obstacle, dipilih vertex keluar terdekat
searah heading / arah kedatangan (PRD §4.2.5).
"""
from __future__ import annotations

import math
from typing import Optional

from sim.geometry import point_in_polygon, regular_polygon, segment_hits_polygon
from sim.types import ActiveRegion, Obstacle, Scene, Vec2


class ActiveRegionSelector:
    """Memilih dan memperluas rintangan aktif berdasarkan garis S-G (PRD §4.2.1)."""

    def __init__(
        self,
        start: Vec2,
        goal: Vec2,
        obstacles: list[Obstacle],
        cfg=None,
    ):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.cfg = cfg

    def _get_obs_radius(self, obs: Obstacle) -> float:
        if self.cfg is not None:
            if hasattr(self.cfg, "safe_radius"):
                return self.cfg.safe_radius(obs)
            robot_radius = getattr(self.cfg, "robot_radius", 0.0)
            safety_margin = getattr(self.cfg, "safety_margin", 0.0)
            return obs.radius + robot_radius + safety_margin
        return obs.radius

    def _get_obs_poly(self, obs: Obstacle) -> list[Vec2]:
        r = self._get_obs_radius(obs)
        if abs(r - obs.radius) < 1e-6:
            return obs.vertices()
        return regular_polygon(obs.center, r, obs.sides)

    def _get_bbox(self, obs_list: list[Obstacle]) -> tuple[float, float, float, float]:
        """Hitung bounding box dari start, goal, dan obstacle aktif."""
        min_x = min(self.start.x, self.goal.x)
        max_x = max(self.start.x, self.goal.x)
        min_y = min(self.start.y, self.goal.y)
        max_y = max(self.start.y, self.goal.y)
        for obs in obs_list:
            r = self._get_obs_radius(obs)
            min_x = min(min_x, obs.center.x - r)
            max_x = max(max_x, obs.center.x + r)
            min_y = min(min_y, obs.center.y - r)
            max_y = max(max_y, obs.center.y + r)
        return min_x, max_x, min_y, max_y

    def select(self) -> tuple[list[Obstacle], ActiveRegion]:
        pending = list(self.obstacles)
        active: list[Obstacle] = []

        # Obstacle yang memotong segmen S-G, atau memuat start/goal
        for obs in list(pending):
            poly = self._get_obs_poly(obs)
            if (
                point_in_polygon(self.start, poly)
                or point_in_polygon(self.goal, poly)
                or segment_hits_polygon(self.start, self.goal, poly)
            ):
                active.append(obs)
                pending.remove(obs)

        if not active:
            return [], ActiveRegion(None, [])

        # Perluas berulang sampai titik tetap (fixed point)
        while True:
            changed = False
            min_x, max_x, min_y, max_y = self._get_bbox(active)
            for obs in list(pending):
                r = self._get_obs_radius(obs)
                obs_min_x = obs.center.x - r
                obs_max_x = obs.center.x + r
                obs_min_y = obs.center.y - r
                obs_max_y = obs.center.y + r
                if not (
                    obs_max_x < min_x
                    or obs_min_x > max_x
                    or obs_max_y < min_y
                    or obs_min_y > max_y
                ):
                    active.append(obs)
                    pending.remove(obs)
                    changed = True
            if not changed:
                break

        bbox = self._get_bbox(active)
        return active, ActiveRegion(bbox, [o.id for o in active])


class VisibilityGraph:
    """Graf keterlihatan untuk DAVG."""

    def __init__(
        self,
        scene: Scene,
        cfg=None,
        start_theta: Optional[float] = None,
    ):
        if hasattr(scene.robot, "pose"):
            self.start = scene.robot.pose.position
            self.start_theta = scene.robot.pose.theta if start_theta is None else start_theta
        elif hasattr(scene.robot, "position"):
            self.start = scene.robot.position
            self.start_theta = getattr(scene.robot, "theta", 0.0) if start_theta is None else start_theta
        else:
            self.start = Vec2(0, 0)
            self.start_theta = 0.0 if start_theta is None else start_theta

        self.goal = scene.goal
        self.obstacles = scene.obstacles
        self.cfg = cfg

        self.active_obstacles: list[Obstacle] = []
        self.active_region: Optional[ActiveRegion] = None
        self.nodes: list[Vec2] = []
        self.adj: dict[Vec2, list[tuple[Vec2, float]]] = {}
        self.edge_list: list[tuple[Vec2, Vec2]] = []

    @property
    def edges(self) -> dict[Vec2, list[tuple[Vec2, float]]]:
        """Kompatibilitas mundur dengan kode yang membaca vg.edges sebagai dict."""
        return self.adj

    @property
    def region_bbox(self) -> Optional[tuple[float, float, float, float]]:
        """Kompatibilitas mundur dengan kode yang membaca vg.region_bbox."""
        return self.active_region.bbox if self.active_region else None

    def _get_obs_radius(self, obs: Obstacle) -> float:
        if self.cfg is not None:
            if hasattr(self.cfg, "safe_radius"):
                return self.cfg.safe_radius(obs)
            robot_radius = getattr(self.cfg, "robot_radius", 0.0)
            safety_margin = getattr(self.cfg, "safety_margin", 0.0)
            return obs.radius + robot_radius + safety_margin
        return obs.radius

    def _get_obs_poly(self, obs: Obstacle) -> list[Vec2]:
        r = self._get_obs_radius(obs)
        if abs(r - obs.radius) < 1e-6:
            return obs.vertices()
        return regular_polygon(obs.center, r, obs.sides)

    def _is_covered(self, point: Vec2, current_obs: Obstacle) -> bool:
        """Cek apakah point berada di dalam poligon obstacle aktif lainnya."""
        for other in self.active_obstacles:
            if other.id != current_obs.id:
                if point_in_polygon(point, self._get_obs_poly(other)):
                    return True
        return False

    def _find_exit_vertex(self, point: Vec2, poly: list[Vec2], direction: Vec2) -> Vec2:
        """Mencari vertex keluar terdekat searah dengan direction."""
        best_v = None
        best_dist = float("inf")
        fallback_v = None
        max_dot = float("-inf")

        for v in poly:
            diff = v - point
            d = diff.norm()
            if d < 1e-9:
                continue
            dot = (diff.x * direction.x + diff.y * direction.y) / d
            if dot > max_dot:
                max_dot = dot
                fallback_v = v
            if dot > 0.0:
                if d < best_dist:
                    best_dist = d
                    best_v = v

        if best_v is not None:
            return best_v
        if fallback_v is not None:
            return fallback_v
        return poly[0]

    def build(self) -> tuple[dict[Vec2, list[tuple[Vec2, float]]], list[Obstacle]]:
        selector = ActiveRegionSelector(self.start, self.goal, self.obstacles, self.cfg)
        self.active_obstacles, self.active_region = selector.select()

        # Deteksi start atau goal di dalam obstacle aktif
        start_obs: Optional[Obstacle] = None
        goal_obs: Optional[Obstacle] = None
        for obs in self.active_obstacles:
            poly = self._get_obs_poly(obs)
            if point_in_polygon(self.start, poly):
                start_obs = obs
            if point_in_polygon(self.goal, poly):
                goal_obs = obs

        v_exit: Optional[Vec2] = None
        if start_obs is not None:
            h = Vec2(math.cos(self.start_theta), math.sin(self.start_theta))
            v_exit = self._find_exit_vertex(self.start, self._get_obs_poly(start_obs), h)

        v_entry: Optional[Vec2] = None
        if goal_obs is not None:
            arr_dir = self.start - self.goal
            if arr_dir.norm() < 1e-9:
                arr_dir = Vec2(-1.0, 0.0)
            v_entry = self._find_exit_vertex(self.goal, self._get_obs_poly(goal_obs), arr_dir)

        # Kumpulkan node
        self.nodes = [self.start, self.goal]
        if v_exit is not None and not any(n.dist(v_exit) < 1e-6 for n in self.nodes):
            self.nodes.append(v_exit)
        if v_entry is not None and not any(n.dist(v_entry) < 1e-6 for n in self.nodes):
            self.nodes.append(v_entry)

        for obs in self.active_obstacles:
            poly = self._get_obs_poly(obs)
            for v in poly:
                if not self._is_covered(v, obs):
                    if not any(n.dist(v) < 1e-6 for n in self.nodes):
                        self.nodes.append(v)

        self.adj = {n: [] for n in self.nodes}
        self.edge_list = []

        # Sambungan khusus jika start / goal di dalam obstacle
        if start_obs is not None and v_exit is not None:
            d = self.start.dist(v_exit)
            self.adj[self.start].append((v_exit, d))
            self.adj[v_exit].append((self.start, d))
            self.edge_list.append((self.start, v_exit))

        if goal_obs is not None and v_entry is not None:
            d = v_entry.dist(self.goal)
            self.adj[v_entry].append((self.goal, d))
            self.adj[self.goal].append((v_entry, d))
            self.edge_list.append((v_entry, self.goal))

        # Jika start dan goal di dalam obstacle yang sama dan bisa terhubung langsung
        if start_obs is not None and goal_obs is not None and start_obs.id == goal_obs.id:
            other_polys = [
                self._get_obs_poly(o) for o in self.active_obstacles if o.id != start_obs.id
            ]
            if not any(segment_hits_polygon(self.start, self.goal, p) for p in other_polys):
                d = self.start.dist(self.goal)
                self.adj[self.start].append((self.goal, d))
                self.adj[self.goal].append((self.start, d))
                self.edge_list.append((self.start, self.goal))

        active_polys = [self._get_obs_poly(obs) for obs in self.active_obstacles]

        n_count = len(self.nodes)
        for i in range(n_count):
            for j in range(i + 1, n_count):
                u, v = self.nodes[i], self.nodes[j]

                # Jangan hubungkan u atau v ke start/goal jika start/goal berada di dalam obstacle
                # karena sudah ditangani lewat v_exit / v_entry
                if start_obs is not None and (u == self.start or v == self.start):
                    continue
                if goal_obs is not None and (u == self.goal or v == self.goal):
                    continue

                hits = False
                for poly in active_polys:
                    if segment_hits_polygon(u, v, poly):
                        hits = True
                        break

                if not hits:
                    d = u.dist(v)
                    self.adj[u].append((v, d))
                    self.adj[v].append((u, d))
                    self.edge_list.append((u, v))

        return self.adj, self.active_obstacles
