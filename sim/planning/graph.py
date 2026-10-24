import math
from typing import List, Dict, Tuple
from sim.types import Vec2, Obstacle, Scene
from sim.geometry import point_in_polygon, segment_hits_polygon, wrap_angle

class VisibilityGraph:
    def __init__(self, scene: Scene):
        self.start = scene.robot.pose.position if hasattr(scene.robot, 'pose') else Vec2(0, 0)
        self.goal = scene.goal
        self.obstacles = scene.obstacles
        self.active_obstacles: List[Obstacle] = []
        self.nodes: List[Vec2] = []
        self.edges: Dict[Vec2, List[Tuple[Vec2, float]]] = {}
        self.region_bbox = None

    def _get_bbox(self, obs_list: List[Obstacle]) -> Tuple[float, float, float, float]:
        """Hitung bounding box dari start, goal, dan obstacle aktif"""
        min_x, max_x = min(self.start.x, self.goal.x), max(self.start.x, self.goal.x)
        min_y, max_y = min(self.start.y, self.goal.y), max(self.start.y, self.goal.y)
        for obs in obs_list:
            min_x, max_x = min(min_x, obs.center.x - obs.radius), max(max_x, obs.center.x + obs.radius)
            min_y, max_y = min(min_y, obs.center.y - obs.radius), max(max_y, obs.center.y + obs.radius)
        return min_x, max_x, min_y, max_y

    def _select_active_region(self):
        """Memilih rintangan aktif berdasarkan garis S-G"""
        pending = self.obstacles[:]
        active = []
        """Rintangan memotong S-G"""
        for obs in pending[:]:
            if segment_hits_polygon(self.start, self.goal, obs.vertices()):
                self.active_obstacles(obs)
                pending.remove(obs)

        """Memperluas rintangan berdasarkan bounding box"""
        changed = False
        bbox = self._get_bbox(active)
        min_x, max_x, min_y, max_y = bbox
        for obs in pending[:]:
            if not (obs.center.x + obs.radius < min_x or obs.center.x - obs.radius > max_x or obs.center.y + obs.radius < min_y or obs.center.y - obs.radius > max_y):
                active.append(obs)
                pending.remove(obs)
                changed = True
        self.active_obstacles = active
        self.region_bbox = self._get_bbox(active)

    def _is_covered(self, point: Vec2) -> bool:
        """Cek keberadaan vertex pada rintangan aktif lain"""
        for obs in self.active_obstacles:
            if point_in_polygon(point, obs.vertices()):
                return True
        return False

    def build(self) -> Tuple[Dict[Vec2, List[Tuple[Vec2, float]]], List[Obstacle]]:
        self._select_active_region()
        self.nodes = [self.start, self.goal]

        """Menambahkan vertex yang tidak tertutup"""
        active_polys = [obs.vertices() for obs in self.active_obstacles]
        for poly in active_polys:
            for v in poly:
                if not self._is_covered:
                    self.nodes.append(v)

        for n in self.nodes:
            self.edges[n] = []

        n_count = len(self.nodes)
        for i in range (n_count):
            for j in range (i+1, n_count):
                u, v = self.nodes[i], self.nodes[j]

                hits = False
                for poly in active_polys:
                    if segment_hits_polygon(u, v, poly):
                        hits = True
                        break
                    if not hits:
                        d = u.dist(v)
                        self.edges[u].append((v, d))
                        self.edges[v].append((u, d))

        return self.edges, self.active_obstacles