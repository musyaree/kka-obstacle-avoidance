import heapq
import math
from typing import List, Dict, Tuple, Optional
from sim.types import Vec2, PlanStatus, PlanResult
from sim.geometry import wrap_angle
from sim.planning.graph import VisibilityGraph

def get_turn_angle(prev: Vec2, curr: Vec2, nxt: Vec2) -> float:
    """Menghitung sudut belok"""
    angle1 = math.atan2(curr.y - prev.y, curr.x - prev.x)
    angle2 = math.atan2(nxt.y - curr.y, nxt.x - curr.x)
    return wrap_angle(angle2 - angle1)

class AugmentedAStar:
    def __init__(self, start: Vec2, goal: Vec2, edges: Dict[Vec2, List[Tuple[Vec2, float]]], lambda_turn: float):
        self.start = start
        self.goal = goal
        self.edges = edges
        self.lambda_turn = lambda_turn

    def search(self) -> Optional[List[Vec2]]:
        """A* dengan augmented states (current, prev) untuk penalti sudut"""
        queue = [(0.0, 0.0, self.start, None, [self.start])]
        visited = set()

        while queue:
            f, g, curr, prev, path = heapq.heappop(queue)
            state = (curr, prev)

            if state in visited:
                continue
            visited.add(state)

            if curr == self.goal:
                return path

            for nxt, dist in self.edges.get(curr, []):
                turn_penalty = 0.0
                if prev is not None:
                    turn_angle = get_turn_angle(prev, curr, nxt)
                    turn_penalty = self.lambda_turn * abs(turn_angle)

                new_g = g + dist + turn_penalty
                h = nxt.dist(self.goal)
                new_f = new_g + h
                heapq.heappush(queue, (new_f, new_g, nxt, curr, path + [nxt]))

        return None

def interpolate_trajectory(path: List[Vec2], v_max: float, dt: float) -> List:
    """Interpolasi linear untuk diumpan ke cf-MPC"""
    if not path or v_max <= 0:
        return []

    trajectory = []

    for p in path:
        trajectory.append(p)
    return trajectory