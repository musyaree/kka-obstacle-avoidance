"""Engine: satu-satunya tempat mesin status, dinamika, gerak obstacle, dan kejadian.

Murni, tanpa pygame. Seluruh masukan lewat Intent pada tick(); seluruh keluaran
berupa Frame. Pada MVP hanya mode MANUAL yang berjalan; AUTO menunggu planning
(v1.1) dan control (v2.0).
"""
from __future__ import annotations

import copy
import math
from typing import Optional

from .config import Config, Scenario, TimedEvent, build_obstacle
from .dynamics import Robot, RobotLimits
from .types import Frame, Intent, Mode, Pose, Scene, Vec2, Velocity


class EventScheduler:
    """Menyerahkan kejadian berwaktu yang sudah jatuh tempo, sekali saja."""

    def __init__(self, events: list[TimedEvent]):
        self.reset(events)

    def reset(self, events: list[TimedEvent]) -> None:
        self._events = sorted(events, key=lambda e: e.t)
        self._next = 0

    def due(self, now: float) -> list[TimedEvent]:
        out = []
        while self._next < len(self._events) and self._events[self._next].t <= now:
            out.append(self._events[self._next])
            self._next += 1
        return out


class Engine:
    def __init__(self, cfg: Config, scenario: Scenario):
        self.cfg = cfg
        self._scheduler = EventScheduler([])
        self.reset(scenario)

    # ----------------------------------------------------------------- reset ---

    def reset(self, scenario: Scenario) -> None:
        """Mengembalikan keadaan ke awal skenario (dipakai juga oleh tombol R)."""
        cfg = self.cfg
        limits = RobotLimits(cfg.max_speed, cfg.max_accel, cfg.max_omega, cfg.max_alpha)
        robot = Robot(scenario.start, cfg.robot_radius, limits)
        obstacles = [copy.deepcopy(o) for o in scenario.obstacles]
        self.scene = Scene(cfg.field_width, cfg.field_height, robot, scenario.goal, obstacles)
        self._next_id = max((o.id for o in obstacles), default=-1) + 1
        self._scheduler.reset(scenario.events)
        self.mode = Mode.MANUAL
        self.paused = False
        self.selected_id: Optional[int] = None
        self.time = 0.0

    # ------------------------------------------------------------------ tick ---

    def tick(self, dt: float, intent: Optional[Intent] = None) -> Frame:
        intent = intent or Intent()
        notice = self._apply_intent(intent)
        if not self.paused:
            self.time += dt
            notice = self._run_events() or notice
            for obstacle in self.scene.obstacles:
                obstacle.step(dt, self.scene.width, self.scene.height)
            self._step_robot(intent.manual, dt)
        return self._frame(notice)

    # --------------------------------------------------------------- internal ---

    def _apply_intent(self, intent: Intent) -> str:
        notice = ""
        if intent.toggle_pause:
            self.paused = not self.paused
        if intent.toggle_auto:
            notice = "Mode AUTO belum tersedia pada MVP (menunggu DAVG dan cf-MPC)"
        if intent.switch_controller:
            notice = "Pengendali belum tersedia pada MVP"
        for move in intent.moves:
            self._apply_move(move.kind, move.pos, move.id)
        for sv in intent.set_velocities:
            obstacle = self.scene.obstacle(sv.id)
            if obstacle is not None:
                obstacle.set_velocity(sv.velocity, self.cfg.obstacle_max_speed)
        if intent.select is not None:
            wanted = intent.select.id
            self.selected_id = wanted if wanted is None or self.scene.obstacle(wanted) else None
        return notice

    def _apply_move(self, kind: str, pos: Vec2, obstacle_id: int) -> None:
        scene = self.scene
        if kind == "robot":
            r = scene.robot.radius
            old = scene.robot.pose
            scene.robot.teleport(Pose(_clamp(pos.x, r, scene.width - r),
                                      _clamp(pos.y, r, scene.height - r), old.theta))
        elif kind == "goal":
            scene.goal = Vec2(_clamp(pos.x, 0.0, scene.width), _clamp(pos.y, 0.0, scene.height))
        elif kind == "obstacle":
            obstacle = scene.obstacle(obstacle_id)
            if obstacle is not None:
                r = obstacle.radius
                obstacle.center = Vec2(_clamp(pos.x, r, scene.width - r), _clamp(pos.y, r, scene.height - r))

    def _run_events(self) -> str:
        notice = ""
        for event in self._scheduler.due(self.time):
            if event.action == "munculkan_obstacle":
                obstacle = build_obstacle(event.params["obstacle"], self._next_id, self.cfg, "kejadian")
                self._next_id += 1
                self.scene.obstacles.append(obstacle)
                notice = f"Obstacle #{obstacle.id} muncul"
            elif event.action == "hapus_obstacle":
                target = event.params["id"]
                before = len(self.scene.obstacles)
                self.scene.obstacles = [o for o in self.scene.obstacles if o.id != target]
                if len(self.scene.obstacles) < before:
                    notice = f"Obstacle #{target} dihapus"
                    if self.selected_id == target:
                        self.selected_id = None
        return notice

    def _step_robot(self, fraction: Velocity, dt: float) -> None:
        """MANUAL: pecahan -1..1 dari WASD/QE diubah menjadi perintah kecepatan."""
        cfg = self.cfg
        fx, fy = fraction.vx, fraction.vy
        norm = math.hypot(fx, fy)
        if norm > 1.0:
            fx, fy = fx / norm, fy / norm
        scale = cfg.manual_speed_ratio
        cmd = Velocity(fx * cfg.max_speed * scale, fy * cfg.max_speed * scale,
                       fraction.omega * cfg.max_omega * scale)
        self.scene.robot.step(cmd, dt, self.scene.width, self.scene.height)

    def _frame(self, notice: str) -> Frame:
        return Frame(
            scene=copy.deepcopy(self.scene),
            mode=self.mode,
            paused=self.paused,
            selected_id=self.selected_id,
            sim_time=self.time,
            notice=notice,
        )


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))
