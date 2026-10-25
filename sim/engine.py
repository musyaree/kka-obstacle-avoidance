"""Engine: satu-satunya tempat mesin status, dinamika, gerak obstacle, dan kejadian.

Murni, tanpa pygame. Seluruh masukan lewat Intent pada tick(); seluruh keluaran
berupa Frame. Mode MANUAL dan AUTO (v1.1) berjalan dengan DAVG dan Controller.
"""
from __future__ import annotations

import copy
import math
from typing import Optional

from .config import Config, Scenario, TimedEvent, build_obstacle
from .control import Controller, FollowerController
from .dynamics import Robot, RobotLimits
from .planning import Planner
from .types import Frame, Intent, Mode, PlanResult, PlanStatus, Pose, Scene, TrackResult, Vec2, Velocity

try:
    from .control import CfMpcController
except ImportError:
    CfMpcController = None


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
        self.planner = Planner(self.cfg)
        self.follower = FollowerController(self.cfg)
        self.cfmpc = CfMpcController(self.cfg) if CfMpcController is not None else None
        # cf-MPC bila cvxpy terpasang; tanpa itu pengikut sederhana (tombol C bergantian bila keduanya ada)
        self.controller: Controller = self.cfmpc if self.cfmpc is not None else self.follower
        self.last_plan: Optional[PlanResult] = None
        self.last_track: Optional[TrackResult] = None
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
        self.last_plan = None
        self.last_track = None
        self._replan_period = 0.1  # detik (10 Hz replan saat obstacle bergerak)
        self._time_since_plan = float("inf")

    # ------------------------------------------------------------------ tick ---

    def tick(self, dt: float, intent: Optional[Intent] = None) -> Frame:
        intent = intent or Intent()
        notice = self._apply_intent(intent)

        if not self.paused:
            self.time += dt
            self._time_since_plan += dt
            evt_notice = self._run_events()
            if evt_notice:
                notice = evt_notice

            for obstacle in self.scene.obstacles:
                obstacle.step(dt, self.scene.width, self.scene.height)

            # Cek apakah robot sampai di goal saat AUTO (jarak <= threshold)
            if self.mode is Mode.AUTO:
                if self.scene.robot.pose.position.dist(self.scene.goal) <= self.cfg.goal_threshold:
                    self.mode = Mode.MANUAL
                    notice = "Sampai di goal"

            if self.mode is Mode.MANUAL:
                self._step_robot(intent.manual, dt)
                # rencana tetap dihitung di MANUAL sebagai pratinjau lapisan 4-7
                self._refresh_plan(bool(intent.moves) or bool(evt_notice))
                self.last_track = None
            else:
                # Mode AUTO: DAVG merencanakan jalur jika sudah waktunya / ada perubahan
                self._refresh_plan(bool(intent.moves) or bool(evt_notice))

                if self.last_plan.status == PlanStatus.NO_PATH:
                    notice = "Tidak ada jalur"
                    self.last_track = self.controller.track(self.scene.robot, [], self.scene.obstacles)
                    self.scene.robot.step(Velocity(0.0, 0.0, 0.0), dt, self.scene.width, self.scene.height)
                else:
                    self.last_track = self.controller.track(
                        self.scene.robot, self.last_plan.trajectory, self.scene.obstacles
                    )
                    self.scene.robot.step(self.last_track.command, dt, self.scene.width, self.scene.height)

                    if self.scene.robot.pose.position.dist(self.scene.goal) <= self.cfg.goal_threshold:
                        self.mode = Mode.MANUAL
                        notice = "Sampai di goal"
        else:
            # Saat paused, jika ada perpindahan objek, perbarui plan (dan perintah bila AUTO)
            if intent.moves:
                self._refresh_plan(True)
                if self.mode is Mode.AUTO:
                    self.last_track = self.controller.track(
                        self.scene.robot, self.last_plan.trajectory, self.scene.obstacles
                    )

        return self._frame(notice)

    # --------------------------------------------------------------- internal ---

    def _refresh_plan(self, force: bool) -> None:
        """Hitung ulang rencana bila belum ada, ada perubahan, tanpa jalur, atau sudah waktunya."""
        if (
            self.last_plan is None
            or force
            or self.last_plan.status == PlanStatus.NO_PATH
            or self._time_since_plan >= self._replan_period
        ):
            self.last_plan = self.planner.plan(self.scene, self.cfg)
            self._time_since_plan = 0.0

    def _apply_intent(self, intent: Intent) -> str:
        notice = ""
        if intent.toggle_pause:
            self.paused = not self.paused

        if intent.switch_controller:
            if self.cfmpc is not None:
                self.controller = self.cfmpc if self.controller is self.follower else self.follower
                notice = f"Pengendali: {self.controller.name}"
            else:
                notice = "cf-MPC belum tersedia (v2.0); memakai pengikut sederhana"

        # Menekan WASD/QE saat AUTO: keluar dari mode AUTO kembali ke MANUAL
        is_manual_active = (
            abs(intent.manual.vx) > 1e-3
            or abs(intent.manual.vy) > 1e-3
            or abs(intent.manual.omega) > 1e-3
        )
        if self.mode is Mode.AUTO and is_manual_active:
            self.mode = Mode.MANUAL
            notice = "Kembali ke mode MANUAL"

        # Menekan Enter untuk toggle AUTO
        if intent.toggle_auto:
            if self.mode is Mode.MANUAL:
                self.mode = Mode.AUTO
                notice = "Masuk mode AUTO"
            else:
                self.mode = Mode.MANUAL
                notice = "Kembali ke mode MANUAL"

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
        controller_name = self.controller.name if self.mode is Mode.AUTO else "-"
        return Frame(
            scene=copy.deepcopy(self.scene),
            mode=self.mode,
            paused=self.paused,
            selected_id=self.selected_id,
            sim_time=self.time,
            controller_name=controller_name,
            notice=notice,
            plan=self.last_plan,
            track=self.last_track,
        )


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))
