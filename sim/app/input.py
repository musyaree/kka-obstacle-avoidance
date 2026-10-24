"""Menerjemahkan peristiwa pygame menjadi Intent (untuk Engine) dan UiCommands (untuk App)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import pygame

from ..types import Frame, Intent, Move, Select, SetVelocity, Vec2, Velocity
from .layers import ARROW_SECONDS, arrow_tip
from .viewport import Viewport

CLICK_PX = 4  # gerakan lebih kecil dari ini masih dianggap klik
HIT_SLACK_PX = 8
ARROW_HIT_PX = 12


@dataclass
class UiCommands:
    """Perintah yang tidak menyentuh simulasi, jadi tidak lewat Engine."""
    quit: bool = False
    reload: bool = False
    toggle_panel: bool = False
    layer_keys: list[str] = field(default_factory=list)


@dataclass
class _Drag:
    kind: str  # 'robot' | 'goal' | 'obstacle' | 'arrow'
    id: int
    offset: Vec2
    start_px: tuple[int, int]
    moved: bool = False


class InputMapper:
    def __init__(self, view: Viewport):
        self.view = view
        self._drag: Optional[_Drag] = None

    def cancel_drag(self) -> None:
        self._drag = None

    @staticmethod
    def manual_command(keys) -> Velocity:
        """Pecahan -1..1: W/S maju-mundur, A/D kiri-kanan, Q/E putar kiri-kanan."""
        return Velocity(
            float(keys[pygame.K_w]) - float(keys[pygame.K_s]),
            float(keys[pygame.K_a]) - float(keys[pygame.K_d]),
            float(keys[pygame.K_q]) - float(keys[pygame.K_e]),
        )

    def handle(self, event: pygame.event.Event, frame: Frame, intent: Intent, ui: UiCommands) -> None:
        if event.type == pygame.KEYDOWN:
            self._key(event, intent, ui)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            self._mouse_down(event.pos, frame, intent)
        elif event.type == pygame.MOUSEMOTION and self._drag is not None:
            self._mouse_move(event.pos, frame, intent)
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self._mouse_up(intent)

    # -------------------------------------------------------------------- keyboard ---

    @staticmethod
    def _key(event: pygame.event.Event, intent: Intent, ui: UiCommands) -> None:
        if event.key == pygame.K_ESCAPE:
            ui.quit = True
        elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            intent.toggle_auto = True
        elif event.key == pygame.K_SPACE:
            intent.toggle_pause = True
        elif event.key == pygame.K_TAB:
            ui.toggle_panel = True
        elif event.key == pygame.K_r:
            ui.reload = True
        elif event.key == pygame.K_c:
            intent.switch_controller = True
        elif event.unicode and event.unicode in "0123456789":
            ui.layer_keys.append(event.unicode)

    # ----------------------------------------------------------------------- mouse ---

    def _hit(self, pos: tuple[int, int], frame: Frame) -> Optional[_Drag]:
        view = self.view
        world = view.to_world(pos)
        scene = frame.scene
        slack = HIT_SLACK_PX / view.scale

        selected = scene.obstacle(frame.selected_id) if frame.selected_id is not None else None
        if selected is not None:
            tip_px = view.to_px(arrow_tip(selected.center, selected.velocity))
            if math.hypot(tip_px[0] - pos[0], tip_px[1] - pos[1]) <= ARROW_HIT_PX:
                return _Drag("arrow", selected.id, Vec2(), pos)

        robot = scene.robot.pose.position
        if world.dist(robot) <= scene.robot.radius + slack:
            return _Drag("robot", -1, world - robot, pos)
        if world.dist(scene.goal) <= 0.3 + slack:
            return _Drag("goal", -1, world - scene.goal, pos)
        for o in reversed(scene.obstacles):
            if world.dist(o.center) <= o.radius:
                return _Drag("obstacle", o.id, world - o.center, pos)
        return None

    def _mouse_down(self, pos: tuple[int, int], frame: Frame, intent: Intent) -> None:
        self._drag = self._hit(pos, frame)
        if self._drag is None:
            intent.select = Select(None)

    def _mouse_move(self, pos: tuple[int, int], frame: Frame, intent: Intent) -> None:
        drag = self._drag
        if not drag.moved and math.hypot(pos[0] - drag.start_px[0], pos[1] - drag.start_px[1]) > CLICK_PX:
            drag.moved = True
        if not drag.moved:
            return
        world = self.view.to_world(pos)
        if drag.kind == "arrow":
            obstacle = frame.scene.obstacle(drag.id)
            if obstacle is not None:
                velocity = (world - obstacle.center) * (1.0 / ARROW_SECONDS)
                intent.set_velocities.append(SetVelocity(drag.id, velocity))
        else:
            intent.moves.append(Move(drag.kind, world - drag.offset, drag.id))

    def _mouse_up(self, intent: Intent) -> None:
        drag, self._drag = self._drag, None
        if drag is not None and drag.kind == "obstacle" and not drag.moved:
            intent.select = Select(drag.id)
