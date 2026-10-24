"""Lapisan visualisasi dan gambar dasar.

Lapisan didaftarkan dengan dekorator @register_layer; panel dan tombol angka
membaca daftar terdaftar itu, jadi menambah lapisan cukup menulis satu kelas.
Lapisan 4 sampai 9 sudah terdaftar tetapi kosong sampai modul penghasil
datanya (planning dan control) selesai.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import pygame

from ..config import Config
from ..types import Frame, Vec2
from .field import draw_markings
from .viewport import Viewport

ARROW_SECONDS = 1.0  # panjang panah kecepatan = kecepatan x 1 detik

BACKGROUND = (24, 28, 34)
FIELD = (42, 128, 56)
FIELD_BORDER = (240, 240, 240)
OBSTACLE_FILL = (200, 70, 70)
OBSTACLE_EDGE = (255, 150, 150)
SELECTED_EDGE = (255, 235, 120)
SAFE_ZONE = (255, 190, 90)
SG_LINE = (180, 200, 255)
ROBOT_FILL = (70, 130, 220)
ROBOT_EDGE = (200, 225, 255)
GOAL = (255, 90, 220)  # bukan putih, supaya tidak tertukar dengan garis lapangan
ARROW = (255, 235, 120)
TEXT = (235, 235, 235)


@dataclass
class RenderContext:
    surface: pygame.Surface
    view: Viewport
    cfg: Config
    fps: float
    font: pygame.font.Font


class Layer(ABC):
    id: str
    hotkey: str
    label: str
    default: bool

    def __init__(self) -> None:
        self.enabled = self.default

    @abstractmethod
    def draw(self, frame: Frame, ctx: RenderContext) -> None:
        ...


_REGISTERED: list[type[Layer]] = []


def register_layer(layer_id: str, hotkey: str, label: str, default: bool = False):
    def decorate(cls: type[Layer]) -> type[Layer]:
        cls.id, cls.hotkey, cls.label, cls.default = layer_id, hotkey, label, default
        _REGISTERED.append(cls)
        return cls
    return decorate


class LayerRegistry:
    def __init__(self, states: Optional[dict[str, bool]] = None):
        self.layers: list[Layer] = [cls() for cls in _REGISTERED]
        self.set_states(states or {})

    def set_states(self, states: dict[str, bool]) -> None:
        """Mengembalikan semua lapisan ke bawaan, lalu menimpa dengan nilai dari skenario."""
        for layer in self.layers:
            layer.enabled = states.get(layer.id, layer.default)

    def toggle(self, layer_id: str) -> None:
        for layer in self.layers:
            if layer.id == layer_id:
                layer.enabled = not layer.enabled

    def toggle_by_key(self, key: str) -> bool:
        for layer in self.layers:
            if layer.hotkey == key:
                layer.enabled = not layer.enabled
                return True
        return False

    def draw_all(self, frame: Frame, ctx: RenderContext) -> None:
        for layer in self.layers:
            if layer.enabled:
                layer.draw(frame, ctx)


# --------------------------------------------------------------- lapisan ---

@register_layer("obstacle", "1", "Obstacle", default=True)
class ObstacleLayer(Layer):
    def draw(self, frame: Frame, ctx: RenderContext) -> None:
        for o in frame.scene.obstacles:
            points = [ctx.view.to_px(v) for v in o.vertices()]
            pygame.draw.polygon(ctx.surface, OBSTACLE_FILL, points)
            selected = o.id == frame.selected_id
            pygame.draw.polygon(ctx.surface, SELECTED_EDGE if selected else OBSTACLE_EDGE,
                                points, 3 if selected else 1)
            label = ctx.font.render(f"#{o.id}", True, TEXT)
            cx, cy = ctx.view.to_px(o.center)
            ctx.surface.blit(label, (cx - label.get_width() // 2, cy - label.get_height() // 2))


@register_layer("safe_zone", "2", "Zona aman", default=True)
class SafeZoneLayer(Layer):
    def draw(self, frame: Frame, ctx: RenderContext) -> None:
        for o in frame.scene.obstacles:
            radius = round(ctx.view.meters_to_px(ctx.cfg.safe_radius(o)))
            pygame.draw.circle(ctx.surface, SAFE_ZONE, ctx.view.to_px(o.center), radius, 1)


@register_layer("sg_line", "3", "Garis S-G", default=True)
class SGLineLayer(Layer):
    def draw(self, frame: Frame, ctx: RenderContext) -> None:
        start = ctx.view.to_px(frame.scene.robot.pose.position)
        goal = ctx.view.to_px(frame.scene.goal)
        pygame.draw.line(ctx.surface, SG_LINE, start, goal, 1)


class _EmptyLayer(Layer):
    """Menunggu modul penghasil datanya (planning atau control)."""

    def draw(self, frame: Frame, ctx: RenderContext) -> None:
        return


@register_layer("active_region", "4", "Active region")
class ActiveRegionLayer(_EmptyLayer):
    pass


@register_layer("visibility_graph", "5", "Visibility graph")
class VisibilityGraphLayer(_EmptyLayer):
    pass


@register_layer("path", "6", "Jalur DAVG", default=True)
class PathLayer(_EmptyLayer):
    pass


@register_layer("trajectory", "7", "Trajektori")
class TrajectoryLayer(_EmptyLayer):
    pass


@register_layer("horizon", "8", "Horizon MPC")
class HorizonLayer(_EmptyLayer):
    pass


@register_layer("tangent_planes", "9", "Bidang singgung")
class TangentPlaneLayer(_EmptyLayer):
    pass


@register_layer("hud", "0", "HUD", default=True)
class HudLayer(Layer):
    def draw(self, frame: Frame, ctx: RenderContext) -> None:
        plan, track = frame.plan, frame.track
        davg = f"{plan.solve_time_ms:.2f} ms" if plan else "-"
        mpc = f"{track.solve_time_ms:.2f} ms" if track else "-"
        slack = f"{track.slack:.3f}" if track else "-"
        lines = [
            f"Mode: {frame.mode.value}" + ("   [JEDA]" if frame.paused else ""),
            f"Pengendali: {frame.controller_name}",
            f"FPS: {ctx.fps:.0f}   Waktu: {frame.sim_time:.1f} s",
            f"DAVG: {davg}   MPC: {mpc}   slack: {slack}",
        ]
        x0, y0, _, _ = ctx.view.field_rect()
        for i, text in enumerate(lines):
            ctx.surface.blit(ctx.font.render(text, True, TEXT), (x0 + 10, y0 + 8 + i * 18))


# ------------------------------------------------------------ gambar dasar ---

def draw_background(ctx: RenderContext) -> None:
    ctx.surface.fill(BACKGROUND)
    x, y, w, h = ctx.view.field_rect()
    pygame.draw.rect(ctx.surface, FIELD, (x, y, w, h))
    draw_markings(ctx.surface, ctx.view)
    pygame.draw.rect(ctx.surface, FIELD_BORDER, (x, y, w, h), max(2, round(0.05 * ctx.view.scale)))


def draw_foreground(frame: Frame, ctx: RenderContext) -> None:
    """Goal, robot, dan panah kecepatan obstacle terpilih; selalu tampil."""
    view, surface = ctx.view, ctx.surface

    gx, gy = view.to_px(frame.scene.goal)
    pygame.draw.circle(surface, GOAL, (gx, gy), 9, 2)
    pygame.draw.line(surface, GOAL, (gx - 13, gy), (gx + 13, gy), 2)
    pygame.draw.line(surface, GOAL, (gx, gy - 13), (gx, gy + 13), 2)

    robot = frame.scene.robot
    center = view.to_px(robot.pose.position)
    radius = max(3, round(view.meters_to_px(robot.radius)))
    pygame.draw.circle(surface, ROBOT_FILL, center, radius)
    pygame.draw.circle(surface, ROBOT_EDGE, center, radius, 2)
    heading = Vec2(robot.pose.position.x + robot.radius * math.cos(robot.pose.theta),
                   robot.pose.position.y + robot.radius * math.sin(robot.pose.theta))
    pygame.draw.line(surface, ROBOT_EDGE, center, view.to_px(heading), 3)

    selected = frame.scene.obstacle(frame.selected_id) if frame.selected_id is not None else None
    if selected is not None:
        tip = arrow_tip(selected.center, selected.velocity)
        start_px, tip_px = view.to_px(selected.center), view.to_px(tip)
        pygame.draw.line(surface, ARROW, start_px, tip_px, 3)
        pygame.draw.circle(surface, ARROW, tip_px, 7)


def arrow_tip(center: Vec2, velocity: Vec2) -> Vec2:
    return center + velocity * ARROW_SECONDS
