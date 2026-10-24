"""Garis lapangan RoboCup Humanoid League kelas AdultSize.

Ukuran dari Tabel 1 aturan RoboCup Humanoid League 2024 (14 m x 9 m):
https://humanoid.robocup.org/wp-content/uploads/RC-HL-2024-Rules.pdf
"""
from __future__ import annotations

import pygame

from ..types import Vec2
from .viewport import Viewport

LINE_COLOR = (240, 240, 240)
LINE_WIDTH_M = 0.05  # garis kira-kira 5 cm menurut aturan

GOAL_WIDTH = 2.6  # jarak antartiang
GOAL_AREA_DEPTH = 1.0
GOAL_AREA_WIDTH = 4.0
PENALTY_AREA_DEPTH = 3.0
PENALTY_AREA_WIDTH = 6.0
PENALTY_MARK_DISTANCE = 2.1
CENTER_CIRCLE_RADIUS = 1.5  # diameter 3 m


def draw_markings(surface: pygame.Surface, view: Viewport) -> None:
    width, height = view.field_width, view.field_height
    mid_y = height / 2.0
    line = max(2, round(LINE_WIDTH_M * view.scale))
    px = view.to_px

    # garis tengah dan lingkaran tengah
    pygame.draw.line(surface, LINE_COLOR, px(Vec2(width / 2, 0)), px(Vec2(width / 2, height)), line)
    pygame.draw.circle(surface, LINE_COLOR, px(Vec2(width / 2, mid_y)),
                       round(view.meters_to_px(CENTER_CIRCLE_RADIUS)), line)
    pygame.draw.circle(surface, LINE_COLOR, px(Vec2(width / 2, mid_y)), max(3, line + 1))

    for side in (0, 1):
        # x_goal: garis gawang; arah: +1 masuk ke lapangan dari kiri, -1 dari kanan
        x_goal, direction = (0.0, 1.0) if side == 0 else (width, -1.0)
        for depth, area_width in ((GOAL_AREA_DEPTH, GOAL_AREA_WIDTH), (PENALTY_AREA_DEPTH, PENALTY_AREA_WIDTH)):
            a = px(Vec2(x_goal, mid_y - area_width / 2))
            b = px(Vec2(x_goal + direction * depth, mid_y + area_width / 2))
            rect = pygame.Rect(min(a[0], b[0]), min(a[1], b[1]), abs(a[0] - b[0]), abs(a[1] - b[1]))
            pygame.draw.rect(surface, LINE_COLOR, rect, line)

        mark = px(Vec2(x_goal + direction * PENALTY_MARK_DISTANCE, mid_y))
        pygame.draw.circle(surface, LINE_COLOR, mark, max(3, line + 1))

        for post_y in (mid_y - GOAL_WIDTH / 2, mid_y + GOAL_WIDTH / 2):
            pygame.draw.circle(surface, LINE_COLOR, px(Vec2(x_goal, post_y)), max(4, line + 2))
            pygame.draw.circle(surface, (30, 30, 30), px(Vec2(x_goal, post_y)), max(4, line + 2), 1)
