"""Panel samping: kotak centang lapisan, status singkat, dan isian kecepatan obstacle."""
from __future__ import annotations

import math
from typing import Optional

import pygame

from ..types import Frame, SetVelocity, Vec2
from .layers import LayerRegistry

PANEL_WIDTH = 300

BG = (38, 43, 52)
BORDER = (70, 78, 92)
TEXT = (235, 235, 235)
MUTED = (150, 158, 170)
ACCENT = (255, 235, 120)
FIELD_BG = (24, 28, 34)
CHECK = (110, 200, 140)

ROW_HEIGHT = 26
FIELD_CHARS = "0123456789.-"


class NumberField:
    """Isian angka sederhana; pygame tidak punya widget teks bawaan."""

    def __init__(self, label: str):
        self.label = label
        self.text = ""
        self.focused = False
        self.rect = pygame.Rect(0, 0, 0, 0)
        self._backup = ""

    def sync(self, value: float) -> None:
        if not self.focused:
            self.text = f"{value:.2f}"

    def focus(self) -> None:
        self.focused = True
        self._backup = self.text
        self.text = ""

    def blur(self) -> None:
        if self.focused:
            self.focused = False
            self.text = self._backup

    def parse(self) -> Optional[float]:
        try:
            value = float(self.text)
        except ValueError:
            return None
        return value if math.isfinite(value) else None


class Panel:
    def __init__(self, registry: LayerRegistry, font: pygame.font.Font, title_font: pygame.font.Font):
        self.registry = registry
        self.font = font
        self.title_font = title_font
        self.visible = True
        self.fields = {"vx": NumberField("vx (m/s)"), "vy": NumberField("vy (m/s)")}
        self._owner: Optional[int] = None
        self._row_rects: list[tuple[object, pygame.Rect]] = []

    @property
    def editing(self) -> bool:
        return any(f.focused for f in self.fields.values())

    def rect(self, window_size: tuple[int, int]) -> pygame.Rect:
        return pygame.Rect(window_size[0] - PANEL_WIDTH, 0, PANEL_WIDTH, window_size[1])

    def toggle(self) -> None:
        self.visible = not self.visible
        self.blur_all()

    def blur_all(self) -> None:
        for f in self.fields.values():
            f.blur()

    # ----------------------------------------------------------------- tata letak ---

    def _layout(self, window_size: tuple[int, int], frame: Frame) -> None:
        if frame.selected_id != self._owner:  # pilihan berganti: batalkan ketikan
            self.blur_all()
            self._owner = frame.selected_id
        x0 = self.rect(window_size).x
        y = 44
        self._row_rects = []
        for layer in self.registry.layers:
            self._row_rects.append((layer, pygame.Rect(x0 + 10, y, PANEL_WIDTH - 20, ROW_HEIGHT)))
            y += ROW_HEIGHT
        self._status_y = y + 16
        self._select_y = self._status_y + 112
        field_y = self._select_y + 54
        for i, f in enumerate(self.fields.values()):
            f.rect = pygame.Rect(x0 + 110, field_y + i * 34, 120, 26)
        selected = frame.scene.obstacle(frame.selected_id) if frame.selected_id is not None else None
        if selected is not None:
            self.fields["vx"].sync(selected.velocity.x)
            self.fields["vy"].sync(selected.velocity.y)

    # --------------------------------------------------------------------- masukan ---

    def handle_event(self, event: pygame.event.Event, frame: Frame,
                     window_size: tuple[int, int]) -> tuple[bool, list[SetVelocity]]:
        """Mengembalikan (apakah panel memakai event ini, perubahan kecepatan yang dikomit)."""
        if not self.visible:
            return False, []
        self._layout(window_size, frame)
        commands: list[SetVelocity] = []

        if event.type == pygame.KEYDOWN and self.editing:
            field = next(f for f in self.fields.values() if f.focused)
            if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER, pygame.K_TAB):
                commands += self._commit(field, frame)
            elif event.key == pygame.K_ESCAPE:
                field.blur()
            elif event.key == pygame.K_BACKSPACE:
                field.text = field.text[:-1]
            elif event.unicode and event.unicode in FIELD_CHARS and len(field.text) < 8:
                field.text += event.unicode
            return True, commands

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            inside = self.rect(window_size).collidepoint(event.pos)
            clicked = None
            if inside and frame.selected_id is not None:
                clicked = next((f for f in self.fields.values() if f.rect.collidepoint(event.pos)), None)
            for field in self.fields.values():
                if field.focused and field is not clicked:
                    commands += self._commit(field, frame)
            if clicked is not None:
                clicked.focus()
            elif inside:
                for layer, rect in self._row_rects:
                    if rect.collidepoint(event.pos):
                        layer.enabled = not layer.enabled
            return inside, commands

        return False, commands

    def _commit(self, field: NumberField, frame: Frame) -> list[SetVelocity]:
        value = field.parse()
        field.blur()
        selected = frame.scene.obstacle(frame.selected_id) if frame.selected_id is not None else None
        if value is None or selected is None:
            return []
        vx, vy = selected.velocity.x, selected.velocity.y
        if field is self.fields["vx"]:
            vx = value
        else:
            vy = value
        return [SetVelocity(selected.id, Vec2(vx, vy))]

    # --------------------------------------------------------------------- gambar ---

    def draw(self, surface: pygame.Surface, frame: Frame) -> None:
        if not self.visible:
            return
        window_size = surface.get_size()
        self._layout(window_size, frame)
        panel = self.rect(window_size)
        pygame.draw.rect(surface, BG, panel)
        pygame.draw.line(surface, BORDER, panel.topleft, panel.bottomleft, 2)
        x0 = panel.x

        self._text(surface, self.title_font, "Lapisan", (x0 + 16, 14), ACCENT)
        for layer, rect in self._row_rects:
            box = pygame.Rect(rect.x + 4, rect.y + 5, 16, 16)
            pygame.draw.rect(surface, FIELD_BG, box)
            pygame.draw.rect(surface, BORDER, box, 1)
            if layer.enabled:
                pygame.draw.rect(surface, CHECK, box.inflate(-6, -6))
            self._text(surface, self.font, f"[{layer.hotkey}] {layer.label}", (rect.x + 30, rect.y + 5),
                       TEXT if layer.enabled else MUTED)

        y = self._status_y
        self._text(surface, self.title_font, "Status", (x0 + 16, y), ACCENT)
        status = [
            f"Mode: {frame.mode.value}" + ("  (jeda)" if frame.paused else ""),
            f"Pengendali: {frame.controller_name}",
            f"Waktu simulasi: {frame.sim_time:.1f} s",
            f"Obstacle: {len(frame.scene.obstacles)}",
        ]
        for i, line in enumerate(status):
            self._text(surface, self.font, line, (x0 + 16, y + 26 + i * 20), TEXT)

        self._text(surface, self.title_font, "Obstacle terpilih", (x0 + 16, self._select_y), ACCENT)
        if frame.selected_id is None:
            self._text(surface, self.font, "Klik obstacle untuk memilih", (x0 + 16, self._select_y + 28), MUTED)
        else:
            self._text(surface, self.font, f"#{frame.selected_id}", (x0 + 16, self._select_y + 28), TEXT)
            for field in self.fields.values():
                self._text(surface, self.font, field.label, (x0 + 16, field.rect.y + 5), TEXT)
                pygame.draw.rect(surface, FIELD_BG, field.rect)
                pygame.draw.rect(surface, ACCENT if field.focused else BORDER, field.rect, 2 if field.focused else 1)
                shown = field.text + ("|" if field.focused else "")
                self._text(surface, self.font, shown, (field.rect.x + 8, field.rect.y + 5), TEXT)

        hints = ["WASD/QE: gerak   Enter: AUTO", "Spasi: jeda   R: muat ulang", "C: pengendali   Tab: panel"]
        for i, line in enumerate(hints):
            self._text(surface, self.font, line, (x0 + 16, panel.bottom - 70 + i * 20), MUTED)

    @staticmethod
    def _text(surface: pygame.Surface, font: pygame.font.Font, text: str, pos: tuple[int, int],
              color: tuple[int, int, int]) -> None:
        surface.blit(font.render(text, True, color), pos)
