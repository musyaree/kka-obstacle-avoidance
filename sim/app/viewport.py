"""Konversi koordinat dunia (meter, y ke atas) ke piksel layar (y ke bawah)."""
from __future__ import annotations

from ..types import Vec2


class Viewport:
    def __init__(self, field_width: float, field_height: float):
        self.field_width = field_width
        self.field_height = field_height
        self.scale = 1.0  # piksel per meter
        self._ox = 0.0
        self._oy = 0.0

    def fit(self, x: int, y: int, width: int, height: int, margin: int = 20) -> None:
        """Memuat seluruh lapangan di dalam area piksel, di tengah."""
        avail_w = max(1, width - 2 * margin)
        avail_h = max(1, height - 2 * margin)
        self.scale = min(avail_w / self.field_width, avail_h / self.field_height)
        self._ox = x + (width - self.field_width * self.scale) / 2.0
        self._oy = y + (height - self.field_height * self.scale) / 2.0

    def to_px(self, p: Vec2) -> tuple[int, int]:
        return (round(self._ox + p.x * self.scale),
                round(self._oy + (self.field_height - p.y) * self.scale))

    def to_world(self, px: tuple[int, int]) -> Vec2:
        return Vec2((px[0] - self._ox) / self.scale,
                    self.field_height - (px[1] - self._oy) / self.scale)

    def meters_to_px(self, meters: float) -> float:
        return meters * self.scale

    def field_rect(self) -> tuple[int, int, int, int]:
        return (round(self._ox), round(self._oy),
                round(self.field_width * self.scale), round(self.field_height * self.scale))
