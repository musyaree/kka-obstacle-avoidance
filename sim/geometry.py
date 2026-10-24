"""Geometri 2D murni: poligon beraturan, titik di dalam poligon, perpotongan segmen."""
from __future__ import annotations

import math

from .types import Vec2

EPS = 1e-9


def wrap_angle(theta: float) -> float:
    """Membungkus sudut ke (-pi, pi]."""
    wrapped = math.fmod(theta + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def regular_polygon(center: Vec2, radius: float, sides: int, rotation: float = 0.0) -> list[Vec2]:
    """Titik sudut poligon beraturan yang terbaca berlawanan arah jarum jam."""
    return [
        Vec2(
            center.x + radius * math.cos(rotation + 2.0 * math.pi * i / sides),
            center.y + radius * math.sin(rotation + 2.0 * math.pi * i / sides),
        )
        for i in range(sides)
    ]


def point_segment_distance(p: Vec2, a: Vec2, b: Vec2) -> float:
    ab = b - a
    length_sq = ab.x * ab.x + ab.y * ab.y
    if length_sq < EPS:
        return p.dist(a)
    t = ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / length_sq
    t = max(0.0, min(1.0, t))
    return p.dist(Vec2(a.x + t * ab.x, a.y + t * ab.y))


def point_in_polygon(p: Vec2, poly: list[Vec2]) -> bool:
    """True hanya jika p benar-benar di dalam; titik di pinggir dianggap di luar."""
    n = len(poly)
    for i in range(n):
        if point_segment_distance(p, poly[i], poly[(i + 1) % n]) < EPS:
            return False
    inside = False
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if (a.y > p.y) != (b.y > p.y):
            x_cross = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y)
            if p.x < x_cross:
                inside = not inside
    return inside


def _orient(a: Vec2, b: Vec2, c: Vec2) -> float:
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)


def _sign(v: float) -> int:
    if v > EPS:
        return 1
    if v < -EPS:
        return -1
    return 0


def segments_cross(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool:
    """True jika segmen ab dan cd berpotongan secara sejati (bukan sekadar bersentuhan)."""
    s1, s2 = _sign(_orient(a, b, c)), _sign(_orient(a, b, d))
    s3, s4 = _sign(_orient(c, d, a)), _sign(_orient(c, d, b))
    return s1 * s2 < 0 and s3 * s4 < 0


def segment_hits_polygon(a: Vec2, b: Vec2, poly: list[Vec2]) -> bool:
    """True jika segmen ab melewati bagian dalam poligon cembung.

    Menyentuh sudut atau menyusuri sisi poligon tidak dihitung menabrak; itu
    yang dibutuhkan visibility graph, yang justru berjalan di antara sudut.
    """
    n = len(poly)
    for i in range(n):
        if segments_cross(a, b, poly[i], poly[(i + 1) % n]):
            return True
    mid = Vec2((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)
    return any(point_in_polygon(p, poly) for p in (a, b, mid))
