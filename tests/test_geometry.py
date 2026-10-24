import math

import pytest

from sim.geometry import (point_in_polygon, point_segment_distance, regular_polygon,
                          segment_hits_polygon, segments_cross, wrap_angle)
from sim.types import Vec2

SQUARE = [Vec2(0, 0), Vec2(2, 0), Vec2(2, 2), Vec2(0, 2)]


def test_wrap_angle_masuk_rentang():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    for theta in (-10.0, -math.pi, 0.3, math.pi, 7.0):
        assert -math.pi < wrap_angle(theta) <= math.pi


def test_regular_polygon_jumlah_dan_jarak_ke_pusat():
    center = Vec2(3.0, -1.0)
    poly = regular_polygon(center, 0.5, 18)
    assert len(poly) == 18
    assert all(center.dist(v) == pytest.approx(0.5) for v in poly)


def test_point_segment_distance():
    assert point_segment_distance(Vec2(1, 1), Vec2(0, 0), Vec2(2, 0)) == pytest.approx(1.0)
    assert point_segment_distance(Vec2(5, 0), Vec2(0, 0), Vec2(2, 0)) == pytest.approx(3.0)


def test_point_in_polygon():
    assert point_in_polygon(Vec2(1, 1), SQUARE)
    assert not point_in_polygon(Vec2(3, 1), SQUARE)
    assert not point_in_polygon(Vec2(-0.1, 1), SQUARE)


def test_titik_di_pinggir_dan_sudut_dianggap_di_luar():
    assert not point_in_polygon(Vec2(1, 0), SQUARE)
    assert not point_in_polygon(Vec2(0, 0), SQUARE)


def test_segments_cross_sejati_saja():
    assert segments_cross(Vec2(0, 0), Vec2(2, 2), Vec2(0, 2), Vec2(2, 0))
    assert not segments_cross(Vec2(0, 0), Vec2(1, 1), Vec2(2, 0), Vec2(2, 2))
    # bersentuhan di ujung tidak dihitung
    assert not segments_cross(Vec2(0, 0), Vec2(1, 1), Vec2(1, 1), Vec2(2, 0))


def test_segmen_menembus_poligon():
    assert segment_hits_polygon(Vec2(-1, 1), Vec2(3, 1), SQUARE)


def test_segmen_di_luar_poligon():
    assert not segment_hits_polygon(Vec2(-1, 3), Vec2(3, 3), SQUARE)


def test_segmen_dengan_ujung_di_dalam():
    assert segment_hits_polygon(Vec2(1, 1), Vec2(5, 5), SQUARE)


def test_diagonal_antar_sudut_menembus_bagian_dalam():
    assert segment_hits_polygon(Vec2(0, 0), Vec2(2, 2), SQUARE)


def test_menyusuri_sisi_poligon_tidak_menabrak():
    # Visibility graph justru berjalan di antara sudut tetangga.
    assert not segment_hits_polygon(Vec2(0, 0), Vec2(2, 0), SQUARE)
    assert not segment_hits_polygon(Vec2(-1, 0), Vec2(3, 0), SQUARE)


def test_menyentuh_satu_sudut_tidak_menabrak():
    assert not segment_hits_polygon(Vec2(-1, 1), Vec2(1, -1), SQUARE)


def test_garis_antidiagonal_lewat_dua_sudut_menembus_bagian_dalam():
    assert segment_hits_polygon(Vec2(-1, 3), Vec2(3, -1), SQUARE)


def test_poligon_18_sisi_ditembus_lewat_pusat():
    poly = regular_polygon(Vec2(0, 0), 1.0, 18)
    assert segment_hits_polygon(Vec2(-3, 0), Vec2(3, 0), poly)
    assert not segment_hits_polygon(Vec2(-3, 1.5), Vec2(3, 1.5), poly)
