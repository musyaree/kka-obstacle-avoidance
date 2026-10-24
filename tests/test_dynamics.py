import math

import pytest

from sim.dynamics import Robot, RobotLimits
from sim.types import Pose, Velocity

LIMITS = RobotLimits(max_speed=2.0, max_accel=2.0, max_omega=2.0, max_alpha=6.0)
W, H = 14.0, 9.0


def make_robot(pose=Pose(7.0, 4.5, 0.0)):
    return Robot(pose, 0.35, LIMITS)


def test_perintah_dipotong_ke_kecepatan_maksimum():
    v = LIMITS.clamp(Velocity(10.0, 0.0, 0.0), Velocity(2.0, 0.0, 0.0), 0.1)
    assert math.hypot(v.vx, v.vy) <= LIMITS.max_speed + 1e-9


def test_perubahan_kecepatan_dibatasi_percepatan():
    v = LIMITS.clamp(Velocity(2.0, 0.0, 0.0), Velocity(), 0.1)
    assert v.vx == pytest.approx(LIMITS.max_accel * 0.1)


def test_perubahan_kecepatan_putar_dibatasi():
    v = LIMITS.clamp(Velocity(0.0, 0.0, 2.0), Velocity(), 0.1)
    assert v.omega == pytest.approx(LIMITS.max_alpha * 0.1)


def test_robot_butuh_waktu_untuk_mencapai_kecepatan_penuh():
    robot = make_robot()
    for _ in range(10):  # 1 detik pada dt 0,1
        robot.step(Velocity(2.0, 0.0, 0.0), 0.1, W, H)
    assert robot.velocity.vx == pytest.approx(2.0)
    # kecepatan 0,2k m/s pada langkah k = 1..10: jarak 0,1 * 0,2 * 55 = 1,1 m
    assert robot.pose.x == pytest.approx(7.0 + 1.1)


def test_maju_mengikuti_arah_hadap():
    robot = make_robot(Pose(7.0, 4.5, math.pi / 2))
    for _ in range(10):
        robot.step(Velocity(2.0, 0.0, 0.0), 0.1, W, H)
    assert robot.pose.y > 4.5 + 0.5
    assert robot.pose.x == pytest.approx(7.0, abs=1e-9)


def test_geser_ke_kiri_adalah_vy_positif():
    robot = make_robot(Pose(7.0, 4.5, 0.0))
    for _ in range(10):
        robot.step(Velocity(0.0, 1.0, 0.0), 0.1, W, H)
    assert robot.pose.y > 4.5


def test_berputar_dan_sudut_tetap_dalam_rentang():
    robot = make_robot()
    for _ in range(400):
        robot.step(Velocity(0.0, 0.0, 2.0), 0.05, W, H)
    assert -math.pi < robot.pose.theta <= math.pi


def test_robot_tidak_keluar_lapangan():
    robot = make_robot(Pose(13.5, 4.5, 0.0))
    for _ in range(100):
        robot.step(Velocity(2.0, 0.0, 0.0), 0.1, W, H)
    assert robot.pose.x == pytest.approx(W - robot.radius)


def test_teleport_menolkan_kecepatan():
    robot = make_robot()
    robot.step(Velocity(2.0, 0.0, 1.0), 0.1, W, H)
    robot.teleport(Pose(3.0, 3.0, 1.0))
    assert robot.pose == Pose(3.0, 3.0, 1.0)
    assert robot.velocity == Velocity()
