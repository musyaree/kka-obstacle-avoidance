"""Pengujian FollowerController (pengikut sederhana) sesuai PRD §4.7 dan PLAN.md T2."""
import math
import pytest

from sim.config import Config
from sim.control import FollowerController
from sim.dynamics import Robot, RobotLimits
from sim.types import Pose, TrajectoryPoint, Vec2, Velocity


@pytest.fixture
def cfg() -> Config:
    return Config(
        max_speed=2.0,
        max_omega=2.0,
        horizon=10,
        dt_mpc=0.25,
        goal_threshold=0.3,
    )


@pytest.fixture
def controller(cfg: Config) -> FollowerController:
    return FollowerController(cfg)


def make_straight_trajectory(
    start: Vec2 = Vec2(2.0, 4.5),
    end: Vec2 = Vec2(8.0, 4.5),
    speed: float = 2.0,
    dt: float = 0.25,
) -> list[TrajectoryPoint]:
    """Trajektori lurus buatan tangan tanpa bergantung pada modul planning."""
    dist = start.dist(end)
    steps = max(2, int(dist / (speed * dt)))
    direction = end - start
    theta = math.atan2(direction.y, direction.x)
    traj = []
    for i in range(steps + 1):
        ratio = i / steps
        pos = Vec2(start.x + ratio * direction.x, start.y + ratio * direction.y)
        t = ratio * (dist / speed)
        traj.append(TrajectoryPoint(pos=pos, theta=theta, speed=speed, t=t))
    return traj


def test_trajektori_kosong_menghasilkan_perintah_nol_dan_horizon_penuh(
    controller: FollowerController, cfg: Config
):
    pose = Pose(2.0, 4.5, 0.0)
    result = controller.track(pose, [], [])

    assert result.command.vx == 0.0
    assert result.command.vy == 0.0
    assert result.command.omega == 0.0
    assert len(result.horizon) == cfg.horizon
    assert all(p == pose for p in result.horizon)


def test_trajektori_lurus_maju_ke_depan(controller: FollowerController):
    # Robot di (2, 4.5) menghadap sumbu-x (theta = 0)
    pose = Pose(2.0, 4.5, 0.0)
    traj = make_straight_trajectory(start=Vec2(2.0, 4.5), end=Vec2(8.0, 4.5))

    result = controller.track(pose, traj, [])

    # Bergerak maju (vx positif), vy dan omega mendekati nol
    assert result.command.vx > 0.5
    assert abs(result.command.vy) < 1e-4
    assert abs(result.command.omega) < 1e-4


def test_trajektori_ke_kiri_menghasilkan_vy_positif(controller: FollowerController):
    # Robot menghadap sumbu-x (theta = 0), jalur ke arah +y (ke kiri relatif thd robot)
    pose = Pose(2.0, 4.5, 0.0)
    traj = make_straight_trajectory(start=Vec2(2.0, 4.5), end=Vec2(2.0, 8.0))

    result = controller.track(pose, traj, [])

    assert result.command.vy > 0.5
    assert abs(result.command.vx) < 1e-4


def test_kecepatan_tidak_pernah_melampaui_batas(controller: FollowerController, cfg: Config):
    # Uji berbagai kombinasi pose dan arah target
    for theta in (-math.pi, -math.pi / 2, 0.0, math.pi / 3, math.pi):
        pose = Pose(2.0, 4.5, theta)
        traj = make_straight_trajectory(start=Vec2(2.0, 4.5), end=Vec2(5.0, 7.0))
        result = controller.track(pose, traj, [])

        v_mag = math.hypot(result.command.vx, result.command.vy)
        assert v_mag <= cfg.max_speed + 1e-9
        assert abs(result.command.omega) <= cfg.max_omega + 1e-9


def test_horizon_berisi_n_titik_dan_bergerak_maju(
    controller: FollowerController, cfg: Config
):
    pose = Pose(2.0, 4.5, 0.0)
    traj = make_straight_trajectory()
    result = controller.track(pose, traj, [])

    assert len(result.horizon) == cfg.horizon
    # Setiap titik berikutnya pada horizon harus maju ke arah +x
    for i in range(len(result.horizon) - 1):
        assert result.horizon[i + 1].x > result.horizon[i].x


def test_perlambatan_saat_mendekati_titik_akhir(controller: FollowerController):
    traj = make_straight_trajectory(start=Vec2(2.0, 4.5), end=Vec2(3.0, 4.5))

    # Saat masih jauh (jarak 1.0 m > 0.6 m zona perlambatan)
    res_jauh = controller.track(Pose(2.0, 4.5, 0.0), traj, [])

    # Saat sudah sangat dekat dengan titik akhir (misal jarak 0.2 m < 0.6 m)
    res_dekat = controller.track(Pose(2.8, 4.5, 0.0), traj, [])

    assert res_dekat.command.vx < res_jauh.command.vx


def test_berhenti_jika_sudah_di_titik_akhir(controller: FollowerController):
    traj = make_straight_trajectory(start=Vec2(2.0, 4.5), end=Vec2(5.0, 4.5))
    # Robot tepat berada di titik akhir jalur
    pose_goal = Pose(5.0, 4.5, 0.0)
    result = controller.track(pose_goal, traj, [])

    assert result.command.vx == 0.0
    assert result.command.vy == 0.0


def test_menerima_state_berupa_robot_maupun_pose(
    controller: FollowerController, cfg: Config
):
    limits = RobotLimits(cfg.max_speed, cfg.max_accel, cfg.max_omega, cfg.max_alpha)
    robot = Robot(Pose(2.0, 4.5, 0.0), cfg.robot_radius, limits)
    pose = Pose(2.0, 4.5, 0.0)
    traj = make_straight_trajectory()

    res_robot = controller.track(robot, traj, [])
    res_pose = controller.track(pose, traj, [])

    assert res_robot.command == res_pose.command
    assert res_robot.horizon == res_pose.horizon


def test_menerima_trajektori_berupa_list_vec2(controller: FollowerController):
    # Kompatibilitas jika trajektori hanya berupa list Vec2 murni
    traj_vec2 = [Vec2(2.0, 4.5), Vec2(3.0, 4.5), Vec2(4.0, 4.5)]
    result = controller.track(Pose(2.0, 4.5, 0.0), traj_vec2, [])

    assert result.command.vx > 0.0
    assert len(result.horizon) == 10

