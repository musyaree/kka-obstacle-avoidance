"""Unit test untuk pengendali cf-MPC (PRD §4.2, §4.6 kasus 6, §5.1 v2.0)."""
import math
import pytest

from sim.config import Config, load_config, load_scenario
from sim.control.cfmpc import CfMpcController
from sim.engine import Engine
from sim.types import Intent, Mode, Obstacle, Pose, TrajectoryPoint, Vec2, Velocity


@pytest.fixture
def cfg() -> Config:
    return load_config("config.json")


@pytest.fixture
def cfmpc(cfg: Config) -> CfMpcController:
    return CfMpcController(cfg)


def test_kasus_6_robot_di_dalam_zona_obstacle_tetap_menghasilkan_solusi(cfg: Config, cfmpc: CfMpcController):
    """PRD §4.6 Kasus 6: LMPC tetap menghasilkan solusi saat robot di dalam zona obstacle."""
    obs = Obstacle(id=1, center=Vec2(5.0, 5.0), radius=0.6)
    safe_r = cfg.safe_radius(obs)

    # Tempatkan robot di dalam zona aman obstacle (jarak 0.2 m < safe_radius)
    robot_pose = Pose(5.2, 5.0, 0.0)
    dist = Vec2(robot_pose.x, robot_pose.y).dist(obs.center)
    assert dist < safe_r, f"Robot harus di dalam zona aman: dist={dist} < safe_r={safe_r}"

    # Trajektori referensi lurus melewati obstacle
    traj = [
        TrajectoryPoint(pos=Vec2(5.0 + 0.5 * (k + 1), 5.0), theta=0.0, speed=2.0, t=0.25 * (k + 1))
        for k in range(cfg.horizon)
    ]

    result = cfmpc.track(robot_pose, traj, [obs])

    # Harus tetap menghasilkan solusi yang valid (bukan crash / nol karena gagal)
    assert result is not None
    assert len(result.horizon) == cfg.horizon + 1
    # Slack harus aktif (> 0) karena posisi robot melanggar batas tanpa slack
    assert result.slack > 0.0
    # Command menghasilkan kecepatan translasi
    speed = math.hypot(result.command.vx, result.command.vy)
    assert speed > 0.0
    assert speed <= cfg.max_speed + 1e-4


def test_cfmpc_trajektori_kosong(cfmpc: CfMpcController, cfg: Config):
    """Jika trajektori kosong, pengendali mengembalikan perintah diam dan slack 0."""
    robot_pose = Pose(2.0, 4.5, 0.0)
    result = cfmpc.track(robot_pose, [], [])

    assert result.command.vx == 0.0
    assert result.command.vy == 0.0
    assert result.command.omega == 0.0
    assert result.slack == 0.0
    assert len(result.horizon) == cfg.horizon + 1


def test_cfmpc_bebas_tanpa_obstacle(cfmpc: CfMpcController, cfg: Config):
    """Tanpa obstacle, robot bergerak maju menuju titik referensi dengan slack 0."""
    robot_pose = Pose(2.0, 4.5, 0.0)
    traj = [
        TrajectoryPoint(pos=Vec2(2.0 + 0.5 * (k + 1), 4.5), theta=0.0, speed=2.0, t=0.25 * (k + 1))
        for k in range(cfg.horizon)
    ]

    result = cfmpc.track(robot_pose, traj, [])

    assert result.command.vx > 0.0
    assert abs(result.command.vy) < 0.2
    assert result.slack == 0.0
    assert len(result.horizon) == cfg.horizon + 1


def test_cfmpc_batas_kecepatan_dan_orientasi(cfmpc: CfMpcController, cfg: Config):
    """Perintah kontrol tidak boleh melebihi max_speed dan max_omega."""
    robot_pose = Pose(2.0, 4.5, 0.0)
    # Titik referensi jauh dan menyamping tajam
    traj = [
        TrajectoryPoint(pos=Vec2(10.0, 8.0), theta=math.pi / 2, speed=5.0, t=1.0)
        for _ in range(cfg.horizon)
    ]

    result = cfmpc.track(robot_pose, traj, [])
    speed = math.hypot(result.command.vx, result.command.vy)
    assert speed <= cfg.max_speed + 1e-4
    assert abs(result.command.omega) <= cfg.max_omega + 1e-4


def test_cfmpc_kinerja_waktu_solve_cepat(cfmpc: CfMpcController, cfg: Config):
    """Solve time dengan Parameter harus cepat (< 25 ms) untuk mendukung 60 FPS (PRD K1)."""
    robot_pose = Pose(2.0, 4.5, 0.0)
    obs = Obstacle(id=1, center=Vec2(5.0, 4.5), radius=0.6)
    traj = [
        TrajectoryPoint(pos=Vec2(2.0 + 0.5 * (k + 1), 4.5), theta=0.0, speed=2.0, t=0.25 * (k + 1))
        for k in range(cfg.horizon)
    ]

    # Warmup
    cfmpc.track(robot_pose, traj, [obs])

    # Ukur
    res = cfmpc.track(robot_pose, traj, [obs])
    # Harus di bawah 25 ms
    assert res.solve_time_ms < 25.0, f"Solve time terlalu lambat: {res.solve_time_ms} ms"


def test_cfmpc_integrasi_engine_dan_tombol_c(cfg: Config):
    """Tombol C mengganti pengendali aktif antara Pengikut sederhana dan cf-MPC."""
    scenario = load_scenario("scenarios/default.json", cfg)
    engine = Engine(cfg, scenario)

    # Bawaan adalah follower
    assert engine.controller.name == "Pengikut sederhana"

    # Tekan C untuk beralih ke cf-MPC
    f1 = engine.tick(0.05, Intent(switch_controller=True))
    assert engine.controller.name == "cf-MPC"
    assert "cf-MPC" in f1.notice

    # Jalankan mode AUTO dengan cf-MPC
    f2 = engine.tick(0.05, Intent(toggle_auto=True))
    assert f2.mode == Mode.AUTO
    assert f2.controller_name == "cf-MPC"
    assert f2.track is not None
    assert f2.track.command is not None

    # Tekan C lagi untuk kembali ke Pengikut sederhana
    f3 = engine.tick(0.05, Intent(switch_controller=True))
    assert engine.controller.name == "Pengikut sederhana"

