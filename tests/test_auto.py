"""Pengujian mode AUTO pada Engine sesuai PRD §2.3 dan §4.6."""
import math
import pytest

from sim.config import Config, Scenario, load_config, load_scenario
from sim.engine import Engine
from sim.types import Intent, Mode, Move, Obstacle, PlanStatus, Pose, Vec2, Velocity

CFG = Config()
DT = 1.0 / 60.0


def make_engine(
    start: Pose = Pose(2.0, 4.5, 0.0),
    goal: Vec2 = Vec2(12.0, 4.5),
    obstacles: list[Obstacle] | None = None,
) -> Engine:
    scenario = Scenario("uji", start, goal, obstacles or [], [])
    return Engine(CFG, scenario)


def test_enter_mengaktifkan_auto_dan_mencapai_goal_tanpa_masukan_manual():
    # Menggunakan skenario dasar default: robot di (2, 4.5), obstacle di (7, 4.5)
    scenario = load_scenario("scenarios/default.json", CFG)
    engine = Engine(CFG, scenario)

    # Mulai di mode MANUAL
    f0 = engine.tick(DT)
    assert f0.mode is Mode.MANUAL

    # Tekan Enter untuk masuk AUTO
    f1 = engine.tick(DT, Intent(toggle_auto=True))
    assert f1.mode is Mode.AUTO
    assert f1.plan is not None
    assert f1.plan.status is PlanStatus.OK

    # Jalankan simulasi sampai mencapai goal (maksimal 15 detik simulasi)
    reached = False
    for _ in range(int(15.0 / DT)):
        frame = engine.tick(DT)
        if frame.mode is Mode.MANUAL and "goal" in frame.notice.lower():
            reached = True
            break

    assert reached
    assert frame.scene.robot.pose.position.dist(frame.scene.goal) <= CFG.goal_threshold + 0.05


def test_wasd_saat_auto_menghentikan_auto():
    engine = make_engine()
    engine.tick(DT, Intent(toggle_auto=True))
    assert engine.mode is Mode.AUTO

    # Kirim masukan manual WASD (maju)
    f = engine.tick(DT, Intent(manual=Velocity(1.0, 0.0, 0.0)))
    assert f.mode is Mode.MANUAL
    assert engine.mode is Mode.MANUAL


def test_enter_saat_auto_menghentikan_auto():
    engine = make_engine()
    engine.tick(DT, Intent(toggle_auto=True))
    assert engine.mode is Mode.AUTO

    # Tekan Enter lagi
    f = engine.tick(DT, Intent(toggle_auto=True))
    assert f.mode is Mode.MANUAL


def test_menggeser_goal_saat_auto_berlanjut_dan_replan():
    engine = make_engine()
    engine.tick(DT, Intent(toggle_auto=True))
    assert engine.mode is Mode.AUTO

    new_goal = Vec2(10.0, 7.0)
    f = engine.tick(DT, Intent(moves=[Move("goal", new_goal)]))

    assert f.mode is Mode.AUTO
    assert f.scene.goal == new_goal
    assert f.plan is not None
    assert f.plan.path[-1] == new_goal


def test_menyeret_robot_saat_auto_berlanjut_dari_posisi_baru():
    engine = make_engine()
    engine.tick(DT, Intent(toggle_auto=True))
    assert engine.mode is Mode.AUTO

    new_pos = Vec2(4.0, 2.0)
    f = engine.tick(DT, Intent(moves=[Move("robot", new_pos)]))

    assert f.mode is Mode.AUTO
    assert f.scene.robot.pose.position.dist(new_pos) < 0.05
    assert f.plan is not None
    assert f.plan.path[0].dist(new_pos) < 1e-4


def test_tidak_ada_jalur_robot_diam_dan_layar_menampilkan_tidak_ada_jalur():
    # Kelilingi goal rapat-rapat dengan cincin obstacle
    ring_obs = []
    goal = Vec2(12.0, 4.5)
    for i in range(8):
        angle = 2.0 * math.pi * i / 8
        cx = goal.x + 0.8 * math.cos(angle)
        cy = goal.y + 0.8 * math.sin(angle)
        ring_obs.append(Obstacle(id=10 + i, center=Vec2(cx, cy), radius=0.6, sides=18))

    engine = make_engine(goal=goal, obstacles=ring_obs)
    f = engine.tick(DT, Intent(toggle_auto=True))

    assert f.mode is Mode.AUTO
    assert f.plan.status is PlanStatus.NO_PATH
    assert "tidak ada jalur" in f.notice.lower()
    # Robot tidak bergerak (kecepatan nol)
    assert f.track.command.vx == 0.0
    assert f.track.command.vy == 0.0

    # Lakukan beberapa tick lagi, robot tetap diam
    pos_before = f.scene.robot.pose.position
    for _ in range(10):
        f = engine.tick(DT)
    assert f.scene.robot.pose.position.dist(pos_before) < 1e-4


def test_tombol_c_memberitahu_status_pengendali():
    engine = make_engine()
    f = engine.tick(DT, Intent(switch_controller=True))
    assert "pengendali" in f.notice.lower() or "cf-mpc" in f.notice.lower()


def test_jeda_menghentikan_robot_saat_auto():
    engine = make_engine()
    engine.tick(DT, Intent(toggle_auto=True))
    assert engine.mode is Mode.AUTO

    # Tekan spasi untuk jeda
    f_pause = engine.tick(DT, Intent(toggle_pause=True))
    assert f_pause.paused

    pos_paused = f_pause.scene.robot.pose.position
    for _ in range(10):
        f = engine.tick(DT)
    assert f.scene.robot.pose.position.dist(pos_paused) < 1e-4
