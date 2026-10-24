"""Pengujian Engine lewat antarmukanya: tick(dt, Intent) -> Frame, tanpa pygame."""
import pytest

from sim.config import Config, Scenario, TimedEvent
from sim.engine import Engine
from sim.types import (Intent, Mode, Move, Obstacle, Pose, Select, SetVelocity, Vec2,
                       Velocity)

DT = 0.05
CFG = Config()


def make_engine(obstacles=(), events=(), start=Pose(2.0, 4.5, 0.0), goal=Vec2(12.0, 4.5)):
    scenario = Scenario("uji", start, goal, list(obstacles), list(events))
    return Engine(CFG, scenario)


def run(engine, seconds, intent=None):
    frame = None
    for _ in range(round(seconds / DT)):
        frame = engine.tick(DT, intent)
    return frame


def test_mulai_di_mode_manual_tanpa_bergerak():
    frame = make_engine().tick(DT)
    assert frame.mode is Mode.MANUAL
    assert frame.scene.robot.pose == Pose(2.0, 4.5, 0.0)


def test_maju_manual_menggerakkan_robot():
    frame = run(make_engine(), 2.0, Intent(manual=Velocity(1.0, 0.0, 0.0)))
    assert frame.scene.robot.pose.x > 3.0
    assert frame.scene.robot.velocity.vx == pytest.approx(CFG.max_speed * CFG.manual_speed_ratio)


def test_diagonal_tidak_lebih_cepat_dari_lurus():
    frame = run(make_engine(), 3.0, Intent(manual=Velocity(1.0, 1.0, 0.0)))
    v = frame.scene.robot.velocity
    assert (v.vx ** 2 + v.vy ** 2) ** 0.5 <= CFG.max_speed * CFG.manual_speed_ratio + 1e-9


def test_jeda_menghentikan_robot_dan_obstacle():
    engine = make_engine([Obstacle(0, Vec2(5, 4), 0.5, 18, Vec2(1.0, 0.0))])
    engine.tick(DT, Intent(toggle_pause=True))
    before = engine.tick(DT, Intent(manual=Velocity(1.0, 0.0, 0.0)))
    after = run(engine, 1.0, Intent(manual=Velocity(1.0, 0.0, 0.0)))
    assert after.paused
    assert after.scene.robot.pose == before.scene.robot.pose
    assert after.scene.obstacles[0].center == before.scene.obstacles[0].center
    assert after.sim_time == before.sim_time


def test_objek_tetap_bisa_digeser_saat_jeda():
    engine = make_engine([Obstacle(0, Vec2(5, 4), 0.5)])
    engine.tick(DT, Intent(toggle_pause=True))
    frame = engine.tick(DT, Intent(moves=[Move("obstacle", Vec2(6, 3), id=0)]))
    assert frame.scene.obstacles[0].center == Vec2(6, 3)


def test_obstacle_bergerak_dan_memantul_di_batas():
    engine = make_engine([Obstacle(0, Vec2(13.0, 4.5), 0.5, 18, Vec2(1.0, 0.0))])
    frame = run(engine, 3.0)
    o = frame.scene.obstacles[0]
    assert o.center.x <= CFG.field_width - o.radius + 1e-9
    assert o.velocity.x < 0


def test_geser_robot_goal_dan_obstacle_dibatasi_lapangan():
    engine = make_engine([Obstacle(0, Vec2(5, 4), 0.5)])
    frame = engine.tick(DT, Intent(moves=[
        Move("robot", Vec2(-5, 100)),
        Move("goal", Vec2(99, -3)),
        Move("obstacle", Vec2(-2, -2), id=0),
    ]))
    r = frame.scene.robot
    assert r.pose.x == pytest.approx(r.radius) and r.pose.y == pytest.approx(CFG.field_height - r.radius)
    assert frame.scene.goal == Vec2(CFG.field_width, 0.0)
    assert frame.scene.obstacles[0].center == Vec2(0.5, 0.5)


def test_geser_robot_menolkan_kecepatan_dan_mempertahankan_arah_hadap():
    engine = make_engine(start=Pose(2.0, 4.5, 1.0))
    run(engine, 1.0, Intent(manual=Velocity(1.0, 0.0, 0.0)))
    frame = engine.tick(DT, Intent(moves=[Move("robot", Vec2(8, 3))]))
    assert frame.scene.robot.velocity == Velocity()
    assert frame.scene.robot.pose.theta == pytest.approx(1.0)


def test_kecepatan_obstacle_diatur_dan_dibatasi():
    engine = make_engine([Obstacle(0, Vec2(5, 4), 0.5)])
    frame = engine.tick(DT, Intent(set_velocities=[SetVelocity(0, Vec2(30.0, 0.0))]))
    assert frame.scene.obstacles[0].velocity.norm() == pytest.approx(CFG.obstacle_max_speed)


def test_pilih_dan_batal_pilih_obstacle():
    engine = make_engine([Obstacle(0, Vec2(5, 4), 0.5)])
    assert engine.tick(DT, Intent(select=Select(0))).selected_id == 0
    assert engine.tick(DT, Intent()).selected_id == 0
    assert engine.tick(DT, Intent(select=Select(None))).selected_id is None
    assert engine.tick(DT, Intent(select=Select(99))).selected_id is None


def test_kejadian_memunculkan_obstacle_pada_waktunya():
    event = TimedEvent(1.0, "munculkan_obstacle", {"obstacle": {"x": 10, "y": 4.5, "radius": 0.5}})
    engine = make_engine([Obstacle(0, Vec2(5, 4), 0.5)], [event])
    assert len(run(engine, 0.5).scene.obstacles) == 1
    frame = run(engine, 1.0)
    assert [o.id for o in frame.scene.obstacles] == [0, 1]
    assert len(run(engine, 2.0).scene.obstacles) == 2  # tidak muncul dua kali


def test_kejadian_menghapus_obstacle_dan_membatalkan_pilihan():
    event = TimedEvent(0.5, "hapus_obstacle", {"id": 0})
    engine = make_engine([Obstacle(0, Vec2(5, 4), 0.5)], [event])
    engine.tick(DT, Intent(select=Select(0)))
    frame = run(engine, 1.0)
    assert frame.scene.obstacles == []
    assert frame.selected_id is None


def test_enter_belum_mengaktifkan_auto_pada_mvp():
    frame = make_engine().tick(DT, Intent(toggle_auto=True))
    assert frame.mode is Mode.MANUAL
    assert "AUTO" in frame.notice


def test_reset_mengembalikan_keadaan_awal():
    obstacle = Obstacle(0, Vec2(5, 4), 0.5, 18, Vec2(1.0, 0.0))
    scenario = Scenario("uji", Pose(2.0, 4.5, 0.0), Vec2(12.0, 4.5), [obstacle], [])
    engine = Engine(CFG, scenario)
    run(engine, 2.0, Intent(manual=Velocity(1.0, 0.0, 0.0)))
    engine.reset(scenario)
    frame = engine.tick(0.0)
    assert frame.scene.robot.pose == Pose(2.0, 4.5, 0.0)
    assert frame.scene.obstacles[0].center == Vec2(5, 4)
    assert frame.sim_time == 0.0
    assert scenario.obstacles[0].center == Vec2(5, 4)  # skenario asli tidak ikut berubah


def test_frame_adalah_potret_yang_terpisah_dari_engine():
    engine = make_engine([Obstacle(0, Vec2(5, 4), 0.5)])
    frame = engine.tick(DT)
    frame.scene.obstacles[0].center = Vec2(0, 0)
    assert engine.scene.obstacles[0].center == Vec2(5, 4)
