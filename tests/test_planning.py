"""Pengujian modul planning (DAVG + A*) sesuai PRD §4.6 dan PLAN.md T1."""
import math
import pytest

from sim.config import Config
from sim.dynamics import Robot, RobotLimits
from sim.geometry import segment_hits_polygon
from sim.planning import (
    ActiveRegion,
    ActiveRegionSelector,
    AugmentedSearch,
    Planner,
    TrajectoryInterpolator,
    VisibilityGraph,
    plan,
)
from sim.types import Obstacle, PlanStatus, Pose, Scene, TrajectoryPoint, Vec2


DEFAULT_LIMITS = RobotLimits(max_speed=2.0, max_accel=2.0, max_omega=2.0, max_alpha=6.0)


def make_scene(
    robot_pos: Vec2 = Vec2(2.0, 4.5),
    robot_theta: float = 0.0,
    goal: Vec2 = Vec2(12.0, 4.5),
    obstacles: list[Obstacle] | None = None,
) -> Scene:
    robot = Robot(Pose(robot_pos.x, robot_pos.y, robot_theta), 0.35, DEFAULT_LIMITS)
    return Scene(
        width=14.0,
        height=9.0,
        robot=robot,
        goal=goal,
        obstacles=obstacles or [],
    )


# Kasus 1: Tanpa obstacle
def test_kasus_1_tanpa_obstacle_jalur_garis_lurus():
    cfg = Config()
    scene = make_scene()
    res = plan(scene, cfg)

    assert res.status == PlanStatus.OK
    assert res.path == [Vec2(2.0, 4.5), Vec2(12.0, 4.5)]
    assert len(res.trajectory) > 0
    assert res.trajectory[0].pos == Vec2(2.0, 4.5)
    assert res.trajectory[-1].pos == Vec2(12.0, 4.5)
    assert res.region.bbox is None
    assert len(res.region.obstacle_ids) == 0


# Kasus 4: Obstacle di luar garis S-G tidak aktif
def test_kasus_4_obstacle_tidak_aktif_tidak_mengubah_hasil():
    cfg = Config()
    # Obstacle ditaruh jauh dari garis S-G (y = 1.0 vs y = 4.5)
    obs = Obstacle(id=0, center=Vec2(7.0, 1.0), radius=0.5, sides=18)
    scene = make_scene(obstacles=[obs])
    res = plan(scene, cfg)

    assert res.status == PlanStatus.OK
    assert res.path == [Vec2(2.0, 4.5), Vec2(12.0, 4.5)]
    assert len(res.region.obstacle_ids) == 0


# Kasus 2: Obstacle di tengah garis S-G, memutari dan sisi berubah sesuai sudut awal (Fig. 11a, b)
def test_kasus_2_obstacle_di_tengah_memutari_dan_sisi_berubah_sesuai_sudut_awal():
    cfg = Config(lambda_turn=2.0)
    obs = Obstacle(id=1, center=Vec2(7.0, 4.5), radius=0.6, sides=18)

    # 11a: Sudut awal menghadap agak ke atas (+pi / 4)
    scene_up = make_scene(robot_theta=math.pi / 4.0, obstacles=[obs])
    res_up = plan(scene_up, cfg)

    assert res_up.status == PlanStatus.OK
    assert len(res_up.path) > 2
    # Jalur tidak boleh menembus poligon obstacle
    poly = obs.vertices()
    for i in range(len(res_up.path) - 1):
        assert not segment_hits_polygon(res_up.path[i], res_up.path[i + 1], poly)
    # Titik tengah jalur harus memutar lewat atas (y > 4.5)
    assert all(p.y >= 4.5 - 1e-6 for p in res_up.path)
    assert any(p.y > 4.5 + 0.1 for p in res_up.path)

    # 11b: Sudut awal menghadap agak ke bawah (-pi / 4)
    scene_down = make_scene(robot_theta=-math.pi / 4.0, obstacles=[obs])
    res_down = plan(scene_down, cfg)

    assert res_down.status == PlanStatus.OK
    assert len(res_down.path) > 2
    for i in range(len(res_down.path) - 1):
        assert not segment_hits_polygon(res_down.path[i], res_down.path[i + 1], poly)
    # Titik tengah jalur harus memutar lewat bawah (y < 4.5)
    assert all(p.y <= 4.5 + 1e-6 for p in res_down.path)
    assert any(p.y < 4.5 - 0.1 for p in res_down.path)


# Kasus 3a: Start berada di dalam obstacle (Fig. 11c)
def test_kasus_3a_start_di_dalam_obstacle():
    cfg = Config()
    obs = Obstacle(id=2, center=Vec2(2.0, 4.5), radius=1.0, sides=18)
    # Robot tepat di pusat obstacle, menghadap ke kanan (theta = 0) menuju goal
    scene = make_scene(robot_pos=Vec2(2.0, 4.5), robot_theta=0.0, obstacles=[obs])
    res = plan(scene, cfg)

    assert res.status == PlanStatus.OK
    assert len(res.path) >= 2
    assert res.path[0] == Vec2(2.0, 4.5)
    # Vertex keluar harus di depan robot (x > 2.0)
    assert res.path[1].x > 2.0
    assert res.path[-1] == Vec2(12.0, 4.5)


# Kasus 3b: Goal berada di dalam obstacle (Fig. 11d)
def test_kasus_3b_goal_di_dalam_obstacle():
    cfg = Config()
    obs = Obstacle(id=3, center=Vec2(12.0, 4.5), radius=1.0, sides=18)
    scene = make_scene(obstacles=[obs])
    res = plan(scene, cfg)

    assert res.status == PlanStatus.OK
    assert len(res.path) >= 2
    assert res.path[0] == Vec2(2.0, 4.5)
    # Vertex masuk harus berada di sisi kedatangan (x < 12.0)
    assert res.path[-2].x < 12.0
    assert res.path[-1] == Vec2(12.0, 4.5)


# Kasus 5: Goal tertutup rapat -> NO_PATH tanpa exception
def test_kasus_5_goal_tertutup_rapat():
    cfg = Config()
    # Buat rintangan yang melingkari goal (12, 4.5) secara rapat sehingga tertutup
    ring_obs = []
    goal = Vec2(12.0, 4.5)
    n_ring = 8
    r_ring = 0.8
    for i in range(n_ring):
        angle = 2.0 * math.pi * i / n_ring
        cx = goal.x + r_ring * math.cos(angle)
        cy = goal.y + r_ring * math.sin(angle)
        ring_obs.append(Obstacle(id=10 + i, center=Vec2(cx, cy), radius=0.6, sides=18))

    scene = make_scene(obstacles=ring_obs)
    res = plan(scene, cfg)

    assert res.status == PlanStatus.NO_PATH
    assert res.path == []
    assert res.trajectory == []


# Determinisme
def test_determinisme_perencanaan():
    cfg = Config()
    obs = Obstacle(id=5, center=Vec2(7.0, 4.5), radius=0.7, sides=18)
    scene = make_scene(robot_theta=0.3, obstacles=[obs])

    res1 = plan(scene, cfg)
    res2 = plan(scene, cfg)

    assert res1.status == res2.status == PlanStatus.OK
    assert res1.path == res2.path
    assert len(res1.trajectory) == len(res2.trajectory)
    for p1, p2 in zip(res1.trajectory, res2.trajectory):
        assert p1.pos == p2.pos
        assert p1.theta == p2.theta
        assert p1.speed == p2.speed
        assert p1.t == p2.t


# Interpolasi trajektori
def test_interpolasi_trajektori():
    path = [Vec2(0.0, 0.0), Vec2(4.0, 0.0)]
    max_speed = 2.0
    dt = 0.25
    traj = TrajectoryInterpolator.interpolate(path, max_speed, dt)

    assert len(traj) > 0
    assert traj[0].t == 0.0
    assert traj[0].pos == Vec2(0.0, 0.0)
    assert traj[-1].pos == Vec2(4.0, 0.0)
    assert traj[-1].t == pytest.approx(2.0)
    for tp in traj:
        assert isinstance(tp, TrajectoryPoint)
        assert tp.speed <= max_speed
    for i in range(len(traj) - 1):
        assert traj[i + 1].t > traj[i].t


# Active region dan struktur edge
def test_active_region_dan_edges():
    cfg = Config()
    obs = Obstacle(id=7, center=Vec2(7.0, 4.5), radius=0.5, sides=18)
    scene = make_scene(obstacles=[obs])
    res = plan(scene, cfg)

    assert res.region is not None
    assert 7 in res.region.obstacle_ids
    assert res.region.bbox is not None
    assert len(res.edges) > 0
    # edges adalah daftar pasangan (u, v)
    u, v = res.edges[0]
    assert isinstance(u, Vec2)
    assert isinstance(v, Vec2)
