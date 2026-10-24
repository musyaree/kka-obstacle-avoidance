import time
from sim.types import Scene, PlanResult, PlanStatus
from sim.geometry import Vec2
from sim.planning.graph import VisibilityGraph
from sim.planning.search import AugmentedAStar, interpolate_trajectory

def plan(scene: Scene, cfg) -> PlanResult:
    """Fungsi utama yang dipanggil Engine pada setiap frame jika terjadi replanning."""
    t0 = time.perf_counter()
    vg = VisibilityGraph(scene)
    edges, active_obs = vg.build()
    
    """ Pencarian Jalur """
    start = scene.robot.pose.position if hasattr(scene.robot, 'pose') else Vec2(0, 0)
    astar = AugmentedAStar(start, scene.goal, edges, getattr(cfg, 'lambda_turn', 1.0))
    path = astar.search()
    
    status = PlanStatus.OK if path else PlanStatus.NO_PATH
    trajectory = interpolate_trajectory(path, getattr(cfg, 'v_max', 2.0), getattr(cfg, 'dt', 0.25)) if path else []
    
    dt_ms = (time.perf_counter() - t0) * 1000.0
    
    return PlanResult(
        status=status,
        path=path or [],
        trajectory=trajectory,
        region=vg.region_bbox, 
        edges=edges,           
        solve_time_ms=dt_ms
    )