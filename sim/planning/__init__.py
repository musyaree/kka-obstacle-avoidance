"""Modul perencanaan rute DAVG + A* (PRD §4.2, §4.3)."""
from sim.planning.graph import ActiveRegion, ActiveRegionSelector, VisibilityGraph
from sim.planning.search import (
    AugmentedAStar,
    AugmentedSearch,
    Planner,
    TrajectoryInterpolator,
    get_turn_angle,
    interpolate_trajectory,
)
from sim.types import PlanResult, Scene

_default_planner = Planner()


def plan(scene: Scene, cfg=None) -> PlanResult:
    """Fungsi utama antarmuka planning: plan(scene, cfg) -> PlanResult (PRD §4.3)."""
    return _default_planner.plan(scene, cfg)


__all__ = [
    "plan",
    "Planner",
    "VisibilityGraph",
    "ActiveRegion",
    "ActiveRegionSelector",
    "AugmentedSearch",
    "AugmentedAStar",
    "TrajectoryInterpolator",
    "interpolate_trajectory",
    "get_turn_angle",
]
