"""Tipe data bersama antarmodul. Tidak mengimpor pygame.

Koordinat dunia dalam meter, sumbu x ke kanan dan y ke atas, sudut theta
berlawanan arah jarum jam (0 = menghadap sumbu x). Kerangka tubuh robot:
x ke depan, y ke kiri.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from .dynamics import Robot


@dataclass(frozen=True)
class Vec2:
    x: float = 0.0
    y: float = 0.0

    def __add__(self, other: Vec2) -> Vec2:
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vec2) -> Vec2:
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, k: float) -> Vec2:
        return Vec2(self.x * k, self.y * k)

    __rmul__ = __mul__

    def norm(self) -> float:
        return math.hypot(self.x, self.y)

    def dist(self, other: Vec2) -> float:
        return (self - other).norm()

    def __lt__(self, other: Vec2) -> bool:
        return (self.x, self.y) < (other.x, other.y)


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    @property
    def position(self) -> Vec2:
        return Vec2(self.x, self.y)


@dataclass(frozen=True)
class Velocity:
    """Kecepatan dalam kerangka tubuh robot: (vx maju, vy kiri, omega putar)."""
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


@dataclass(frozen=True)
class TrajectoryPoint:
    """Satu titik pada trajektori berinterpolasi waktu.

    Dihasilkan oleh planning.TrajectoryInterpolator dan dikonsumsi oleh
    FollowerController maupun CfMpcController.

    Atribut:
        pos   -- posisi dunia (meter).
        theta -- sudut hadap (radian, CCW dari sumbu-x).
        speed -- kelajuan yang disarankan di titik ini (m/s).
        t     -- waktu relatif dari awal trajektori (detik).
    """
    pos: Vec2
    theta: float = 0.0
    speed: float = 1.0
    t: float = 0.0


class Mode(Enum):
    MANUAL = "MANUAL"
    AUTO = "AUTO"


class PlanStatus(Enum):
    OK = "OK"
    NO_PATH = "NO_PATH"


@dataclass
class Obstacle:
    """Poligon beraturan yang boleh diam atau bergerak dengan kecepatan konstan."""
    id: int
    center: Vec2
    radius: float
    sides: int = 18
    velocity: Vec2 = Vec2()

    def vertices(self) -> list[Vec2]:
        from .geometry import regular_polygon  # impor lokal: geometry mengimpor types
        return regular_polygon(self.center, self.radius, self.sides)

    def contains(self, point: Vec2) -> bool:
        from .geometry import point_in_polygon
        return point_in_polygon(point, self.vertices())

    def set_velocity(self, velocity: Vec2, max_speed: float) -> None:
        speed = velocity.norm()
        if speed > max_speed > 0.0:
            velocity = velocity * (max_speed / speed)
        self.velocity = velocity

    def step(self, dt: float, width: float, height: float) -> None:
        """Maju satu langkah; memantul saat pinggir poligon menyentuh batas lapangan."""
        vx, vy = self.velocity.x, self.velocity.y
        x = self.center.x + vx * dt
        y = self.center.y + vy * dt
        r = self.radius
        if x < r:
            x, vx = r, abs(vx)
        elif x > width - r:
            x, vx = width - r, -abs(vx)
        if y < r:
            y, vy = r, abs(vy)
        elif y > height - r:
            y, vy = height - r, -abs(vy)
        self.center = Vec2(x, y)
        self.velocity = Vec2(vx, vy)


@dataclass
class Scene:
    width: float
    height: float
    robot: "Robot"
    goal: Vec2
    obstacles: list[Obstacle] = field(default_factory=list)

    def obstacle(self, obstacle_id: int) -> Optional[Obstacle]:
        for o in self.obstacles:
            if o.id == obstacle_id:
                return o
        return None


# ---------------------------------------------------------------- masukan ---

@dataclass(frozen=True)
class Move:
    """Memindahkan 'robot', 'goal', atau 'obstacle' (dengan id) ke posisi dunia."""
    kind: str
    pos: Vec2
    id: int = -1


@dataclass(frozen=True)
class SetVelocity:
    id: int
    velocity: Vec2


@dataclass(frozen=True)
class Select:
    """id None berarti membatalkan pilihan."""
    id: Optional[int]


@dataclass
class Intent:
    """Seluruh aksi pengguna untuk satu frame; satu-satunya masukan Engine.tick()."""
    manual: Velocity = Velocity()  # pecahan dari batas, masing-masing -1..1
    toggle_auto: bool = False
    toggle_pause: bool = False
    switch_controller: bool = False
    moves: list[Move] = field(default_factory=list)
    set_velocities: list[SetVelocity] = field(default_factory=list)
    select: Optional[Select] = None


# ----------------------------------------------------------------- keluaran ---

@dataclass
class ActiveRegion:
    """Wilayah aktif DAVG (bounding box dan ID obstacle yang memotong/diperluas)."""
    bbox: Optional[tuple[float, float, float, float]] = None
    obstacle_ids: list[int] = field(default_factory=list)

    def __iter__(self):
        if self.bbox is not None:
            return iter(self.bbox)
        return iter(())


@dataclass
class PlanResult:
    """Diisi modul planning (v1.1). Rangka ini sudah memuat data debug lapisan 4-7."""
    status: PlanStatus = PlanStatus.OK
    path: list[Vec2] = field(default_factory=list)
    trajectory: list[TrajectoryPoint] = field(default_factory=list)
    region: Optional[ActiveRegion] = None
    edges: list = field(default_factory=list)
    solve_time_ms: float = 0.0


@dataclass
class TrackResult:
    """Diisi modul control (v2.0). Rangka ini sudah memuat data debug lapisan 8-9."""
    command: Velocity = Velocity()
    horizon: list[Pose] = field(default_factory=list)
    tangent_planes: list = field(default_factory=list)
    slack: float = 0.0
    solve_time_ms: float = 0.0


@dataclass
class Frame:
    """Potret keadaan setelah satu tick; yang digambar oleh app."""
    scene: Scene
    mode: Mode
    paused: bool
    selected_id: Optional[int]
    sim_time: float
    controller_name: str = "-"
    notice: str = ""
    plan: Optional[PlanResult] = None
    track: Optional[TrackResult] = None
