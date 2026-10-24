"""Pemuatan dan validasi config.json serta berkas skenario.

Semua galat dilaporkan sebagai ConfigError dengan pesan yang menyebut berkas
dan bagian yang salah, supaya aplikasi bisa menampilkannya di layar dan tetap
memakai konfigurasi terakhir yang valid.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, Optional

from .types import Obstacle, Pose, Vec2

LAYER_IDS = (
    "obstacle", "safe_zone", "sg_line", "active_region", "visibility_graph",
    "path", "trajectory", "horizon", "tangent_planes", "hud",
)

EVENT_ACTIONS = ("munculkan_obstacle", "hapus_obstacle")


class ConfigError(Exception):
    """Berkas konfigurasi atau skenario tidak bisa dipakai."""


@dataclass(frozen=True)
class Config:
    # Lapangan RoboCup pada paper.
    field_width: float = 14.0
    field_height: float = 9.0

    # Robot. Kecepatan maksimum mengikuti paper (humanoid berlari sekitar 2 m/s);
    # percepatan dan kecepatan putar adalah asumsi kelompok.
    robot_radius: float = 0.35
    max_speed: float = 2.0
    max_accel: float = 2.0
    max_omega: float = 2.0
    max_alpha: float = 6.0
    manual_speed_ratio: float = 0.5  # WASD/QE memakai sebagian dari batas

    # Obstacle.
    obstacle_max_speed: float = 2.0
    polygon_sides: int = 18  # Fig. 10 pada paper
    safety_margin: float = 0.2  # tambahan pada zona aman di luar radius robot

    # Disiapkan untuk planning dan control; belum dipakai pada MVP.
    lambda_turn: float = 1.0
    rho_slack: float = 100.0
    horizon: int = 10
    dt_mpc: float = 0.25
    goal_threshold: float = 0.3
    noise_std: float = 0.0

    def safe_radius(self, obstacle: Obstacle) -> float:
        """Radius zona aman di sekitar obstacle, yang dipakai MPC kelak."""
        return obstacle.radius + self.robot_radius + self.safety_margin


# nama: (jenis, batas bawah, batas bawah eksklusif, batas atas)
_RULES: dict[str, tuple[str, Optional[float], bool, Optional[float]]] = {
    "field_width": ("float", 0.0, True, None),
    "field_height": ("float", 0.0, True, None),
    "robot_radius": ("float", 0.0, True, None),
    "max_speed": ("float", 0.0, True, None),
    "max_accel": ("float", 0.0, True, None),
    "max_omega": ("float", 0.0, True, None),
    "max_alpha": ("float", 0.0, True, None),
    "manual_speed_ratio": ("float", 0.0, True, 1.0),
    "obstacle_max_speed": ("float", 0.0, True, None),
    "polygon_sides": ("int", 3, False, None),
    "safety_margin": ("float", 0.0, False, None),
    "lambda_turn": ("float", 0.0, False, None),
    "rho_slack": ("float", 0.0, True, None),
    "horizon": ("int", 1, False, None),
    "dt_mpc": ("float", 0.0, True, None),
    "goal_threshold": ("float", 0.0, True, None),
    "noise_std": ("float", 0.0, False, None),
}


def _number(label: str, value: Any, kind: str = "float", lo: Optional[float] = None,
            lo_strict: bool = False, hi: Optional[float] = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{label}: harus berupa angka, bukan {value!r}")
    if kind == "int" and not isinstance(value, int):
        raise ConfigError(f"{label}: harus bilangan bulat, bukan {value!r}")
    if not math.isfinite(value):
        raise ConfigError(f"{label}: harus berhingga, bukan {value!r}")
    if lo is not None and (value <= lo if lo_strict else value < lo):
        relation = "lebih besar dari" if lo_strict else "minimal"
        raise ConfigError(f"{label}: harus {relation} {lo}, nilainya {value}")
    if hi is not None and value > hi:
        raise ConfigError(f"{label}: harus paling besar {hi}, nilainya {value}")
    return int(value) if kind == "int" else float(value)


def _read_json(path: Path) -> Any:
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError as e:
        raise ConfigError(f"{path}: berkas tidak bisa dibaca ({e.strerror or e})") from e
    try:
        return json.loads(text)
    except json.JSONDecodeError as e:
        raise ConfigError(f"{path}: JSON tidak valid (baris {e.lineno}, kolom {e.colno}): {e.msg}") from e


def _check_keys(label: str, data: dict, allowed: set[str], required: set[str] = frozenset()) -> None:
    unknown = set(data) - allowed
    if unknown:
        raise ConfigError(f"{label}: kunci tidak dikenal {sorted(unknown)}; yang valid {sorted(allowed)}")
    missing = required - set(data)
    if missing:
        raise ConfigError(f"{label}: kunci wajib tidak ada {sorted(missing)}")


def load_config(path: Path | str) -> Config:
    path = Path(path)
    data = _read_json(path)
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: isi berkas harus berupa objek JSON")
    _check_keys(str(path), data, {f.name for f in fields(Config)})
    values = {}
    for name, value in data.items():
        kind, lo, lo_strict, hi = _RULES[name]
        values[name] = _number(f"{path}: {name}", value, kind, lo, lo_strict, hi)
    return Config(**values)


# -------------------------------------------------------------- skenario ---

@dataclass(frozen=True)
class TimedEvent:
    t: float
    action: str
    params: dict = field(default_factory=dict)


@dataclass
class Scenario:
    name: str
    start: Pose
    goal: Vec2
    obstacles: list[Obstacle]
    events: list[TimedEvent] = field(default_factory=list)
    layers: dict[str, bool] = field(default_factory=dict)


def build_obstacle(data: Any, obstacle_id: int, cfg: Config, label: str) -> Obstacle:
    """Membuat Obstacle dari kamus JSON; dipakai juga oleh kejadian berwaktu."""
    if not isinstance(data, dict):
        raise ConfigError(f"{label}: harus berupa objek")
    _check_keys(label, data, {"x", "y", "radius", "sides", "vx", "vy"}, {"x", "y", "radius"})
    radius = _number(f"{label}.radius", data["radius"], lo=0.0, lo_strict=True)
    x = _number(f"{label}.x", data["x"], lo=radius, hi=cfg.field_width - radius)
    y = _number(f"{label}.y", data["y"], lo=radius, hi=cfg.field_height - radius)
    sides = _number(f"{label}.sides", data.get("sides", cfg.polygon_sides), "int", 3)
    vx = _number(f"{label}.vx", data.get("vx", 0.0))
    vy = _number(f"{label}.vy", data.get("vy", 0.0))
    if math.hypot(vx, vy) > cfg.obstacle_max_speed:
        raise ConfigError(f"{label}: kelajuan melebihi obstacle_max_speed={cfg.obstacle_max_speed}")
    return Obstacle(obstacle_id, Vec2(x, y), radius, sides, Vec2(vx, vy))


def _parse_event(data: Any, cfg: Config, label: str) -> TimedEvent:
    if not isinstance(data, dict):
        raise ConfigError(f"{label}: harus berupa objek")
    action = data.get("aksi")
    if action not in EVENT_ACTIONS:
        raise ConfigError(f"{label}.aksi: harus salah satu dari {list(EVENT_ACTIONS)}, bukan {action!r}")
    t = _number(f"{label}.t", data.get("t"), lo=0.0)
    if action == "munculkan_obstacle":
        _check_keys(label, data, {"t", "aksi", "obstacle"}, {"t", "aksi", "obstacle"})
        build_obstacle(data["obstacle"], -1, cfg, f"{label}.obstacle")  # hanya memeriksa
        return TimedEvent(t, action, {"obstacle": data["obstacle"]})
    _check_keys(label, data, {"t", "aksi", "id"}, {"t", "aksi", "id"})
    return TimedEvent(t, action, {"id": int(_number(f"{label}.id", data["id"], "int", 0))})


def load_scenario(path: Path | str, cfg: Config) -> Scenario:
    path = Path(path)
    data = _read_json(path)
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: isi berkas harus berupa objek JSON")
    _check_keys(str(path), data, {"nama", "start", "goal", "obstacles", "events", "layers"},
                {"start", "goal"})

    start = data["start"]
    _check_keys(f"{path}: start", start if isinstance(start, dict) else {}, {"x", "y", "theta"}, {"x", "y"})
    r = cfg.robot_radius
    start_pose = Pose(
        _number(f"{path}: start.x", start["x"], lo=r, hi=cfg.field_width - r),
        _number(f"{path}: start.y", start["y"], lo=r, hi=cfg.field_height - r),
        _number(f"{path}: start.theta", start.get("theta", 0.0)),
    )

    goal = data["goal"]
    _check_keys(f"{path}: goal", goal if isinstance(goal, dict) else {}, {"x", "y"}, {"x", "y"})
    goal_pos = Vec2(
        _number(f"{path}: goal.x", goal["x"], lo=0.0, hi=cfg.field_width),
        _number(f"{path}: goal.y", goal["y"], lo=0.0, hi=cfg.field_height),
    )

    raw_obstacles = data.get("obstacles", [])
    if not isinstance(raw_obstacles, list):
        raise ConfigError(f"{path}: obstacles harus berupa daftar")
    obstacles = [build_obstacle(o, i, cfg, f"{path}: obstacles[{i}]") for i, o in enumerate(raw_obstacles)]

    raw_events = data.get("events", [])
    if not isinstance(raw_events, list):
        raise ConfigError(f"{path}: events harus berupa daftar")
    events = sorted((_parse_event(e, cfg, f"{path}: events[{i}]") for i, e in enumerate(raw_events)),
                    key=lambda e: e.t)

    raw_layers = data.get("layers", {})
    if not isinstance(raw_layers, dict):
        raise ConfigError(f"{path}: layers harus berupa objek")
    layers: dict[str, bool] = {}
    for layer_id, on in raw_layers.items():
        if layer_id not in LAYER_IDS:
            raise ConfigError(f"{path}: layers.{layer_id}: lapisan tidak dikenal; yang valid {list(LAYER_IDS)}")
        if not isinstance(on, bool):
            raise ConfigError(f"{path}: layers.{layer_id}: harus true atau false, bukan {on!r}")
        layers[layer_id] = on

    name = data.get("nama", path.stem)
    if not isinstance(name, str):
        raise ConfigError(f"{path}: nama harus berupa teks")
    return Scenario(name, start_pose, goal_pos, obstacles, events, layers)
