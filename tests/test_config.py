import json
from pathlib import Path

import pytest

from sim.config import ConfigError, load_config, load_scenario

ROOT = Path(__file__).resolve().parent.parent


def write(tmp_path, name, data):
    path = tmp_path / name
    path.write_text(data if isinstance(data, str) else json.dumps(data), encoding="utf-8")
    return path


@pytest.fixture
def cfg():
    return load_config(ROOT / "config.json")


def test_config_bawaan_valid(cfg):
    assert (cfg.field_width, cfg.field_height) == (14.0, 9.0)
    assert cfg.polygon_sides == 18


def test_config_sebagian_memakai_nilai_bawaan(tmp_path):
    cfg = load_config(write(tmp_path, "c.json", {"robot_radius": 0.5}))
    assert cfg.robot_radius == 0.5
    assert cfg.max_speed == 2.0


def test_kunci_tidak_dikenal_ditolak(tmp_path):
    with pytest.raises(ConfigError, match="robot_radiu"):
        load_config(write(tmp_path, "c.json", {"robot_radiu": 0.5}))


def test_radius_negatif_ditolak(tmp_path):
    with pytest.raises(ConfigError, match="robot_radius"):
        load_config(write(tmp_path, "c.json", {"robot_radius": -1}))


def test_jumlah_sisi_harus_bilangan_bulat_minimal_tiga(tmp_path):
    with pytest.raises(ConfigError):
        load_config(write(tmp_path, "c.json", {"polygon_sides": 2}))
    with pytest.raises(ConfigError):
        load_config(write(tmp_path, "c.json", {"polygon_sides": 6.5}))


def test_boolean_bukan_angka(tmp_path):
    with pytest.raises(ConfigError):
        load_config(write(tmp_path, "c.json", {"max_speed": True}))


def test_json_rusak_dilaporkan_dengan_nama_berkas(tmp_path):
    path = write(tmp_path, "c.json", "{ bukan json")
    with pytest.raises(ConfigError, match="c.json"):
        load_config(path)


def test_berkas_tidak_ada():
    with pytest.raises(ConfigError):
        load_config(ROOT / "tidak-ada.json")


def test_skenario_bawaan_valid(cfg):
    sc = load_scenario(ROOT / "scenarios" / "default.json", cfg)
    assert len(sc.obstacles) == 3
    assert [o.id for o in sc.obstacles] == [0, 1, 2]
    assert sc.events[0].action == "munculkan_obstacle"
    assert sc.layers["active_region"] is False


BASE = {"start": {"x": 2, "y": 4}, "goal": {"x": 12, "y": 4}}


def scenario(tmp_path, cfg, **extra):
    return load_scenario(write(tmp_path, "s.json", {**BASE, **extra}), cfg)


def test_skenario_minimal(tmp_path, cfg):
    sc = scenario(tmp_path, cfg)
    assert sc.obstacles == [] and sc.events == [] and sc.layers == {}


def test_obstacle_di_luar_lapangan_ditolak(tmp_path, cfg):
    with pytest.raises(ConfigError, match="obstacles"):
        scenario(tmp_path, cfg, obstacles=[{"x": 0.1, "y": 4, "radius": 0.5}])


def test_radius_obstacle_harus_positif(tmp_path, cfg):
    with pytest.raises(ConfigError, match="radius"):
        scenario(tmp_path, cfg, obstacles=[{"x": 5, "y": 4, "radius": 0}])


def test_kelajuan_obstacle_melebihi_batas_ditolak(tmp_path, cfg):
    with pytest.raises(ConfigError, match="obstacle_max_speed"):
        scenario(tmp_path, cfg, obstacles=[{"x": 5, "y": 4, "radius": 0.5, "vx": 5.0}])


def test_lapisan_tidak_dikenal_ditolak(tmp_path, cfg):
    with pytest.raises(ConfigError, match="lapisan tidak dikenal"):
        scenario(tmp_path, cfg, layers={"hantu": True})


def test_aksi_kejadian_tidak_dikenal_ditolak(tmp_path, cfg):
    with pytest.raises(ConfigError, match="aksi"):
        scenario(tmp_path, cfg, events=[{"t": 1, "aksi": "ledakan"}])


def test_kejadian_diurutkan_menurut_waktu(tmp_path, cfg):
    sc = scenario(tmp_path, cfg, events=[
        {"t": 4, "aksi": "hapus_obstacle", "id": 0},
        {"t": 1, "aksi": "munculkan_obstacle", "obstacle": {"x": 5, "y": 4, "radius": 0.5}},
    ])
    assert [e.t for e in sc.events] == [1, 4]


def test_start_wajib_ada(tmp_path, cfg):
    with pytest.raises(ConfigError, match="start"):
        load_scenario(write(tmp_path, "s.json", {"goal": {"x": 1, "y": 1}}), cfg)
