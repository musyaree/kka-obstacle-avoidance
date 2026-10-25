# Rencana Kerja: Menutup v1.1 (DAVG + A*), Pengikut, AUTO, dan Lapisan 4–6

Berkas ini melengkapi `PRD.md`, bukan menggantikannya. `PRD.md` tetap acuan;
berkas ini mencatat urutan kerja, temuan lapangan, dan keputusan yang perlu diambil.
Diperbarui: 25 Oktober 2026 (tenggat ETS).

Cakupan rencana ini: memperbaiki `planning/`, menguji `control/`, menyalakan mode
AUTO di `Engine`, dan menambah lapisan 4–6 supaya DAVG bisa diperagakan.
**`cvxpy` / cf-MPC (v2.0) sengaja di luar cakupan.**

---

## 1. Posisi saat ini

Riwayat commit (3 penulis, sesuai pembagian kerja PRD §4.7):

| Commit | Tanggal | Penulis | Isi |
|---|---|---|---|
| `0b500c4` | 2026-10-24 | musyaree | `init simulation` — MVP lengkap: `types`, `geometry`, `config`, `dynamics`, `engine` (MANUAL), `app/` penuh, 4 berkas tes, `PRD.md`, `README.md`, `config.json`, `scenarios/default.json` |
| `72765ad` | 2026-10-24 | xbggnr (Ben) | `DAVG + A*` — `planning/__init__.py`, `planning/graph.py`, `planning/search.py` |
| `2557a47` | 2026-10-25 | fahmiilmi14 | `followerController` — `control/__init__.py`, `control/follower.py` |

Baca: commit pertama adalah MVP yang utuh dan sudah diuji. Dua commit terakhir
adalah v1.1 yang **belum pernah tersambung ke `Engine` dan belum punya satu pun tes.**

### Peta status terhadap fase PRD §5.1

| Fase | Isi | Status |
|---|---|---|
| MVP | pygame, lapangan, robot manual, obstacle geser, panel, lapisan 0–3 | **selesai** (commit 1) |
| v1.1 | DAVG + A* + lapisan 4–7, mode AUTO dengan pengikut sederhana, muat ulang R | **kode ada, rusak, belum tersambung** |
| v2.0 | cf-MPC + lapisan 8–9, tombol C, skenario berwaktu, kasus uji lengkap | **di luar cakupan sekarang** |

### Yang sudah ada vs yang bolong

| Bagian | Keadaan |
|---|---|
| `types`, `geometry`, `config`, `dynamics` | beres, ada tes |
| `app/` (panel, input, viewport, field, 10 lapisan terdaftar) | beres |
| `engine.py` | hanya MANUAL. `Enter` masih menampilkan "belum tersedia pada MVP" |
| `planning/` | terisi tapi **gagal di ketiga kasus probe** (§2) |
| `control/follower.py` | terisi, **belum pernah dipanggil dari mana pun**, belum ada tes |
| `control/cfmpc.py` | tidak ada; `control/__init__.py` menelan `ImportError` → `CfMpcController = None` |
| lapisan 4–9 di `app/layers.py` | `_EmptyLayer`, menggambar nol |
| `tests/` | `test_config`, `test_dynamics`, `test_engine`, `test_geometry`. **Tidak ada tes untuk planning/control** |

---

## 2. Bukti kerusakan DAVG

Dijalankan langsung lewat antarmuka modul (`sim.planning.plan`), lapangan 14×9,
robot di (2, 4.5), goal di (12, 4.5):

| Kasus | Harapan (PRD §4.6) | Hasil nyata |
|---|---|---|
| tanpa obstacle | `OK`, jalur = garis lurus | `NO_PATH`, 0 titik |
| obstacle di luar garis S-G | `OK`, jalur = garis lurus | `NO_PATH`, 0 titik |
| obstacle di garis S-G (kasus 2) | `OK`, jalur memutari | **`TypeError: 'list' object is not callable`** |

Penyebab pasti, per berkas:

**`sim/planning/graph.py`**

1. `_select_active_region`: `self.active_obstacles(obs)` — memanggil *list*, bukan
   method. Harusnya `active.append(obs)`. Ini sumber `TypeError`.
2. `_select_active_region`: perluasan wilayah hanya **satu lintasan**, dan variabel
   `changed` dihitung tetapi tidak dipakai. PRD §4.2.1 minta "diperluas berulang
   sampai tidak ada obstacle lain yang memotongnya" → harus loop sampai titik tetap.
3. `build()`: `if not self._is_covered:` — kurang `()`. Bound method selalu truthy,
   jadi tidak ada vertex yang pernah disaring. Filter oklusi mati total.
4. `build()`: uji keterlihatan **berada di dalam** `for poly in active_polys:` di
   bawah `if not hits:`. Akibatnya:
   - tanpa obstacle aktif, badan loop tidak jalan → **edge start↔goal tidak pernah
     dibuat** (ini yang membuat dua kasus pertama `NO_PATH`);
   - begitu satu poligon kena, `break` keluar tanpa menambah edge;
   - saat bebas, edge ditambahkan **sekali per poligon** → edge ganda.
5. Tidak ada penanganan start/goal **di dalam** obstacle (PRD §4.2.5): harus pilih
   vertex keluar terdekat, dan itu **bukan** `NO_PATH`.
6. Graf dibangun dari poligon mentah; radius robot + `safety_margin` belum
   diperhitungkan. Perlu keputusan (lihat §4, K3).

**`sim/planning/search.py`**

7. `heapq` menerima tuple `(f, g, Vec2, Vec2|None, list)`. `Vec2` adalah
   `@dataclass(frozen=True)` tanpa `order=True`, jadi **tidak punya `__lt__`**:
   begitu dua entri seri pada `(f, g)`, perbandingan naik jadi `TypeError`.
   Butuh penghitung tie-break sebagai elemen ketiga.
8. `path + [nxt]` menyalin seluruh jalur pada setiap push → O(n²) waktu dan memori.
9. `interpolate_trajectory(path, v_max, dt)` **mengabaikan kedua argumennya** dan
   mengembalikan `Vec2` mentah. Akibatnya tidak ada timestamp (PRD §4.2.6, lapisan 7)
   dan `FollowerController.track()` yang membaca `target.pos`, `target.speed`,
   `target.theta` akan `AttributeError` saat dijalankan.

**`sim/planning/__init__.py`**

10. `getattr(cfg, 'v_max', 2.0)` dan `getattr(cfg, 'dt', 0.25)` — nama field yang
    benar adalah `max_speed` dan `dt_mpc`. Karena `getattr` bertoleransi, kesalahan
    ini **diam-diam** memakai nilai default.
11. `PlanResult.edges` bertipe `list` tetapi diisi `dict`; `PlanResult.region` berisi
    tuple bbox saja, tidak cukup bagi lapisan 4 untuk menyorot obstacle aktif.
12. Nama kelas menyimpang dari PRD §4.3: `AugmentedAStar` vs `AugmentedSearch`,
    `interpolate_trajectory` vs `TrajectoryInterpolator`, dan `Planner` belum ada.

---

## 3. Rencana kerja bertahap

Urutan mengikuti permintaan: **DAVG → pengikut → AUTO → lapisan 4–6**, dengan
README dan PRD dirapikan setelahnya. Setiap tahap berhenti di keadaan hijau
(tes lulus) sebelum tahap berikutnya dimulai.

### T1 — Perbaiki dan uji `planning/` (DAVG + A*)

Berkas: `sim/planning/graph.py`, `sim/planning/search.py`,
`sim/planning/__init__.py`, `tests/test_planning.py` (baru).

Langkah:

1. Perbaiki butir 1–6 di `graph.py`: `append`, loop titik tetap, panggil
   `_is_covered()`, pindahkan uji keterlihatan **keluar** dari loop poligon, dan
   penanganan start/goal di dalam obstacle.
2. Perbaiki butir 7–9 di `search.py`: tie-break counter, rekonstruksi jalur lewat
   `came_from` (bukan salin list), dan interpolasi berwaktu.
3. Perbaiki butir 10–11 di `__init__.py`: pakai `cfg.max_speed` / `cfg.dt_mpc`,
   dan sepakati bentuk `PlanResult` (lihat §4 K2).
4. Tulis `tests/test_planning.py` memetakan PRD §4.6 kasus 1–5.

Kriteria selesai:

- `plan()` pada kasus tanpa obstacle → `OK`, `path == [start, goal]`.
- obstacle di tengah → `OK`, jalur tidak memotong poligon obstacle.
- obstacle tidak aktif → hasil **identik** dengan tanpa obstacle.
- start (dan goal) di dalam obstacle → `OK`, bukan `NO_PATH`.
- goal tertutup rapat → `NO_PATH` **tanpa exception**.
- Arah putar berubah sesuai sudut awal (PRD §4.6 kasus 2, Fig. 11a/b).
- Determinisme: masukan sama → jalur sama persis.

### T2 — Uji dan rapikan `control/follower.py`

Berkas: `sim/control/follower.py`, `tests/test_follower.py` (baru).

1. Sambungkan ke tipe trajektori hasil T1 (`TrajectoryPoint` dengan `pos`, `speed`,
   `theta`, `t`) supaya tidak `AttributeError`.
2. Pastikan `command` tidak pernah melampaui `cfg.max_speed` / `cfg.max_omega`.
3. Tulis tes: trajektori kosong → perintah nol; trajektori lurus → arah benar;
   horizon berisi `cfg.horizon` titik; kecepatan ≤ batas.

Kriteria selesai: `FollowerController.track()` dapat diuji dengan trajektori buatan
tangan **tanpa** DAVG (sesuai catatan PRD §4.7 untuk orang 4).

### T3 — Mode AUTO di `Engine` (PRD §2.3)

Berkas: `sim/engine.py`, `tests/test_engine.py` (diperbarui), `tests/test_auto.py`
(baru, opsional).

1. `Engine` memegang planner dan pengendali aktif.
2. `_frame()` mengisi `plan`, `track`, dan `controller_name` → HUD dan waktu solve ikut hidup.
3. Terapkan tabel mesin status PRD §2.3 **persis**: Enter masuk AUTO; WASD/QE keluar;
   Enter keluar; sampai goal (`cfg.goal_threshold`) keluar; geser goal/obstacle →
   replan; seret robot saat AUTO → pindah lalu lanjut; tidak ada jalur → robot diam
   + pesan "Tidak ada jalur".
4. Pesan "belum tersedia pada MVP" dihapus, dan
   `test_enter_belum_mengaktifkan_auto_pada_mvp` di `tests/test_engine.py` **harus
   ditulis ulang** — tes itu sengaja mengunci perilaku MVP yang sekarang kita ganti.
5. Alur diuji lewat daftar `Intent` per frame, tanpa pygame (PRD §4.6 tambahan).

Kriteria selesai: robot berjalan dari start ke goal pada `scenarios/default.json`
tanpa input manual, dan tetap stabil saat goal digeser dan saat goal ditutup.

### T4 — Lapisan visualisasi 4–6

Berkas: `sim/app/layers.py`, `sim/types.py` (bila `PlanResult` dirapikan).

| Lapisan | Isi |
|---|---|
| 4 Active region | kotak wilayah aktif + obstacle aktif disorot |
| 5 Visibility graph | seluruh edge pada graf |
| 6 Jalur DAVG | polyline jalur + vertex |

Kriteria selesai: menekan `4`, `5`, `6` menampilkan data nyata dari `PlanResult`,
dan `6` menyala secara default (PRD §2.5).

### T5 — README dan status PRD

- `README.md`: status lapisan (4–6 jadi "ada"), tabel kontrol (perilaku `C`),
  cara menjalankan (venv + `pytest`), dan struktur ( `planning/` dan `control/`
  tidak lagi hanya `.gitkeep`).
- `PRD.md` §5.1: tandai fase MVP dan v1.1 selesai; catat cf-MPC sebagai tertunda.

### T6 — Kebersihan

- `.venv` proyek + `pytest` terpasang (lihat §4 K4).
- Satu commit per area kerja, pesan mengikuti gaya riwayat yang ada
  (`Feat: ...`, `Fix: ...`).

---

## 4. Keputusan yang perlu diambil

| # | Pertanyaan | Usulan |
|---|---|---|
| K1 | Berhenti untuk ditinjau tiap tahap, atau kerjakan T1–T4 sekali jalan? | Berhenti setelah T1 dan T3 (dua titik paling berisiko) |
| K2 | Bentuk `PlanResult.edges` dan `.region`? | `edges` = daftar pasangan `(u, v)`; `region` = objek `ActiveRegion(bbox, obstacle_ids)` agar lapisan 4 bisa menyorot |
| K3 | Graf memakai poligon mentah atau yang sudah digelembungi radius robot + `safety_margin`? | **Digelembungi** — konsisten dengan zona aman `R_obs` yang sudah dipakai lapisan 2, meski ongkos tepi naik |
| K4 | Lingkungan Python: `.venv` proyek atau Python yang terpasang? | **`.venv` proyek.** Python di PATH adalah milik ESP-IDF (`C:\Espressif\tools\python`); jangan menaruh `pytest`/`pygame` di sana |
| K5 | Cadence replanning: tiap frame atau hanya saat ada perubahan? | Mulai tiap frame, **ukur** dulu; kalau > ~15 ms/plan, beralih ke deteksi perubahan (PRD §4.2 dan risiko §5.2) |
| K6 | Perilaku tombol `C` selagi cf-MPC di luar cakupan? | Tampilkan hanya pengikut sederhana + pesan bahwa cf-MPC belum terpasang (jangan tombol mati yang membingungkan) |
| K7 | Tambah tipe `TrajectoryPoint` di `types.py`? | Ya — dibutuhkan lapisan 7 dan `FollowerController`; perlu sekali sentuh `types.py`, titik koordinasi paling berat menurut PRD §4.7 |
| K8 | Samakan nama kelas dengan PRD (`AugmentedSearch`, `TrajectoryInterpolator`, `Planner`)? | Ya, sekalian saat T1 karena berkasnya sedang dibuka |

---

## 5. Di luar cakupan (sengaja ditunda)

- `control/cfmpc.py` (LMPC dengan slack) dan lapisan 8–9.
- Perbandingan cf-MPC vs pengikut sederhana lewat tombol `C`.
- Lapisan 7 (trajektori) — murah, tetapi tidak diminta pada urutan ini.
- NMPC, prediksi gerak lawan, tangkapan layar/perekaman (PRD §2.6).

---

## 6. Risiko

| Risiko | Penanganan |
|---|---|
| Replanning tiap frame lambat di Python | Ukur di T1/T3, siap beralih ke deteksi perubahan (K5) |
| `pytest` belum terpasang, jadi klaim "beres" tak terbukti | Pasang `.venv` sebelum T1 (K4); T1–T3 wajib lulus tes |
| Kelas penamaan PRD vs kode menyimpang, membingungkan saat presentasi | Samakan di T1 (K8) |
| Tenggat hari ini | Pengikut sederhana sudah cukup untuk demo; cf-MPC tetap cadangan |

---

## 7. Berkas yang akan disentuh

| Berkas | Tahap |
|---|---|
| `sim/planning/graph.py`, `sim/planning/search.py`, `sim/planning/__init__.py` | T1 |
| `tests/test_planning.py` (baru) | T1 |
| `sim/types.py` | T1 / T4 (bergantung K2, K7) |
| `sim/control/follower.py` | T2 |
| `tests/test_follower.py` (baru) | T2 |
| `sim/engine.py` | T3 |
| `tests/test_engine.py` (diperbarui) | T3 |
| `sim/app/layers.py` | T4 |
| `README.md`, `PRD.md` | T5 |
