# PRD: Simulasi Obstacle Avoidance Humanoid (DAVG + A* dan LMPC)

| | |
|---|---|
| Proyek | Demo program mata kuliah KKA, kelompok 8 |
| Acuan | Hou, Fernandez, Zhu, dan Hong, "Model Predictive Control with Visibility Graphs for Humanoid Path Planning and Tracking Against Adversarial Opponents" (ICRA 2025, arXiv 2504.02184) |
| Batas waktu | 25 Oktober 2026 |
| Status | Presentasi slide sudah dilakukan; tersisa demo program |

Jika kode berbeda dengan dokumen ini, dokumen diperbarui terlebih dahulu.

---

## 1. Ringkasan Eksekutif

**Pernyataan masalah.** Paper menjelaskan DAVG dan cf-MPC lewat gambar dan persamaan, tetapi kelompok belum punya sesuatu yang bisa dijalankan untuk memperlihatkan bagaimana jalur terbentuk dan bagaimana robot menghindar saat obstacle bergerak atau muncul mendadak.

**Solusi yang diusulkan.** Simulasi 2D berbasis pygame yang menjalankan DAVG + A* untuk perencanaan jalur dan LMPC (cf-MPC) untuk pelacakan, dengan lapisan visualisasi yang bisa dinyalakan satu per satu, objek yang bisa digeser dengan mouse, dan parameter yang diubah lewat file `.json`.

**Kriteria keberhasilan.**

| # | Kriteria | Ukuran |
|---|---|---|
| K1 | Kelancaran | Sekitar 60 FPS pada laptop yang dipakai demo, dengan skenario berisi 6 obstacle |
| K2 | Replanning | Jalur DAVG terhitung ulang tanpa menurunkan FPS di bawah 30 saat obstacle digeser |
| K3 | Kebenaran | Enam kasus uji di bagian 4.6 lulus (`pytest`) |
| K4 | Keterulangan | Setelah menekan R, tampilan skenario sama persis dengan sebelumnya |
| K5 | Kemudahan demo | Dalam demo 15 menit, kelompok dapat menyalakan lapisan satu per satu mengikuti urutan slide tanpa membuka kode |

---

## 2. Pengalaman Pengguna dan Fungsionalitas

### 2.1 Persona

- **Presenter (anggota kelompok):** mengoperasikan simulasi di depan kelas, perlu kontrol cepat dan tampilan yang jelas.
- **Penonton (dosen dan teman sekelas):** melihat layar, mengajukan pertanyaan seperti "bagaimana jika obstacle ini digeser?".
- **Pengembang (anggota kelompok):** mengubah parameter lewat JSON dan menambah skenario atau lapisan.

### 2.2 Cerita Pengguna dan Kriteria Penerimaan

**S1. Mengemudikan robot secara manual**
Sebagai presenter, saya ingin menggerakkan robot dengan WASD dan QE agar bisa menempatkannya di posisi demo.
- W/S untuk maju dan mundur, A/D untuk bergeser ke samping, Q/E untuk berputar.
- Robot memakai model omnidirectional (state x, y, θ; input vₓ, v_y, ω).
- Kecepatan dan percepatan robot dibatasi sesuai `config.json`.

**S2. Menjalankan perencanaan dan pelacakan otomatis**
Sebagai presenter, saya ingin menekan satu tombol agar robot menuju goal dengan DAVG + cf-MPC.
- Enter dari mode MANUAL masuk ke mode AUTO.
- Mode aktif terlihat jelas di HUD.
- Perilaku mode mengikuti tabel mesin status (bagian 2.3).

**S3. Menggeser objek dengan mouse**
Sebagai presenter, saya ingin menyeret robot, goal, dan obstacle agar bisa menjawab pertanyaan penonton secara langsung.
- Klik-seret memindahkan robot, goal, atau obstacle.
- Saat AUTO berjalan, jalur dihitung ulang pada perubahan berikutnya.
- Menyeret robot saat AUTO memindahkannya seketika dan AUTO berlanjut dari posisi baru.

**S4. Mengatur kecepatan obstacle**
Sebagai presenter, saya ingin membuat obstacle bergerak sebagai lawan yang melintas.
- Klik tanpa seret memilih obstacle; obstacle terpilih menampilkan panah kecepatan.
- Kecepatan diatur dengan menyeret ujung panah atau mengetik vₓ dan v_y di panel; keduanya tersinkron.
- Obstacle memantul di batas lapangan; kelajuan dibatasi nilai dari `config.json`.
- Tombol R mengembalikan kecepatan ke nilai awal dari JSON.

**S5. Menyalakan dan mematikan lapisan visualisasi**
Sebagai presenter, saya ingin menampilkan lapisan satu per satu agar penonton mengikuti urutan penjelasan.
- Sepuluh lapisan (bagian 2.5) bisa diatur dari panel (klik) atau tombol angka.
- Panel disembunyikan atau ditampilkan dengan Tab; lapangan melebar penuh saat panel tersembunyi.
- Status awal tiap lapisan dapat diatur per skenario di JSON.

**S6. Mengubah parameter dan skenario lewat JSON**
Sebagai pengembang, saya ingin mengubah parameter tanpa menyentuh kode.
- Parameter algoritma dan robot ada di `config.json`; skenario ada di `scenarios/*.json`.
- Tombol R memuat ulang keduanya tanpa menutup program dan mengembalikan skenario ke keadaan awal.
- JSON yang salah atau bernilai tidak wajar menampilkan pesan galat di layar dan tidak menghentikan program; konfigurasi terakhir yang valid tetap dipakai.

**S7. Membandingkan cf-MPC dengan pengikut sederhana**
Sebagai presenter, saya ingin memperlihatkan apa yang ditambahkan cf-MPC.
- Tombol C beralih antara cf-MPC dan pengikut sederhana (bergerak menuju titik berikutnya tanpa menghindar).
- Pengendali yang aktif tampil di HUD.

**S8. Kasus tanpa jalur**
Sebagai presenter, saya ingin program tetap stabil ketika goal tertutup.
- Robot diam dan layar menampilkan "Tidak ada jalur".
- Robot bergerak lagi begitu jalur tersedia, tanpa tindakan tambahan.

### 2.3 Mesin Status

Mode awal adalah MANUAL.

| Kejadian | Hasil |
|---|---|
| Menekan **Enter** saat MANUAL | Masuk mode AUTO: DAVG merencanakan jalur, cf-MPC melacaknya |
| Menekan **WASD/QE** saat AUTO | AUTO berhenti, kembali ke MANUAL |
| Menekan **Enter** saat AUTO | AUTO berhenti, kembali ke MANUAL |
| Robot sampai di goal (jarak di bawah ambang) | AUTO berhenti, kembali ke MANUAL |
| Menggeser goal atau obstacle saat AUTO | AUTO berlanjut, jalur dihitung ulang |
| Menyeret robot dengan mouse saat AUTO | Robot berpindah ke posisi baru, AUTO berlanjut dari posisi tersebut |
| Tidak ada jalur ke goal | Robot diam dan layar menampilkan "Tidak ada jalur"; robot bergerak lagi begitu jalur tersedia |

Tombol tambahan: **Spasi** menjeda simulasi (objek tetap dapat digeser), **R** memuat ulang konfigurasi dan skenario, dan **C** beralih antara cf-MPC dan pengikut sederhana.

Perpindahan robot saat AUTO sengaja diizinkan. Kondisi ini meniru kasus pada paper ketika estimator menempatkan robot di dalam batas obstacle, sehingga manfaat slack variable dapat diperlihatkan.

### 2.4 Kontrol dan Interaksi

**Keyboard:** WASD/QE untuk robot; Enter untuk mulai atau berhenti AUTO; Spasi untuk jeda; R untuk muat ulang; C untuk ganti pengendali; Tab untuk panel; 1–9 dan 0 untuk lapisan.

**Mouse:**
- Klik dan seret pada robot, goal, atau obstacle memindahkannya.
- Klik tanpa seret pada obstacle memilihnya.
- Menyeret ujung panah kecepatan atau mengetik angka di panel mengatur (vₓ, v_y) obstacle terpilih.

### 2.5 Lapisan Visualisasi

| Tombol | Lapisan | Isi | Default |
|---|---|---|---|
| 1 | Obstacle | Poligon beraturan | ON |
| 2 | Zona aman | Lingkaran radius R_obs yang dipakai MPC | ON |
| 3 | Garis S-G | Garis lurus dari start ke goal | ON |
| 4 | Active region | Kotak wilayah aktif, obstacle aktif disorot | OFF |
| 5 | Visibility graph | Seluruh edge pada graf | OFF |
| 6 | Jalur DAVG | Jalur terpendek hasil A*, lengkap dengan vertex | ON |
| 7 | Trajektori | Titik berwaktu hasil interpolasi linear | OFF |
| 8 | Horizon MPC | Prediksi N langkah beserta referensinya | OFF |
| 9 | Bidang singgung | Constraint linear LMPC pada tiap obstacle | OFF |
| 0 | HUD | Mode, pengendali aktif, FPS, waktu solve DAVG dan MPC, nilai slack | ON |

**Panel samping** di sisi kanan berisi kotak centang kesepuluh lapisan, status singkat (mode, waktu solve, slack), dan isian kecepatan obstacle yang terpilih. Panel tidak dipakai untuk mengubah parameter algoritma; itu dilakukan lewat JSON.

### 2.6 Non-Goals

- NMPC, karena paper memakai SNOPT yang berlisensi. Simulasi hanya mengimplementasikan LMPC.
- Prediksi gerak lawan. Paper menganggap obstacle diam di posisinya saat ini pada setiap siklus, dan simulasi mengikuti asumsi tersebut.
- Kendali bersama (input WASD sebagai kecepatan yang diinginkan, lalu dikoreksi MPC).
- Tangkapan layar dan perekaman video dari dalam program. Keduanya memakai alat bawaan sistem operasi.
- Perbandingan kuantitatif dengan metode lain (A* biasa, RRT, CBF). Simulasi ini bukan alat evaluasi, dan pengikut sederhana hanya alat demo.
- Perencanaan 3D, dinamika humanoid yang sebenarnya, dan perilaku lawan yang mengejar robot.

---

## 3. Persyaratan Sistem AI

Tidak berlaku. Proyek ini tidak memakai model bahasa atau komponen pembelajaran mesin. LMPC adalah optimasi konveks (QP) yang dapat dites secara deterministik; strategi pengujiannya ada di bagian 4.6.

---

## 4. Spesifikasi Teknis

### 4.1 Model

**Robot:** lingkaran dengan radius tertentu, ditambah satu garis radius ke bagian dalam yang menunjukkan arah hadap. Dinamikanya kinematik omnidirectional dengan batas kecepatan dan percepatan. Batas kecepatan mengikuti paper (humanoid berlari sekitar 2 m/s). Batas percepatan tidak tercantum di paper, sehingga nilainya merupakan asumsi kelompok dan harus dilabeli demikian saat dijelaskan. Noise pada estimasi posisi bersifat opsional dan default-nya mati.

**Obstacle:** poligon beraturan (default 18 sisi, sesuai Fig. 10 pada paper) yang dapat diam atau bergerak dengan kecepatan konstan dan memantul di batas lapangan. Obstacle yang saling tumpang tindih diperlakukan sebagai satu wilayah terlarang, bukan sebagai galat.

**Lapangan:** 14 m × 9 m. Koordinat dunia dinyatakan dalam meter, dan satu fungsi (`Viewport`) bertugas mengonversinya ke piksel.

### 4.2 Algoritma

**DAVG:**
1. Obstacle yang memotong garis S-G menjadi active obstacle awal. Wilayah diperluas berulang sampai tidak ada obstacle lain yang memotongnya.
2. Visibility graph dibangun dari start, goal, dan vertex active obstacle yang tidak tertutup obstacle lain. Bobot edge awal adalah jarak Euclidean.
3. Graf diaugmentasi: state berupa pasangan (vertex sebelumnya, vertex sekarang), dan bobot edge menjadi jarak + λ·|sudut belok|. Rumus ini merupakan keputusan kelompok karena paper tidak merinci bobotnya.
4. Pencarian memakai A* dengan heuristik jarak Euclidean ke goal. Heuristik ini tetap admissible karena bobot sudut tidak pernah negatif.
5. Jika start atau goal berada di dalam obstacle, dipilih vertex keluar terdekat: searah heading untuk start, dan searah kedatangan untuk goal. Kasus ini tidak dihitung sebagai "tidak ada jalur".
6. Titik-titik hasil pencarian diinterpolasi secara linear dan diberi timestamp.

Jalur dihitung ulang pada setiap frame. Jika di Python ternyata terlalu lambat, perhitungan diubah menjadi hanya saat ada perubahan.

**cf-MPC (LMPC):** berupa QP yang diselesaikan dengan `cvxpy` memakai `Parameter` agar kompilasi cukup dilakukan sekali. Dinamika dilinearisasi di sekitar trajektori referensi. Constraint tabrakan berupa bidang singgung pada tiap obstacle, dilengkapi slack variable δ ≥ 0 dengan penalti ρΣδ² agar masalah tetap memiliki solusi ketika robot berada di dalam zona obstacle. Constraint kecepatan mengikuti batas robot. Formulasinya tunggal tanpa perpindahan mode. Default horizon 10 langkah dan dt 0,25 s, sesuai simulasi pada paper.

**Pengikut sederhana:** pengendali kedua yang bergerak menuju titik berikutnya pada trajektori dengan batas kecepatan robot, tanpa menghindari obstacle. Fungsinya sebagai alat demo (tombol C), cadangan jika `cvxpy` bermasalah di komputer demo, dan memungkinkan perencana diuji sebelum LMPC selesai.

### 4.3 Arsitektur

Rancangan memakai beberapa modul yang dalam dengan antarmuka kecil, bukan banyak berkas kecil. `Engine` murni (tanpa pygame) dan menjadi satu-satunya tempat mesin status, dinamika, gerak obstacle, dan kejadian berwaktu. Lapisan pygame hanya menggambar `Frame` dan menerjemahkan masukan menjadi `Intent`.

| Modul | Antarmuka | Yang disembunyikan |
|---|---|---|
| `planning` | `plan(scene, cfg) -> PlanResult` | Active region, visibility graph, augmentasi, A*, interpolasi |
| `control` | `track(state, trajectory, obstacles) -> TrackResult` | Linearisasi, QP, slack, batas kecepatan |
| `engine` | `tick(dt, intent) -> Frame` | Mesin status, dinamika robot, gerak obstacle, kejadian berwaktu, replanning |
| `app` | menggambar `Frame`, menghasilkan `Intent` | Panel, lapisan, mouse, pygame |

Keputusan perancangan:

- `PlanResult` selalu memuat jalur, trajektori, status, serta data debug (active region dan seluruh edge graf), tanpa parameter `debug`. Data itu sudah dihitung sebagai bagian pencarian, dan dengan begitu `Engine` tidak perlu tahu lapisan mana yang menyala.
- `TrackResult` memuat kecepatan perintah, prediksi horizon, bidang singgung, nilai slack, dan waktu solve.
- `Engine` hanya punya satu pintu masuk, `tick(dt, intent)`. `Intent` memuat seluruh aksi pada frame itu: perintah manual, tombol, dan daftar perpindahan objek.
- `control` punya dua adapter (cf-MPC dan pengikut sederhana), sehingga seam-nya nyata. `planning` hanya punya satu implementasi, jadi tidak diberi antarmuka yang dapat diganti.
- Seluruh lapisan visualisasi berada di satu berkas, `app/layers.py`, dengan dekorator `@layer(...)` yang mendaftarkannya. Panel dan tombol angka membaca daftar terdaftar itu.

Aturan dependensi: `app` boleh mengimpor semuanya; `engine` mengimpor `planning` dan `control`; `planning` dan `control` hanya mengimpor `types`, `geometry`, dan `config`; hanya `app` yang mengimpor pygame.

```
obstacle-avoidance-sim/
├── pyproject.toml
├── PRD.md
├── config.json
├── scenarios/
├── sim/
│   ├── types.py          Vec2, Pose, Velocity, Mode, Obstacle, Scene, Intent, Frame, PlanResult, TrackResult
│   ├── geometry.py       perpotongan, titik di dalam poligon, poligon beraturan
│   ├── dynamics.py       Robot, RobotLimits (langkah dinamika dengan batas)
│   ├── config.py         Config, Scenario, load_config, load_scenario, ConfigError
│   ├── planning/         antarmuka: plan(scene, cfg) -> PlanResult (masih .gitkeep)
│   │   ├── graph.py      ActiveRegionSelector, VisibilityGraph
│   │   └── search.py     AugmentedSearch, TrajectoryInterpolator, Planner
│   ├── control/          antarmuka: Controller.track(...) -> TrackResult (masih .gitkeep)
│   │   ├── cfmpc.py      CfMpcController (LMPC dengan slack)
│   │   └── follower.py   FollowerController
│   ├── engine.py         Engine, EventScheduler
│   └── app/              satu-satunya yang mengimpor pygame
│       ├── __main__.py   menjalankan: python -m sim.app
│       ├── main.py       App
│       ├── viewport.py   Viewport (meter ke piksel)
│       ├── field.py      garis lapangan RoboCup AdultSize
│       ├── input.py      InputMapper (peristiwa pygame menjadi Intent)
│       ├── panel.py      Panel, NumberField
│       └── layers.py     LayerRegistry, Layer dan sepuluh lapisan
└── tests/                test_geometry, test_dynamics, test_config, test_engine
```

`planning/` dan `control/` baru berisi `.gitkeep`; berkas yang tercantum di dalamnya dibuat pada v1.1 dan v2.0.

`Controller` dan `Layer` adalah kelas abstrak dengan masing-masing dua dan sepuluh subkelas. Diagram kelas Mermaid disusun terpisah dan mengikuti struktur ini.

### 4.4 Konfigurasi dan Skenario

```
config.json            parameter algoritma dan robot
scenarios/*.json       satu file per skenario
```

`config.json` memuat: λ (bobot sudut belok), ρ (penalti slack), horizon dan dt MPC, jumlah sisi poligon, radius robot, batas kecepatan dan percepatan, batas kelajuan obstacle, ambang sampai di goal, serta besar noise (default 0).

Setiap skenario memuat: posisi start dan goal, daftar obstacle (posisi, radius, jumlah sisi, kecepatan awal dengan default 0), kejadian berwaktu (misalnya `{"t": 3.0, "aksi": "munculkan_obstacle", ...}`), dan status awal lapisan.

### 4.5 Persyaratan Non-Fungsional dan Titik Integrasi

- Target sekitar 60 FPS pada laptop yang dipakai untuk demo (K1, K2).
- Logika (geometri, DAVG, A*, LMPC, `Engine`) tidak mengimpor pygame, sehingga dapat diuji tanpa jendela.
- Waktu solve pada HUD adalah hasil ukur simulasi ini sendiri. Angka tersebut tidak boleh dibandingkan langsung dengan Tabel I dan II pada paper, yang diukur di HP Elite Mini 800 G9 dengan SNOPT dan Gurobi.
- Dependensi: `pygame`, `numpy`, `cvxpy` (dengan OSQP), dan `pytest`, dipasang di virtual environment di dalam folder proyek.
- Simulasi berjalan lokal; tidak ada jaringan, basis data, atau autentikasi.

### 4.6 Pengujian

Pengujian memakai `pytest` untuk logika murni dan tidak mencakup tampilan. Pengujian dilakukan lewat antarmuka modul, bukan lewat bagian dalamnya. Kasus yang wajib ada (K3), sebagian besar bersumber dari paper:

1. Tanpa obstacle di garis S-G, jalur berupa garis lurus.
2. Satu obstacle di tengah: jalur memutarinya, dan sisi yang dipilih berubah sesuai sudut awal (Fig. 11a, b).
3. Start atau goal berada di dalam obstacle: vertex keluar yang dipilih benar (Fig. 11c, d).
4. Obstacle yang tidak aktif tidak mengubah hasil.
5. Goal yang tertutup rapat: A* melaporkan "tidak ada jalur" tanpa galat.
6. LMPC tetap menghasilkan solusi saat robot berada di dalam zona obstacle.

Tambahan: pengujian alur lengkap pada `Engine`. Skenario ditulis sebagai daftar `Intent` per frame (misalnya frame 0 menekan Enter, frame 90 menggeser obstacle), dijalankan tanpa pygame, lalu posisi robot dan status diperiksa.

### 4.7 Pembagian Kerja (5 orang)

| Orang | Tanggung jawab | Kelas dan berkas | Bergantung pada |
|---|---|---|---|
| 1 | Fondasi, lalu `Engine` | `types.py`, `geometry.py`, `config.py` (`ScenarioLoader`, `Config`, `ConfigError`), lalu `engine.py` (`Engine`, `EventScheduler`) | Paling awal; yang lain bergantung padanya |
| 2 | Graf visibilitas | `planning/graph.py`: `ActiveRegionSelector`, `ActiveRegion`, `VisibilityGraph` | `types`, `geometry` |
| 3 | Pencarian dan `plan()` | `planning/search.py`: `AugmentedSearch`, `TrajectoryInterpolator`, `Trajectory`, `Planner` | Keluaran orang 2 |
| 4 | Dinamika dan kontrol | `dynamics.py` (`Robot`, `RobotLimits`), `control/`: `Controller`, `CfMpcController`, `FollowerController` | `types`, hasil `plan()` |
| 5 | Aplikasi pygame | `app/`: `App`, `InputMapper`, `Viewport`, `Panel`, `NumberField`, `LayerRegistry`, `Layer` dan sepuluh lapisannya | `Frame` dari `Engine` |

Catatan pelaksanaan:

- `types.py` disepakati bersama dalam satu sesi singkat sebelum kode ditulis. Itulah titik koordinasi yang paling berat.
- Orang 2 dan 3 berbagi satu seam internal: `VisibilityGraph.build()` menghasilkan graf, lalu `AugmentedSearch.findPath()` memakainya. Orang 3 dapat menguji pencarian dengan graf buatan tangan sebelum orang 2 selesai.
- Orang 4 tidak menunggu `planning`; `FollowerController` dan `Robot` dapat diuji dengan trajektori buatan tangan.
- Orang 5 tidak menunggu `Engine`; panel dan lapisan dapat digambar dari objek `Frame` palsu yang dibuat dari `types.py`.
- Setiap orang menulis tes untuk modulnya sendiri.

### 4.8 Keamanan dan Privasi

Tidak ada data pengguna, akun, atau akses jaringan. Satu-satunya masukan eksternal adalah berkas JSON lokal, yang divalidasi sebelum dipakai (bagian S6).

---

## 5. Risiko dan Roadmap

### 5.1 Peluncuran Bertahap

Batas akhir seluruh pekerjaan adalah 25 Oktober 2026. Tanggal untuk tiap fase di bawah ini belum ditetapkan (TBD).

| Fase | Isi | Tanggal |
|---|---|---|
| MVP | Kerangka pygame, lapangan, robot manual (WASD/QE), obstacle yang bisa digeser, panel dan lapisan dasar (1, 2, 3, 0), `types.py` dan `geometry.py` | TBD |
| v1.1 | DAVG + A* lengkap dengan lapisan 4–7, mode AUTO dengan pengikut sederhana, pemuatan JSON dan tombol R | TBD |
| v2.0 | cf-MPC (LMPC dengan slack) dengan lapisan 8–9, tombol C, kecepatan obstacle, skenario berwaktu, kasus uji lengkap | 25 Oktober 2026 |

### 5.2 Risiko Teknis

| Risiko | Dampak | Penanganan |
|---|---|---|
| Waktu hanya sekitar enam hari sejak dokumen ini | Fitur v2.0 tidak selesai | Pengikut sederhana memungkinkan demo tanpa LMPC; urutan fase di atas memberi hasil yang dapat didemokan lebih awal |
| `cvxpy` lambat atau sulit dipasang di komputer demo | cf-MPC tidak berjalan pada 60 FPS | Ukur lebih awal; jika perlu pindahkan modul MPC ke `osqp` langsung tanpa mengubah antarmuka; pengikut sederhana sebagai cadangan |
| Replanning tiap frame terlalu lambat di Python | FPS di bawah 30 saat obstacle digeser | Ubah menjadi replanning hanya saat ada perubahan |
| Jumlah edge graf besar pada banyak obstacle berpoligon 18 sisi | Penurunan FPS dan memori | Kurangi jumlah sisi di `config.json`; tambahkan parameter debug belakangan bila perlu |
| Kesepakatan `types.py` terlambat | Orang 2–5 tertahan | Sesi singkat di awal; Orang 5 memakai `Frame` palsu |
| Orang 1 memegang beban terbesar | Keterlambatan `Engine` | Pindahkan `config.py` ke orang 5 bila perlu |
| LMPC adalah bagian paling sulit secara matematis | Orang 4 terhambat | Serahkan ke anggota yang paling kuat di optimasi |
| Rumus bobot sudut belok tidak ada di paper | Jalur terlihat aneh | Gunakan jarak + λ·|sudut|; nilai λ dicoba dan dapat diubah lewat JSON |

### 5.3 Hal yang Masih Terbuka

- Nilai ambang sampai di goal dan batas percepatan robot.
- Versi Python di komputer demo. Belum diperiksa; yang terpasang saat ini `pygame` 2.5.2 dan `numpy` 1.26.4, sedangkan `cvxpy` dan `osqp` belum ada.
- Apakah demo ditampilkan langsung atau direkam. Hal ini belum dijawab dan memengaruhi jumlah skenario berwaktu yang perlu disiapkan.
- Tanggal untuk fase MVP dan v1.1.
- Rumus pasti bobot sudut belok (usulan: jarak + λ·|sudut|; nilai λ perlu dicoba).
