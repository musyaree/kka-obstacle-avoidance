# Simulasi Obstacle Avoidance: DAVG + A* dan LMPC

Simulasi pygame untuk demo mata kuliah KKA (kelompok 8), perencanaan jalur dengan Dynamic Augmented Visibility Graph (DAVG) dan pengendalian dengan collision-free MPC (cf-MPC) untuk robot humanoid di lapangan RoboCup.

## Menjalankan

Butuh Python 3.10 atau lebih baru.

```
pip install pygame numpy
python -m sim.app
```

## Kontrol

| Masukan | Fungsi |
|---|---|
| W / S | maju / mundur |
| A / D | geser kiri / kanan |
| Q / E | putar berlawanan / searah jarum jam |
| Seret robot, goal, atau obstacle | pindahkan posisinya |
| Klik obstacle | pilih; seret ujung panahnya untuk mengatur kecepatan |
| Enter | mulai / hentikan AUTO |
| Spasi | jeda |
| R | muat ulang `config.json` dan skenario |
| C | ganti pengendali |
| Tab | tampil / sembunyikan panel |
| 1–9, 0 | nyala / mati lapisan visualisasi |
| Esc | keluar |

Kecepatan obstacle juga bisa diisi sebagai angka lewat panel.

## Lapisan visualisasi

| Tombol | Lapisan | Status |
|---|---|---|
| 1 | Obstacle | ada |
| 2 | Zona aman | ada |
| 3 | Garis S-G | ada |
| 4 | Active region | menunggu DAVG |
| 5 | Visibility graph | menunggu DAVG |
| 6 | Jalur DAVG | menunggu DAVG |
| 7 | Trajektori | menunggu AUTO |
| 8 | Horizon MPC | menunggu LMPC |
| 9 | Bidang singgung | menunggu LMPC |
| 0 | HUD | ada |

## Konfigurasi

Parameter ada di `config.json` (ukuran lapangan, radius robot, batas kecepatan dan percepatan, margin aman, λ sudut belok, bobot slack, horizon MPC, noise). Skenario ada di `scenarios/*.json`: posisi awal, goal, obstacle, kejadian berwaktu, dan lapisan yang menyala di awal. File yang salah ketik kuncinya ditolak dengan pesan galat, dan tombol R tidak akan membuang keadaan lama kalau muat ulang gagal.

## Struktur

```
sim/
  types.py, geometry.py, config.py   tipe data, geometri, pemuat JSON
  dynamics.py                        model robot dan batasnya
  engine.py                          Engine.tick(dt, Intent) -> Frame, tanpa pygame
  planning/                          DAVG + A* (v1.1)
  control/                           pengendali: pengikut sederhana, LMPC (v2.0)
  app/                               satu-satunya yang mengimpor pygame
tests/                               pytest, tanpa jendela
```

Aturan dependensinya: hanya `sim/app` yang boleh mengimpor pygame, jadi seluruh logika bisa diuji tanpa layar.

## Tes

```
pip install pytest
pytest
```
