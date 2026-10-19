# Validasi akurasi tool posisi matahari (langkah 1 peta kerja)

Dibuat otomatis oleh `validate_sun.py`. Titik panel: lintang -7.276006, bujur 112.794327, zona Asia/Jakarta, ketinggian 5 m (asumsi).
Rentang: seluruh tahun 2026, langkah 10 menit (52560 titik waktu), hanya titik dengan elevasi geometrik acuan > 0
(n = 26266). Pysolar: semua titik. Kondisi atmosfer seragam: 101325 Pa, 12 C.

Acuan: pvlib SPA (`nrel_numpy`). Ukuran utama: **sudut pisah** antara dua vektor arah matahari (deg).
Kolom daz/del: selisih azimuth (terbungkus ke +-180) dan elevasi terpisah, hanya untuk perbandingan dengan hasil awal satu hari.
Kesalahan azimuth meledak di dekat zenith secara artifisial, sehingga maks daz tidak mencerminkan kesalahan arah sebenarnya.

## Hasil: elevasi geometrik (tanpa refraksi)

| Tool | N titik | maks (deg) | rata-rata | P99 | RMSE | maks &#124;daz&#124; | maks &#124;del&#124; |
|---|---|---|---|---|---|---|---|
| pvlib ephemeris | 26266 | 0.01082 | 0.00423 | 0.01000 | 0.00474 | 0.49330 | 0.01074 |
| Pysolar | 26266 | 0.00596 | 0.00513 | 0.00587 | 0.00515 | 0.61809 | 0.00595 |
| SolTrack (C) | 26266 | 0.00889 | 0.00346 | 0.00875 | 0.00420 | 0.53658 | 0.00889 |
| astropy (independen) | 26266 | 0.00058 | 0.00021 | 0.00054 | 0.00024 | 0.02479 | 0.00057 |

## Hasil: elevasi tampak (dengan refraksi, astropy tidak termasuk karena dihitung tanpa refraksi)

| Tool | N titik | maks (deg) | rata-rata | P99 | RMSE | maks &#124;daz&#124; | maks &#124;del&#124; |
|---|---|---|---|---|---|---|---|
| pvlib ephemeris | 26266 | 0.01042 | 0.00393 | 0.00884 | 0.00440 | 0.49330 | 0.01027 |
| Pysolar | 26266 | 0.00595 | 0.00510 | 0.00586 | 0.00513 | 0.61809 | 0.00593 |
| SolTrack (C) | 26266 | 0.00890 | 0.00344 | 0.00874 | 0.00418 | 0.53658 | 0.00890 |

SolTrack: azimut hanya tersedia satu (tanpa koreksi refraksi), dipakai di kedua tabel.
Pysolar: refraksi dihitung dengan `get_refraction_correction` dari elevasi geometriknya sendiri.

## Pemeriksaan independen

astropy `get_sun` + `AltAz` (ERFA/IAU, tanpa refraksi, tabel IERS bawaan tanpa unduhan) tidak berbagi kode dengan SPA, jadi
baris astropy pada tabel geometrik adalah pemeriksaan independen terhadap acuan. Keterbatasan: astropy dan SPA sama-sama
efemeris analitik tingkat detik busur, bukan data observasi; tidak ada pembanding berbasis JPL DE yang dijalankan.

## Benchmark waktu hitung (indikasi, satu tahun = 52560 titik, semua titik termasuk malam)

Mesin: 13th Gen Intel(R) Core(TM) i5-13450HX (WSL Ubuntu, satu proses, tanpa paralelisasi). SolTrack diukur dengan proses C terpisah
(termasuk start proses dan pembacaan teks); Pysolar berupa loop Python per titik; pvlib dan astropy vektor.

| Tool | Total (s) | per titik (us) |
|---|---|---|
| pvlib SPA (acuan) | 0.34 | 6.5 |
| pvlib ephemeris | 0.04 | 0.7 |
| Pysolar | 16.98 | 323.0 |
| SolTrack (C) | 0.32 | 6.0 |
| astropy (independen) | 9.79 | 186.3 |

## Catatan mikrokontroler dan lisensi (satu kalimat per tool)

- pvlib: pustaka Python/NumPy, tidak cocok langsung untuk mikrokontroler; lisensi BSD-3.
- Pysolar: pustaka Python murni, tidak cocok langsung untuk mikrokontroler; lisensi GPL.
- SolTrack: ditulis dalam C ringan tanpa dependensi selain `libm` (driver di sini hanya dikompilasi dengan gcc di PC); README proyek
  menyebut port Arduino, tetapi port tersebut tidak dijalankan atau diverifikasi di sini; lisensi LGPL-3.
- astropy: pustaka Python besar, hanya dipakai sebagai pemeriksaan di PC; lisensi BSD-3 (tidak diverifikasi ulang di sini).

## Keterbatasan

- Acuan SPA bukan kebenaran mutlak; hanya astropy yang independen, dan ia hanya diuji tanpa refraksi.
- Selisih tampak tergantung model refraksi tiap tool; seluruhnya memakai tekanan dan suhu yang sama.
- Delta T / UT1 mengikuti default tiap tool (pvlib: perkiraan, astropy: tabel IERS bawaan, SolTrack dan Pysolar: model internal).
- Benchmark hanya indikasi (satu mesin, satu kali jalan, tanpa pengulangan).
