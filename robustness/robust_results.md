# Uji ketahanan Monte Carlo terhadap parameter sensor (dibuat otomatis oleh robust_mc.py)

128 sampel parameter sensor (Sobol scrambled, seed 20261004), hari 2026-10-04, cerah (seed 0) dan sebagian (rata-rata seed 1 dan 2), salah pasang 0/0, 3/2, 6/4 deg. Total 4608 simulasi, waktu jalan 159 s (16 proses). Rentang: r log-seragam [0.5, 3]; mism seragam [0, 0.10]; L0 log-seragam [5000, 80000]; gam seragam [0.5, 0.9]; vn log-seragam [0.001, 0.005] V. Semua rentang adalah asumsi (sensor belum diukur). RMSE = galat pointing (deg), zenith <= 45 deg. Lantai galat = hypot(mism/sqrt2, 0.707 vn/V) / kemiringan e per deg, diukur numerik pada kasus cerah.


## (a) Sebaran RMSE (deg) antar sampel sensor

| weather | mp | ctrl | median | P10 | P90 |
|---|---|---|---|---|---|
| cerah | 0/0 | astro | 0.0363 | 0.0363 | 0.0363 |
| cerah | 0/0 | bias | 4.17 | 0.969 | 8.32 |
| cerah | 0/0 | ldr | 5.32 | 1.95 | 17.1 |
| cerah | 0/0 | saputra | 5.27 | 1.35 | 12.1 |
| cerah | 3/2 | astro | 2.4 | 2.4 | 2.4 |
| cerah | 3/2 | bias | 4.35 | 1.04 | 9.18 |
| cerah | 3/2 | ldr | 5.38 | 1.71 | 14.9 |
| cerah | 3/2 | saputra | 5.21 | 1.32 | 12.3 |
| cerah | 6/4 | astro | 4.84 | 4.84 | 4.84 |
| cerah | 6/4 | bias | 4.73 | 1.06 | 9.99 |
| cerah | 6/4 | ldr | 5.41 | 1.69 | 12.7 |
| cerah | 6/4 | saputra | 5.12 | 1.31 | 12.3 |
| sebagian | 0/0 | astro | 0.0363 | 0.0363 | 0.0363 |
| sebagian | 0/0 | bias | 2.51 | 0.744 | 4.17 |
| sebagian | 0/0 | ldr | 18.2 | 7.94 | 25.7 |
| sebagian | 0/0 | saputra | 3.21 | 0.994 | 6.89 |
| sebagian | 3/2 | astro | 2.4 | 2.4 | 2.4 |
| sebagian | 3/2 | bias | 2.2 | 0.782 | 3.81 |
| sebagian | 3/2 | ldr | 18.6 | 6.92 | 26 |
| sebagian | 3/2 | saputra | 3.6 | 1.8 | 7.06 |
| sebagian | 6/4 | astro | 4.84 | 4.84 | 4.84 |
| sebagian | 6/4 | bias | 1.77 | 0.731 | 3.56 |
| sebagian | 6/4 | ldr | 18.8 | 7.65 | 26.9 |
| sebagian | 6/4 | saputra | 4.44 | 3.1 | 7.6 |


## (b) Fraksi sampel sensor di mana RMSE kontroler < RMSE astro (LDR membantu)

| weather | mp | frac_bias_lt_astro | frac_saputra_lt_astro | frac_ldr_lt_astro | astro_rmse | floor_median |
|---|---|---|---|---|---|---|
| cerah | 0/0 | 0 | 0 | 0 | 0.0363 | 18.2 |
| cerah | 3/2 | 0.242 | 0.234 | 0.172 | 2.4 | 18.2 |
| cerah | 6/4 | 0.5 | 0.492 | 0.484 | 4.84 | 18.2 |
| sebagian | 0/0 | 0 | 0 | 0 | 0.0363 | 18.2 |
| sebagian | 3/2 | 0.617 | 0.211 | 0.0234 | 2.4 | 18.2 |
| sebagian | 6/4 | 0.977 | 0.594 | 0.0547 | 4.84 | 18.2 |


### Titik impas lantai galat per level (ambang lantai galat dengan akurasi pemisahan menang/kalah tertinggi)

| weather | mp | floor_thr_deg | akurasi | n_win | n |
|---|---|---|---|---|---|
| cerah | 3/2 | 6.48 | 0.938 | 31 | 128 |
| cerah | 6/4 | 14.9 | 0.859 | 64 | 128 |
| sebagian | 3/2 | 23.2 | 0.812 | 79 | 128 |
| sebagian | 6/4 | 139 | 0.977 | 125 | 128 |


## (c) Aturan desain turunan dari data

- Cuaca cerah: hibrid bias mengalahkan astro bila (lantai galat sensor) / (RMSE astro akibat salah pasang) <= 3.10 (akurasi klasifikasi 89% pada level 3/2 dan 6/4 digabung; LDR menang di 37% sampel).
- Cuaca sebagian: hibrid bias mengalahkan astro bila (lantai galat sensor) / (RMSE astro akibat salah pasang) <= 15.10 (akurasi klasifikasi 87% pada level 3/2 dan 6/4 digabung; LDR menang di 80% sampel).
- Dalam derajat (cerah): ambang lantai galat 6.48 deg untuk 3/2 (RMSE astro 2.40 deg), 14.89 deg untuk 6/4 (RMSE astro 4.84 deg).
- Dalam derajat (sebagian): ambang lantai galat 23.19 deg untuk 3/2 (RMSE astro 2.40 deg), 139.35 deg untuk 6/4 (RMSE astro 4.84 deg).
- Tanpa salah pasang (0/0): hibrid bias mengalahkan astro hanya di 0% (cerah) dan 0% (sebagian) sampel, jadi LDR tidak menambah apa pun di sana.

## (d) Korelasi peringkat Spearman antara parameter sensor dan RMSE hibrid bias

| weather | mp | r | mism | L0 | gam | vn | floor |
|---|---|---|---|---|---|---|---|
| cerah | 0/0 | -0.566 | 0.783 | 0.0426 | 0.0156 | 0.0434 | 0.879 |
| cerah | 3/2 | -0.585 | 0.775 | 0.015 | 0.0231 | 0.0171 | 0.892 |
| cerah | 6/4 | -0.601 | 0.761 | -0.0125 | 0.0247 | -0.0169 | 0.903 |
| sebagian | 0/0 | -0.524 | 0.802 | 0.112 | 0.0213 | 0.088 | 0.836 |
| sebagian | 3/2 | -0.587 | 0.767 | 0.0287 | 0.0194 | 0.0111 | 0.871 |
| sebagian | 6/4 | -0.603 | 0.741 | -0.0157 | 0.0327 | -0.0295 | 0.875 |


Peringkat pengaruh (rata-rata |rho| pada salah pasang 3/2 dan 6/4, cerah dan sebagian): floor (0.88), mism (0.76), r (0.59), gam (0.02), vn (0.02), L0 (0.02).


## Keterbatasan

- Semua rentang parameter adalah asumsi; kesimpulan hanya berlaku di dalam rentang itu, bukan prediksi untuk sensor nyata. Ukur sensor, lalu bandingkan dengan peta ini.
- Parameter disampel bebas satu sama lain (tanpa korelasi fisik). Jumlah sampel terbatas, jadi ambang adalah taksiran, bukan batas tegas.
- Satu hari (2026-10-04), satu seed cerah, dua seed sebagian (awan sintetis). Realisasi ketidakcocokan antar LDR berasal dari seed simulate() (seed+77), sehingga pola relatif empat LDR sama antar sampel dan hanya skalanya (mism) yang berubah.
- Lantai galat teoretis memakai kemiringan sinyal pada satu pose dan irradiance (DNI 800, DHI 100 W/m2); lantai nyata berubah menurut waktu dan pose. Parameter kontroler (kp, dead, kcthr) tidak ikut diubah.
- Model kuasi-statis, tanpa dinamika motor dan angin. Salah pasang hanya tiga level, tafsiran di antaranya adalah interpolasi.

