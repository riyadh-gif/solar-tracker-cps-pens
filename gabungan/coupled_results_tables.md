# Tabel hasil penggabungan dinamika dan kontrol hibrid (dibuat otomatis oleh coupled_sim.py)

2026-10-04, salah pasang 3/2 deg, rata-rata antar seed cuaca (cerah 1, sebagian 2) dan seed angin (3). Galat pointing hanya saat zenith <= 45 deg.


## E1: angin di bawah/di atas torsi tahan, tanpa menerapkan lepas langkah (kejadian hanya dihitung)

ev_az dan ev_el = rata-rata kejadian melewati torsi tahan per hari siang (12 jam), vmax = angin maksimum (m/s).

| sensor | weather | wind | ctrl | rmse | p95 | emax | gain | acts | ev_az | ev_el | vmax |
|---|---|---|---|---|---|---|---|---|---|---|---|
| default | cerah | tanpa_dinamika | astro | 2.4 | 2.88 | 3 | 19.6 | 2.29e+03 | 0 | 0 | 0 |
| default | cerah | gravitasi | astro | 2.58 | 3.15 | 3.27 | 19.7 | 2.29e+03 | 0 | 0 | 0 |
| default | cerah | tenang | astro | 2.58 | 3.16 | 3.37 | 19.7 | 2.29e+03 | 0 | 0 | 7.15 |
| default | cerah | berangin | astro | 2.59 | 3.3 | 4.1 | 19.7 | 2.29e+03 | 6 | 0 | 19.7 |
| default | cerah | badai | astro | 2.66 | 3.86 | 5.74 | 19.7 | 2.29e+03 | 1.06e+03 | 227 | 36 |
| default | cerah | tanpa_dinamika | ldr | 4.48 | 4.79 | 5.03 | 19.3 | 690 | 0 | 0 | 0 |
| default | cerah | gravitasi | ldr | 4.49 | 4.79 | 5 | 19.3 | 709 | 0 | 0 | 0 |
| default | cerah | tenang | ldr | 4.49 | 4.79 | 5.02 | 19.3 | 697 | 0 | 0 | 7.15 |
| default | cerah | berangin | ldr | 4.48 | 4.87 | 5.6 | 19.3 | 736 | 6 | 0.333 | 19.7 |
| default | cerah | badai | ldr | 4.34 | 5.02 | 7.04 | 19.3 | 1.31e+03 | 1.06e+03 | 236 | 36 |
| default | cerah | tanpa_dinamika | saputra | 4.31 | 4.86 | 5.1 | 19.5 | 1.33e+03 | 0 | 0 | 0 |
| default | cerah | gravitasi | saputra | 4.3 | 4.83 | 5.07 | 19.5 | 1.35e+03 | 0 | 0 | 0 |
| default | cerah | tenang | saputra | 4.3 | 4.84 | 5.15 | 19.5 | 1.36e+03 | 0 | 0 | 7.15 |
| default | cerah | berangin | saputra | 4.26 | 4.86 | 5.6 | 19.5 | 1.36e+03 | 6 | 0 | 19.7 |
| default | cerah | badai | saputra | 4.17 | 5.15 | 6.95 | 19.5 | 1.79e+03 | 1.06e+03 | 229 | 36 |
| default | cerah | tanpa_dinamika | bias | 3.85 | 4.61 | 4.84 | 19.5 | 2.33e+03 | 0 | 0 | 0 |
| default | cerah | gravitasi | bias | 3.82 | 4.62 | 4.82 | 19.6 | 2.32e+03 | 0 | 0 | 0 |
| default | cerah | tenang | bias | 3.83 | 4.6 | 4.86 | 19.6 | 2.33e+03 | 0 | 0 | 7.15 |
| default | cerah | berangin | bias | 3.85 | 4.62 | 5.34 | 19.6 | 2.36e+03 | 6 | 0 | 19.7 |
| default | cerah | badai | bias | 3.87 | 5 | 6.79 | 19.6 | 2.84e+03 | 1.06e+03 | 228 | 36 |
| default | sebagian | tanpa_dinamika | astro | 2.4 | 2.88 | 3 | 15 | 2.29e+03 | 0 | 0 | 0 |
| default | sebagian | gravitasi | astro | 2.58 | 3.15 | 3.27 | 15 | 2.29e+03 | 0 | 0 | 0 |
| default | sebagian | tenang | astro | 2.58 | 3.16 | 3.37 | 15 | 2.29e+03 | 0 | 0 | 7.15 |
| default | sebagian | berangin | astro | 2.59 | 3.3 | 4.1 | 15 | 2.29e+03 | 6 | 0 | 19.7 |
| default | sebagian | badai | astro | 2.66 | 3.86 | 5.74 | 15 | 2.29e+03 | 1.06e+03 | 227 | 36 |
| default | sebagian | tanpa_dinamika | ldr | 13.6 | 28.8 | 52 | 14 | 7.04e+03 | 0 | 0 | 0 |
| default | sebagian | gravitasi | ldr | 13.6 | 28.7 | 52.2 | 14 | 7.02e+03 | 0 | 0 | 0 |
| default | sebagian | tenang | ldr | 13.6 | 28.8 | 52.2 | 14 | 7.03e+03 | 0 | 0 | 7.15 |
| default | sebagian | berangin | ldr | 13.6 | 28.6 | 52.1 | 14 | 7.06e+03 | 6 | 0.167 | 19.7 |
| default | sebagian | badai | ldr | 13.6 | 28.7 | 52.8 | 14 | 7.33e+03 | 1.06e+03 | 233 | 36 |
| default | sebagian | tanpa_dinamika | saputra | 3.4 | 4.79 | 17 | 14.9 | 2.78e+03 | 0 | 0 | 0 |
| default | sebagian | gravitasi | saputra | 3.45 | 4.93 | 17 | 14.9 | 2.79e+03 | 0 | 0 | 0 |
| default | sebagian | tenang | saputra | 3.45 | 4.92 | 17 | 14.9 | 2.79e+03 | 0 | 0 | 7.15 |
| default | sebagian | berangin | saputra | 3.44 | 4.97 | 17.3 | 14.9 | 2.8e+03 | 6 | 0 | 19.7 |
| default | sebagian | badai | saputra | 3.43 | 5.09 | 17.7 | 14.9 | 2.97e+03 | 1.06e+03 | 228 | 36 |
| default | sebagian | tanpa_dinamika | bias | 1.94 | 2.89 | 3.62 | 15 | 2.33e+03 | 0 | 0 | 0 |
| default | sebagian | gravitasi | bias | 1.94 | 2.86 | 3.89 | 15 | 2.33e+03 | 0 | 0 | 0 |
| default | sebagian | tenang | bias | 1.92 | 2.85 | 3.92 | 15 | 2.33e+03 | 0 | 0 | 7.15 |
| default | sebagian | berangin | bias | 1.94 | 2.87 | 4.06 | 15 | 2.34e+03 | 6 | 0 | 19.7 |
| default | sebagian | badai | bias | 2.12 | 3.21 | 4.94 | 15 | 2.58e+03 | 1.06e+03 | 230 | 36 |
| baik | cerah | tanpa_dinamika | astro | 2.4 | 2.88 | 3 | 19.6 | 2.29e+03 | 0 | 0 | 0 |
| baik | cerah | gravitasi | astro | 2.58 | 3.15 | 3.27 | 19.7 | 2.29e+03 | 0 | 0 | 0 |
| baik | cerah | tenang | astro | 2.58 | 3.16 | 3.37 | 19.7 | 2.29e+03 | 0 | 0 | 7.15 |
| baik | cerah | berangin | astro | 2.59 | 3.3 | 4.1 | 19.7 | 2.29e+03 | 6 | 0 | 19.7 |
| baik | cerah | badai | astro | 2.66 | 3.86 | 5.74 | 19.7 | 2.29e+03 | 1.06e+03 | 227 | 36 |
| baik | cerah | tanpa_dinamika | ldr | 1.37 | 3.06 | 4.42 | 19.5 | 829 | 0 | 0 | 0 |
| baik | cerah | gravitasi | ldr | 1.36 | 3.03 | 4.43 | 19.5 | 828 | 0 | 0 | 0 |
| baik | cerah | tenang | ldr | 1.36 | 3.03 | 4.38 | 19.5 | 831 | 0 | 0 | 7.15 |
| baik | cerah | berangin | ldr | 1.32 | 3.01 | 4.33 | 19.5 | 992 | 6 | 0.333 | 19.7 |
| baik | cerah | badai | ldr | 1.3 | 2.87 | 4.55 | 19.5 | 2.51e+03 | 1.06e+03 | 238 | 36 |
| baik | cerah | tanpa_dinamika | saputra | 0.897 | 1.2 | 1.42 | 19.7 | 1.46e+03 | 0 | 0 | 0 |
| baik | cerah | gravitasi | saputra | 0.895 | 1.2 | 1.42 | 19.7 | 1.47e+03 | 0 | 0 | 0 |
| baik | cerah | tenang | saputra | 0.895 | 1.2 | 1.44 | 19.7 | 1.46e+03 | 0 | 0 | 7.15 |
| baik | cerah | berangin | saputra | 0.858 | 1.22 | 1.87 | 19.7 | 1.53e+03 | 6 | 0 | 19.7 |
| baik | cerah | badai | saputra | 0.888 | 1.42 | 3.65 | 19.7 | 2.68e+03 | 1.06e+03 | 228 | 36 |
| baik | cerah | tanpa_dinamika | bias | 0.723 | 0.899 | 2.09 | 19.7 | 2.32e+03 | 0 | 0 | 0 |
| baik | cerah | gravitasi | bias | 0.721 | 0.903 | 2.43 | 19.7 | 2.33e+03 | 0 | 0 | 0 |
| baik | cerah | tenang | bias | 0.723 | 0.929 | 2.45 | 19.7 | 2.34e+03 | 0 | 0 | 7.15 |
| baik | cerah | berangin | bias | 0.741 | 1.07 | 2.45 | 19.7 | 2.48e+03 | 6 | 0 | 19.7 |
| baik | cerah | badai | bias | 0.847 | 1.39 | 3.46 | 19.7 | 3.62e+03 | 1.06e+03 | 228 | 36 |
| baik | sebagian | tanpa_dinamika | astro | 2.4 | 2.88 | 3 | 15 | 2.29e+03 | 0 | 0 | 0 |
| baik | sebagian | gravitasi | astro | 2.58 | 3.15 | 3.27 | 15 | 2.29e+03 | 0 | 0 | 0 |
| baik | sebagian | tenang | astro | 2.58 | 3.16 | 3.37 | 15 | 2.29e+03 | 0 | 0 | 7.15 |
| baik | sebagian | berangin | astro | 2.59 | 3.3 | 4.1 | 15 | 2.29e+03 | 6 | 0 | 19.7 |
| baik | sebagian | badai | astro | 2.66 | 3.86 | 5.74 | 15 | 2.29e+03 | 1.06e+03 | 227 | 36 |
| baik | sebagian | tanpa_dinamika | ldr | 10.4 | 21.7 | 24.7 | 11.5 | 2.31e+03 | 0 | 0 | 0 |
| baik | sebagian | gravitasi | ldr | 6.98 | 15.1 | 19.7 | 14.9 | 2.55e+03 | 0 | 0 | 0 |
| baik | sebagian | tenang | ldr | 9.18 | 18.8 | 23.1 | 13.3 | 2.28e+03 | 0 | 0 | 7.15 |
| baik | sebagian | berangin | ldr | 6.88 | 14.2 | 19.9 | 14.7 | 2.49e+03 | 6 | 0.333 | 19.7 |
| baik | sebagian | badai | ldr | 7.4 | 15.6 | 21.4 | 13.8 | 3.37e+03 | 1.06e+03 | 234 | 36 |
| baik | sebagian | tanpa_dinamika | saputra | 1.58 | 2.79 | 4.39 | 15.1 | 2.19e+03 | 0 | 0 | 0 |
| baik | sebagian | gravitasi | saputra | 1.68 | 3.05 | 4.47 | 15.1 | 2.2e+03 | 0 | 0 | 0 |
| baik | sebagian | tenang | saputra | 1.68 | 3.05 | 4.48 | 15.1 | 2.2e+03 | 0 | 0 | 7.15 |
| baik | sebagian | berangin | saputra | 1.68 | 3.08 | 4.8 | 15.1 | 2.26e+03 | 6 | 0 | 19.7 |
| baik | sebagian | badai | saputra | 1.74 | 3.47 | 5.71 | 15.1 | 2.89e+03 | 1.06e+03 | 228 | 36 |
| baik | sebagian | tanpa_dinamika | bias | 0.461 | 0.633 | 2.06 | 15.1 | 2.33e+03 | 0 | 0 | 0 |
| baik | sebagian | gravitasi | bias | 0.473 | 0.677 | 2.44 | 15.1 | 2.33e+03 | 0 | 0 | 0 |
| baik | sebagian | tenang | bias | 0.475 | 0.65 | 2.41 | 15.1 | 2.33e+03 | 0 | 0 | 7.15 |
| baik | sebagian | berangin | bias | 0.575 | 0.851 | 2.44 | 15.1 | 2.41e+03 | 6 | 0 | 19.7 |
| baik | sebagian | badai | bias | 0.847 | 1.53 | 3.75 | 15.1 | 3.07e+03 | 1.06e+03 | 228 | 36 |


## E2: pemulihan setelah lepas langkah disuntikkan (kedua sumbu, searah, pada sepertiga hari)

rmse_post = RMSE setelah kejadian, err_end = galat rata-rata 15 menit terakhir (jendela terjangkau), recover_min = menit sampai galat < 3 deg bertahan 10 menit (rata-rata hanya seed yang pulih), frac_pulih = fraksi seed yang pulih.

| sensor | weather | slip_deg | ctrl | rmse_post | err_end | recover_min | frac_pulih |
|---|---|---|---|---|---|---|---|
| default | cerah | 7.2 | astro | 10.4 | 12.2 | nan | 0 |
| default | cerah | 21.6 | astro | 26.8 | 31.1 | nan | 0 |
| default | cerah | 72 | astro | 88.2 | 100 | nan | 0 |
| default | cerah | 7.2 | ldr | 4.79 | 7.59 | nan | 0 |
| default | cerah | 21.6 | ldr | 8.71 | 21.5 | nan | 0 |
| default | cerah | 72 | ldr | 112 | 71.8 | nan | 0 |
| default | cerah | 7.2 | saputra | 3.93 | 4.2 | 111 | 1 |
| default | cerah | 21.6 | saputra | 9.11 | 21.5 | 107 | 1 |
| default | cerah | 72 | saputra | 88.2 | 100 | nan | 0 |
| default | cerah | 7.2 | bias | 4.03 | 4.38 | 94.7 | 1 |
| default | cerah | 21.6 | bias | 15.4 | 17.8 | nan | 0 |
| default | cerah | 72 | bias | 90.5 | 104 | nan | 0 |
| default | sebagian | 7.2 | astro | 10.4 | 12.2 | nan | 0 |
| default | sebagian | 21.6 | astro | 26.8 | 31.1 | nan | 0 |
| default | sebagian | 72 | astro | 88.2 | 100 | nan | 0 |
| default | sebagian | 7.2 | ldr | 15.4 | 23.1 | 76.4 | 1 |
| default | sebagian | 21.6 | ldr | 18.4 | 42.1 | 0.6 | 0.5 |
| default | sebagian | 72 | ldr | 82 | 81.7 | nan | 0 |
| default | sebagian | 7.2 | saputra | 7.26 | 8.56 | 76.2 | 1 |
| default | sebagian | 21.6 | saputra | 19.6 | 21.1 | 71.2 | 0.5 |
| default | sebagian | 72 | saputra | 88.2 | 100 | nan | 0 |
| default | sebagian | 7.2 | bias | 1.06 | 1.05 | 0.25 | 1 |
| default | sebagian | 21.6 | bias | 15.2 | 17.8 | nan | 0 |
| default | sebagian | 72 | bias | 83.7 | 94.7 | nan | 0 |
| baik | cerah | 7.2 | astro | 10.4 | 12.2 | nan | 0 |
| baik | cerah | 21.6 | astro | 26.8 | 31.1 | nan | 0 |
| baik | cerah | 72 | astro | 88.2 | 100 | nan | 0 |
| baik | cerah | 7.2 | ldr | 2.09 | 7 | 0.167 | 1 |
| baik | cerah | 21.6 | ldr | 7.83 | 21.4 | 0.6 | 1 |
| baik | cerah | 72 | ldr | 112 | 71.8 | nan | 0 |
| baik | cerah | 7.2 | saputra | 2.08 | 7.01 | 0.133 | 1 |
| baik | cerah | 21.6 | saputra | 8.49 | 21.4 | 42.3 | 1 |
| baik | cerah | 72 | saputra | 88.2 | 100 | nan | 0 |
| baik | cerah | 7.2 | bias | 0.709 | 0.725 | 0.167 | 1 |
| baik | cerah | 21.6 | bias | 15.2 | 17.8 | nan | 0 |
| baik | cerah | 72 | bias | 86.3 | 98.7 | nan | 0 |
| baik | sebagian | 7.2 | astro | 10.4 | 12.2 | nan | 0 |
| baik | sebagian | 21.6 | astro | 26.8 | 31.1 | nan | 0 |
| baik | sebagian | 72 | astro | 88.2 | 100 | nan | 0 |
| baik | sebagian | 7.2 | ldr | 11.2 | 24.6 | 24.6 | 1 |
| baik | sebagian | 21.6 | ldr | 9.64 | 22 | 24.8 | 1 |
| baik | sebagian | 72 | ldr | 56.4 | 71.5 | nan | 0 |
| baik | sebagian | 7.2 | saputra | 7.1 | 7.89 | 24.4 | 1 |
| baik | sebagian | 21.6 | saputra | 19.9 | 31.1 | 61.6 | 1 |
| baik | sebagian | 72 | saputra | 88.2 | 100 | nan | 0 |
| baik | sebagian | 7.2 | bias | 0.246 | 0.197 | 0.1 | 1 |
| baik | sebagian | 21.6 | bias | 15.2 | 17.8 | nan | 0 |
| baik | sebagian | 72 | bias | 84.9 | 96.3 | nan | 0 |


## E3: statistik angin sintetis (10 seed, 12 jam siang) dan kejadian melewati torsi tahan

| wind | vmax | frac_v_gt_15 | frac_v_gt_20 | ev_az | ev_el |
|---|---|---|---|---|---|
| tenang | 7.41 | 0 | 0 | 0 | 0 |
| berangin | 20.8 | 0.000668 | 2.26e-05 | 5.2 | 0.7 |
| badai | 38.3 | 0.156 | 0.0134 | 1.15e+03 | 299 |


Kecepatan angin yang melewati torsi tahan (analitik, Cn 1.2, lengan asumsi): elevasi 19.4 m/s (dengan gravitasi terburuk), azimuth 17.8 m/s.

