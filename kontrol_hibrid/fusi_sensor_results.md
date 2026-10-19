# Penggabungan LDR dan arus dengan bobot dari ketidakpastian

Dibangkitkan `kontrol_hibrid/fusi_sensor.py`. Model LDR dari `hybrid_sim.py`, model arus dari `panel_listrik.py` dan `sensor_arus.py`, langit dari pvlib (7 Okt 2026).

Galat astro dianggap 1.5 deg (ASUMSI: salah pasang + defleksi gravitasi + lepas langkah kecil).

## 1. Ketidakpastian tiap sumber (Monte Carlo, 400 cuplikan)

Simpangan baku taksiran SUDUT, bukan simpangan baku sinyal mentah: derau sinyal dibagi kepekaan de/dq.

Panel selalu dianggap TEPAT menunjuk matahari, jadi kemiringan mengikuti elevasi matahari (q2 = 90 - el).

LDR dipisah dua: `acak` dari derau ADC (bisa ditekan dengan merata-rata) dan `bias` dari ketidakcocokan antar LDR (TETAP untuk satu alat, hanya hilang dengan kalibrasi).

| Langit | Elevasi (deg) | q2 (deg) | LDR q2 acak | LDR q2 bias | LDR q1 acak | LDR q1 bias | Arus (deg) |
|---|---|---|---|---|---|---|---|
| okta 0 | 45 | 45 | 0.443 | 2.855 | 0.597 | 4.684 | 0.85 |
| okta 0 | 60 | 30 | 0.469 | 2.882 | 0.880 | 6.656 | 0.80 |
| okta 0 | 70 | 20 | 0.477 | 2.909 | 1.307 | 9.738 | 0.80 |
| okta 0 | 80 | 10 | 0.483 | 2.922 | 2.744 | 20.329 | 0.80 |
| okta 2 | 45 | 45 | 0.443 | 2.877 | 0.598 | 4.723 | 0.85 |
| okta 2 | 60 | 30 | 0.465 | 2.906 | 0.879 | 6.711 | 0.80 |
| okta 2 | 70 | 20 | 0.477 | 2.926 | 1.306 | 9.801 | 0.80 |
| okta 2 | 80 | 10 | 0.482 | 2.944 | 2.754 | 20.457 | 0.80 |
| okta 4 | 45 | 45 | 0.463 | 3.123 | 0.627 | 5.125 | 0.95 |
| okta 4 | 60 | 30 | 0.496 | 3.163 | 0.937 | 7.301 | 0.85 |
| okta 4 | 70 | 20 | 0.509 | 3.192 | 1.380 | 10.683 | 0.85 |
| okta 4 | 80 | 10 | 0.512 | 3.216 | 2.908 | 22.340 | 0.85 |
| okta 6 | 45 | 45 | 0.691 | 5.202 | 0.935 | 8.543 | 1.30 |
| okta 6 | 60 | 30 | 0.776 | 5.496 | 1.466 | 12.698 | 1.15 |
| okta 6 | 70 | 20 | 0.812 | 5.672 | 2.238 | 18.951 | 1.10 |
| okta 6 | 80 | 10 | 0.851 | 5.788 | 4.805 | 40.126 | 1.10 |

Tiga pola terbaca:
1. **Derau acak LDR kecil, tetapi bias antar perangkat jauh lebih besar dan menguasai.** Tanpa kalibrasi, keunggulan LDR terhadap arus sebagian besar hilang. Ini membalik perkiraan awal saya bahwa LDR unggul satu sampai dua orde; yang unggul hanya derau acaknya.
2. **Ketelitian azimuth memburuk saat matahari makin tinggi** (kemiringan mengecil). Itu wujud nyata Jacobian yang hampir singular di dekat zenith: de/dq1 sebanding sin(q2). Ketelitian kemiringan TIDAK ikut memburuk, karena sumbu itu tetap teramati di mana pun.
3. **Langit tertutup memperburuk LDR tetapi hampir tidak menyentuh arus.** Derau acak LDR naik sekitar 1.6 kali dari okta 0 ke okta 6, dan biasnya pun naik; angka arus hampir tetap 0.8 sampai 1.3 derajat. LDR tetap tahan terhadap perubahan irradiance menyeluruh karena memakai NISBAH antar sisi, tetapi nisbah itu tidak menolong terhadap ketidakcocokan antar sensornya sendiri.


**Peringatan membandingkan kolom:** kolom arus adalah jari-jari 1 sigma untuk MENENTUKAN LETAK puncak arus, sedangkan puncak arus itu sendiri bergeser dari arah matahari sebesar -2.75 sampai +2.75 derajat menurut albedo tanah (lihat `panel_listrik_results.md`). Jadi arus juga punya bias sebesar itu, yang hanya hilang bila albedo tempat pemasangan diukur. Kedua sensor sama-sama perlu kalibrasi; bedanya kalibrasi LDR mengenai perangkatnya, kalibrasi arus mengenai lingkungannya.


## 2. Bobot penggabungan astro dan LDR

Bobot kebalikan ragam: w = 1/sigma^2, taksiran gabungan = (w_astro x astro + w_ldr x ldr) / (w_astro + w_ldr). Dengan sigma astro 1.5 deg.

| Langit | q2 (deg) | Bobot LDR untuk q2 | Bobot LDR untuk q1 | sigma gabungan q1 (deg) |
|---|---|---|---|---|
| okta 0 | 45 | 21.2 % | 9.2 % | 1.430 |
| okta 0 | 30 | 20.9 % | 4.8 % | 1.464 |
| okta 0 | 20 | 20.6 % | 2.3 % | 1.483 |
| okta 0 | 10 | 20.4 % | 0.5 % | 1.496 |
| okta 2 | 45 | 21.0 % | 9.0 % | 1.431 |
| okta 2 | 30 | 20.6 % | 4.7 % | 1.464 |
| okta 2 | 20 | 20.4 % | 2.2 % | 1.483 |
| okta 2 | 10 | 20.2 % | 0.5 % | 1.496 |
| okta 4 | 45 | 18.4 % | 7.8 % | 1.440 |
| okta 4 | 30 | 18.0 % | 4.0 % | 1.470 |
| okta 4 | 20 | 17.7 % | 1.9 % | 1.486 |
| okta 4 | 10 | 17.5 % | 0.4 % | 1.497 |
| okta 6 | 45 | 7.6 % | 3.0 % | 1.478 |
| okta 6 | 30 | 6.8 % | 1.4 % | 1.490 |
| okta 6 | 20 | 6.4 % | 0.6 % | 1.495 |
| okta 6 | 10 | 6.2 % | 0.1 % | 1.499 |

Bobot ini keluar dari pengukuran, bukan disetel tangan. Perhatikan bobot LDR untuk azimuth TURUN sendiri saat matahari makin tinggi, tanpa aturan `if q2 < 5 derajat` seperti pada `hybrid_sim.py` (parameter `guard`). Itulah keuntungan memakai ketidakpastian sebagai bobot.


## 3. Nilai kalibrasi LDR

Bias antar LDR adalah satu satunya hal yang membuat LDR kalah dari model astro. Tabel ini menunjukkan apa yang didapat bila ketidakcocokan antar LDR dikalibrasi (langit cerah, elevasi 45 deg).

| Ketidakcocokan antar LDR | sigma LDR q2 (deg) | sigma LDR q1 (deg) | Bobot LDR q2 | Bobot LDR q1 |
|---|---|---|---|---|
| 3.0 % (tanpa kalibrasi (ASUMSI sekarang)) | 2.915 | 4.081 | 20.9 % | 11.9 % |
| 1.0 % (kalibrasi kasar) | 1.034 | 1.458 | 67.8 % | 51.4 % |
| 0.3 % (kalibrasi teliti) | 0.514 | 0.708 | 89.5 % | 81.8 % |
| 0.0 % (sempurna (batas derau ADC)) | 0.440 | 0.602 | 92.1 % | 86.1 % |

**Inilah hasil terpenting di langkah ini.** Tanpa kalibrasi, penggabungan yang jujur akan memberi LDR bobot kecil saja, artinya LDR nyaris tidak berguna dan tracker praktis berjalan secara astro saja. Dengan kalibrasi teliti, bobot LDR melonjak dan LDR menjadi sumber utama. Jadi **kalibrasi LDR bukan penyempurnaan, melainkan syarat agar LDR layak dipasang sama sekali.**


## 4. Arus sebagai pemeriksa, bukan sebagai pengoreksi

Arus terlalu kasar untuk mengoreksi arah (satu sampai dua orde lebih buruk daripada LDR), tetapi ia mengukur hasil yang SEBENARNYA dipanen, sehingga bisa menangkap kesalahan yang tidak terlihat LDR: LDR kotor atau tertutup, lepas langkah besar, bayangan sebagian, dan panel retak. Bila arus terukur turun lebih dari 5 persen di bawah ramalan dari sudut beban, ragam LDR dibesarkan sehingga penggabungan otomatis lebih percaya astro.

| Arus terukur / ramalan | Faktor pembesar ragam LDR |
|---|---|
| 1.00 | 1.0 |
| 0.98 | 1.0 |
| 0.95 | 2.0 |
| 0.90 | 5.0 |
| 0.80 | 17.0 |
| 0.50 | 101.0 |

## 4. Batas

- Penggabungan ini statis per cuplikan, tanpa ingatan. Ia tidak memakai model dinamika untuk meramal antar pengukuran, dan tidak menaksir lepas langkah terkumpul. Itu tugas tapis Kalman di langkah berikut.
- sigma astro 1.5 derajat masih ASUMSI dan sangat menentukan bobotnya. Nilai nyatanya ditentukan oleh ketelitian pemasangan dan seberapa sering lepas langkah terjadi.
- Seluruh tetapan LDR (tinggi sekat r, ketidakcocokan 3 persen, kurva lux) masih ASUMSI.
- Bias LDR dari ketidakcocokan antar sensor TIDAK hilang dengan merata-rata; itu memerlukan kalibrasi, dan hanya sebagian yang dapat dipisahkan (lihat catatan `hybrid_results.md`).
