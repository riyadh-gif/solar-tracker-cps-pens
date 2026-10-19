# Model listrik panel MOS C-50-18-M

Dibangkitkan `kontrol_hibrid/panel_listrik.py` (model dioda tunggal De Soto, pvlib 0.16.1; parameter dicocokkan ke pelat nama).

## 1. Parameter hasil pencocokan (acuan STC)

| Besaran | Nilai |
|---|---|
| Arus cahaya I_L | 3.0092 A |
| Arus jenuh I_o | 7.024e-10 A |
| Tahanan seri R_s | 0.3720 ohm |
| Tahanan bocor R_sh | 121.5 ohm |
| a_ref (n Ns Vt) | 1.0174 V, faktor ideal n = 1.100 |
| Sel seri (ASUMSI) | 36 |

## 2. Reproduksi pelat nama di STC

| Besaran | Pelat nama | Model | Selisih |
|---|---|---|---|
| Isc | 3.00 A | 3.000 A | +0.00 % |
| Voc | 22.50 V | 22.500 V | +0.00 % |
| Imp | 2.70 A | 2.700 A | +0.00 % |
| Vmp | 18.50 V | 18.500 V | -0.00 % |
| Pmax | 50.00 W | 49.950 W | -0.10 % |

## 3. Arus terhadap irradiance dan suhu

| POA (W/m2) | Suhu sel (C) | Isc (A) | Imp (A) | Vmp (V) | Pmp (W) |
|---|---|---|---|---|---|
| 100 | 33.1 | 0.302 | 0.272 | 16.26 | 4.42 |
| 200 | 36.2 | 0.605 | 0.544 | 16.54 | 9.00 |
| 400 | 42.5 | 1.213 | 1.089 | 16.43 | 17.89 |
| 600 | 48.8 | 1.824 | 1.634 | 16.06 | 26.23 |
| 800 | 55.0 | 2.437 | 2.178 | 15.58 | 33.92 |
| 1000 | 61.2 | 3.054 | 2.720 | 15.05 | 40.92 |

Arus hubung singkat sebanding dengan irradiance dalam 1 persen, dan hampir tidak terpengaruh suhu. Itulah dasar memakai arus sebagai sensor arah: arus mengukur irradiance bidang, dan irradiance bidang mengukur kosinus galat pointing.


## 4. Arus sebagai sensor arah

Langit cerah NYATA dari pvlib (7 Okt 2026, matahari di Timur, elevasi 44.8 deg): DNI 703, DHI 171, GHI 667 W/m2, albedo 0.25. Nilai ini saling konsisten (GHI = DNI sin el + DHI); nilai pilihan tangan mudah melanggarnya dan membesar-besarkan suku albedo.

Simpangan diukur dari ARAH MATAHARI. Perhatikan puncak arus TIDAK berada tepat di 0 derajat.

| Simpangan dari matahari (deg) | POA (W/m2) | Imp (A) | Beda arus |
|---|---|---|---|
| -20 | 830.0 | 2.2591 | -5.029 % |
| -10 | 863.1 | 2.3490 | -1.251 % |
| -5 | 871.4 | 2.3715 | -0.305 % |
| -2 | 873.7 | 2.3776 | -0.045 % |
| 0 | 874.1 | 2.3787 | +0.000 % |
| 1 | 873.9 | 2.3783 | -0.016 % |
| 2 | 873.6 | 2.3773 | -0.058 % |
| 2.6 | 873.3 | 2.3765 | -0.095 % |
| 5 | 871.1 | 2.3707 | -0.337 % |
| 10 | 862.6 | 2.3475 | -1.314 % |
| 20 | 828.9 | 2.2561 | -5.153 % |
| 30 | 774.1 | 2.1074 | -11.407 % |

**Temuan 1: memaksimalkan arus TIDAK mengarahkan panel ke matahari, dan arah simpangannya bergantung tempat pemasangan.** Pada albedo 0.25 puncak arus bergeser -0.10 derajat, dan arus di puncak hanya +0.0002 persen lebih tinggi. Penyebabnya dua komponen sebaran yang menarik ke arah BERLAWANAN: albedo menarik optimum ke arah lebih tegak (panel melihat lebih banyak tanah), baur langit menarik ke arah lebih datar (panel melihat lebih banyak langit). Dengan komponen langsung saja puncaknya tepat di 0 derajat (diperiksa di selftest).


| Albedo tanah | Puncak arus (deg dari matahari) | Contoh permukaan |
|---|---|---|
| 0.10 | -2.75 | aspal, tanah basah |
| 0.20 | -1.00 | rumput, tanah kering |
| 0.25 | -0.10 | nilai bawaan model |
| 0.40 | +2.75 | beton terang, pasir |

Tandanya berbalik di sekitar albedo 0.25. Jadi **albedo tempat pemasangan harus diukur** bila arus dipakai sebagai acuan posisi; tanpa itu, arah simpangannya pun tidak diketahui.


**Temuan 2: arus buta terhadap galat kecil.** Di dekat optimum arus berubah sebagai kosinus, jadi turunannya NOL. Simpangan 2 derajat hanya mengubah arus 0.09 persen, jauh di bawah toleransi pabrik 3 persen dan di bawah derau pengukuran yang wajar. Arus juga tidak memberi ARAH: simpangan +5 dan -5 derajat memberi arus yang mirip. Arus baru berarti di atas sekitar 10 derajat (turun 0.6 persen) dan jelas di atas 20 derajat (turun 4 persen).


**Akibatnya untuk rancangan kontrol:** arus dipakai sebagai (a) pemeriksa kasar bahwa penjejakan tidak melenceng jauh, (b) pembanding antara dua posisi pada pendakian bukit yang lambat, (c) pendeteksi lepas langkah besar. Koreksi halus dan informasi arah tetap dari LDR. Keduanya saling melengkapi: LDR peka dan berarah tetapi terpengaruh awan dan kotor, arus lamban dan buta arah tetapi langsung mengukur hasil yang dipanen.


## 5. Asumsi yang dipakai

- Sel seri 36 (dari nama modul 18 V nominal; Voc per sel 0.625 V)
- Koefisien suhu Isc +0.05 %/C, Voc -0.30 %/C (khas mono-Si, TIDAK ada di pelat nama)
- NOCT 45 C, suhu udara 30 C
- IAM ASHRAE b0 0.05
- Toleransi pabrik +-3 %, belum termasuk kotor, penuaan, dan ketidakcocokan sel
