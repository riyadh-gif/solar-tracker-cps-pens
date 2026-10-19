# Analisis zenith (keyhole) tracker 2 sumbu, PENS 2026

Dihasilkan otomatis oleh `zenith_analysis.py` (jalankan ulang untuk memperbarui angka).

## Asumsi dan metode

- Titik panel: lintang -7.276006, bujur 112.794327, Asia/Jakarta, ketinggian 5 m (asumsi). Posisi matahari: pvlib (nrel_numpy) via `sun.loc`; zenith = 90 - elevasi tampak.
- Zeta minimum dan waktunya: kulminasi dicari di grid 10 s, lalu diperhalus pada grid 0.1 s (+-60 s). Resolusi zeta pada grid halus sekitar 0.0002 deg (laju matahari 0.004 deg/s kali 0.05 s).
- Laju azimuth sendi (panel mengikuti matahari sempurna, cabang q2 >= 0, q1 = azimuth matahari): selisih azimuth antar sampel dibagi dt. Grid 10 s seharian, 1 s pada +-20 menit sekitar kulminasi, 0.1 s pada +-60 s. Hanya interval dengan matahari di atas horizon.
- Estimasi analitik: v/zeta dengan v = laju sudut matahari yang diukur di kulminasi (deg/s) dan zeta dalam radian (hampiran lintasan lurus melewati zenith).
- Strategi keyhole: q1 mengejar azimuth matahari dengan laju maks `lim`; q2 dianggap tanpa batas laju dan dipilih optimal dalam [0, q2_maks] (cabang q2 >= 0). Galat = sudut antara normal panel dan arah matahari. Galat maks hanya dilaporkan saat matahari terjangkau (zenith <= q2_maks); saat zenith > q2_maks galat besar bukan akibat keyhole.
- Rugi energi: integral DNI*cos(galat), langit cerah (sun_day okta=0, DNI grid 1 s diinterpolasi ke grid simulasi 1 s, dan 0.1 s dekat kulminasi). Dua pembanding: rugi keyhole = relatif terhadap tracker laju tak terbatas dengan q2_maks sama; rugi total = relatif terhadap pointing sempurna (cos = 1).
- q2_maks = 45 deg adalah asumsi kasar (belum diukur); 60 dan 90 deg hanya sensitivitas. Laju q2 tidak dibatasi. Rasio gigi sendi belum diketahui: laju sendi dianggap sama dengan laju poros motor.

## 1. Jarak zenith minimum

| zeta < | jumlah hari (dari 365) |
|---|---|
| 0.5 deg | 6 |
| 1 deg | 10 |
| 2 deg | 22 |
| 5 deg | 53 |
| 10 deg | 111 |

- Hari terdekat zenith: 2026-03-02, zeta = 0.0653 deg, pukul 11:40 WIB.
- 2026-10-04: zeta = 2.936 deg, pukul 11:17 WIB.
- Musim zeta < 2 deg: 25 Feb sampai 07 Mar; 07 Oct sampai 17 Oct.
- Musim zeta < 5 deg: 17 Feb sampai 14 Mar; 29 Sep sampai 25 Oct.
- Musim zeta < 10 deg: 01 Feb sampai 27 Mar; 16 Sep sampai 10 Nov.

![zeta](zeta_min_vs_hari.png)

## 2. Laju azimuth sendi maksimum (pointing sempurna)

- Terburuk sepanjang tahun: 3.63 deg/s pada 2026-03-02 (zeta 0.0653 deg; analitik 3.63 deg/s).
- Rasio numerik/analitik pada hari zeta < 5 deg: median 1.000, rentang 1.000 sampai 1.001.
- Jumlah hari dengan laju maks melebihi ambang: > 0.1 deg/s: 25 hari, > 0.5 deg/s: 5 hari, > 1.0 deg/s: 2 hari.
- Durasi laju melebihi ambang per hari, maksimum tahunan: > 0.1: 574 s, > 0.5: 114 s, > 1.0: 57 s.

| tanggal | zeta (deg) | laju maks numerik (deg/s) | analitik v/zeta (deg/s) | detik > 0.1 | detik > 0.5 | detik > 1 |
|---|---|---|---|---|---|---|
| 2026-03-02 | 0.0653 | 3.627 | 3.626 | 188 | 79 | 51 |
| 2026-10-04 | 2.9361 | 0.081 | 0.081 | 0 | 0 | 0 |

Contoh hari lain (laju maks, deg/s): 01-15: 0.0162, 03-20: 0.0337, 06-21: 0.0075, 09-23: 0.0332, 12-21: 0.0137.

![laju](laju_azimuth_vs_hari.png)

## 3. Strategi keyhole: batas laju azimuth

### 2026-03-02 (hari terburuk), zeta 0.0653 deg

| batas laju (deg/s) | galat maks (deg), q2maks 45 | rugi keyhole % (45) | rugi total % (45) | galat maks (deg), q2maks 90 | rugi keyhole % (90) |
|---|---|---|---|---|---|
| 0.02 | 20.11 | 0.79 | 1.86 | 20.11 | 0.78 |
| 0.05 | 7.64 | 0.046 | 1.12 | 7.64 | 0.045 |
| 0.1 | 3.59 | 0.005 | 1.08 | 3.59 | 0.0049 |
| 0.5 | 0.53 | 1.9e-05 | 1.08 | 0.53 | 1.9e-05 |
| 1 | 0.19 | 1.2e-06 | 1.08 | 0.19 | 1.1e-06 |
| 2 | 0.05 | 2.6e-08 | 1.08 | 0.05 | 2.5e-08 |
| 5 | 0.00 | 0 | 1.08 | 0.00 | 0 |
| 10 | 0.00 | 0 | 1.08 | 0.00 | 0 |

Energi DNI harian (kWh/m2): pointing sempurna 6.623; tracker ideal q2maks 45 6.552; q2maks 90 6.623.

### 2026-10-04 (4 Okt), zeta 2.9361 deg

| batas laju (deg/s) | galat maks (deg), q2maks 45 | rugi keyhole % (45) | rugi total % (45) | galat maks (deg), q2maks 90 | rugi keyhole % (90) |
|---|---|---|---|---|---|
| 0.02 | 10.43 | 0.17 | 1.39 | 10.43 | 0.17 |
| 0.05 | 1.45 | 0.00096 | 1.22 | 1.45 | 0.00094 |
| 0.1 | 0.00 | 0 | 1.22 | 0.00 | 0 |
| 0.5 | 0.00 | 0 | 1.22 | 0.00 | 0 |
| 1 | 0.00 | 0 | 1.22 | 0.00 | 0 |
| 2 | 0.00 | 0 | 1.22 | 0.00 | 0 |
| 5 | 0.00 | 0 | 1.22 | 0.00 | 0 |
| 10 | 0.00 | 0 | 1.22 | 0.00 | 0 |

Energi DNI harian (kWh/m2): pointing sempurna 6.895; tracker ideal q2maks 45 6.811; q2maks 90 6.895.

![keyhole](galat_rugi_vs_batas_laju.png)

### Fraksi matahari dapat dijangkau (zenith <= q2_maks), langit cerah, nilai: % waktu siang / % energi DNI

| hari | q2maks 45 | 60 (sensitivitas) | 90 (sensitivitas) |
|---|---|---|---|
| 2026-03-20 (ekuinoks Mar) | 49 / 70 | 66 / 88 | 99 / 100 |
| 2026-06-21 (solstis Jun) | 39 / 56 | 60 / 83 | 99 / 100 |
| 2026-10-04 (4 Okt) | 50 / 69 | 66 / 87 | 99 / 100 |
| 2026-12-21 (solstis Des) | 47 / 67 | 64 / 87 | 99 / 100 |
| 2026-03-02 (hari terburuk) | 50 / 70 | 66 / 89 | 99 / 100 |

Pada 90 deg seluruh waktu siang terjangkau menurut definisi (zenith <= 90 deg).

## 4. Kesimpulan desain

- Laju azimuth teoretis (tanpa batas) di kulminasi: 3.6 deg/s pada hari terburuk, 0.08 deg/s pada 4 Okt, hanya selama detik hingga puluhan detik. Dari sisi kecepatan langkah laju itu ringan (tabel di bawah: 3.6 deg/s sekitar 32 langkah/detik pada 1/16). Yang belum diketahui adalah percepatan, torsi, dan resonansi saat laju berubah tajam (butuh model dinamika, Fase 4), jadi batas laju (strategi keyhole) dipertahankan sebagai pengaman, bukan karena kecepatan langkah tidak mencukupi.
- Batas laju terkecil dalam daftar yang menjaga rugi keyhole <= 0.01% energi DNI harian dan galat maks <= 1 deg (langit cerah, q2maks 45): hari terburuk 0.5 deg/s, 4 Okt 0.1 deg/s. Rugi pada batas lain ada di tabel bagian 3.
- Rugi energi keyhole dibanding rugi total (kolom rugi total, q2maks 45): bagian besar rugi total berasal dari waktu saat zenith > 45 deg, bukan dari zenith. Keyhole hanya berpengaruh pada hari zeta kecil dan sekitar kulminasi.
- Langkah per detik = laju (deg/s) / (1.8 deg / microstep), poros motor tanpa gearbox (jika ada rasio gigi N, kalikan N):

| laju (deg/s) | 1/8 | 1/16 | 1/32 |
|---|---|---|---|
| 0.004 | 0.0178 | 0.0356 | 0.0711 |
| 0.05 | 0.222 | 0.444 | 0.889 |
| 0.1 | 0.444 | 0.889 | 1.78 |
| 0.5 | 2.22 | 4.44 | 8.89 |
| 1 | 4.44 | 8.89 | 17.8 |
| 2 | 8.89 | 17.8 | 35.6 |
| 5 | 22.2 | 44.4 | 88.9 |

- Pelacakan biasa (matahari 0.004 deg/s) hanya perlu sekitar 0.02 sampai 0.07 langkah per detik pada 1/8 sampai 1/32 (baris 0.004), jadi microstep halus dipilih demi resolusi dan kehalusan, bukan kecepatan. Batas laju keyhole 1 sampai 5 deg/s memerlukan sekitar 18 sampai 89 langkah per detik pada 1/32 (tabel), masih ringan bagi driver umum. Kontrol: pasang pembatas laju (rate limiter) pada setpoint q1 dan, dekat zenith, jangan memutar azimuth dari umpan balik LDR, karena arah azimuth dari selisih LDR tidak terdefinisi di sana.
- Belum diverifikasi: batas q2 nyata, rasio gigi, torsi dan kehilangan langkah stepper pada laju dan beban nyata, serta efek awan (hanya langit cerah). Simulasi tidak membatasi laju q2 dan tidak memodelkan LDR.
