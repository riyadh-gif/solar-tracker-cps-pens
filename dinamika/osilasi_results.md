# Osilasi dua sumbu dan dampaknya pada energi

Dibangkitkan `dinamika/osilasi_energi.py` (tanggal 2026-10-07, aktuator `pid`, microstep 1/16, jendela jejak 60 s dan kejadian 20 s pada 1 kHz, deret angin seed 3).

Panel MOS C-50-18-M: 0.670 x 0.445 m, 2.55 kg, Pmax 50 W, efisiensi 16.8 %. Gravitasi 10 m/s2.

## 1. Jejak matahari biasa (jam saat matahari terjangkau)

Osilasi = simpangan sudut beban terhadap perintah, dipisah jadi bias (defleksi tetap) dan ayunan (`pp` = puncak ke puncak, diambil jendela TERBURUK karena sebagian jendela tidak ada langkah sama sekali). `galat dinamik` = galat pointing beban dikurangi galat pointing perintah, jadi batas jangkauan 45 deg tidak ikut terhitung. Rugi = daya pada sudut beban dibandingkan daya pada sudut perintah (negatif berarti defleksi kebetulan membantu).

| angin | strategi | ayun az pp (deg) | ayun el pp (deg) | galat total (deg) | galat dinamik (deg) | galat dinamik maks (deg) | angin > torsi tahan (% waktu) | jendela lepas sinkron | rugi (%) | energi hari (Wh) |
|---|---|---|---|---|---|---|---|---|---|---|
| none | langsung | 0.1774 | 0.1621 | 0.8165 | -0.02192 | 0.5033 | 0 | 0 | -0.009016 | 431.1 |
| none | scurve | 0.1774 | 0.1621 | 0.8165 | -0.02192 | 0.5033 | 0 | 0 | -0.009016 | 431.1 |
| none | zv | 0.1774 | 0.1621 | 0.8165 | -0.02191 | 0.5033 | 0 | 0 | -0.009016 | 431.1 |
| none | zvd | 0.1774 | 0.1621 | 0.8164 | -0.02191 | 0.5033 | 0 | 0 | -0.009016 | 431.1 |
| berangin | langsung | 770.2 | 1.012 | 6.354 | 5.516 | 90.09 | 10.83 | 4 | 3.079 | 417.7 |
| berangin | scurve | 770.2 | 1.012 | 6.354 | 5.516 | 90.09 | 10.83 | 4 | 3.079 | 417.7 |
| berangin | zv | 770.2 | 1.012 | 6.354 | 5.516 | 90.09 | 10.83 | 4 | 3.079 | 417.7 |
| berangin | zvd | 770.2 | 1.012 | 6.355 | 5.516 | 90.09 | 10.83 | 4 | 3.079 | 417.7 |

Energi sehari pada sudut perintah (tanpa dinamika): **431.0 Wh**; panel datar 362.3 Wh; tracker ideal 432.9 Wh. Kolom energi hari adalah EKSTRAPOLASI: energi perintah dikali (1 - rugi), dengan rugi rata-rata dari jendela terpilih, bukan simulasi 1 kHz sehari penuh.


## 2. Kejadian gerak besar (di sinilah strategi berbeda)

`nyala_awal` = dari parkir datar ke pose jejak, `stow` = kemiringan ke 0 deg, `putar_90` = azimuth berputar 90 deg. Jejak harian terlalu lambat untuk membedakan strategi: setelah dibulatkan ke microstep, perintahnya identik.

| angin | kejadian | strategi | ayun az pp (deg) | ayun el pp (deg) | galat dinamik maks (deg) | daya rata (W) | rugi (%) |
|---|---|---|---|---|---|---|---|
| none | nyala_awal | langsung | 0 | 18.36 | 11.06 | 50.24 | 0.01008 |
| none | stow | langsung | 0 | 19.49 | 10.91 | 49.53 | 0.05161 |
| none | putar_90 | langsung | 137 | 0 | 15.39 | 48.9 | -0.0184 |
| none | nyala_awal | scurve | 0 | 1.036 | 0.5923 | 50.23 | 0.002846 |
| none | stow | scurve | 0 | 1.01 | 0.5805 | 49.55 | 0.04033 |
| none | putar_90 | scurve | 1.46 | 0 | 0.1497 | 49.03 | 0.001984 |
| none | nyala_awal | zv | 0 | 0.6262 | 0.4425 | 50.23 | 0.002901 |
| none | stow | zv | 0 | 0.6126 | 0.3262 | 49.55 | 0.04006 |
| none | putar_90 | zv | 0.8144 | 0 | 0.08929 | 49.04 | 0.001967 |
| none | nyala_awal | zvd | 0 | 0.5281 | 0.3999 | 50.23 | 0.002966 |
| none | stow | zvd | 0 | 0.5181 | 0.2806 | 49.55 | 0.03999 |
| none | putar_90 | zvd | 0.4979 | 0 | 0.05929 | 49.04 | 0.001957 |
| berangin | nyala_awal | langsung | 1.251 | 20.67 | 10.76 | 50.24 | 0.006963 |
| berangin | stow | langsung | 1.251 | 17.52 | 10.83 | 49.57 | -0.03703 |
| berangin | putar_90 | langsung | 175.3 | 0.699 | 15.1 | 48.79 | 0.2006 |
| berangin | nyala_awal | scurve | 1.251 | 1.053 | 0.6112 | 50.24 | -0.003807 |
| berangin | stow | scurve | 1.251 | 1.009 | 0.6952 | 49.59 | -0.0379 |
| berangin | putar_90 | scurve | 1.587 | 0.699 | 0.6952 | 48.99 | 0.09678 |
| berangin | nyala_awal | zv | 1.251 | 0.7193 | 0.5409 | 50.24 | -0.003883 |
| berangin | stow | zv | 1.251 | 0.7561 | 0.6952 | 49.59 | -0.03788 |
| berangin | putar_90 | zv | 1.261 | 0.699 | 0.6952 | 48.99 | 0.09652 |
| berangin | nyala_awal | zvd | 1.251 | 0.6988 | 0.5409 | 50.23 | -0.003943 |
| berangin | stow | zvd | 1.251 | 0.7825 | 0.6952 | 49.59 | -0.03785 |
| berangin | putar_90 | zvd | 1.251 | 0.699 | 0.6952 | 49 | 0.09627 |

## 3. Pengaruh microstep pada jejak biasa (tanpa angin)

Saat menjejak, sumber ayunan satu-satunya adalah tangga microstep. Shaper tidak bisa menolong di sini (satu microstep tidak bisa dipecah dua), jadi satu-satunya tuas adalah memperhalus langkah.

| microstep | satu langkah (deg) | ayun az pp (deg) | ayun el pp (deg) | galat dinamik maks (deg) | rugi (%) |
|---|---|---|---|---|---|
| 1/8 | 0.225 | 0.3946 | 0.3745 | 0.5033 | -0.008993 |
| 1/16 | 0.1125 | 0.1774 | 0.1621 | 0.5033 | -0.009016 |
| 1/32 | 0.05625 | 0.06858 | 0.05958 | 0.5034 | -0.009024 |
| 1/64 | 0.02813 | 0.03143 | 0.03188 | 0.5034 | -0.009021 |

## 3b. Respons langkah tunggal (tanpa angin)

Lewatan terhadap langkah, dan waktu menetap ke pita 2 %.

| sumbu | strategi | langkah (deg) | lewatan (%) | menetap (s) |
|---|---|---|---|---|
| azimuth | langsung | 1 | 89.34 | 1.41 |
| azimuth | langsung | 20 | 71.21 | 3.547 |
| azimuth | scurve | 1 | 69.86 | 1.259 |
| azimuth | scurve | 20 | 2.972 | 1.017 |
| azimuth | zv | 1 | 0 | 0.182 |
| azimuth | zv | 20 | 0 | 0.774 |
| azimuth | zvd | 1 | 0 | 0.353 |
| azimuth | zvd | 20 | 0.08385 | 0.854 |
| elevasi | langsung | 1 | 82.17 | 0.627 |
| elevasi | langsung | 20 | 66.03 | 1.436 |
| elevasi | scurve | 1 | 48.85 | 0.545 |
| elevasi | scurve | 20 | 1.433 | 0.709 |
| elevasi | zv | 1 | 0 | 0.178 |
| elevasi | zv | 20 | 0.008582 | 0.755 |
| elevasi | zvd | 1 | 0 | 0.267 |
| elevasi | zvd | 20 | 0.002104 | 0.805 |

## 4. Rincian per jendela jejak

| angin | strategi | jam | ayun az pp | ayun el pp | bias az | bias el | galat total (deg) | galat dinamik (deg) | daya (W) | rugi (%) |
|---|---|---|---|---|---|---|---|---|---|---|
| none | langsung | 9.5 | 0 | 0.1621 | 0 | 0.2467 | 0.2454 | 0.1791 | 48.67 | -0.002961 |
| none | langsung | 11.2 | 0.1774 | 0.1605 | 0.002971 | -0.1262 | 0.1414 | 0.06116 | 50.57 | 0.0004119 |
| none | langsung | 13 | 0 | 0.1621 | 0 | 0.2378 | 0.2416 | 0.1753 | 48.76 | -0.002857 |
| none | langsung | 14.5 | 0 | 0 | 0 | 0.5033 | 2.637 | -0.5033 | 42.37 | -0.03066 |
| none | scurve | 9.5 | 0 | 0.1621 | 0 | 0.2467 | 0.2454 | 0.1791 | 48.67 | -0.002961 |
| none | scurve | 11.2 | 0.1774 | 0.1605 | 0.002971 | -0.1262 | 0.1414 | 0.06116 | 50.57 | 0.0004119 |
| none | scurve | 13 | 0 | 0.1621 | 0 | 0.2378 | 0.2416 | 0.1753 | 48.76 | -0.002857 |
| none | scurve | 14.5 | 0 | 0 | 0 | 0.5033 | 2.637 | -0.5033 | 42.37 | -0.03066 |
| none | zv | 9.5 | 0 | 0.1621 | 0 | 0.2467 | 0.2456 | 0.1793 | 48.67 | -0.002961 |
| none | zv | 11.2 | 0.1774 | 0.1605 | 0.002971 | -0.1262 | 0.1413 | 0.06111 | 50.57 | 0.0004117 |
| none | zv | 13 | 0 | 0.1621 | 0 | 0.2378 | 0.2415 | 0.1753 | 48.76 | -0.002858 |
| none | zv | 14.5 | 0 | 0 | 0 | 0.5033 | 2.637 | -0.5033 | 42.37 | -0.03066 |
| none | zvd | 9.5 | 0 | 0.1621 | 0 | 0.2467 | 0.2457 | 0.1794 | 48.67 | -0.00296 |
| none | zvd | 11.2 | 0.1774 | 0.1605 | 0.00297 | -0.1262 | 0.1413 | 0.06108 | 50.57 | 0.0004115 |
| none | zvd | 13 | 0 | 0.1621 | 0 | 0.2378 | 0.2414 | 0.1752 | 48.76 | -0.002859 |
| none | zvd | 14.5 | 0 | 0 | 0 | 0.5033 | 2.637 | -0.5033 | 42.37 | -0.03066 |
| berangin | langsung | 9.5 | 770.2 | 1.007 | -96.93 | -0.1974 | 6.123 | 6.057 | 47.36 | 2.673 |
| berangin | langsung | 11.2 | 769.8 | 1.012 | -96.6 | -0.5707 | 0.8043 | 0.7241 | 50.56 | 0.01566 |
| berangin | langsung | 13 | 770.1 | 1.008 | -96.91 | -0.2065 | 6.001 | 5.935 | 47.51 | 2.573 |
| berangin | langsung | 14.5 | 769.3 | 1.008 | -97.19 | 0.06001 | 12.49 | 9.348 | 39.37 | 7.055 |
| berangin | scurve | 9.5 | 770.2 | 1.007 | -96.93 | -0.1974 | 6.123 | 6.057 | 47.36 | 2.673 |
| berangin | scurve | 11.2 | 769.8 | 1.012 | -96.6 | -0.5707 | 0.8043 | 0.7241 | 50.56 | 0.01566 |
| berangin | scurve | 13 | 770.1 | 1.008 | -96.91 | -0.2065 | 6.001 | 5.935 | 47.51 | 2.573 |
| berangin | scurve | 14.5 | 769.3 | 1.008 | -97.19 | 0.06001 | 12.49 | 9.348 | 39.37 | 7.055 |
| berangin | zv | 9.5 | 770.2 | 1.005 | -96.93 | -0.1974 | 6.123 | 6.057 | 47.36 | 2.673 |
| berangin | zv | 11.2 | 769.8 | 1.012 | -96.6 | -0.5707 | 0.8043 | 0.724 | 50.56 | 0.01566 |
| berangin | zv | 13 | 770.1 | 1.008 | -96.91 | -0.2065 | 6.001 | 5.935 | 47.51 | 2.573 |
| berangin | zv | 14.5 | 769.3 | 1.008 | -97.19 | 0.06001 | 12.49 | 9.348 | 39.37 | 7.055 |
| berangin | zvd | 9.5 | 770.2 | 1.005 | -96.93 | -0.1974 | 6.123 | 6.057 | 47.36 | 2.673 |
| berangin | zvd | 11.2 | 769.8 | 1.012 | -96.61 | -0.5707 | 0.8042 | 0.724 | 50.56 | 0.01566 |
| berangin | zvd | 13 | 770.1 | 1.008 | -96.91 | -0.2066 | 6.001 | 5.935 | 47.51 | 2.573 |
| berangin | zvd | 14.5 | 769.3 | 1.008 | -97.19 | 0.06001 | 12.49 | 9.348 | 39.37 | 7.055 |

## 5. Batas

- Model aktuator `pid` (linier dengan kejenuhan 3 N.m). Stepper sebenarnya bertorsi `Th sin(Nr delta)` yang MENURUN setelah 1.8 deg, jadi angka di sini optimistis saat selisih besar; lepas langkah tidak dimodelkan.
- Redaman sendi 0.2 N.m.s/rad dan gesekan 0.05 N.m masih ASUMSI dan paling menentukan lama ayunan.
- Suku Coriolis diabaikan; kopling antar sumbu hanya lewat J_az(q2).
- Matahari dianggap tetap di dalam satu jendela 12 s (benar sampai sekitar 0.05 deg).
- Hanya jendela terpilih yang disimulasikan 1 kHz; energi harian adalah ekstrapolasi.
