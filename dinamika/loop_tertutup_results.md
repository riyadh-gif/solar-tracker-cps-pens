# Dampak stepper loop tertutup (enkoder di rotor)
Dihasilkan oleh `dinamika/loop_tertutup.py`.

**PERHATIAN, tabel ini belum tervalidasi.** Angka Gazebo yang dulu tervalidasi (azimuth 4,79 Hz prediksi berbanding 5,19 Hz terukur, lewatan 93,3 % berbanding 90 %) berasal dari motor LAMA 57HSE3.0N pada loop terbuka. Dengan 57HSE1.5N kolom loop terbuka di bawah menjadi 3,46 Hz, jadi tabel ini TIDAK lagi berjangkar pada pengukuran Gazebo mana pun. Validasi ulang lewat `rec_step.py` dan `rec_shaped.py` belum dijalankan.

## A. Loop terbuka berbanding loop tertutup ideal (rotor terpaku)
| sumbu | kasus | K_ef | f_n_Hz | zeta | lewatan_pct | tau_s | siklus_dering | defleksi_grav_deg |
|---|---|---|---|---|---|---|---|---|
| azimuth | loop terbuka | 71.6 | 3.46 | 0.0304 | 90.9 | 1.51 | 5.2 | 0.000 |
| azimuth | loop tertutup ideal | 1600.0 | 16.36 | 0.0064 | 98.0 | 1.51 | 24.8 | 0.000 |
| kemiringan | loop terbuka | 60.6 | 5.32 | 0.0551 | 84.1 | 0.54 | 2.9 | 1.205 |
| kemiringan | loop tertutup ideal | 317.0 | 12.16 | 0.0241 | 92.7 | 0.54 | 6.6 | 0.230 |

Lebih kaku tetapi KURANG teredam. `tau` tidak berubah karena hanya bergantung J dan b, sehingga dering yang sama panjangnya berisi jauh lebih banyak siklus.

## B. Driver nyata: penguatan posisi berhingga
`Kp` = kekakuan tambahan yang diberikan loop posisi driver ke rotor, dalam N.m/rad. `Kp = 0` sama dengan loop terbuka. Nilai sebenarnya bergantung driver dan belum diketahui.

| sumbu | Kp | f_n_Hz | zeta | lewatan_pct | defleksi_deg |
|---|---|---|---|---|---|
| azimuth | 0 | 3.46 | 0.0304 | 90.9 | 0.000 |
| azimuth | 150 | 5.74 | 0.0183 | 94.4 | 0.000 |
| azimuth | 500 | 8.41 | 0.0125 | 96.1 | 0.000 |
| azimuth | 1500 | 11.52 | 0.0091 | 97.2 | 0.000 |
| azimuth | 5000 | 14.27 | 0.0074 | 97.7 | 0.000 |
| azimuth | tak hingga | 16.36 | 0.0064 | 98.0 | 0.000 |
| kemiringan | 0 | 5.32 | 0.0551 | 84.1 | 1.205 |
| kemiringan | 150 | 7.83 | 0.0374 | 88.9 | 0.555 |
| kemiringan | 500 | 9.76 | 0.0300 | 91.0 | 0.357 |
| kemiringan | 1500 | 11.09 | 0.0264 | 92.0 | 0.277 |
| kemiringan | 5000 | 11.80 | 0.0248 | 92.5 | 0.245 |
| kemiringan | tak hingga | 12.16 | 0.0241 | 92.7 | 0.230 |

## C. ZVD yang TIDAK diretune
Shaper masih dirancang untuk frekuensi loop terbuka tetapi dipakai pada sistem loop tertutup.

| sumbu | f_rancang_Hz | f_nyata_Hz | tanpa_shaper_pct | zvd_lama_pct | zvd_diretune_pct |
|---|---|---|---|---|---|
| azimuth | 3.46 | 16.36 | 100.0 | 15.4 | 0.0 |
| kemiringan | 5.32 | 12.16 | 100.0 | 67.2 | 0.0 |

Ini temuan paling mendesak: ZVD lama kehilangan sebagian besar kemampuannya, dan pada sumbu kemiringan hampir tidak berguna lagi.

## D. Puntiran poros: galat yang enkoder TIDAK bisa lihat
Enkoder ada di rotor. Torsi beban memuntir poros antara rotor dan panel, dan puntiran itu tetap ada walau driver menolkan galat rotor.

| sumbu | sumber | torsi_Nm | puntir_terbuka_deg | puntir_tertutup_deg | rugi_energi_pct |
|---|---|---|---|---|---|
| kemiringan | gravitasi maks | 1.2740 | 1.2051 | 0.2303 | 0.00081 |
| kemiringan | angin terburuk 11 thn (6.22 m/s) | 0.5549 | 0.5249 | 0.1003 | 0.00015 |

## E. Resolusi enkoder berbanding microstep 1/16
Microstep 1/16 = 0.1125 deg. Kuantisasi inilah yang membangkitkan riak saat menjejak.

| enkoder_ppr | hitungan_per_putaran | resolusi_deg | berbanding_microstep_1_16 | lantai_kuantisasi |
|---|---|---|---|---|
| 1000 | 4000 | 0.0900 | 0.80 x | microstep |
| 2500 | 10000 | 0.0360 | 0.32 x | microstep |
| 4000 | 16000 | 0.0225 | 0.20 x | microstep |

Enkoder 1000 ppr yang lazim pada driver stepper loop tertutup TIDAK memperhalus riak jejak, karena resolusinya sebanding dengan microstep. Loop tertutup memperbaiki lepas langkah, bukan kehalusan jejak.

## F. Rugi energi dari galat arah statis
| galat_deg | rugi_kosinus_pct |
|---|---|
| 0.05 | 0.00004 |
| 0.10 | 0.00015 |
| 0.23 | 0.00081 |
| 0.50 | 0.00381 |
| 1.00 | 0.01523 |
| 2.00 | 0.06092 |
| 5.00 | 0.38053 |
| 7.20 | 0.78853 |

Pembanding: suhu sel 55 C = 14,9 %, toleransi pabrik 3 %, ayunan motor saat menjejak 0,007 %.
