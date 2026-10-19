# Dampak stepper loop tertutup (enkoder di rotor)
Dihasilkan oleh `dinamika/loop_tertutup.py`. Kolom loop terbuka mereproduksi angka yang sudah tervalidasi terhadap Gazebo (4,79 Hz, 93,3 %, 1,51 s), jadi modelnya berjangkar.

## A. Loop terbuka berbanding loop tertutup ideal (rotor terpaku)
| sumbu | kasus | K_ef | f_n_Hz | zeta | lewatan_pct | tau_s | siklus_dering | defleksi_grav_deg |
|---|---|---|---|---|---|---|---|---|
| azimuth | loop terbuka | 137.1 | 4.79 | 0.0220 | 93.3 | 1.51 | 7.2 | 0.000 |
| azimuth | loop tertutup ideal | 1600.0 | 16.36 | 0.0064 | 98.0 | 1.51 | 24.8 | 0.000 |
| kemiringan | loop terbuka | 101.7 | 6.89 | 0.0425 | 87.5 | 0.54 | 3.7 | 0.718 |
| kemiringan | loop tertutup ideal | 317.0 | 12.16 | 0.0241 | 92.7 | 0.54 | 6.6 | 0.230 |

Lebih kaku tetapi KURANG teredam. `tau` tidak berubah karena hanya bergantung J dan b, sehingga dering yang sama panjangnya berisi jauh lebih banyak siklus.

## B. Driver nyata: penguatan posisi berhingga
`Kp` = kekakuan tambahan yang diberikan loop posisi driver ke rotor, dalam N.m/rad. `Kp = 0` sama dengan loop terbuka. Nilai sebenarnya bergantung driver dan belum diketahui.

| sumbu | Kp | f_n_Hz | zeta | lewatan_pct | defleksi_deg |
|---|---|---|---|---|---|
| azimuth | 0 | 4.79 | 0.0220 | 93.3 | 0.000 |
| azimuth | 150 | 6.50 | 0.0162 | 95.0 | 0.000 |
| azimuth | 500 | 8.79 | 0.0120 | 96.3 | 0.000 |
| azimuth | 1500 | 11.66 | 0.0090 | 97.2 | 0.000 |
| azimuth | 5000 | 14.29 | 0.0074 | 97.7 | 0.000 |
| azimuth | tak hingga | 16.36 | 0.0064 | 98.0 | 0.000 |
| kemiringan | 0 | 6.89 | 0.0425 | 87.5 | 0.718 |
| kemiringan | 150 | 8.48 | 0.0346 | 89.7 | 0.474 |
| kemiringan | 500 | 9.97 | 0.0294 | 91.2 | 0.343 |
| kemiringan | 1500 | 11.14 | 0.0263 | 92.1 | 0.275 |
| kemiringan | 5000 | 11.80 | 0.0248 | 92.5 | 0.244 |
| kemiringan | tak hingga | 12.16 | 0.0241 | 92.7 | 0.230 |

## C. ZVD yang TIDAK diretune
Shaper masih dirancang untuk frekuensi loop terbuka tetapi dipakai pada sistem loop tertutup.

| sumbu | f_rancang_Hz | f_nyata_Hz | tanpa_shaper_pct | zvd_lama_pct | zvd_diretune_pct |
|---|---|---|---|---|---|
| azimuth | 4.79 | 16.36 | 100.0 | 34.7 | 0.0 |
| kemiringan | 6.89 | 12.16 | 100.0 | 75.9 | 0.0 |

Ini temuan paling mendesak: ZVD lama kehilangan sebagian besar kemampuannya, dan pada sumbu kemiringan hampir tidak berguna lagi.

## D. Puntiran poros: galat yang enkoder TIDAK bisa lihat
Enkoder ada di rotor. Torsi beban memuntir poros antara rotor dan panel, dan puntiran itu tetap ada walau driver menolkan galat rotor.

| sumbu | sumber | torsi_Nm | puntir_terbuka_deg | puntir_tertutup_deg | rugi_energi_pct |
|---|---|---|---|---|---|
| kemiringan | gravitasi maks | 1.2740 | 0.7176 | 0.2303 | 0.00081 |
| kemiringan | angin terburuk 11 thn (6.22 m/s) | 0.5549 | 0.3125 | 0.1003 | 0.00015 |

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
