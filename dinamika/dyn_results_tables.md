# Tabel hasil dinamika (dibuat otomatis oleh dyn_model.py)

SEMUA parameter adalah asumsi atau placeholder: inersia beban (menunggu CAD), kekakuan poros, redaman dan gesekan beban, lengan angin. Lihat dyn_results.md.


## A. Respons satu microstep (default). f_eigen = mode terendah analitik; final_norm = posisi akhir beban / langkah (1 = tercapai penuh)

| sumbu | microstep | f_eigen_hz | overshoot_pct | settle_s | final_norm |
|---|---|---|---|---|---|
| elevasi | 1/8 | 8.54 | 61.4 | 0.163 | 1 |
| elevasi | 1/16 | 8.54 | 38.4 | 0.57 | 1 |
| elevasi | 1/32 | 8.54 | 0 | 0.378 | 1 |
| azimuth | 1/8 | 8.43 | 70.8 | 0.261 | 1 |
| azimuth | 1/16 | 8.43 | 53.4 | 0.437 | 1 |
| azimuth | 1/32 | 8.43 | 18.5 | 0.46 | 1 |


## A2. Sensitivitas redaman viskos beban (microstep 1/16)

| sumbu | bl | overshoot_pct | settle_s |
|---|---|---|---|
| elevasi | 0.05 | 46.8 | 0.282 |
| elevasi | 0.2 | 38.4 | 0.57 |
| elevasi | 0.5 | 24.3 | 0.639 |
| elevasi | 1 | 6.76 | 0.37 |
| azimuth | 0.05 | 60.2 | 0.493 |
| azimuth | 0.2 | 53.4 | 0.437 |
| azimuth | 0.5 | 41.3 | 0.116 |
| azimuth | 1 | 25.2 | 0.462 |


## B. Sapuan frekuensi langkah: riak terbesar dan lokasinya

| sumbu | microstep | riak_langkah_deg | riak_maks_deg | di_langkah_per_s | di_deg_per_s | f_eigen_langkah_per_s |
|---|---|---|---|---|---|---|
| elevasi | 1/8 | 0.225 | 0.509 | 7.8 | 1.76 | 8.54 |
| elevasi | 1/16 | 0.113 | 0.139 | 2.35 | 0.265 | 8.54 |
| elevasi | 1/32 | 0.0563 | 0.0487 | 2.35 | 0.132 | 8.54 |
| azimuth | 1/8 | 0.225 | 0.722 | 7.8 | 1.76 | 8.43 |
| azimuth | 1/16 | 0.113 | 0.207 | 7.8 | 0.878 | 8.43 |
| azimuth | 1/32 | 0.0563 | 0.06 | 2.35 | 0.132 | 8.43 |


## B2. Riak pada laju keyhole (acuan analisis_zenith: 0.5 sampai 2 deg/s). Baris valid = tidak: jendela 4 s terlalu sedikit langkah untuk regresi tren, abaikan

| sumbu | microstep | laju_deg_per_s | langkah_per_s | riak_pp_deg | rasio_kecepatan | valid |
|---|---|---|---|---|---|---|
| elevasi | 1/8 | 0.1 | 0.444 | 0.357 | 1.12 | tidak (kurang dari 2 langkah/detik) |
| elevasi | 1/8 | 0.5 | 2.22 | 0.335 | 1 | ya |
| elevasi | 1/8 | 1 | 4.44 | 0.314 | 0.995 | ya |
| elevasi | 1/8 | 2 | 8.89 | 0.579 | 0.998 | ya |
| elevasi | 1/16 | 0.1 | 0.889 | 0.161 | 1.11 | tidak (kurang dari 2 langkah/detik) |
| elevasi | 1/16 | 0.5 | 4.44 | 0.107 | 0.998 | ya |
| elevasi | 1/16 | 1 | 8.89 | 0.0675 | 1 | ya |
| elevasi | 1/16 | 2 | 17.8 | 0.0215 | 1 | ya |
| elevasi | 1/32 | 0.1 | 1.78 | 0.0509 | 0.984 | tidak (kurang dari 2 langkah/detik) |
| elevasi | 1/32 | 0.5 | 8.89 | 0.03 | 1 | ya |
| elevasi | 1/32 | 1 | 17.8 | 0.0107 | 1 | ya |
| elevasi | 1/32 | 2 | 35.6 | 0.0029 | 1 | ya |
| azimuth | 1/8 | 0.1 | 0.444 | 0.378 | 1.12 | tidak (kurang dari 2 langkah/detik) |
| azimuth | 1/8 | 0.5 | 2.22 | 0.362 | 1 | ya |
| azimuth | 1/8 | 1 | 4.44 | 0.335 | 0.995 | ya |
| azimuth | 1/8 | 2 | 8.89 | 0.716 | 0.997 | ya |
| azimuth | 1/16 | 0.1 | 0.889 | 0.176 | 1.09 | tidak (kurang dari 2 langkah/detik) |
| azimuth | 1/16 | 0.5 | 4.44 | 0.168 | 0.995 | ya |
| azimuth | 1/16 | 1 | 8.89 | 0.19 | 0.999 | ya |
| azimuth | 1/16 | 2 | 17.8 | 0.0212 | 1 | ya |
| azimuth | 1/32 | 0.1 | 1.78 | 0.0644 | 0.979 | tidak (kurang dari 2 langkah/detik) |
| azimuth | 1/32 | 0.5 | 8.89 | 0.0316 | 1 | ya |
| azimuth | 1/32 | 1 | 17.8 | 0.0104 | 1 | ya |
| azimuth | 1/32 | 2 | 35.6 | 0.00437 | 1 | ya |


## C. Hembusan angin (naik 0.3 s, beban gravitasi awal = grav sumbu, elevasi 0.64 N.m kasus terburuk)

| sumbu | angin_m_per_s | torsi_Nm | defleksi_puncak_deg | defleksi_akhir_deg | lepas_sinkron |
|---|---|---|---|---|---|
| elevasi | 5 | 0.797 | 0.09 | 0.09 | False |
| elevasi | 10 | 1.27 | 0.367 | 0.367 | False |
| elevasi | 15 | 2.05 | 0.911 | 0.872 | False |
| elevasi | 20 | 3.15 | 2.52e+03 | 2.52e+03 | True |
| elevasi | 25 | 4.56 | 3.82e+03 | 3.82e+03 | True |
| elevasi | 30 | 6.29 | 5.29e+03 | 5.29e+03 | True |
| elevasi | 35 | 8.32 | 6.4e+03 | 6.4e+03 | True |
| azimuth | 5 | 0.238 | 0.0994 | 0.0994 | False |
| azimuth | 10 | 0.95 | 0.415 | 0.403 | False |
| azimuth | 15 | 2.14 | 1.06 | 0.986 | False |
| azimuth | 20 | 3.8 | 3.12e+03 | 3.12e+03 | True |
| azimuth | 25 | 5.94 | 5.02e+03 | 5.02e+03 | True |
| azimuth | 30 | 8.55 | 7.31e+03 | 7.31e+03 | True |
| azimuth | 35 | 11.6 | 1e+04 | 1e+04 | True |

