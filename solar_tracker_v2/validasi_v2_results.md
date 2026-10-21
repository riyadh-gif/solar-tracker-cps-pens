# Validasi Gazebo konfigurasi sekarang (gigi 5:1, loop tertutup Kp 300)

Konfigurasi: `N = 5,0` pada kemiringan, `Kp = 300`, `Th = 1,5` N·m, `k_el = 163,8`, `k_az = 1600`,
`J_el = 0,0587`, `J_az = 0,1514`. Gain Gazebo `p = 303,80` (az) dan `160,99` (el), `cmd_max` 1,5 dan 7,5 N·m.
`/joint_states` terukur 1000 Hz pada seluruh rekaman. Satu penerbit per topik, diperiksa sebelum mengukur.

Prediksi: `f_n` azimuth 7,13 Hz, kemiringan 8,33 Hz; lewatan analitik orde dua 95,5 % dan 90,3 %.

---

## A. Perbandingan strategi perintah (`rec_shaped.py`)

Lewatan dalam persen. Kolom model dari `dinamika/osilasi_energi.py` pada konfigurasi sama.

| Uji | langsung Gazebo/model | scurve | zv | **zvd** |
|---|---|---|---|---|
| **az 1°** | **85,61 / 84,65** | 68,76 / 62,39 | 0,21 / 0,00 | **2,16 / 2,28** |
| az 5° | 58,40 / 75,24 | 2,99 / 5,01 | 1,68 / 0,86 | **0,80 / 0,88** |
| az 90° | 3,21 / 40,64 | 0,38 / 0,29 | 0,10 / 0,01 | **0,02 / 0,01** |
| el 20° | 5,15 / 62,77 | 0,52 / 0,55 | 0,00 / 0,06 | **0,11 / 0,06** |

**ZVD bekerja di seluruh rentang:** 85,61 % menjadi 2,16 % pada langkah kecil (40 kali), dan 3,21 % menjadi
0,02 % pada langkah besar. Seluruh strategi berprofil cocok dengan model dalam sekitar satu titik persen.

---

## B. Mengapa langkah MENTAH tidak cocok, dan kenapa itu bukan cacat model

URDF membatasi kecepatan sendi di `velocity="1"` rad/s. Model Python tidak punya batas itu, sehingga pada
langkah besar Gazebo direm sedangkan model bebas melesat. Percepatan jenuh `a = T_{h,L}/J` memberi
kecepatan puncak `v = sqrt(2 a θ)`:

| Sumbu | a (rad/s²) | 1° | 5° | 20° | 90° |
|---|---|---|---|---|---|
| azimuth | 9,91 | **0,59 bebas** | 1,31 dibatasi | 2,63 dibatasi | 5,58 dibatasi |
| kemiringan | 127,77 | 2,11 dibatasi | 4,72 dibatasi | 9,44 dibatasi | 20,03 dibatasi |

Cocokkan dengan tabel A dan polanya sempurna:

- **az 1° adalah SATU-SATUNYA kasus yang tidak terkena batas**, dan di situ Gazebo dan model sepakat
  dalam **1,1 titik persen** (85,61 berbanding 84,65).
- Makin jauh kecepatan puncak melewati batas, makin besar selisihnya: az 5° (1,31 rad/s) meleset 17 titik,
  az 90° (5,58 rad/s) meleset 37 titik.
- Kemiringan terkena batas pada semua ukuran langkah, karena percepatan jenuhnya 13 kali azimuth
  (`T_{h,L} = 7,5` N·m dibagi `J = 0,0587`), jadi langkah mentah kemiringan tidak pernah cocok.

**Jadi ketidakcocokan langkah mentah adalah artefak `velocity="1"` di URDF, bukan kesalahan dinamika.**
Begitu batas itu tidak aktif, kesepakatannya sangat baik.

---

## C. Lewatan analitik berbanding terukur

| | Analitik orde dua | Model dua massa | Gazebo |
|---|---|---|---|
| azimuth, langkah 1° | 95,5 % | 84,65 % | **85,61 %** |

Rumus analitik `exp(-pi zeta / sqrt(1-zeta^2))` memberi 95,5 % karena mengabaikan gesekan Coulomb
(0,05 N·m) dan kejenuhan torsi. Model dua massa memasukkan keduanya dan jatuh ke 84,65 %, dan Gazebo
mengukur 85,61 %. Pola yang sama pernah terlihat pada desain lama (93,3 % analitik berbanding 90 %
terukur).

---

## D. Koreksi terhadap catatan sebelumnya

Pada pengukuran Fase 4 saya melaporkan `rec_step.py` memberi lewatan **0,86 %** untuk langkah azimuth 1°
dan menandainya sebagai **selisih besar yang belum terdiagnosis** terhadap prediksi 95 %.

**Itu salah, dan penyebabnya definisi metrik, bukan fisika.** `rec_step.py` dan `rec_shaped.py` menghitung
lewatan dengan cara berbeda. Pada langkah dan konfigurasi yang sama, `rec_shaped.py` mengukur **85,61 %**
dan cocok dengan model. Tidak ada anomali yang tersisa.

`rec_step.py` juga tidak lagi memakai frekuensi prediksi yang dipatok (dulu 4,79 dan 6,89 Hz dari motor
lama); nilainya kini diturunkan dari `dyn_model.AXES`, jadi ikut berubah saat `Th`, `k`, `Kp` atau `N`
diubah.

### Frekuensi dering

Pada `rec_step.py` langkah kemiringan 5°, azimuth yang kembali ke nol berdering pada **6,49 Hz**
berbanding prediksi **7,13 Hz**, selisih 9 %. Nilai `f_ayun` pada baris sumbu yang justru diperintah
sering tidak berarti (tertulis 0,75 atau 0,46 Hz) karena responsnya hampir tanpa dering sehingga pendeteksi
persilangan tidak punya siklus untuk dihitung; abaikan kolom itu bila lewatannya kecil.

---

## E. Yang masih terbuka

- Batas `velocity="1"` rad/s di URDF adalah ASUMSI yang diwarisi desain lama dan sekarang terbukti
  mendominasi seluruh uji langkah mentah. Nilai sebenarnya harus diambil dari kurva torsi-kecepatan
  driver, dan sampai itu ada, hanya hasil berprofil (scurve, zv, zvd) yang layak dikutip.
- Backlash gigi belum dimodelkan sama sekali, dan enkoder rotor secara struktural tidak dapat melihatnya.
