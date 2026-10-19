# Validasi Gazebo setelah motor 57HSE1.5N (Fase 4)

Konfigurasi: `Th = 1.5` N·m, **`Kp = 0` (masih loop terbuka)**, microstep 1/16, gravitasi 10 m/s², panel 2,55 kg.
Gazebo headless, `/joint_states` terukur 1000 Hz pada semua rekaman.

Tujuan utama: menyelesaikan perselisihan antara prediksi kecil-sinyal (3,46 Hz) dan ODE `osilasi_energi`
(1,37 Hz) dengan bukti pihak ketiga.

---

## A. Perbandingan strategi perintah (`rec_shaped.py`, aktuator `pid`)

Lewatan dalam persen. Kolom model dari `dinamika/osilasi_energi.py` pada konfigurasi sama.

### Azimuth 90 derajat

| Strategi | Gazebo lewatan | Model lewatan | Gazebo menetap | Model menetap |
|---|---|---|---|---|
| langsung | 40,62 | 40,64 | 3,72 s | 3,72 s |
| scurve | 1,23 | 1,28 | 3,00 s | 3,00 s |
| zv | 0,01 | 0,00 | 3,06 s | 3,06 s |
| **zvd** | **0,00** | **0,00** | 3,15 s | 3,15 s |

### Azimuth 5 derajat

| Strategi | Gazebo | Model |
|---|---|---|
| langsung | 74,63 | 75,38 |
| scurve | 44,37 | 46,34 |
| zv | 0,00 | 0,51 |
| **zvd** | **0,00** | **0,26** |

### Kemiringan 20 derajat

| Strategi | Gazebo | Model |
|---|---|---|
| langsung | **8,01** | **59,66** |
| scurve | 2,97 | 2,99 |
| zv | 0,13 | 0,00 |
| **zvd** | **0,00** | **0,00** |

**Kesimpulan A.** Model Python dan Gazebo cocok dalam 0,05 titik persen pada azimuth, di dua ukuran langkah
yang berbeda 18 kali. Itu validasi kuat. ZVD menekan lewatan azimuth 90 derajat dari 40,62 % ke **0,00 %**.

Ketidakcocokan langkah mentah kemiringan (8,01 berbanding 59,66) **bukan regresi**: itu artefak
`velocity="1"` rad/s di URDF yang berlaku sebagai rem sempurna, sudah terdokumentasi sebelumnya pada
konfigurasi lama dengan angka 5,98 berbanding 66,03. Model Python tidak punya rem itu. Semua strategi
berbentuk profil tetap cocok, jadi artefaknya hanya menyentuh langkah mentah.

---

## B. Respons langkah (`rec_step.py`, aktuator `pid`)

Langkah azimuth 1 derajat:

| Besaran | Terukur |
|---|---|
| Nilai akhir | 0,982 dari 1,000 derajat (sisa -0,018) |
| Lewatan | **0,85 %** |
| Waktu menetap | 0,78 s |
| Defleksi gravitasi kemiringan saat diam | **-0,244 derajat** |

**Dua catatan penting.**

1. **Defleksi gravitasi -0,244 derajat memvalidasi kekakuan baru.** Prediksi `Mg(0)/K_el = 0,276 / 60,65 =
   0,261 derajat`. Cocok. Pada konfigurasi lama (Th 3,0) nilainya 0,155 derajat prediksi berbanding 0,161
   terukur. Jadi rantai Th menuju K_ser menuju defleksi benar ujung ke ujung.

2. **Lewatan 0,85 % sama sekali tidak cocok dengan prediksi orde dua (sekitar 91 % pada zeta 0,0304).**
   Pada konfigurasi lama angka ini 93,3 % teori berbanding 90 % terukur. Sebabnya **belum terdiagnosis**.
   Sisa galat tetap -0,018 derajat setara torsi 0,023 N·m, di bawah gesekan Coulomb 0,05 N·m, jadi sendi
   memang berhenti karena gesekan. Dugaan sementara: dengan Th separuh, torsi pegas pada simpangan kecil
   ikut separuh sedangkan gesekan Coulomb tetap 0,05 N·m, sehingga rasio gesekan terhadap pegas berlipat
   dan dering teredam habis. **Dugaan ini belum diuji.**

`rec_step.py` juga masih memuat prediksi frekuensi yang dipatok (4,79 dan 6,89 Hz) dari motor lama. Angka
itu salah sekarang dan harus diturunkan dari `dyn_model`.

---

## C. Angin, aktuator stepper (`rec_wind.py`)

Hari 2026-10-07, pengali 1,0, 120 detik, v maks 1,5 m/s di panel.

| Sumbu | Torsi maks | Terhadap Th 1,5 | Defleksi diprediksi | Lepas langkah |
|---|---|---|---|---|
| Azimuth | 0,07 N·m | 4,7 % | 0,059 derajat | tidak, sesuai prediksi |
| Kemiringan | 0,03 N·m | 2,0 % | 0,031 derajat | **YA, pada cuplikan pertama** |

**Angin bukan penyebabnya.** Torsi angin kemiringan hanya 0,03 N·m, yaitu 2 % torsi tahan.

### Lepas langkah kemiringan: satu siklus listrik, terkunci saat start

Keadaan tunak terukur: perintah 0,000 derajat, beban **-7,648 derajat**, menetap di situ.

Itu persis satu siklus listrik ditambah ketertinggalan gravitasi pada pose baru:

```
satu siklus listrik      = 360 / Nr = 7,200 derajat
gravitasi pada q2 = -7,648 deg = 0,477 N.m
ketertinggalan rotor     = asin(0,477 / 1,5) / 50 = 0,371 derajat
jumlah                   = 7,571 derajat   (terukur 7,648)
```

**Ini bukan kegagalan menahan beban dalam keadaan tunak.** Untuk menahan gravitasi 0,276 N·m pada q2 = 0
hanya dibutuhkan ketertinggalan rotor `asin(0,276/1,5)/50 = 0,212 derajat`, jauh di bawah puncak sinus
1,8 derajat. Jadi motor **mampu** menahan panel saat diam.

Yang terjadi adalah transien saat start: sebelum `stepper_node` mulai menerbitkan torsi, sendi kemiringan
jatuh bebas di bawah gravitasi, dan saat torsi mulai bekerja simpangan sudah melewati setengah siklus
listrik, sehingga model mengunci ke cabang yang salah.

**Mengapa ini lebih mudah terjadi pada 57HSE1.5N:** ketertinggalan rotor yang dibutuhkan berlipat dua
(0,212 berbanding 0,105 derajat pada Th 3,0), dan margin dinamis ke puncak sinus 1,8 derajat menyusut.

Azimuth tidak lepas langkah sama sekali (0,000 derajat), dan azimuth memang tidak memikul gravitasi. Itu
menguatkan bahwa penyebabnya gravitasi saat start, bukan angin.

**Kaitan ke Fase 0:** pemberat penyeimbang menurunkan gravitasi puncak 4,6 kali (1,274 ke 0,276 N·m) dan
gravitasi pada q2 = 0 menjadi nol, yang besar kemungkinan menghapus kejadian ini seluruhnya. Belum diuji.

---

## D. Yang belum dijalankan

- `rec_day.py` (pelacakan sehari penuh). Prioritas terendah menurut rencana: riak jejak ditentukan
  kuantisasi microstep dan `k`, bukan oleh perubahan ini.
- Seluruh validasi di atas memakai **`Kp = 0`**, jadi belum ada satu pun pengukuran Gazebo untuk
  konfigurasi loop tertutup. `loop_tertutup_results.md` tetap tanpa jangkar pengukuran.

## E. Yang masih terbuka setelah Fase 4

1. **Lewatan 0,85 % berbanding prediksi 91 %** pada langkah 1 derajat. Belum terdiagnosis.
2. **Dering ODE 1,37 Hz berbanding prediksi kecil-sinyal 3,46 Hz** di `osilasi_energi.self_check`.
   Gazebo tidak menyelesaikan ini secara langsung karena aktuator `pid` adalah pegas linier, bukan hukum
   sinus stepper. Keduanya kemungkinan besar bergejala sama: gesekan dan ketaklinieran mendominasi pada
   amplitudo kecil setelah Th diperkecil.
3. Prediksi frekuensi yang dipatok di `rec_step.py` masih dari motor lama.
