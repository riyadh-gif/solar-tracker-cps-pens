# Parameter yang Masih Diasumsikan, dan Cara Mengukurnya

Dokumen kerja untuk mengubah model dari **Digital Model** (sekarang) menjadi **Digital Shadow**.
Semua nilai di sini adalah tebakan saya, bukan hasil pengukuran alat Anda. Dokumen induk parameter ada di
`parameter_kinematik.md`; berkas ini khusus membahas apa yang belum diukur dan bagaimana mengukurnya.

Ringkasan: **ada sekitar 31 parameter yang masih diasumsikan.** Tetapi tidak semuanya sama penting. Lima di
antaranya menentukan hampir seluruh hasil, dan semuanya dapat diukur tanpa integrasi perangkat lunak apa pun.

---

## 1. Lima pengukuran dengan nilai tertinggi

Kerjakan ini lebih dulu. Semuanya pekerjaan meja, beberapa jam, tanpa ROS dan tanpa mikrokontroler.

| # | Pengukuran | Alat | Memperbaiki | Kalau salah, akibatnya |
|---|---|---|---|---|
| 1 | **Uji peluruhan bebas** | perekam sudut sementara (video lambat, IMU murah, atau enkoder pinjam) | redaman `bl`, gesekan `fc`, kekakuan `k`, frekuensi alami | **seluruh hasil dinamika dan peredam osilasi** |
| 2 | **Kurva LDR** (lux terhadap tegangan) | lux meter dan multimeter | `gam`, `L0` | batas atas SEMUA kalibrasi dan ketelitian LDR |
| 3 | **Batas kemiringan** | gerakkan sampai menumbuk, busur derajat | `EL_LIMIT`, `Q2_MAX` | jangkauan harian dan energi panen |
| 4 | **Arah hadap rel** | kompas | `HEAD` | seluruh kinematik dan arah penjejakan |
| 5 | **Ketidakcocokan antar LDR** | prosedur A saat mendung | `mism` | **menentukan LDR berguna atau justru merugikan** |

### Mengapa uji peluruhan bebas paling penting

Satu percobaan ini mengukur empat parameter sekaligus. Caranya: kunci satu sumbu di posisi diam, beri satu
langkah perintah (misal 5 derajat), rekam sudutnya, lalu baca dari grafik:

- **frekuensi ayunan** memberi kekakuan seri K bila inersia J sudah diketahui dari CAD, lewat f = akar(K/J) dibagi 2 pi
- **laju peluruhan** memberi redaman `bl`, lewat zeta = -ln(rasio puncak berurutan) dibagi 2 pi, lalu bl = 2 zeta akar(K J)
- **simpangan sisa** yang tidak kembali ke nol memberi gesekan Coulomb `fc`
- **lewatan** memvalidasi seluruh rantai

Ramalan model sekarang yang harus Anda bandingkan:

| Sumbu | Frekuensi | Redaman zeta | Lewatan langkah | Waktu peluruhan |
|---|---|---|---|---|
| Azimuth | 4,79 Hz | 0,022 | 93 % | 1,51 s |
| Elevasi | 6,89 Hz | 0,043 | 88 % | 0,54 s |

Kalau hasil ukur jauh berbeda, seluruh bab dinamika dan peredam osilasi harus dihitung ulang. Kalau cocok,
Anda punya validasi kuat untuk tesis.

---

## 2. Mekanik dan struktur

| Parameter | Nilai sekarang | Lokasi di kode | Cara mengukur |
|---|---|---|---|
| Redaman viskos beban `bl` | 0,2 N.m.s/rad | `dinamika/dyn_model.py` | uji peluruhan bebas |
| Gesekan Coulomb beban `fc` | 0,05 N.m | `dyn_model.py` | torsi minimum yang mulai menggerakkan sumbu |
| Kekakuan puntir poros `k` | elevasi 317, azimuth 1600 N.m/rad | `dyn_model.py` pada `AXES` | uji peluruhan bebas, atau beri torsi statis lalu ukur puntirannya |
| Rasio redaman mode poros `zeta` | 0,02 | `dyn_model.py` | uji peluruhan bebas |
| Redaman rotor dan driver `bm` | 0,005 N.m.s/rad | `dyn_model.py` | lembar data driver, atau abaikan (pengaruhnya kecil) |
| Inersia rotor `Jm` | 1,1e-4 kg.m2 | `dyn_model.py` | lembar data motor 57HSE3.0N |
| Redaman dan gesekan sendi URDF | 0,2 dan 0,05 | `build_model.py` pada `JOINT_DAMP` dan `JOINT_FRIC` | ikut hasil uji peluruhan bebas |
| **Batas kemiringan** `EL_LIMIT` | 45 derajat | `build_model.py`, `tracker.py` pada `Q2_MAX` | gerakkan sampai menumbuk rangka, ukur dengan busur derajat |
| **Arah hadap rel** `HEAD` | 90 derajat (rel menghadap Utara) | `tracker.py` | kompas, catat arah rel panjang |
| Batas kecepatan sendi elevasi | 1 rad/s di URDF | `build_model.py` | laju maksimum driver sebelum lepas langkah |
| Massa motor | 1,2 kg per motor | `inersia/inersia_cad.py` pada `MOTOR_KG` | timbang, atau lembar data |
| Isi tiang miring (bagian 8 dan 9) | 15 % volume pembungkus | `inersia_cad.py` pada `SPEC` | timbang bagiannya |
| Isi rumah pillow block (bagian 13) | 30 % volume pembungkus | `inersia_cad.py` | timbang; bila besi cor, massanya 2,6 kali lebih besar |
| Bahan pengencang dan braket kecil | baja atau aluminium | `inersia_cad.py` | lihat dan timbang |
| Posisi dan orientasi panel di rel | sisi 0,670 m searah engsel | `build_model.py` pada `PANEL` | meteran, catat pusat panel terhadap sumbu engsel |
| Ketinggian lokasi | 5 m | `sun.py` | GPS atau peta |

**Catatan penting tentang batas kecepatan URDF.** Nilai `velocity="1"` rad/s pada sendi elevasi berperilaku
sebagai rem sempurna di Gazebo. Terbukti menyebabkan selisih besar antara model Python dan Gazebo pada perintah
mentah (lewatan 66 persen berbanding 6 persen). Stepper nyata tidak punya rem itu, ia akan lepas langkah.
Ukur laju maksimum yang sebenarnya, dan sadari bahwa di atas itu perilakunya lepas langkah, bukan melambat.

---

## 3. Sensor LDR

Kelompok ini paling menentukan apakah LDR berguna. Hasil terukur: **LDR tak terkalibrasi membuat galat pointing
5,63 derajat, lebih buruk daripada tanpa LDR sama sekali (2,52 derajat).** Setelah kalibrasi: 0,16 derajat.

| Parameter | Nilai sekarang | Lokasi | Cara mengukur |
|---|---|---|---|
| **Eksponen LDR `gam`** | 0,7 | `kontrol_hibrid/hybrid_sim.py` | di meja: lux meter dan multimeter, sapu tingkat cahaya, cocokkan kurvanya |
| **Setengah jenuh `L0`** | 20000 lux | `hybrid_sim.py` | dari kurva yang sama |
| **Ketidakcocokan antar LDR `mism`** | 0,03 (3 %) | `hybrid_sim.py` | prosedur A: saat mendung tebal, rekam keempat LDR, bedanya adalah ketidakcocokannya |
| Tinggi sekat per lebar `r` | 1,0 | `hybrid_sim.py` | meteran: tinggi sekat dibagi lebar jendela LDR |
| Derau ADC `vn` | 2 mV | `hybrid_sim.py` | rekam LDR pada cahaya tetap, hitung simpangan bakunya |
| Tegangan catu dan bit ADC | 3,3 V dan 12 bit | `hybrid_sim.py` | keputusan rancangan, tinggal dicatat |
| Laju maksimum kemiringan | 1,0 derajat/s | `hybrid_sim.py` pada `rate2` | laju maksimum driver |
| Ambang hibrid Saputra `wthr` | 400 W/m2 | `hybrid_sim.py` | hanya untuk pembanding metode pustaka |

**Urutan kalibrasi yang disarankan** (rinciannya di `kontrol_hibrid/kalibrasi_ldr_results.md`):

1. **Ukur kurva LDR di meja lebih dulu.** Galat 10 persen pada `gam` menyisakan 0,26 persen ketidakcocokan,
   dua puluh kali lebih buruk daripada batas dari derau. Memperbanyak cuplikan tidak menolong kalau kurvanya salah.
2. **Prosedur A saat mendung tebal.** Sekat hanya membayangi komponen langsung, jadi di bawah cahaya baur
   keempat LDR menerima irradiance sama berapa pun arah panel. Rekam beberapa ratus cuplikan, simpan faktor koreksi.
3. **Prosedur B sepanjang satu hari cerah** untuk memisahkan sisa salah pasang azimuth dari ketidakcocokan.
   Hanya azimuth yang terpisahkan; kemiringan tidak (bilangan kondisi 23936).
4. **Ulangi prosedur A berkala**, karena LDR menua dan kotor dengan laju berbeda.

---

## 4. Listrik panel

Pelat nama sudah cukup untuk modelnya. Yang diasumsikan hanya besaran yang tidak tercantum di pelat nama.

| Parameter | Nilai sekarang | Lokasi | Cara mengukur |
|---|---|---|---|
| Jumlah sel seri | 36 | `kontrol_hibrid/panel_listrik.py` pada `NS` | hitung sel pada panel, atau lembar data lengkap |
| Koefisien suhu Isc | +0,05 %/C | `panel_listrik.py` pada `ALPHA_SC` | lembar data lengkap, atau ukur Isc pada dua suhu |
| Koefisien suhu Voc | -0,30 %/C | `panel_listrik.py` pada `BETA_VOC` | lembar data lengkap, atau ukur Voc pada dua suhu |
| NOCT | 45 C | `panel_listrik.py` | lembar data, atau tempel termokopel di belakang panel |
| Suhu udara | 30 C | `panel_listrik.py` pada `suhu_sel` | termometer di lokasi |
| Koefisien IAM `b0` | 0,05 (ASHRAE) | `panel_listrik.py` pada `B0_IAM` | ukur arus pada beberapa sudut datang |
| Faktor ideal dioda `n` | 1,1 | `panel_listrik.py` pada `N_IDEAL` | **tidak perlu diukur**: sudah diperiksa, n 1,0 sampai 1,5 mengubah arus hanya 0,13 persen |
| **Albedo tanah** | 0,25 | `panel_listrik.py` pada `ALB` | dua piranometer (menghadap atas dan bawah), atau tabel menurut jenis permukaan |

**Albedo lebih penting daripada kelihatannya.** Puncak arus bergeser dari arah matahari sebesar -2,75 derajat
pada albedo 0,10 sampai +2,75 derajat pada albedo 0,40. Jadi kalau Anda memakai arus sebagai acuan posisi,
**arah simpangannya pun tidak diketahui** sebelum albedo lokasi diukur.

Yang **tidak** perlu diukur karena sudah dari pelat nama: Pmax 50 W, Voc 22,5 V, Isc 3 A, Vmp 18,5 V,
Imp 2,7 A, dimensi 670 x 445 x 25 mm, massa 2,55 kg.

---

## 5. Sensor arus

Seluruh rantai ini adalah rancangan yang belum dipilih. Ganti setelah perangkatnya ditentukan.

| Parameter | Nilai sekarang | Lokasi | Cara mendapatkan |
|---|---|---|---|
| Bit dan jangkauan ADC | 12 bit, 0 sampai 4 A | `kontrol_hibrid/sensor_arus.py` | keputusan rancangan |
| Derau acak | 2 mA rms | `sensor_arus.py` pada `NOISE_A` | rekam pada arus tetap, hitung simpangan bakunya |
| Hanyutan offset | 5 mA | `sensor_arus.py` pada `DRIFT_A` | rekam beberapa jam pada arus nol |
| Laju cuplik | 100 Hz | `sensor_arus.py` pada `F_SAMPLE` | keputusan rancangan |
| Ketidakpastian model arus | 3 % | `kontrol_hibrid/ekf.py` pada `K_MODEL_ARUS` | bandingkan arus terukur dengan ramalan selama sehari |

**Peringatan yang sudah terbukti merusak:** parameter terakhir itu wajib memuat galat **model**, bukan hanya
derau sensor. Mengisinya dengan derau ADC saja membuat penguatan Kalman ribuan kali terlalu besar dan tapis
rusak total (galat pointing melonjak dari 0,18 ke 20,5 derajat).

---

## 6. Angin

**Seluruh kelompok ini belum dapat dipakai untuk mengambil keputusan rancangan.**

| Parameter | Nilai sekarang | Lokasi | Cara mendapatkan |
|---|---|---|---|
| Koefisien gaya normal `Cn` | 1,2 | `dyn_model.py` pada `wind_torque` | tabel beban angin standar, terowongan angin, atau CFD |
| Lengan momen angin elevasi | 0,0668 m | `dyn_model.py` pada `AXES` | letak pusat tekanan pelat datar miring |
| Lengan momen angin azimuth | 0,152 m | `dyn_model.py` pada `AXES` | **murni tebakan**, tidak ada dasar fisis |

Saya sempat melaporkan batas angin tarik-lepas azimuth 9,6 m/s dan menyebutnya risiko sehari-hari.
**Klaim itu harus ditarik.** Hasilnya berubah enam kali lipat tergantung lengan yang saya tebak:

| Lengan azimuth | v tarik-lepas | Dasar |
|---|---|---|
| 0,009 m | 39,4 m/s | eksentrisitas geometris nyata: pusat massa panel hanya 9,2 mm dari sumbu azimuth |
| 0,152 m | 9,6 m/s | tebakan saya, tanpa dasar fisis |
| 0,335 m | 6,5 m/s | kasus terburuk, seluruh gaya di satu ujung |

Panel hampir simetris terhadap sumbu azimuth, jadi angin merata nyaris tidak menghasilkan momen puntir tegak.
Momen azimuth yang nyata datang dari turbulensi, yang tidak dimodelkan di sini. Elevasi lebih berdasar
(11,2 sampai 17,7 m/s) karena pusat tekanan pelat datar memang bergeser 10 sampai 25 persen sisi melintang.

---

## 7. Penaksir (EKF)

Parameter penyetelan, bukan besaran fisis. Diperbaiki dengan data lapangan, bukan dengan alat ukur.

| Parameter | Nilai sekarang | Lokasi | Cara memperbaiki |
|---|---|---|---|
| Galat astro `SIG_ASTRO` | 1,5 derajat | `kontrol_hibrid/fusi_sensor.py` | dari ketelitian pemasangan dan frekuensi lepas langkah yang teramati |
| Jalan acak `Q_DRIFT` | 0,01 derajat per akar detik | `kontrol_hibrid/ekf.py` | dari hanyutan yang teramati sepanjang hari |
| Ambang pencilan `nis_batas` | 25 | `solar_tracker/ekf_node.py` | dari sebaran NIS yang teramati |
| Batas penolakan beruntun | 10 | `ekf_node.py` pada `TOLAK_MAKS` | dari pengalaman lapangan |

**Diketahui kurang:** tapis masih terlalu percaya diri sekitar 3,4 kali pada azimuth. Sigma yang dilaporkan
0,23 derajat sedangkan sebaran galat sebenarnya 0,79 derajat. Sebabnya `Q` berbentuk jalan acak halus,
sedangkan lepas langkah datang sebagai lompatan besar sesekali.

---

## 8. Yang SUDAH terukur, jangan diukur lagi

| Besaran | Sumber |
|---|---|
| Panel: 50 W, Voc 22,5 V, Isc 3 A, Vmp 18,5 V, Imp 2,7 A, 670 x 445 x 25 mm, 2,55 kg | pelat nama MOS C-50-18-M |
| Titik poros azimuth (0, 0, 0,165) m dan sumbu elevasi (0, 0, 0,2045) m | CAD |
| Kemiringan rel pada pose CAD 21,24 derajat | CAD |
| Massa profil aluminium 2020 | volume mesh CAD dikali 2700 kg/m3 |
| Motor: torsi tahan 3 N.m, 50 gigi rotor, 1,8 derajat per langkah | spesifikasi 57HSE3.0N |
| Lintang dan bujur PENS -7,276006 dan 112,794327 | keputusan |
| Gravitasi 10 m/s2 | keputusan pengguna |
| Posisi matahari (pvlib SPA) | tervalidasi terhadap astropy, SolTrack, Pysolar, selisih di bawah 0,011 derajat |

---

## 9. Lembar kerja: urutan pengerjaan

### Tahap 1, meja dan lapangan, tanpa perangkat lunak

- [ ] Kompas: arah rel panjang, catat derajat dari Utara. Perbarui `tracker.HEAD`.
- [ ] Busur derajat: batas kemiringan ke dua arah. Perbarui `EL_LIMIT` dan `Q2_MAX`.
- [ ] Timbangan: massa motor, tiang, rumah pillow block. Perbarui `inersia_cad.py`.
- [ ] Meteran: tinggi sekat dan lebar jendela LDR, posisi panel terhadap sumbu engsel.
- [ ] Lux meter dan multimeter: kurva LDR. Perbarui `gam` dan `L0`.
- [ ] **Uji peluruhan bebas pada kedua sumbu.** Perbarui `bl`, `fc`, `k`, dan `zeta`.
- [ ] Termometer: suhu udara khas siang di lokasi.

### Tahap 2, elektronik minimal, belum ROS

- [ ] Mikrokontroler membaca 4 LDR dan arus panel, kirim ke serial, catat ke berkas.
- [ ] Ukur derau dan hanyutan sensor arus pada keadaan tetap.
- [ ] **Prosedur A saat mendung tebal.** Simpan faktor koreksi per LDR.
- [ ] Bandingkan arus terukur dengan ramalan model sehari penuh. Perbarui `K_MODEL_ARUS`.

### Tahap 3, integrasi ROS

- [ ] Node pembaca serial yang menerbitkan ke `/tracker/diag` dengan bentuk pesan sama seperti `sensor_node.py`.
      Seluruh rantai di belakangnya (EKF, panel web, grafik) jalan tanpa diubah.
- [ ] Jalankan EKF pada data nyata. Ini sudah **Digital Shadow**.
- [ ] **Prosedur B** sepanjang satu hari cerah untuk sisa bias azimuth.
- [ ] Tutup loop ke driver. Ini **Digital Twin**.

### Tahap 4, yang hanya bisa dilakukan setelah ada alat nyata

- [ ] Backlash pada kopling motor ke poros (tidak dimodelkan sama sekali).
- [ ] Mode getar rangka (model sekarang menganggap rangka kaku kecuali poros).
- [ ] Laju pengotoran LDR dan panel.
- [ ] Koefisien angin, bila memang akan dijadikan dasar rancangan parkir.

---

## 10. Bacaan pendukung

| Berkas | Isi |
|---|---|
| `parameter_kinematik.md` | dokumen induk seluruh parameter, 16 bagian |
| `dinamika/osilasi_results.md` | osilasi dua sumbu dan dampaknya pada energi |
| `kontrol_hibrid/kalibrasi_ldr_results.md` | prosedur A dan B, lengkap dengan angkanya |
| `kontrol_hibrid/ekf_results.md` | hasil EKF dan batas mendasarnya |
| `kontrol_hibrid/panel_listrik_results.md` | model dioda tunggal dan arus sebagai sensor |
| `kontrol_hibrid/sensor_arus_results.md` | rantai pengukuran arus dan batas keterbacaannya |
| `kontrol_hibrid/fusi_sensor_results.md` | ketidakpastian tiap sensor dan nilai kalibrasi |
| `solar_tracker/README_gazebo.md` | cara menjalankan simulasi dan isi topik diagnostik |
