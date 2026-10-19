# Parameter Penting Desain Kinematik Solar Tracker 2 Sumbu

Dokumen ini merangkum parameter kinematik tracker (azimuth dan kemiringan, 4 LDR, titik PENS Surabaya) dan **di berkas mana tiap parameter didefinisikan**. Kolom Status: **ukur** = berasal dari CAD atau spesifikasi, **ASUMSI** = belum diukur di alat nyata (harus diganti setelah pengukuran), **keputusan** = dipilih bersama pengguna.

Semua jalur relatif terhadap folder proyek `modelkinematik/`.

## 1. Konvensi koordinat dan sudut

| Besaran            | Definisi                                                                                     | Sumber                                                       |
| ------------------ | -------------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| Frame dunia = dasar | x = Utara (ASUMSI arah rel panjang), y = Barat, z = atas. Tidak ada rotasi tambahan antara dunia dan dasar |
| q1 (azimuth joint) | Putar sekitar sumbu z, positif searah jarum jam dari atas, tanpa batas (continuous). Arah miring kompas = q1 + HEAD | `tracker.py`, URDF `azimuth_joint` di `build_model.py` |
| q2 (kemiringan)    | Sudut panel dari horizontal, 0 = datar, positif memiringkan panel ke arah -y alat (pada q1 = 0 dan q2 > 0: ke Timur)                 | `tracker.py`, URDF `elevation_joint`                     |
| Azimuth matahari   | Searah jarum jam dari Utara (kompas), elevasi dari horizon                                   | `sun.py` (`sun_day`)                                     |
| Zenith matahari    | 90 derajat dikurangi elevasi                                                                 | `tracker.py` (`ik_track`)                                |

## 2. Persamaan kinematik

**Kinematika maju** (normal panel di frame dasar), `tracker.fk(q1, q2)`:

```
n = ( sin q2 cos(q1+HEAD) , -sin q2 sin(q1+HEAD) , cos q2 )     HEAD = 90 derajat (`tracker.HEAD`)
```

**Vektor matahari** dari azimuth A dan elevasi e, `tracker.sun_vec(A, e)`:

```
s = ( cos e cos A , -cos e sin A , sin e )
```

**Galat pointing** (sudut antara normal panel dan matahari): `arccos(n . s)`. Dipakai di `gabungan/coupled_sim.py`, `solar_tracker/rec_day.py`, `solar_tracker/control_web.py`.

**Kinematika balik** (`tracker.ik_track`): untuk matahari (A, zenith z) ada dua solusi, (q1, q2) = (A - HEAD, z) dan (A - HEAD + 180, -z). Fungsi memilih cabang yang kontinu dari langkah sebelumnya, sehingga lewat zenith tidak perlu memutar azimuth 180 derajat. Bila z > batas kemiringan hasilnya NaN (di luar jangkauan).

**Singularitas:** di zenith (z = 0) q1 tidak terdefinisi (kutub koordinat, efek keyhole). Dekat zenith, laju azimuth yang dibutuhkan membesar. Dianalisis di `analisis_zenith/`.

## 3. Geometri mekanik (dari CAD)

| Parameter                               | Nilai                                                                       | Status                                                          | Lokasi                                                                                                                  |
| --------------------------------------- | --------------------------------------------------------------------------- | --------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| Titik poros azimuth`P_AZ`             | (0, 0, 0.165) m dari dasar                                                  | ukur (CAD)                                                      | `build_model.py`, konstanta `P_AZ`                                                                                  |
| Titik sumbu elevasi`P_EL`             | (0, 0, 0.2045) m                                                            | ukur (CAD)                                                      | `build_model.py`, konstanta `P_EL`                                                                                  |
| Offset sumbu elevasi dari poros azimuth | (0, 0, 0.0395) m                                                            | turunan                                                         | `build_model.py`, origin `elevation_joint`                                                                          |
| Kemiringan rel pada pose CAD asli       | 21.24 derajat (diputar balik agar pose nol horizontal)                      | ukur (CAD)                                                      | `build_model.py`, konstanta `TILT`                                                                                  |
| Sumbu`azimuth_joint`                  | (0, 0, -1), kontinu                                                         | keputusan                                                       | `build_model.py`, URDF di `main()`                                                                                  |
| Sumbu`elevation_joint`                | (1, 0, 0) searah rel, revolute                                                         | keputusan                                                       | `build_model.py`, URDF di `main()`                                                                                  |
| Batas elevation_joint                   | +-45 derajat (+-0.7854 rad), effort 3 N.m, velocity 1 rad/s                 | **ASUMSI** (rel menumbuk rangka, belum diukur)            | `build_model.py` (`EL_LIMIT`), `tracker.py` (`Q2_MAX`), `solar_tracker/sun_command_node.py` (param `q2max`) |
| Ukuran panel                            | 0.670 x 0.445 x 0.025 m (sisi 0.670 m searah sumbu engsel)                  | ukur (spesifikasi MOS C-50-18-M); orientasi = keputusan    | `build_model.py`, konstanta `PANEL`                                                                                 |
| Massa link                              | base 2.134 kg, azimuth_link 1.439 kg, elevation_link 0.505 kg, panel 2.55 kg | motor 1.2 kg**ASUMSI**; panel 2.55 kg dari spesifikasi; sisanya dari CAD | `inersia/inersia_link.json` (dibaca `build_model.py`, `inertial_xml`)                                             |
| Inersia beban sumbu                     | elevasi 0.0543 kg.m2 (sumbu x), azimuth 0.1514 kg.m2 (q2 = 0)               | dari CAD + massa panel spesifikasi                              | `dinamika/dyn_model.py`, `AXES`                                                                                     |
| Peredam dan gesekan sendi               | 0.2 N.m.s/rad dan 0.05 N.m                                                  | **ASUMSI**                                                | `build_model.py`, `JOINT_DAMP`, `JOINT_FRIC`                                                                      |
| Mesh                                    | 3 link (base, azimuth_link, elevation_link) dari`parts_v2`                | ukur (CAD)                                                      | `build_model.py`, `GROUPS`; berkas di `solar_tracker/meshes/`                                                     |

| Gravitasi                               | 10 m/s2                                                                     | keputusan (bukan 9.80665)                                       | `inersia/inersia_cad.py` (`G`), `solar_tracker/world.sdf` dan `world_stepper.sdf` (`<gravity>`) |

## 4. Titik pemasangan dan matahari

| Parameter                 | Nilai                                                                                      | Status           | Lokasi                                                  |
| ------------------------- | ------------------------------------------------------------------------------------------ | ---------------- | ------------------------------------------------------- |
| Lintang, bujur            | -7.276006, 112.794327 (PENS)                                                               | keputusan        | `sun.py`, `LAT`, `LON`                            |
| Zona waktu                | Asia/Jakarta                                                                               | keputusan        | `sun.py`, `TZ`                                      |
| Ketinggian                | 5 m                                                                                        | **ASUMSI** | `sun.py`, `pvlib.location.Location(... altitude=5)` |
| Algoritma posisi matahari | pvlib SPA (NREL), divalidasi terhadap astropy, SolTrack, Pysolar (selisih < 0.011 derajat) | ukur             | `sun.py`; validasi di `validasi_matahari/`          |
| Irradiance                | Ineichen clear-sky, faktor awan Kasten-Czeplak (okta), dekomposisi Erbs                    | model            | `sun.py` (`sun_day`), `skenario_cuaca/`           |

## 5. Jangkauan dan laju gerak

| Parameter                              | Nilai                                                                                                          | Status                                    | Lokasi                                                                      |
| -------------------------------------- | -------------------------------------------------------------------------------------------------------------- | ----------------------------------------- | --------------------------------------------------------------------------- |
| Matahari terjangkau                    | hanya saat zenith <= batas kemiringan 45 derajat; di PENS sekitar 43% (21 Jun) sampai 54% (Mar, Okt) jam siang | bergantung pada batas 45 derajat (ASUMSI) | `tracker.py` (`ik_track`), hasil di `solar_tracker/README_gazebo.md`  |
| Laju maksimum azimuth (kontrol hibrid) | 0.5 derajat/s                                                                                                  | hasil analisis zenith                     | `kontrol_hibrid/hybrid_sim.py`, `P.rate1`                               |
| Laju maksimum kemiringan               | 1.0 derajat/s                                                                                                  | **ASUMSI**                          | `kontrol_hibrid/hybrid_sim.py`, `P.rate2`                               |
| Pembatas laju perintah node matahari   | az_rate dan tilt_rate 90 derajat/s (waktu nyata, untuk demo)                                                   | demo                                      | `solar_tracker/sun_command_node.py`, `DEFAULTS`                         |
| Pembatas percepatan dan peredam ayunan node matahari | az_accel dan tilt_accel 300 derajat/s2 (profil trapesium), shaper `zvd` | hasil `dinamika/osilasi_energi.py`, tervalidasi di Gazebo | `solar_tracker/sun_command_node.py`, `DEFAULTS`, `shaper_taps` |
| Profil perintah stepper di Gazebo      | laju 30 derajat/s, percepatan 300 derajat/s2                                                                   | demo                                      | `solar_tracker/stepper_node.py`, `DEFAULTS`                             |
| Tanggal dan waktu simulasi             | tanggal 2026 bebas, kecepatan waktu 1 sampai 1200x                                                             | opsi                                      | `solar_tracker/sun_command_node.py` (param `date`, `speed`, `hour`) |

## 6. Aktuator (memengaruhi presisi kinematik)

| Parameter                           | Nilai                                                                                | Status                        | Lokasi                                                                                                |
| ----------------------------------- | ------------------------------------------------------------------------------------ | ----------------------------- | ----------------------------------------------------------------------------------------------------- |
| Motor                               | 2x stepper 57HSE3.0N, torsi tahan 3 N.m                                              | ukur (spesifikasi)            | `dinamika/dyn_model.py`, `Axis.Th`                                                                |
| Mode loop                           | Terbuka (tanpa umpan balik posisi), loop tertutup hanya lewat LDR di tingkat sistem  | keputusan                     | `dinamika/dyn_model.py` (bagian "Definisi dan keputusan")                                           |
| Transmisi                           | langsung ke poros, tanpa gearbox                                                     | keputusan                     | `dinamika/dyn_model.py`                                                                             |
| Langkah dan microstep               | 1.8 derajat per langkah, microstep default 1/16, resolusi 0.1125 derajat             | microstep keputusan sementara | `dyn_model.py` (`Axis.step`), `hybrid_sim.py` (`P.step`), `stepper_node.py` (`microstep`) |
| Gigi rotor                          | 50                                                                                   | ukur                          | `dyn_model.py`, `Axis.Nr`                                                                         |
| Kekakuan puntir poros               | elevasi 317 N.m/rad (baja 8 mm), azimuth 1600 N.m/rad                                | **ASUMSI**              | `dyn_model.py`, `AXES`                                                                            |
| Kekakuan seri (aktuator pid Gazebo) | elevasi 101.82, azimuth 137.14 N.m/rad                                               | turunan                       | `build_model.py`, `stiffness_gains()`                                                             |
| Defleksi gravitasi elevasi          | M(q) = m g (y cos q - z sin q), maks 1.274 N.m pada -45 derajat = 0.717 derajat      | model (g = 10, panel 2.55 kg) | `dyn_model.py` (`grav`); terukur di Gazebo 0.161 derajat pada q2 = 0 (prediksi 0.155)             |
| Angin tarik-lepas (statis)          | azimuth 6.5 sampai 39 m/s, elevasi 11 sampai 18 m/s (rentang, lihat bagian 13) | **TIDAK DAPAT DIPAKAI** sampai lengan angin diukur | `dinamika/dyn_model.py` (`wind_torque`), `gabungan/coupled_sim.py`                              |

## 7. Sensor LDR (4 buah) dan kontrol hibrid

| Parameter                | Nilai                                                                                                           | Status           | Lokasi                                                                  |
| ------------------------ | --------------------------------------------------------------------------------------------------------------- | ---------------- | ----------------------------------------------------------------------- |
| Posisi LDR               | tengah tiap sisi panel, di permukaan atas; sumbu z frame LDR = normal panel                                     | ukur (foto alat) | `build_model.py`, `LDRS` dan blok `ldr_xml`                       |
| Tinggi sekat per lebar   | r = 1.0 (bersekat silang)                                                                                       | **ASUMSI** | `kontrol_hibrid/hybrid_sim.py`, `P.r`                               |
| Ketidakcocokan antar LDR | 3% (simpangan baku)                                                                                             | **ASUMSI** | `hybrid_sim.py`, `P.mism`                                           |
| Pembagi tegangan, ADC    | Vcc 3.3 V, 12 bit, derau 2 mV, setengah jenuh 20000 lux, eksponen 0.7                                           | **ASUMSI** | `hybrid_sim.py`, `P.vcc`, `P.bits`, `P.vn`, `P.L0`, `P.gam` |
| Penguatan dan zona mati  | kp 60 derajat/s per satuan galat, zona mati 0.002 (sekitar 1.5 derajat saat cerah)                              | disetel          | `hybrid_sim.py`, `P.kp`, `P.dead`                                 |
| Batas koreksi LDR        | memori maksimum 10 derajat, azimuth tidak dikoreksi bila kemiringan < 5 derajat, siaga bila elevasi < 5 derajat | disetel          | `hybrid_sim.py`, `P.dmax`, `P.guard`, `P.elmin`                 |
| Strategi                 | astro (pvlib feedforward), ldr, saputra, bias (feedforward astro + memori bias LDR + indeks kecerahan)          | keputusan        | `hybrid_sim.py` (pengendali)                                          |

## 8. Peta kode

| Berkas                                            | Isi                                                        | Parameter utama                                                          |
| ------------------------------------------------- | ---------------------------------------------------------- | ------------------------------------------------------------------------ |
| `sun.py`                                        | posisi matahari dan irradiance di PENS                     | `LAT`, `LON`, `TZ`, ketinggian                                     |
| `tracker.py`                                    | FK, IK, vektor matahari, batas kemiringan                  | `Q2_MAX`, rumus bagian 2                                               |
| `build_model.py`                                | CAD ke 3 mesh dan URDF (kinematik, inersia, plugin Gazebo) | `P_AZ`, `P_EL`, `TILT`, `EL_LIMIT`, `PANEL`, `LDRS`, redaman |
| `inersia/inersia_cad.py`, `inersia_link.json` | massa dan inersia per link dari CAD                        | massa, pusat massa, tensor                                               |
| `dinamika/dyn_model.py`                         | dinamika stepper, poros, angin (Python, RK4)               | `Axis`, `AXES`                                                       |
| `dinamika/osilasi_energi.py`                     | osilasi DUA sumbu sekaligus dan dampaknya pada energi; bandingkan strategi perintah | strategi, `micros`, `events`; hasil di `osilasi_results.md` |
| `kontrol_hibrid/hybrid_sim.py`                  | LDR dan kontrol hibrid pvlib + LDR                         | dataclass`P`                                                           |
| `kontrol_hibrid/panel_listrik.py`                | model listrik panel (dioda tunggal), arus sebagai sensor arah | pelat nama MOS C-50-18-M; hasil di `panel_listrik_results.md` |
| `kontrol_hibrid/sensor_arus.py`                  | rantai pengukuran arus (derau, kuantisasi, hanyutan), batas keterbacaan, pendakian bukit | hasil di `sensor_arus_results.md` |
| `kontrol_hibrid/fusi_sensor.py`                  | ketidakpastian tiap sensor, bobot kebalikan ragam, nilai kalibrasi LDR | `SIG_ASTRO`; hasil di `fusi_sensor_results.md` |
| `kontrol_hibrid/ekf.py`                          | EKF penaksir simpangan sudut (salah pasang + lepas langkah), LDR dan arus sebagai pengukuran | `Q_DRIFT`, `K_MODEL_ARUS`; hasil di `ekf_results.md` |
| `kontrol_hibrid/kalibrasi_ldr.py`                | prosedur kalibrasi ketidakcocokan antar LDR dan pemisahan bias pasang | hasil di `kalibrasi_ldr_results.md` |
| `angin/angin_openmeteo.py`                       | angin NYATA Surabaya berarah dari Open-Meteo, koreksi tinggi, torsi berarah | `ALPHA`, `CN`, `E_CHORD`, `E_SPAN` |
| `gabungan/coupled_sim.py`, `stow_sim.py`      | gabungan hibrid dan dinamika, strategi stow angin          | `WINDS`, `Ext`                                                       |
| `analisis_zenith/`                              | singularitas dan laju azimuth dekat zenith                 | batas laju                                                               |
| `solar_tracker/`                                | Gazebo, ROS 2, node matahari, node stepper, panel kontrol  | lihat`README_gazebo.md`                                                |

## 9. Yang masih harus diukur di alat nyata

1. Batas kemiringan maksimum (sekarang 45 derajat, sangat memengaruhi jangkauan harian).
2. Arah hadap sumbu +x alat (sekarang Utara = arah rel panjang dan engsel kemiringan; panel berguling ke Timur-Barat).
3. Orientasi panel di rel (model: sisi 0.670 m searah engsel) dan titik pasangnya; ukuran dan massa sudah dari spesifikasi MOS C-50-18-M.
4. Merek driver stepper, microstep, dan arus (sekarang 1/16 default).
5. Kekakuan puntir, redaman, dan gesekan sendi nyata.
6. Geometri sekat dan karakteristik LDR (r, ketidakcocokan, kurva lux).
7. Ketinggian lokasi dan koefisien gaya angin panel (Cn, lengan).

## 10. Catatan sumbu engsel (keputusan pengguna)

Engsel kemiringan searah rel panjang: panel hanya berguling kanan-kiri (tidak jungkat-jungkit sepanjang rel). Mesh motor tilt dari CAD masih berporos tegak lurus rel, jadi tampilan motor tidak sejalan dengan sumbu engsel (kosmetik). **Belum diperbarui:** inersia beban elevasi `JL = 0.0353` di `dinamika/dyn_model.py` dihitung terhadap sumbu lama (tegak lurus rel); terhadap sumbu rel nilainya lebih kecil (panel 1.5 kg: sekitar 0.011 vs 0.024 kg.m2 untuk panel saja) dan lengan gravitasi ikut berubah.

## 11. Spesifikasi panel surya (MOS C-50-18-M, dari pelat nama)

| Besaran | Nilai |
|---|---|
| Daya maksimum Pmax | 50 W |
| Tegangan sirkuit terbuka Voc | 22.5 V |
| Arus hubung singkat Isc | 3 A |
| Tegangan daya maksimum Vmp | 18.5 V |
| Arus daya maksimum Imp | 2.7 A |
| Toleransi | +-3 % |
| Dimensi | 670 x 445 x 25 mm |
| Massa | 2.55 kg |
| Acuan uji | STC: AM 1.5, E = 1000 W/m2, Tc = 25 C |

Efisiensi modul turunan: 50 W / (0.670 x 0.445 m2 x 1000 W/m2) = **16.8 %** (dipakai `solar_tracker/control_web.py`, `EFF`).
Besaran listrik sudah dimodelkan di `kontrol_hibrid/panel_listrik.py` (dioda tunggal, pvlib). Parameter hasil pencocokan: I_L 3.0092 A, I_o 7.02e-10 A, R_s 0.372 ohm, R_sh 121.5 ohm, n = 1.1 (36 sel seri, ASUMSI). Pelat nama direproduksi persis di STC.

**Dua temuan tentang arus sebagai sensor arah:**
1. Memaksimalkan arus TIDAK mengarahkan panel tepat ke matahari, dan **arah simpangannya bergantung albedo tanah**: -2.75 derajat pada albedo 0.10, sekitar 0 pada 0.25, +2.75 derajat pada 0.40 (langit cerah pvlib, elevasi 45 derajat). Baur langit menarik optimum ke arah lebih datar, albedo menarik ke arah lebih tegak, berlawanan. Dengan komponen langsung saja puncaknya tepat di 0 derajat. **Albedo tempat pemasangan harus diukur** bila arus dipakai sebagai acuan posisi.
   *(Koreksi: angka +2.6 derajat yang saya laporkan lebih dulu memakai DNI, DHI, dan GHI pilihan tangan yang tidak konsisten, GHI 800 padahal seharusnya 691, sehingga suku albedo membesar.)*
2. Arus buta terhadap galat kecil: simpangan 2 derajat hanya mengubah arus 0.09 persen, di bawah toleransi pabrik 3 persen. Arus baru berarti di atas 10 derajat. Koreksi halus dan informasi arah tetap dari LDR.

**Hasil yang KEDALUWARSA setelah perubahan ini** (dihitung dengan panel 1.5 kg, 0.44 x 0.30 m, g = 9.81, sumbu engsel lama):
`dinamika/dyn_results.md` dan tabelnya, `gabungan/coupled_results.md`, `gabungan/stow_results.md`, `kontrol_hibrid/hybrid_results.md`, `robustness/`, `skenario_cuaca/`. Jalankan ulang sebelum dikutip.

## 12. Osilasi dan energi (ringkas, dari `dinamika/osilasi_results.md`)

| Keadaan | Ayun puncak ke puncak | Rugi energi |
|---|---|---|
| Jejak biasa, microstep 1/16 | az 0.18 deg, el 0.16 deg | -0.009 % (defleksi kebetulan sedikit membantu) |
| Jejak biasa, microstep 1/64 | az 0.03 deg, el 0.03 deg | -0.009 % (tidak berubah) |
| Putar azimuth 90 deg, perintah mentah | az 137 deg | kecil, tapi mekanis berat |
| Putar azimuth 90 deg, ZVD | az 0.50 deg | kecil |
| Lompat jam 09 ke 15 di Gazebo, tanpa shaper | lewatan terburuk 19.4 deg | kecil |
| Lompat jam 09 ke 15 di Gazebo, ZVD | lewatan terburuk 2.3 deg | kecil |
| Angin `berangin` (puncak 11.9 m/s) | az lepas sinkron | **3.1 %** |

Kesimpulan: **osilasi hampir tidak merugikan energi**; yang merugikan adalah lepas sinkron karena angin. Peredaman osilasi tetap perlu untuk beban mekanik, umur motor, dan derau pembacaan LDR.

## 13. KOREKSI: batas angin tarik-lepas belum dapat dipakai

Saya sempat melaporkan azimuth lepas sinkron pada 9.6 m/s dan menyebutnya risiko sehari-hari. **Klaim itu terlalu kuat.** Angka itu bergantung pada lengan momen angin yang saya TEBAK, dan hasilnya berubah enam kali lipat:

| Lengan azimuth (m) | v tarik-lepas (m/s) | Dasar |
|---|---|---|
| 0.009 | 39.4 | eksentrisitas geometris nyata: pusat massa panel hanya 9.2 mm dari sumbu azimuth |
| 0.050 | 16.7 | asimetri sedang |
| 0.152 | 9.6 | tebakan saya sebelumnya (0.23 x sisi panjang), TANPA dasar fisis |
| 0.335 | 6.5 | kasus terburuk, seluruh gaya di satu ujung panel |

Panel hampir simetris terhadap sumbu azimuth, jadi angin merata nyaris tidak menghasilkan momen puntir tegak. Momen azimuth yang nyata datang dari turbulensi dan beban tak simetris, yang tidak dimodelkan di sini.

Elevasi lebih berdasar, karena pusat tekanan pelat datar memang bergeser 10 sampai 25 persen sisi melintang: **11.2 sampai 17.7 m/s**. Tetap rentang, bukan satu angka.

**Yang diperlukan untuk memakai angka ini:** koefisien gaya angin dan titik pusat tekanan panel miring, dari pengukuran, tabel standar beban angin, atau CFD. Sampai itu ada, hasil angin di `osilasi_results.md`, `coupled_results.md`, dan `stow_results.md` hanya berlaku sebagai skenario, bukan ramalan.

## 14. Penggabungan sensor: kalibrasi LDR adalah syarat, bukan penyempurnaan

Dari `kontrol_hibrid/fusi_sensor.py`, ketidakpastian taksiran sudut diukur Monte Carlo lalu dipakai sebagai bobot (kebalikan ragam). Galat astro dianggap 1.5 derajat (ASUMSI).

| Ketidakcocokan antar LDR | sigma LDR kemiringan | sigma LDR azimuth | Bobot LDR kemiringan | Bobot LDR azimuth |
|---|---|---|---|---|
| 3 % (tanpa kalibrasi) | 2.92 deg | 4.08 deg | 20.9 % | 11.9 % |
| 1 % (kalibrasi kasar) | 1.03 deg | 1.46 deg | 67.8 % | 51.4 % |
| 0.3 % (kalibrasi teliti) | 0.51 deg | 0.71 deg | 89.5 % | 81.8 % |

Derau acak LDR hanya 0.43 sampai 0.85 derajat; yang menguasai adalah BIAS dari ketidakcocokan antar sensor, dan bias tidak hilang dengan merata-rata. Tanpa kalibrasi, penggabungan yang jujur memberi LDR bobot kecil, artinya tracker praktis berjalan astro saja dan LDR nyaris tidak berguna.

Catatan lain:
- Ketelitian azimuth dari LDR memburuk saat matahari makin tinggi (de/dq1 sebanding sin q2). Bobotnya turun sendiri, tanpa perlu aturan `if q2 < 5 derajat` seperti parameter `guard` di `hybrid_sim.py`.
- Ketidakpastian arus 0.8 sampai 1.3 derajat dan hampir tidak terpengaruh cuaca, tetapi puncak arus sendiri bergeser -2.75 sampai +2.75 derajat menurut albedo. Kedua sensor sama-sama perlu kalibrasi: LDR terhadap perangkatnya, arus terhadap lingkungannya.

## 15. EKF penaksir simpangan sudut (loop terbuka)

Keadaan yang ditaksir adalah SIMPANGAN b antara sudut perintah dan sudut beban (salah pasang + lepas langkah terkumpul), bukan sudutnya sendiri. Hari 7 Okt 2026, lepas langkah azimuth 7.2 derajat disuntikkan di tengah hari. Galat diukur hanya saat matahari terjangkau.

| Susunan | Galat pointing rata-rata | p95 |
|---|---|---|
| Astro saja (tanpa tapis) | 2.52 deg | 5.30 deg |
| **EKF + LDR terkalibrasi** | **0.18 deg** | 0.39 deg |
| EKF + LDR 3 % TIDAK terkalibrasi | 5.63 deg | 6.10 deg |
| EKF + LDR terkalibrasi + arus | 0.18 deg | 0.39 deg |
| EKF + arus saja | 2.24 deg | 5.17 deg |

Tiga kesimpulan:
1. LDR terkalibrasi dengan EKF memperbaiki 14 kali, dan pulih dari lepas langkah 7.2 derajat dalam sekitar 350 detik tanpa diberi tahu bahwa lepas langkah terjadi.
2. **LDR yang tidak dikalibrasi lebih buruk daripada tidak memakai LDR sama sekali.** Bias LDR tidak dapat dibedakan dari salah pasang mekanik (tidak teramati secara terpisah), jadi tapis menyerapnya lalu mengarahkan panel keliru.
3. **Arus hampir tidak menambah apa apa** (0.181 tetap 0.181). Sendirian hanya memperbaiki 11 persen. Sesuai Jacobian: dI/db hampir nol di dekat puncak.

Catatan penting: R untuk arus HARUS memuat ketidakpastian MODEL (toleransi pabrik 3 persen, kotor, suhu, albedo), bukan hanya derau ADC 0.2 mA. Memakai derau ADC saja membuat penguatan Kalman ribuan kali terlalu besar dan galat melonjak ke 20.5 derajat.

Tapis masih **terlalu percaya diri sekitar 3.4 kali** pada azimuth (sigma dilaporkan 0.23 derajat, sebaran galat sebenarnya 0.79 derajat), karena Q berbentuk jalan acak halus sedangkan lepas langkah datang sebagai lompatan.

## 16. Kalibrasi LDR: prosedur dan hasilnya

**Prosedur A, cahaya baur merata.** Sekat hanya membayangi komponen LANGSUNG, jadi di bawah cahaya baur murni keempat LDR menerima irradiance yang sama berapa pun arah panel. Diperiksa: faktor koreksi yang dihasilkan sama sampai 1 bagian per juta pada tiga pose berbeda. Di lapangan: tunggu mendung tebal (okta 8 menyisakan DNI hanya 2.2 persen) atau tutup panel dengan penyebar cahaya, rekam beberapa ratus cuplikan, simpan faktor koreksi.

| Keadaan LDR | Ketidakcocokan sisa | Galat pointing lewat EKF |
|---|---|---|
| Tanpa LDR (astro saja) | - | 2.52 deg |
| Tanpa kalibrasi | 3.0 % | **5.63 deg (lebih buruk daripada tanpa LDR)** |
| Prosedur A di mendung nyata | 0.69 % | 1.46 deg |
| Prosedur A, eksponen gam meleset 10 % | 0.26 % | 0.58 deg |
| Prosedur A, cahaya baur murni | 0.011 % | **0.16 deg** |

**Yang paling menentukan bukan jumlah cuplikan, melainkan pengetahuan tentang kurva LDR.** Galat 10 persen pada eksponen `gam` menyisakan 0.26 persen ketidakcocokan, sedangkan merata-rata lebih banyak hanya menolong sampai batas kuantisasi ADC. Jadi mengukur kurva lux terhadap tegangan di meja adalah langkah pertama.

**Prosedur B, regresi di tempat sepanjang hari.** Salah pasang mekanik menghasilkan e = k x bias yang ikut berubah ketika kepekaan k berubah; ketidakcocokan LDR menghasilkan offset yang hampir tetap. Regresi e terhadap k memisahkan keduanya. Kepekaan azimuth berubah 25.6 kali sepanjang hari (sebanding sin q2), kemiringan hanya 1.1 kali. Hasilnya: **bias azimuth terpisahkan dengan baik** (taksiran 1.50 dari sebenarnya 1.50 derajat), **bias kemiringan tidak** (0.84 dari 1.00, bilangan kondisi 23936).

Urutan yang disarankan: (1) ukur kurva LDR di meja, (2) prosedur A saat mendung tebal, (3) prosedur B satu hari cerah untuk sisa bias azimuth, (4) ulangi prosedur A berkala karena LDR menua dan kotor dengan laju berbeda.

## 17. Angin NYATA menggantikan angin sintetis, dan risikonya jauh lebih kecil

Sumber: Open-Meteo Archive API (gratis, tanpa kunci), reanalisis per jam, 1 Jan 2015 sampai 30 Sep 2026, **102984 jam (11 tahun, 2015 sampai 2026)**, tersimpan di `angin/angin_pens_2015-2026.csv`. Modul `angin/angin_openmeteo.py`.

**Angin terukur di PENS, jam siang (07-17):**

| | Pada 10 m | Di panel (0.3 m, alpha 0.30) |
|---|---|---|
| Kecepatan rata-rata | 2.74 m/s | 0.96 m/s |
| Kecepatan p99 | 5.78 m/s | 2.02 m/s |
| Kecepatan maksimum | 7.08 m/s | 2.47 m/s |
| Hembusan maksimum | 16.81 m/s | **5.87 m/s** |

**Arah dominan: Tenggara 31.3 % dan Timur 26.3 %**, jadi 58 % waktu angin datang dari Timur sampai Tenggara (pola monsun). Barat 16.7 %, Barat Laut 10.2 %.

**Dua koreksi penting terhadap pekerjaan sebelumnya:**

1. **Skenario sintetis saya 2 sampai 3 kali terlalu berat.** Skenario `berangin` (7 m/s) dan `badai` (11 m/s) dimaksudkan sebagai angin DI PANEL, padahal hembusan terburuk setahun di tinggi panel hanya 5.87 m/s.
2. **Model torsi lama buta arah dan buta orientasi.** Model baru memakai gaya tegak lurus pelat: `tau_el = -e_c F` dan `tau_az = -e_s F sin(q2)` dengan `F = 0.5 rho v^2 A Cn (w . n)`. Torsi azimuth NOL saat panel datar, yang benar secara fisika dan tidak dimiliki model lama.

**Akibatnya, pada hembusan terburuk setahun (5.87 m/s di panel, kemiringan 45 derajat):**

| Sumbu | Torsi angin | Terhadap torsi tahan 3 N.m |
|---|---|---|
| Azimuth | 0.052 N.m | 1.7 % |
| Elevasi | 0.247 N.m | 8.2 % |

**Kesimpulan: angin BUKAN risiko bagi alat ini.** Margin terhadap lepas langkah sekitar 12 kali pada kasus terburuk setahun. Seluruh kekhawatiran lepas sinkron karena angin yang saya laporkan sebelumnya tidak berlaku, karena alat ini hanya 0.3 m di atas tanah dan terlindung.

**Yang masih asumsi:** eksponen profil alpha 0.30, koefisien Cn 1.2, pergeseran pusat tekanan `e_c` 0.0668 m dan `e_s` 0.02 m. Reanalisis pada petak sekitar 9 km juga tidak dapat melihat gedung dan pohon di kampus, jadi angka ini iklim regional, bukan beban di titik pemasangan. Anemometer di lokasi tetap yang paling kuat, dan sekarang ada pembandingnya.

## 18. Preset angin sintetis dibuang, diganti pengali terhadap data nyata

Rekaman 11 tahun menunjukkan maksimum hembusan tahunan Surabaya sangat stabil:

| | Pada 10 m |
|---|---|
| Rata-rata maksimum tahunan | 16.28 m/s |
| Simpangan baku | 1.29 m/s |
| Terendah (2015) | 13.81 m/s |
| **Tertinggi (2017, 2018, 2021)** | **17.81 m/s** |

Maksimum 11 tahun hanya **1.06 kali** maksimum 1 tahun. Surabaya terlalu dekat ekuator untuk siklon tropis dan tidak kena badai lintang menengah, jadi ekstremnya datar. Karena itu:

- Preset `tenang` 3, `berangin` 7, `badai` 11 m/s **dibuang dari jalur langsung**. Preset itu salah kalibrasi: dimaksudkan sebagai angin DI PANEL, padahal hembusan terburuk 11 tahun di tinggi panel hanya 6.22 m/s.
- Penggantinya parameter **`skala`**, pengali terhadap data nyata. "Tiga kali hembusan terburuk 11 tahun" dapat dipertanggungjawabkan; "badai 11 m/s" tidak.
- `gabungan/coupled_sim.wind_series` dipertahankan dan ditandai USANG, hanya agar studi luring lama tetap jalan.

**Margin terhadap lepas langkah (kemiringan 45 derajat, torsi tahan 3 N.m):**

| Kasus | v di panel | % torsi tahan |
|---|---|---|
| Terburuk 11 tahun | 6.22 m/s | **9.2 %** |
| Pengali 1.5 | 9.33 m/s | 20.8 % |
| Pengali 2 | 12.44 m/s | 37.0 % |
| Pengali 3 | 18.66 m/s | 83.1 % |

**Angin bukan risiko bagi alat ini.** Bahkan terhadap rekaman 11 tahun, marginnya sekitar 11 kali. Angin baru jadi masalah pada pengali tiga, yang berarti bukan lagi Surabaya di ketinggian 0.3 m (misalnya bila alat dipindah ke atap setinggi 10 m).
