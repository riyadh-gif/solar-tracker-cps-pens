# Solar Tracker PENS sebagai Cyber-Physical System

Dokumen ini mengikuti kerangka `Solar Tracking Systems and Cyber-Physical Systems` yang diberikan dosen,
tetapi seluruh isinya adalah sistem yang benar-benar dibangun dan diukur, bukan rumusan umum. Setiap angka
di sini berasal dari CAD, pelat nama, data cuaca nyata, atau pengukuran di Gazebo, dan setiap asumsi diberi
tanda. Nomor bagian sengaja disejajarkan dengan dokumen rujukan agar mudah dibandingkan.

**Alat:** tracker 2 sumbu azimuth dan kemiringan, panel MOS C-50-18-M 50 W, dua motor stepper 57HSE3.0N
terhubung langsung ke poros tanpa gearbox, empat LDR bersekat di tepi panel, dipasang di PENS Surabaya
(-7,276006; 112,794327).

**Perbedaan paling penting dengan dokumen rujukan:** dokumen itu mengandaikan servo bertorsi dengan enkoder
(lihat lapisan antarmuka, "Quadrature Encoder Counter"). Alat ini **stepper loop terbuka tanpa enkoder**.
Akibatnya dua skema kontrol utama di sana, Computed Torque Control dan Sliding Mode Control, tidak dapat
diterapkan apa adanya. Penggantinya diuraikan di bagian 6.

---

## 1. Kerangka koordinat acuan

Dokumen rujukan memakai ENU (x Timur, y Utara, z atas). Sistem ini memakai **NWU**:

```
x̂ = Utara     ŷ = Barat     ẑ = atas (zenith)
```

Pilihan ini membuat azimuth kompas (searah jarum jam dari Utara) muncul langsung sebagai rotasi negatif
terhadap ẑ, tanpa konversi tambahan. Keduanya setara; selisihnya hanya konvensi.

Sumbu engsel kemiringan **searah rel panjang**, yaitu sumbu x alat. Panel berguling ke kanan dan kiri,
tidak jungkat-jungkit sepanjang rel. Arah hadap rel diasumsikan Utara (`tracker.HEAD` = 90 derajat,
**ASUMSI**, belum diukur dengan kompas).

| Besaran | Lambang | Definisi |
|---|---|---|
| Sudut sendi azimuth | q1 | putar terhadap ẑ, positif searah jarum jam dilihat dari atas, tanpa batas |
| Sudut sendi kemiringan | q2 | 0 = panel datar, positif miring ke Timur; batas +-45 derajat (**ASUMSI**) |
| Arah miring kompas | q1 + HEAD | HEAD = 90 derajat |

---

## 2. Representasi vektor posisi matahari

Dokumen rujukan memakai rumus deklinasi sederhana:

```
sin αs = sin φ sin δ + cos φ cos δ cos ω
```

Sistem ini memakai **algoritma SPA (NREL) lewat pvlib**, yang jauh lebih teliti. Vektor satuan menuju
matahari pada azimuth A (kompas) dan elevasi e:

```
ŝ = (cos e cos A, −cos e sin A, sin e)
```

**Tervalidasi:** posisi matahari dibandingkan dengan astropy, SolTrack, dan Pysolar. Selisih maksimum
**di bawah 0,011 derajat**. Rumus deklinasi sederhana tipikal meleset puluhan kali lebih besar.

Irradiance memakai model clear-sky Ineichen, faktor awan Kasten-Czeplak (okta), dan dekomposisi Erbs untuk
memisahkan DNI dan DHI.

---

## 3. Kinematika vektor normal panel

Normal panel diperoleh dengan memutar vektor zenith n0 = [0,0,1]:

```
n̂ = Rz(−q1) Rx(q2) ẑ
n̂ = ( sin q2 cos(q1+HEAD), −sin q2 sin(q1+HEAD), cos q2 )
```

Struktur ini **identik** dengan dokumen rujukan `n = Rz(θA) Ry(θE) n0`, berbeda hanya karena sumbu engsel
di sini searah x (rel) dan azimuth diukur dari Utara searah jarum jam.

**Tervalidasi di Gazebo:** normal panel dari TF dibandingkan dengan rumus di atas pada (q1,q2) = (0,30),
(90,30), (0,−30). Selisih **0,14 sampai 0,34 derajat**, seluruhnya terjelaskan oleh defleksi gravitasi.

### Jacobian kinematik

```
∂n̂/∂q1 = −sin(q2) ŷp        ∂n̂/∂q2 = x̂p
```

Nilai singular matriks Jacobian adalah sin(q2) dan 1, sehingga bilangan kondisinya 1/sin(q2):

| q2 | Bilangan kondisi |
|---|---|
| 1 derajat | 57,3 |
| 5 derajat | 11,5 |
| 30 derajat | 2,0 |
| 45 derajat | 1,4 |

**Inilah efek keyhole**, dinyatakan sebagai hilangnya rank, bukan sebagai gejala terpisah. Di dekat zenith
sumbu azimuth hampir tidak teramati. Konsekuensinya muncul lagi di bagian 7 (bobot sensor) dan bagian 11
(kovarians EKF), tanpa perlu aturan tangan `if q2 < 5 derajat`.

### Galat pointing

```
θ_err = arccos(n̂ · ŝ)
```

Rugi energi dari galat pointing adalah `1 − cos θ_err`, dipakai di seluruh dokumen ini sebagai ukuran dampak.

---

## 4. Dinamika Lagrange

Bentuk baku sama dengan dokumen rujukan:

```
M(q) q̈ + C(q,q̇) q̇ + g(q) + F(q̇) + τw = τ
```

### Matriks massa M(q)

Inersia azimuth bergantung kemiringan, persis bentuk yang diturunkan dokumen rujukan
`J1(q2) = Iz1 + (m2 rc² + Iy2) cos²q2 + Iz2 sin²q2`. Dari CAD dengan panel 2,55 kg:

```
J1(q2) = 0,1106 + 0,0408 cos²(q2)   kg m²
J2     = 0,0543                      kg m²   (tetap)
ΔI     = 0,0408                      kg m²
```

Dicocokkan dari dua titik CAD: J1(0) = 0,1514 dan J1(45 derajat) = 0,1310 kg m².

### Gravitasi g(q)

Massa yang berputar di sumbu kemiringan (elevation_link + panel) **3,055 kg**, dengan pusat massa bergeser
y = 9,04 mm dan z = 49,93 mm dari sumbu engsel:

```
τ_grav(q2) = −m g ( y cos q2 − z sin q2 )
```

dengan **g = 10 m/s²** (keputusan pengguna, bukan 9,80665).

| Posisi | Momen gravitasi |
|---|---|
| q2 = 0 | 0,276 N m |
| **q2 = −45 derajat** | **1,274 N m** (maksimum dalam batas) |

**Tervalidasi di Gazebo:** defleksi pada q2 = 0 terukur **0,161 derajat**, prediksi 0,155 derajat.

### Gesekan F(q̇)

```
F(q̇) = b q̇ + τc sgn(q̇)       b = 0,2 N m s/rad,  τc = 0,05 N m   (keduanya ASUMSI)
```

Kedua nilai ini **belum diukur** dan paling menentukan lama ayunan. Pengukurannya lewat uji peluruhan bebas.

### Suku Coriolis dan sentripetal C(q,q̇)

```
azimuth  : −ΔI sin(2q2) q̇1 q̇2
kemiringan: +½ ΔI sin(2q2) q̇1²
```

**Belum diterapkan** dalam simulasi. Besarnya sudah dihitung:

| Keadaan | Coriolis azimuth | Terhadap torsi tahan 3 N m |
|---|---|---|
| Jejak harian khas | 4,3e-8 N m | 0,0000 % |
| Jejak laju maksimum | 5,4e-6 N m | 0,0002 % |
| Slew 90 derajat/s | 0,087 N m | **2,9 %** |

Tidak berpengaruh saat menjejak, tetapi mencapai 3,4 persen saat slew cepat. Layak ditambahkan, dan tidak
akan mengubah kesimpulan yang sudah ada.

### Beban angin τw

Dokumen rujukan menulis `τw = ½ ρ v² Apv Cm Lref`, buta arah. Sistem ini memakai model berarah:

```
F  = ½ ρ v² A Cn ( ŵ · n̂ )
τ_kemiringan = −e_c F
τ_azimuth    = −e_s F sin(q2)
```

dengan ŵ vektor perjalanan angin. **Torsi azimuth NOL saat panel datar**, yang benar secara fisika: gaya
tegak lurus panel datar tidak dapat memuntir sumbu tegak. Rinciannya di bagian 9.

### Nilai eigen dan kestabilan

Linearisasi di sekitar titik kerja, x = [q, q̇], memberi matriks Jacobian dinamika F = ∂f/∂x. Nilai eigennya:

| Sumbu | Nilai eigen | Re(λ) | Frekuensi | Redaman ζ | Waktu peluruhan |
|---|---|---|---|---|---|
| Azimuth | −0,661 ± 30,09j | −0,661 | **4,79 Hz** | 0,0219 | 1,51 s |
| Kemiringan | −1,842 ± 43,26j | −1,842 | **6,89 Hz** | 0,0425 | 0,54 s |

**Bagian nyata negatif, jadi sistem stabil dan pasti kembali ke titik setimbang.** Itu pembuktian formal
bahwa panel kembali tenang setelah bergerak.

Lewatan teoretis `exp(−πζ/√(1−ζ²))` = **93,3 %** dan **87,5 %**. **Terukur di Gazebo: 90 % dan 82 %**
untuk langkah 1 derajat. Selisihnya dari gesekan Coulomb yang tidak masuk model linier.

---

## 5. Aktuator: stepper loop terbuka

Bagian ini **tidak ada** di dokumen rujukan, tetapi menentukan segalanya di sini.

Motor digerakkan dengan pulsa posisi, bukan perintah torsi. Hukum torsi-sudut:

```
τ = Th sin( Nr ( θ_cmd − θ_beban − τ/k ) )
```

dengan Th = 3 N m (torsi tahan), Nr = 50 (gigi rotor), k kekakuan puntir poros. Untuk simpangan kecil ini
setara pegas dengan kekakuan seri:

```
K = 1 / ( 1/(Th Nr) + 1/k )      azimuth 137,14   kemiringan 101,82  N m/rad
```

Perintah posisi dibulatkan ke tangga microstep 1/16, yaitu **0,1125 derajat per langkah**. Tangga inilah
sumber eksitasi ayunan saat menjejak, bukan angin atau gangguan luar.

**Sifat yang membedakan dari servo:** bila torsi luar melewati Th, motor kehilangan sinkron dan beban lari.
Lepas langkah **tidak pernah pulih sendiri** karena tidak ada enkoder. Inilah alasan utama memerlukan
penaksir keadaan di bagian 11.

---

## 6. Skema kontrol yang benar-benar dipakai

Dokumen rujukan mengusulkan Computed Torque Control dan Sliding Mode Control. **Keduanya tidak dapat
diterapkan di sini**, karena:

- CTC membutuhkan q dan q̇ terukur tiap siklus. Tidak ada enkoder.
- SMC membutuhkan hal yang sama, ditambah kemampuan mengatur torsi sesaat. Driver stepper hanya menerima
  pulsa posisi.

Penggantinya tiga lapis, dan ketiganya sudah berjalan serta terukur.

### 6.1 Umpan maju astronomis

Lintasan sendi dihitung dari posisi matahari pvlib lewat kinematika balik. Untuk matahari di (A, zenith z)
ada dua solusi, (A − HEAD, z) dan (A − HEAD + 180, −z); dipilih yang kontinu dari langkah sebelumnya agar
melewati zenith tidak perlu memutar azimuth 180 derajat.

### 6.2 Profil gerak dan input shaper ZVD

Perintah melewati **profil trapesium** (laju 90 derajat/s, percepatan 300 derajat/s²) lalu **input shaper
ZVD** tiga impuls, dengan jeda dihitung dari frekuensi dan redaman tiap sumbu.

Hasil terukur di Gazebo, lompatan azimuth 181 derajat, lima ulangan:

| Strategi | Lewatan median | Lewatan terburuk |
|---|---|---|
| Hanya profil trapesium | 0,58 % (1,05 derajat) | **10,71 % (19,35 derajat)** |
| **Profil + ZVD** | 0,11 % (0,20 derajat) | **1,27 % (2,30 derajat)** |

ZVD menekan kasus terburuk sekitar **8 kali**, dan yang sama pentingnya, membuatnya **dapat diramalkan**.

Perbandingan strategi pada langkah tunggal (model Python berbanding Gazebo):

| Uji | Mentah | Trapesium | ZV | ZVD |
|---|---|---|---|---|
| Azimuth 90 derajat | 52,43 / 52,41 % | 0,88 / 0,77 % | 3,07 / 0,00 % | 0,26 / 0,00 % |
| Azimuth 5 derajat | 82,60 / 82,78 % | 15,46 / 15,77 % | 0,32 / 0,40 % | 0,97 / 0,85 % |

**Catatan penting:** input shaper **tidak berguna saat menjejak pelan**. Satu microstep tidak dapat dipecah
menjadi dua setengah langkah, sehingga setelah pembulatan perintahnya sama saja. Satu-satunya tuas saat
menjejak adalah memperhalus microstep.

### 6.3 Koreksi dari penaksir

Taksiran simpangan dari EKF (bagian 11) menggeser perintah. Bersifat opt-in lewat parameter `pakai_ekf`.

---

## 7. Sensor dan penggabungannya

Bagian ini jauh lebih rinci daripada dokumen rujukan, karena di sinilah letak temuan paling menentukan.

### 7.1 LDR

Empat LDR bersekat di tepi panel, digabung jadi dua sinyal selisih:

```
ex = (v_x+ − v_x−) / (v_x+ + v_x−)        ey = (v_y+ − v_y−) / (v_y+ + v_y−)
```

Pembagian dengan jumlahnya membuat sinyal tahan terhadap perubahan irradiance menyeluruh.

Ketidakpastian diukur Monte Carlo, dipisah antara derau acak (bisa ditekan dengan merata-rata) dan bias
antar perangkat (tidak bisa, hanya hilang dengan kalibrasi):

| Langit | Derau acak kemiringan | **Bias kemiringan** | Derau acak azimuth | **Bias azimuth** |
|---|---|---|---|---|
| Cerah | 0,44 derajat | **2,86 derajat** | 0,60 derajat | **4,68 derajat** |
| Mendung okta 6 | 0,69 derajat | **5,20 derajat** | 0,94 derajat | **8,54 derajat** |

**Yang menguasai bukan derau, melainkan bias.** Bias 5 sampai 10 kali derau acaknya.

### 7.2 Arus panel sebagai sensor

Model dioda tunggal dicocokkan ke pelat nama MOS C-50-18-M secara persis:

| Parameter | Nilai |
|---|---|
| Arus cahaya I_L | 3,0092 A |
| Arus jenuh I_o | 7,02e-10 A |
| Tahanan seri R_s | 0,372 ohm |
| Tahanan bocor R_sh | 121,5 ohm |
| Faktor ideal n | 1,1 (tidak kritis: n 1,0 sampai 1,5 mengubah arus 0,13 persen) |

Reproduksi STC persis: Isc 3,000 A, Voc 22,50 V, Imp 2,700 A, Vmp 18,50 V, Pmax 49,95 W. Efisiensi 16,75 %.

**Dua temuan yang membatasi kegunaan arus:**

1. **Memaksimalkan arus TIDAK mengarahkan panel ke matahari.** Puncak arus bergeser −2,75 derajat pada
   albedo 0,10 sampai +2,75 derajat pada albedo 0,40. Penyebabnya baur langit menarik optimum ke arah lebih
   datar sedangkan albedo menarik ke arah lebih tegak. Dengan komponen langsung saja, puncaknya tepat di
   0 derajat. **Albedo lokasi harus diukur** bila arus dipakai sebagai acuan posisi.
2. **Arus buta terhadap galat kecil.** Simpangan 2 derajat hanya mengubah arus 0,09 persen, di bawah
   toleransi pabrik 3 persen. Arus baru berarti di atas 10 derajat.

Dan yang paling membatasi: **perubahan arus karena awan dua sampai tiga orde lebih besar daripada karena
salah arah.** Pendakian bukit memakai arus berhasil saat cerah dan tenang (berhenti 0,30 derajat dari
puncak), tetapi **tersesat sampai 51 derajat** ketika awan menipis 2 persen per detik.

### 7.3 Bobot penggabungan dari ketidakpastian

Bobot kebalikan ragam, galat astro dianggap 1,5 derajat (**ASUMSI**):

| Ketidakcocokan LDR | sigma kemiringan | sigma azimuth | Bobot LDR kemiringan | Bobot LDR azimuth |
|---|---|---|---|---|
| **3 % (tanpa kalibrasi)** | 2,92 derajat | 4,08 derajat | **20,9 %** | **11,9 %** |
| 1 % (kalibrasi kasar) | 1,03 derajat | 1,46 derajat | 67,8 % | 51,4 % |
| 0,3 % (kalibrasi teliti) | 0,51 derajat | 0,71 derajat | **89,5 %** | **81,8 %** |

Bobot LDR untuk azimuth **turun sendiri** dari 9,2 persen pada q2 = 45 derajat menjadi 0,5 persen pada
q2 = 10 derajat, karena `de/dq1` sebanding sin(q2). Efek keyhole dari bagian 3 muncul kembali sebagai angka.

### 7.4 Prosedur kalibrasi LDR

**Prosedur A, cahaya baur merata.** Sekat hanya membayangi komponen langsung, jadi di bawah cahaya baur
keempat LDR menerima irradiance sama berapa pun arah panel (diperiksa: faktor koreksi sama sampai satu
bagian per juta pada tiga pose berbeda). Di lapangan: tunggu mendung tebal (okta 8 menyisakan DNI hanya
2,2 persen), rekam beberapa ratus cuplikan, simpan faktor koreksi.

**Prosedur B, regresi sepanjang hari.** Salah pasang menghasilkan e = k x bias yang ikut berubah ketika
kepekaan k berubah; ketidakcocokan LDR menghasilkan offset hampir tetap. Kepekaan azimuth berubah 25,6 kali
sepanjang hari, kemiringan hanya 1,1 kali. Hasilnya **bias azimuth terpisahkan** (taksiran 1,50 dari
sebenarnya 1,50 derajat), **bias kemiringan tidak** (0,84 dari 1,00, bilangan kondisi 23936).

**Yang paling menentukan bukan jumlah cuplikan, melainkan pengetahuan kurva LDR.** Galat 10 persen pada
eksponen `gam` menyisakan 0,26 persen ketidakcocokan, dua puluh kali lebih buruk daripada batas dari derau.

---

## 8. Arsitektur Cyber-Physical System

```
 ┌──────────────────────────────────────────────────────────────────────────┐
 │                           LAPISAN CYBER                                  │
 │  ┌────────────────────┐  ┌──────────────────┐  ┌────────────────────┐   │
 │  │ sun_command_node   │─►│ profil trapesium │  │ ekf_node           │   │
 │  │ pvlib SPA, IK      │  │ + input shaper   │◄─│ taksir simpangan b │   │
 │  │ q_d[k] @ 100 Hz    │  │ ZVD              │  │ + kovarians P      │   │
 │  └────────────────────┘  └────────┬─────────┘  └─────────▲──────────┘   │
 │  ┌────────────────────┐           │                      │              │
 │  │ sensor_node        │───────────┼──────────────────────┘              │
 │  │ model LDR, dioda   │           │  /tracker/diag @ 5 Hz               │
 │  │ panel, energi      │           │  /ldr/state                         │
 │  └────────────────────┘           │                                     │
 │  ┌────────────────────┐           │                                     │
 │  │ wind_node          │───────────┤  angin NYATA berarah                │
 │  │ Open-Meteo 11 thn  │           │                                     │
 │  └────────────────────┘           │                                     │
 └───────────────────────────────────┼─────────────────────────────────────┘
                                     │ perintah posisi (rad)
 ┌───────────────────────────────────┼─────────────────────────────────────┐
 │ LAPISAN ANTARMUKA                 ▼                                     │
 │   ros_gz_bridge (parameter_bridge, YAML)                                │
 │   pembulatan microstep 0,1125 derajat  <-- kuantisasi Q(.)              │
 │   kejenuhan torsi 3 N m                <-- Sat(.)                       │
 └───────────────────────────────────┬─────────────────────────────────────┘
                                     │ τ(t)                ▲ /joint_states @ 1 kHz
 ┌───────────────────────────────────┼─────────────────────┼───────────────┐
 │ LAPISAN FISIK (Gazebo, DART, 1 ms)                      │               │
 │   M(q)q̈ + C(q,q̇)q̇ + g(q) + F(q̇) + τw = τ                              │
 │   massa dan inersia dari CAD, gravitasi 10 m/s²                         │
 │   gangguan: angin nyata berarah, defleksi, lepas langkah                │
 └─────────────────────────────────────────────────────────────────────────┘
```

Pemetaan ke dokumen rujukan:

| Dokumen rujukan | Di sistem ini | Status |
|---|---|---|
| Plant dinamika kontinu | Gazebo DART, langkah 1 ms | berjalan |
| Kuantisasi Q(.) | pembulatan microstep 0,1125 derajat | berjalan |
| Kejenuhan Sat(.) | batas torsi 3 N m | berjalan |
| Astronomical orbit model | pvlib SPA | berjalan, tervalidasi 0,011 derajat |
| Discrete controller | node ROS 2, 100 Hz | berjalan |
| State estimator (EKF) | `ekf_node` | berjalan |
| Quadrature encoder | **tidak ada** | diganti EKF + LDR |
| Latensi jaringan, jitter | **belum dimodelkan** | kekurangan |
| Serangan FDI, NTP poisoning | **belum dimodelkan** | kekurangan |
| Event-triggered control | **belum ada** | kekurangan |

**Catatan penting tentang jam:** seluruh node memakai `use_sim_time`. Tanpa itu profil gerak dan input
shaper berjalan di jam dinding sedangkan plant di jam simulasi (Gazebo sekitar 85 persen real-time),
sehingga lewatan menjadi tidak dapat diulang. Terukur: 7,6 persen sampai 0,35 persen pada uji yang sama.

---

## 9. Beban angin dari data nyata

Dokumen rujukan memodelkan angin sebagai `τw = ½ ρ v² Apv Cm Lref` tanpa arah. Sistem ini memakai data
pengukuran nyata.

**Sumber:** Open-Meteo Archive API, reanalisis per jam untuk titik PENS, **102.984 jam (11 tahun,
2015 sampai 2026)**, lengkap dengan arah dan hembusan.

### 9.1 Koreksi ketinggian

Reanalisis berlaku pada 10 meter; panel ini hanya 0,3 meter di atas tanah. Profil pangkat
`v(z) = v10 (z/10)^alpha` dengan alpha 0,30 (**ASUMSI**, kampus berbangunan sedang):

| | Pada 10 m | Di panel 0,3 m |
|---|---|---|
| Kecepatan rata-rata | 2,83 m/s | 0,99 m/s |
| Kecepatan maksimum | 9,33 m/s | 3,26 m/s |
| **Hembusan maksimum 11 tahun** | **17,81 m/s** | **6,22 m/s** |

### 9.2 Arah

Angin didominasi **Timur 34,3 persen**, diikuti Tenggara. Itu pola monsun, bukan acak. Model lama yang
hanya memberi tanda plus minus acak tidak dapat menangkapnya.

### 9.3 Model torsi berarah

```
ŵ = vektor perjalanan angin, dari bearing phi menuju phi + 180
F = ½ ρ v² A Cn ( ŵ · n̂ )
τ_kemiringan = −e_c F                e_c = 0,0668 m  (ASUMSI, 15 % sisi melintang)
τ_azimuth    = −e_s F sin(q2)        e_s = 0,02 m    (ASUMSI, eksentrisitas + turbulensi)
```

**Tervalidasi di Gazebo:** pada 13:00 tanggal 7 Okt, angin dari 105 derajat (Timur) menuju 285 derajat
(Barat), pengali 1 sampai 3 memberi torsi 0,45, 1,80, dan 4,04 persen torsi tahan. Rasio 4 dan 9 terbaca
persis, sesuai `v²`. Tandanya juga berbalik antara pagi dan siang karena orientasi panel berubah, sesuatu
yang model buta arah tidak dapat hasilkan.

### 9.4 Margin terhadap lepas langkah

| Kasus | v di panel | Persen torsi tahan 3 N m |
|---|---|---|
| **Terburuk 11 tahun** | 6,22 m/s | **9,2 %** |
| Pengali 1,5 | 9,33 m/s | 20,8 % |
| Pengali 2 | 12,44 m/s | 37,0 % |
| Pengali 3 | 18,66 m/s | 83,1 % |

**Angin bukan risiko bagi alat ini**, dengan margin sekitar 11 kali. Maksimum hembusan tahunan sangat stabil
(rata 16,28 m/s, simpangan baku 1,29 atas 11 tahun) karena Surabaya terlalu dekat ekuator untuk siklon
tropis. Karena itu preset sintetis `tenang`, `berangin`, `badai` dibuang dan diganti pengali terhadap data
nyata: "tiga kali hembusan terburuk 11 tahun" dapat dipertanggungjawabkan, "badai 11 m/s" tidak.

---

## 10. Osilasi dan dampaknya pada energi

Pertanyaan yang dijawab: apakah ayunan akibat gerakan motor merugikan panen energi, sehingga perlu algoritma
peredam?

### 10.1 Besar ayunan terukur

Diukur pada laju penuh `/joint_states` 1000 Hz, dengan gerak jejak lambat dibuang lewat penapis lolos tinggi:

| Keadaan | Ayunan puncak ke puncak | Frekuensi |
|---|---|---|
| Menjejak dekat transit zenith | az 1,90 derajat, el 0,72 derajat | 5,19 dan 6,57 Hz |
| Setelah lompat jam besar | az 4,53 derajat, el 5,44 derajat | |
| 9 detik kemudian | 0,000 derajat | |

Frekuensi terukur 5,19 dan 6,57 Hz **cocok dengan prediksi nilai eigen** 4,79 dan 6,89 Hz. Peluruhan 9 detik
sesuai konstanta waktu 1,51 detik dari bagian nyata nilai eigen.

### 10.2 Rugi energi

| Keadaan | Ayunan | Rugi energi |
|---|---|---|
| Jejak biasa, microstep 1/16 | 0,18 derajat | **0,0001 %** |
| Jejak dekat transit | 1,90 derajat | 0,0069 % |
| Setelah lompat jam | 4,53 derajat | 0,039 % |
| Putar 90 derajat perintah mentah | 137 derajat | 32,7 % (hanya beberapa detik) |

Dampak harian lompatan besar, lima kali sehari: **0,19 mWh dari panen 431 Wh**.

### 10.3 Pembanding

| Penyebab rugi | Besarnya |
|---|---|
| **Suhu sel 55 C dibanding 25 C** | **14,9 %** |
| Toleransi pabrik panel | 3,0 % |
| Defleksi gravitasi 0,72 derajat | 0,008 % |
| **Ayunan motor saat menjejak** | **0,007 %** |

**Kesimpulan: osilasi hampir tidak merugikan energi**, dua ribu kali lebih kecil daripada suhu. Peredam ZVD
tetap perlu, tetapi alasannya **beban mekanik, risiko lepas langkah, dan derau ke sensor**, bukan panen.

Memperhalus microstep dari 1/8 ke 1/64 menurunkan ayunan dari 0,39 ke 0,03 derajat tetapi **tidak mengubah
rugi energi sama sekali** (tetap −0,009 persen).

---

## 11. Penaksir keadaan: EKF sebagai pengganti enkoder

### 11.1 Keadaan yang ditaksir

Dokumen rujukan mengusulkan UKF menaksir parameter fisis `θ = [m2, rc, b1, b2, Cm,A, Cm,E]`. Sistem ini
menaksir sesuatu yang lebih mendesak, karena tidak ada enkoder:

```
b = [b_azimuth, b_kemiringan] = sudut beban − sudut perintah
```

memuat salah pasang mekanik, **lepas langkah terkumpul**, dan sisa defleksi.

```
Prediksi : b[k+1] = b[k] + w ,   P⁻ = P + Q        (F = I, jalan acak)
Koreksi  : z = e(c + b) ,        H = ∂e/∂b         (Jacobian pengukuran)
```

### 11.2 Mengapa lepas langkah lebih penting daripada osilasi

| | Osilasi | Simpangan b |
|---|---|---|
| Sifat | cepat (5 sampai 7 Hz), sementara | lambat, **menumpuk** |
| Hilang sendiri | ya, 9 detik | **tidak pernah** |
| Rugi energi | 0,007 % | tergantung besarnya |

| Simpangan terkumpul | Rugi energi, permanen |
|---|---|
| 7,2 derajat | 0,79 % |
| 14,4 derajat | 3,1 % |
| 21,6 derajat | 7,0 % |
| 30 derajat | 13,4 % |

### 11.3 Hasil terukur

Hari 7 Okt 2026, lepas langkah azimuth 7,2 derajat disuntikkan di tengah hari, galat diukur hanya saat
matahari terjangkau:

| Susunan | Galat rata-rata | p95 |
|---|---|---|
| Astro saja, tanpa tapis | 2,52 derajat | 5,30 derajat |
| **EKF + LDR terkalibrasi** | **0,18 derajat** | 0,39 derajat |
| EKF + LDR 3 % tidak terkalibrasi | **5,63 derajat** | 6,10 derajat |
| EKF + LDR terkalibrasi + arus | 0,18 derajat | 0,39 derajat |
| EKF + arus saja | 2,24 derajat | 5,17 derajat |

Pulih dari lepas langkah 7,2 derajat dalam sekitar **350 detik tanpa pernah diberi tahu** bahwa lepas
langkah terjadi.

**Tiga kesimpulan:**

1. LDR terkalibrasi memperbaiki **14 kali**.
2. **LDR tidak terkalibrasi lebih buruk daripada tidak memakai LDR sama sekali.** Bias LDR tidak dapat
   dibedakan dari salah pasang mekanik, jadi tapis menyerapnya lalu mengarahkan panel keliru. Ini batas
   mendasar susunan sensornya, bukan kekurangan tapisnya.
3. **Arus hampir tidak menambah apa pun**, sesuai Jacobian: dI/db hampir nol di dekat puncak.

### 11.4 Keyhole sebagai kovarians

| Keadaan | de_y/db_az | sigma azimuth dilaporkan |
|---|---|---|
| Matahari elevasi 45 derajat | 0,00091 | 0,67 derajat |
| Matahari elevasi 85 derajat | 0,00019 | **2,65 derajat** |

Kolom azimuth pada H mengecil sebanding sin(q2), sehingga ketidakpastian membesar sendiri. **Tidak ada
aturan `if q2 < 5 derajat` di mana pun.** Inilah pertemuan Jacobian bagian 3 dengan ketidakpastian.

### 11.5 Dua pengaman yang wajib

Keduanya ditemukan ketika loop tertutup menyimpang di Gazebo:

1. **LDR hanya dipakai saat matahari terjangkau.** Saat zenith melewati 45 derajat, kemiringan terpotong
   dan LDR membaca galat besar yang bukan dari simpangan sendi.
2. **Anti-penyimpangan pada gerbang pencilan.** Bila taksiran telanjur salah, inovasi membesar, gerbang
   menolak semua pengukuran, dan tapis terkunci selamanya (terukur: NIS tetap 82 sampai 90, taksiran macet
   di −6,47 derajat). Setelah 10 penolakan beruntun, P digembungkan.

### 11.6 Pelajaran tentang R

`R` untuk arus **wajib memuat galat model**, bukan hanya derau sensor. Mengisinya dengan derau ADC 0,2 mA
saja membuat penguatan Kalman `K = P H / R` ribuan kali terlalu besar, dan **galat pointing melonjak dari
0,18 ke 20,5 derajat**. Ketidakpastian ramalan arus didominasi toleransi pabrik 3 persen, kotor, suhu sel,
dan albedo.

### 11.7 Kejujuran tapis

Tapis masih **terlalu percaya diri sekitar 3,4 kali** pada azimuth: sigma dilaporkan 0,23 derajat sedangkan
sebaran galat sebenarnya 0,79 derajat. Sebabnya `Q` berbentuk jalan acak halus sedangkan lepas langkah
datang sebagai lompatan.

---

## 12. Kerentanan CPS

| Dimensi | Asal fisik | Asal cyber | Dampak | Status di sistem ini |
|---|---|---|---|---|
| Pencuplikan dan latensi | batas percepatan sendi | waktu komputasi, jitter | penundaan fase menurunkan margin kendali | **belum dimodelkan**; ditemukan gejalanya lewat `use_sim_time` |
| Aliasing pemantauan | dering 5 sampai 7 Hz | laju terbit 5 Hz | ayunan tidak terbaca sama sekali | **ditemukan dan diperbaiki**: statistik dihitung pada 1000 Hz |
| Parkir angin | tekanan dinamis angin | ambang anemometer | gagal memerintahkan parkir | risiko rendah: margin 11 kali |
| Lepas langkah | torsi luar melewati 3 N m | tidak ada umpan balik posisi | galat permanen menumpuk | **ditangani EKF**, pulih 350 detik |
| Kalibrasi sensor | ketidakcocokan antar LDR | tidak ada deteksi | LDR memperburuk hasil | **ditemukan dan diukur**, prosedur kalibrasi tersedia |
| Keamanan siber | end-stop mekanik | NTP poisoning, spoofing | aktuator menumbuk batas | **belum dimodelkan** |

---

## 13. Lapisan Digital Twin

### 13.1 Posisi sekarang

Menurut taksonomi Kritzinger dkk. (2018):

| Tingkat | Aliran data | Posisi |
|---|---|---|
| **Digital Model** | tidak ada, manual | **di sini** |
| Digital Shadow | otomatis, fisik ke digital | belum |
| Digital Twin | otomatis, dua arah | belum |

Yang ada adalah Digital Model yang **tervalidasi silang tiga jalur** (nilai eigen analitik, model Python
dua sumbu, dan Gazebo saling cocok). Yang belum ada adalah hubungan ke alat nyata.

### 13.2 Mengapa jembatannya pendek

Arsitekturnya sudah benar. `sensor_node` menghasilkan data sensor dan menerbitkannya ke `/tracker/diag`;
`ekf_node` memakannya dari topik itu. **Mengganti `sensor_node` dengan node pembaca perangkat keras yang
menerbitkan pesan sama membuat seluruh rantai di belakangnya jalan tanpa diubah.** Jembatan ke Digital
Shadow adalah satu node, bukan penulisan ulang.

### 13.3 Yang sudah ada dari usulan dokumen rujukan

| Usulan | Status |
|---|---|
| Recursive parameter estimation | sebagian: EKF menaksir simpangan b, belum parameter fisis θ |
| Predictive optimization | belum |
| Structural fatigue tracking | belum, tetapi `D = ‖τ(k) − τ(k−1)‖²` memberi pembenaran kuantitatif untuk ZVD |
| Health Index dari hanyutan gesekan | belum; perluasan wajar dari EKF yang ada |
| Intelligent backtracking | **tidak berlaku**: untuk ladang surya banyak panel, alat ini satu panel |
| Predictive aero-elastic wind stow | tidak mendesak: margin angin 11 kali |

### 13.4 Langkah menuju Digital Shadow

1. Ukur parameter yang masih asumsi, terutama **uji peluruhan bebas** (sekaligus mengukur redaman,
   gesekan, kekakuan, dan frekuensi) dan **kurva LDR**.
2. Instrumentasi: mikrokontroler membaca 4 LDR dan arus panel, kirim ke serial.
3. Kalibrasi LDR dengan prosedur A dan B.
4. Node pembaca serial menerbitkan ke `/tracker/diag`. **Ini Digital Shadow.**
5. Tutup loop ke driver. **Ini Digital Twin.**

---

## 14. Ringkasan hasil terukur

| Besaran | Nilai | Cara diperoleh |
|---|---|---|
| Posisi matahari | selisih < 0,011 derajat | pvlib SPA vs astropy, SolTrack, Pysolar |
| Kinematika normal panel | selisih 0,14 sampai 0,34 derajat | Gazebo TF vs rumus |
| Defleksi gravitasi q2 = 0 | 0,161 terukur vs 0,155 prediksi | Gazebo |
| Frekuensi alami azimuth | 4,79 prediksi, 5,19 terukur Hz | nilai eigen vs Gazebo 1 kHz |
| Lewatan langkah 1 derajat | 93,3 teori, 90 terukur persen | analitik vs Gazebo |
| ZVD menekan lewatan terburuk | 19,35 jadi 2,30 derajat | Gazebo, 5 ulangan |
| Rugi energi dari osilasi | 0,007 persen | model dua sumbu 1 kHz |
| Energi harian 7 Okt | 431,0 Wh tracker, 362,3 panel datar | model dioda tunggal |
| Margin angin terburuk 11 tahun | 9,2 persen torsi tahan | Open-Meteo + model berarah |
| EKF dengan LDR terkalibrasi | 2,52 jadi 0,18 derajat | simulasi hari penuh |
| Pemulihan lepas langkah 7,2 derajat | 350 detik | simulasi hari penuh |

---

## 15. Kekurangan yang diakui

**Belum dimodelkan:** suku Coriolis (besarnya sudah dihitung, 3,4 persen saat slew), latensi dan jitter
jaringan, serangan siber, event-triggered control, optimasi multi-objektif, backlash kopling, mode getar
rangka.

**Masih asumsi dan paling menentukan:** redaman sendi 0,2 N m s/rad dan gesekan 0,05 N m (menentukan seluruh
hasil dinamika), kurva LDR `gam` 0,7 dan `L0` 20000 lux (menentukan batas kalibrasi), ketidakcocokan LDR
3 persen, batas kemiringan 45 derajat, arah hadap rel, massa motor 1,2 kg, koefisien angin Cn 1,2 dan
pergeseran pusat tekanan.

**Dua klaim yang sudah ditarik:** batas angin tarik-lepas 9,6 m/s (lengan momen tebakan, rentangnya 6,5
sampai 39 m/s) dan pergeseran puncak arus +2,6 derajat (irradiance pilihan tangan tidak konsisten).

Daftar lengkap dan cara mengukurnya ada di `parameter_asumsi_dan_pengukuran.md`.

---

## 16. Peta berkas

| Berkas | Isi |
|---|---|
| `parameter_kinematik.md` | dokumen induk seluruh parameter, 18 bagian |
| `parameter_asumsi_dan_pengukuran.md` | yang masih asumsi dan cara mengukurnya |
| `tracker.py`, `build_model.py` | kinematika, URDF, inersia |
| `dinamika/dyn_model.py` | dinamika satu sumbu, stepper, poros |
| `dinamika/osilasi_energi.py` | osilasi dua sumbu dan dampaknya pada energi |
| `kontrol_hibrid/panel_listrik.py` | model dioda tunggal panel |
| `kontrol_hibrid/sensor_arus.py` | rantai pengukuran arus dan batasnya |
| `kontrol_hibrid/fusi_sensor.py` | ketidakpastian tiap sensor dan bobot |
| `kontrol_hibrid/ekf.py` | penaksir simpangan sudut |
| `kontrol_hibrid/kalibrasi_ldr.py` | prosedur kalibrasi A dan B |
| `angin/angin_openmeteo.py` | angin nyata berarah 11 tahun |
| `solar_tracker/` | simulasi Gazebo, node ROS 2, panel pemantauan |
