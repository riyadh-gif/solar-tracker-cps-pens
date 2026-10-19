# EKF: menaksir simpangan sudut tracker loop terbuka

Dibangkitkan `kontrol_hibrid/ekf.py`. Keadaan yang ditaksir adalah SIMPANGAN `b` antara sudut perintah dan sudut beban (salah pasang + lepas langkah terkumpul), bukan sudutnya sendiri.

Hari 7 Okt 2026, selang tapis 10 s, jalan acak 0.01 deg per akar detik (ASUMSI), ketidakpastian awal 3.0 deg. Lepas langkah azimuth 7.2 deg (dua siklus listrik) disuntikkan pada 30 persen hari.

## 1. Perbandingan galat pointing sehari

| Susunan | Galat rata-rata (deg) | Galat p95 (deg) | Galat maks (deg) |
|---|---|---|---|
| astro saja (tanpa tapis) | 2.523 | 5.300 | 5.439 |
| EKF + LDR terkalibrasi | 0.181 | 0.386 | 3.788 |
| EKF + LDR 3 % tak terkalibrasi | 5.629 | 6.104 | 6.884 |
| EKF + LDR terkalibrasi + arus | 0.181 | 0.385 | 3.788 |
| EKF + arus saja | 2.240 | 5.167 | 5.359 |

Tiga hal yang terbaca:
1. **LDR terkalibrasi memperbaiki 14 kali** (2.52 menjadi 0.18 derajat).
2. **LDR yang TIDAK dikalibrasi lebih buruk daripada tidak memakai LDR sama sekali** (5.63 berbanding 2.52 derajat). Tapis menyerap bias LDR sebagai salah pasang lalu mengarahkan panel keliru sebesar itu. Ini menegaskan temuan langkah sebelumnya: kalibrasi adalah syarat, bukan penyempurnaan.
3. **Arus hampir tidak menambah apa apa** di atas LDR (0.181 menjadi 0.181), dan sendirian hanya memperbaiki 11 persen (2.52 menjadi 2.24). Sesuai dugaan dari Jacobian: dI/db hampir nol di dekat puncak.


**Jebakan yang saya temui:** mula mula R arus saya isi hanya derau ADC (0.2 mA), dan tapis RUSAK, galat melonjak ke 20.5 derajat. Sebabnya penguatan Kalman K = P H / R menjadi ribuan kali terlalu besar ketika R jauh lebih kecil daripada galat MODEL ramalan arus. Ketidakpastian ramalan arus didominasi toleransi pabrik 3 persen, kotor, suhu sel, dan albedo, bukan oleh deraunya ADC. Setelah R memuat suku model, arus menjadi tidak berbahaya sekaligus hampir tidak berguna. Pelajarannya umum: **dalam tapis Kalman, R harus mewakili selisih antara pengukuran dan MODEL, bukan hanya derau sensor.**


## 2. Ketidakpastian yang dilaporkan tapis

Akar diagonal P, yaitu seberapa yakin tapis terhadap taksirannya sendiri.

| Saat | sigma azimuth (deg) | sigma kemiringan (deg) |
|---|---|---|
| awal | 0.648 | 0.565 |
| sebelum lepas langkah | 0.155 | 0.117 |
| sesudah lepas langkah | 0.168 | 0.116 |
| tengah hari | 0.448 | 0.121 |
| akhir hari | 0.130 | 0.122 |

**Pemeriksaan kejujuran tapis:** sigma yang DILAPORKAN harus sepadan dengan galat yang SEBENARNYA. Rata-rata sigma yang dilaporkan [0.233, 0.118] derajat, sedangkan sebaran galat taksiran sebenarnya [0.789, 0.165] derajat. Tapis ini **terlalu percaya diri** sekitar 3.4 kali pada azimuth, terutama karena Q (jalan acak) tidak mewakili lepas langkah yang datang sebagai lompatan besar sesekali. Memperbesar Q, atau menambahkan uji pencilan pada inovasi, akan membuat laporannya lebih jujur.


## 3. Jacobian pengukuran dan efek keyhole

H = de/db. Kolom azimuth mengecil sebanding sin(q2), jadi saat matahari tinggi LDR hampir tidak memberi tahu apa apa tentang azimuth, dan tapis melaporkannya sendiri lewat P yang membesar.

| Keadaan | dex/db_az | dex/db_q2 | dey/db_az | dey/db_q2 |
|---|---|---|---|---|
| elevasi 45 deg (q2 45) | +0.00000 | -0.00129 | +0.00091 | +0.00000 |
| elevasi 80 deg (q2 10) | +0.00000 | -0.00116 | +0.00019 | +0.00000 |

Inilah pertemuan Jacobian dan ketidakpastian yang dibahas sebelumnya: keyhole bukan aturan tangan `if q2 < 5 derajat`, melainkan akibat langsung dari H yang kehilangan rank.


## 4. Pemulihan setelah lepas langkah

Lepas langkah azimuth 7.2 deg disuntikkan pada langkah 1161. Galat memuncak 3.79 deg, lalu turun di bawah 1 deg setelah 350 detik. Tapis tidak perlu diberi tahu bahwa lepas langkah terjadi: selisih pengukuran terhadap ramalan menanganinya sendiri.


## 5. Batas mendasar: bias LDR tidak dapat dipisahkan dari salah pasang

Keduanya menggeser pembacaan dengan cara yang sama, sehingga tidak teramati secara terpisah. EKF akan menyerap bias LDR ke dalam taksiran `b` lalu MENGARAHKAN panel keliru sebesar bias itu. Terlihat pada baris ketiga tabel 1: galat naik dari 7.976 ke 10.549 derajat hanya karena LDR tidak dikalibrasi. Tapis yang lebih pintar tidak menolong; yang menolong kalibrasi, atau sensor tambahan yang biasnya tidak berkorelasi.


## 6. Yang masih asumsi

- Jalan acak Q 0.01 deg per akar detik, dan ketidakpastian awal 3.0 deg.
- Lepas langkah dimodelkan sebagai lompatan tunggal 7.2 derajat; nyatanya bergantung angin dan beban.
- Seluruh tetapan LDR (sekat, kurva lux, ketidakcocokan) dan rantai pengukuran arus masih ASUMSI.
- Tapis bekerja pada SIMPANGAN, bukan pada dinamika 5 Hz. Ayunan tidak diamati dan tidak diredam tapis (lihat `dinamika/osilasi_results.md`: peredamannya dari sisi perintah).
