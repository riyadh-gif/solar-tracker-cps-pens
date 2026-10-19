# Arus panel sebagai sensor arah: derau dan batasnya

Dibangkitkan `kontrol_hibrid/sensor_arus.py`. Arus sebenarnya dari `panel_listrik.py` (dioda tunggal, pelat nama MOS C-50-18-M), lalu dilewatkan rantai pengukuran.

## 1. Rantai pengukuran (ASUMSI)

| Besaran | Nilai |
|---|---|
| ADC | 12 bit, jangkauan 0 sampai 4.0 A, 0.977 mA per LSB |
| Derau acak | 2 mA rms |
| Hanyutan offset | 5 mA (lambat) |
| Laju cuplik | 100 Hz |

## 2. Galat arah terkecil yang masih terbaca

Ambang 3 sigma pada SELISIH dua pembacaan (deraunya akar 2 kali derau satu pembacaan). Diukur ke sisi yang menjauh dari puncak albedo, yaitu sisi yang lebih peka.

| Cuplikan dirata-rata | Waktu (s) | Derau pembacaan (mA) | Galat arah terkecil (deg) |
|---|---|---|---|
| 1 | 0.01 | 2.020 | 5.45 |
| 10 | 0.10 | 0.639 | 3.15 |
| 100 | 1.00 | 0.202 | 1.85 |
| 1000 | 10.00 | 0.064 | 1.10 |
| 10000 | 100.00 | 0.020 | 0.70 |

Merata-rata menolong, tetapi hasilnya **melambat jauh lebih cepat daripada ketelitiannya membaik**: arus berubah sebagai kosinus, jadi menuruni galat dua kali lipat menuntut ketelitian empat kali lipat, yaitu 16 kali lebih banyak cuplikan. Merata-rata 100 detik pun tidak membawa arus ke ketelitian derajat.


## 3. Pembanding: awan jauh lebih berpengaruh daripada salah arah

| Tutupan awan (okta) | Imp (A) | Turun terhadap cerah | Turun bila salah arah 5 deg |
|---|---|---|---|
| 0 | 2.3787 | +0.0 % | -0.305 % |
| 2 | 2.3549 | -1.0 % | -0.286 % |
| 4 | 2.1234 | -10.7 % | -0.064 % |
| 6 | 1.4075 | -40.8 % | +0.960 % |

Inilah hambatan pokok memakai arus sebagai umpan balik arah: **perubahan arus karena awan dua sampai tiga ORDE lebih besar daripada perubahan karena salah arah beberapa derajat.** Pengendali yang menyimpulkan arah dari naik turunnya arus akan tertipu awan, kecuali perbandingannya dilakukan jauh lebih cepat daripada perubahan awan, atau irradiance diukur terpisah sebagai acuan.


## 4. Pendakian bukit (perturb and observe) memakai arus saja

Mulai dari simpangan 12 deg, langkah 2 deg, 100 cuplikan (1 detik) tiap pembandingan, 60 iterasi.

| Keadaan | Berhenti di (deg) | Puncak sebenarnya (deg) | Selisih |
|---|---|---|---|
| cerah dan tenang | +0.20 (sebaran +0.2 sampai +0.2) | -0.10 | +0.30 |
| hanyutan offset 1 mA/s | -0.36 (sebaran -1.4 sampai +0.6) | -0.10 | -0.26 |
| awan menipis 2 %/s | -51.00 (sebaran -51.0 sampai -51.0) | -0.10 | -50.90 |
| awan menebal 2 %/s | +13.00 (sebaran +13.0 sampai +13.0) | -0.10 | +13.10 |
| awan menebal 5 %/s | +13.00 (sebaran +13.0 sampai +13.0) | -0.10 | +13.10 |

Saat cerah dan tenang pendakian bukit berhasil, tetapi hanya sampai ketelitian seukuran langkahnya, dan butuh sekitar satu menit untuk menempuh 12 derajat. Begitu irradiance berubah lambat, pendakian **tertarik mengikuti perubahan irradiance, bukan arah matahari**: arus yang naik karena awan menipis dibaca sebagai bukti arah yang benar.


## 5. Kesimpulan untuk rancangan kontrol

- Arus **tidak layak** jadi umpan balik arah yang berdiri sendiri: buta di bawah beberapa derajat, tidak memberi arah, dan mudah tertipu awan.
- Arus **layak** untuk: memeriksa bahwa penjejakan tidak melenceng jauh (di atas 10 derajat), mendeteksi lepas langkah besar, dan menjadi ukuran hasil yang sebenarnya untuk menilai algoritma.
- Agar arus berguna untuk arah, perlu acuan irradiance terpisah (sel acuan, piranometer, atau LDR itu sendiri) supaya pengaruh awan dapat dibagi keluar.
- Ini memperkuat rencana penggabungan: LDR memberi arah dan kepekaan, arus memberi kebenaran hasil. Bobot penggabungan harus mengikuti ketidakpastian masing-masing, bukan disetel tangan.


## 6. Yang masih asumsi

- Seluruh rantai pengukuran (bit ADC, derau, hanyutan); ganti setelah sensor arus dipilih.
- Model awan hanya penskalaan irradiance seragam (Kasten-Czeplak lewat okta); awan nyata membuat bayangan sebagian pada panel, yang pengaruhnya lebih rumit dan lebih buruk.
- Pendakian bukit di sini menggeser kemiringan saja, satu sumbu.
