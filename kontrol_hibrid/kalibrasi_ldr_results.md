# Kalibrasi ketidakcocokan antar LDR

Dibangkitkan `kontrol_hibrid/kalibrasi_ldr.py`. Ketidakcocokan antar LDR adalah satu satunya hal yang membuat LDR kalah dari model astro (lihat `fusi_sensor_results.md` dan `ekf_results.md`).

## 1. Prosedur A: kalibrasi dengan cahaya baur merata

**Dasar:** sekat hanya membayangi komponen LANGSUNG. Di bawah cahaya baur murni, keempat LDR menerima irradiance yang sama berapa pun arah panel, sehingga setiap beda pembacaan murni ketidakcocokan. Diperiksa di selftest: dengan DNI nol, e sama persis pada pose (0,0), (30,20), dan (-40,45) derajat.

**Cara di lapangan:** tunggu langit mendung tebal (atau tutup panel dengan penyebar cahaya), rekam keempat LDR beberapa ratus cuplikan, balik ke ruang x = v/(vcc - v), lalu simpan faktor koreksi per sensor.

| Keadaan | Sisa ketidakcocokan |
|---|---|
| Sebelum kalibrasi | 3.00 % |
| Prosedur A, cahaya baur murni, 400 cuplikan | **0.011 %** |
| Prosedur A di mendung okta 8 nyata (masih ada sisa DNI) | 0.693 % |
| Prosedur A dengan eksponen gam meleset 10 % | 0.255 % |

Pengaruh banyaknya cuplikan (cahaya baur murni):

| Cuplikan dirata-rata | Sisa ketidakcocokan |
|---|---|
| 10 | 0.071 % |
| 50 | 0.060 % |
| 200 | 0.010 % |
| 800 | 0.007 % |

Dua hal yang membatasi prosedur A: derau ADC (bisa ditekan dengan merata-rata) dan pengetahuan tentang kurva LDR, terutama eksponen `gam`. Galat 10 persen pada `gam` menyisakan ketidakcocokan 0.25 persen, yang sudah cukup untuk menghapus sebagian besar manfaat kalibrasi. Jadi **mengukur kurva LDR lebih penting daripada memperbanyak cuplikan.**


## 2. Prosedur B: memisahkan salah pasang dari ketidakcocokan

Keduanya menggeser e dengan cara yang mirip, tetapi salah pasang menghasilkan e = k x bias yang ikut berubah ketika kepekaan k berubah, sedangkan ketidakcocokan menghasilkan offset yang hampir tetap. Regresi e terhadap k sepanjang hari memisahkan keduanya, sejauh k cukup berubah.

| Sumbu | Rentang perubahan k sehari | Bilangan kondisi regresi | Taksiran bias pasang (sebenarnya) |
|---|---|---|---|
| Azimuth | 25.6 kali | 3859.1 | 1.50 deg (1.50) |
| Kemiringan | 1.1 kali | 23936.1 | 0.84 deg (1.00) |

**Azimuth terpisahkan, kemiringan tidak.** Kepekaan azimuth sebanding sin(q2) sehingga berubah lebar sepanjang hari, memberi regresi tumpuan yang cukup. Kepekaan kemiringan hampir tetap, sehingga bias pasang dan ketidakcocokan nyaris tidak terbedakan. Ini sejalan dengan catatan lama di proyek ini bahwa hanya bias azimuth yang dapat dipisahkan.


## 3. Hasil akhir: galat pointing lewat EKF

Ketidakcocokan sisa dari tabel 1 dimasukkan ke `ekf.py`, hari 7 Okt 2026 dengan lepas langkah azimuth 7.2 deg di tengah hari. Galat diukur hanya saat matahari terjangkau.

| Keadaan LDR | Ketidakcocokan sisa | Galat pointing rata-rata | p95 |
|---|---|---|---|
| (tanpa LDR, astro saja) | - | 2.523 deg | 5.300 deg |
| tanpa kalibrasi | 3.000 % | 5.629 deg | 6.104 deg |
| prosedur A di mendung nyata | 0.693 % | 1.459 deg | 1.661 deg |
| prosedur A, gam meleset 10 % | 0.255 % | 0.582 deg | 0.775 deg |
| prosedur A, baur murni | 0.011 % | 0.164 deg | 0.324 deg |

Kalibrasi mengubah LDR dari **merugikan** menjadi **sangat menguntungkan**. Perhatikan prosedur A yang dilakukan di mendung nyata (masih ada 2.2 persen DNI) sudah cukup untuk membalik keadaan, meski hasilnya 60 kali lebih buruk daripada dengan cahaya baur murni.


## 4. Urutan kalibrasi yang disarankan

1. **Ukur kurva LDR di meja** (lux terhadap tegangan) untuk mendapat `gam` dan `L0`. Ini menentukan batas atas ketelitian seluruh prosedur, lebih menentukan daripada jumlah cuplikan.
2. **Prosedur A saat mendung tebal** atau dengan penutup penyebar cahaya. Satu kali, beberapa menit, menurunkan ketidakcocokan dari 3 persen ke bawah 0.5 persen bila `gam` diketahui baik.
3. **Prosedur B sepanjang satu hari cerah** untuk menyerap sisa salah pasang azimuth.
4. **Ulangi prosedur A berkala**, karena LDR menua dan kotor dengan laju berbeda beda.


## 4. Batas

- Model LDR (sekat `r`, eksponen `gam`, setengah jenuh `L0`, ketidakcocokan 3 persen) seluruhnya ASUMSI. Angka di sini menunjukkan BENTUK hubungannya, bukan ramalan untuk alat nyata.
- Prosedur A mengoreksi ketidakcocokan PENGUATAN. Ia tidak mengoreksi beda sudut pandang, beda tinggi pemasangan, atau sekat yang tidak sama tinggi; ketiganya memberi tanda yang bergantung arah.
- Prosedur B mengandaikan bias pasang TETAP sepanjang hari. Lepas langkah yang terjadi di tengah hari melanggar itu; pakai EKF untuk menanganinya, lalu prosedur B pada bagian hari yang bersih saja.
- Kotor yang tidak merata antar LDR berperilaku seperti ketidakcocokan yang berubah lambat, jadi kalibrasi perlu diulang, bukan sekali selamanya.
