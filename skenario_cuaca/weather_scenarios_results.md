# Skenario cuaca dan rasio sinyal LDR (PENS, -7.276006, 112.794327)

Dihasilkan oleh `weather_scenarios.py` (jalankan ulang untuk memperbarui tabel). Resolusi 5 menit, irradiance langit cerah dari `sun.py` (pvlib Ineichen), awan dengan Kasten-Czeplak, pemecahan DNI/DHI dengan Erbs kecuali disebut lain. Gambar: `F_vs_waktu.png`, `irradiance_per_skenario.png`, `gain_tracker.png`.

## 1. Skenario dan pilihan okta

| skenario | okta | faktor GHI (Kasten-Czeplak) | alasan |
|---|---|---|---|
| cerah | 0 | 1.00 | batas atas, acuan langit cerah |
| berawan sebagian | 4 | 0.93 | setengah langit tertutup, sepadan dengan "berawan sebagian" pada paper Saputra |
| mendung tebal | 8 | 0.25 | tutupan penuh, batas bawah |

Catatan penting: pada model ini okta 4 hanya memotong GHI sekitar 7 persen (0.75 x 0.5^3.4), jadi secara radiometrik skenario "berawan sebagian" masih sangat dekat dengan cerah. Okta 6 (faktor 0.64) ditambahkan di uji sensitivitas (bagian 5) sebagai pembanding yang lebih "berawan".

Hari representatif: 21 Mar (ekuinoks), 21 Jun (matahari paling utara, deklinasi +23.4), 4 Okt 2026 (hari kerja eksperimen), 21 Des (matahari paling selatan, deklinasi -23.4). Surabaya di lintang -7.3, jadi matahari melintas di utara panel pada Jun dan di selatan pada Des.

## 2 dan 4. Irradiance harian, F, energi, dan gain (Erbs)

Satuan energi kWh/m2/hari. E tetap = bidang miring 7 deg menghadap utara. E tracker = bidang selalu tegak lurus matahari (tilt = zenith, azimuth bidang = azimuth matahari, model langit isotropik pvlib, albedo 0.25). F rata-rata dihitung pada elevasi > 10 deg, F berbobot energi = sum(DNI cos z)/sum(GHI).

| hari | skenario | GHI kWh/m2 | DNI kWh/m2 | DHI kWh/m2 | F rata-rata (el>10) | F berbobot energi | E tetap kWh/m2 | E tracker kWh/m2 | gain % |
|---|---|---|---|---|---|---|---|---|
| 21 Mar | cerah (0) | 7.047 | 6.520 | 1.931 | 0.654 | 0.726 | 7.091 | 8.357 | 17.858 |
| 21 Mar | berawan sebagian (4) | 6.546 | 5.307 | 2.347 | 0.571 | 0.642 | 6.580 | 7.512 | 14.157 |
| 21 Mar | mendung tebal (8) | 1.762 | 0.036 | 1.734 | 0.015 | 0.015 | 1.757 | 1.644 | -6.436 |
| 21 Jun | cerah (0) | 5.650 | 5.957 | 1.583 | 0.660 | 0.720 | 5.973 | 7.459 | 24.882 |
| 21 Jun | berawan sebagian (4) | 5.248 | 4.836 | 1.919 | 0.575 | 0.634 | 5.509 | 6.613 | 20.033 |
| 21 Jun | mendung tebal (8) | 1.412 | 0.033 | 1.391 | 0.015 | 0.015 | 1.410 | 1.267 | -10.136 |
| 04 Okt | cerah (0) | 7.230 | 6.895 | 1.839 | 0.684 | 0.746 | 7.209 | 8.657 | 20.077 |
| 04 Okt | berawan sebagian (4) | 6.716 | 5.660 | 2.251 | 0.603 | 0.665 | 6.698 | 7.783 | 16.211 |
| 04 Okt | mendung tebal (8) | 1.807 | 0.038 | 1.779 | 0.015 | 0.016 | 1.802 | 1.690 | -6.261 |
| 21 Des | cerah (0) | 7.212 | 6.866 | 1.994 | 0.652 | 0.723 | 6.918 | 8.761 | 26.650 |
| 21 Des | berawan sebagian (4) | 6.699 | 5.584 | 2.420 | 0.569 | 0.639 | 6.458 | 7.848 | 21.524 |
| 21 Des | mendung tebal (8) | 1.803 | 0.038 | 1.775 | 0.015 | 0.015 | 1.796 | 1.667 | -7.222 |


Ringkasan rata-rata 4 hari dan pembanding Saputra 2026:

| skenario | F berbobot energi (rata 4 hari) | gain % (rata 4 hari) | gain % Saputra |
|---|---|---|---|
| cerah (0) | 0.729 | 22.367 | 28.400 |
| berawan sebagian (4) | 0.645 | 17.981 | 19.700 |
| mendung tebal (8) | 0.016 | -7.514 | 10.000 |


## 3. Mengapa F adalah proksi kekuatan sinyal diferensial

F = DNI cos(zen) / GHI adalah fraksi GHI horizontal yang berasal dari berkas langsung. Pada sensor 4 kuadran bersekat, tiap LDR menerima komponen difus yang hampir sama (langit terlihat serupa dari semua kuadran) ditambah komponen langsung yang hanya mengenai kuadran yang tidak terhalang sekat. Selisih antar LDR (kiri-kanan atau atas-bawah) yang dipakai kontrol datang dari komponen langsung dan sebanding dengan DNI x (fungsi sudut kesalahan pelacakan), sedangkan komponen difus muncul sebagai common-mode yang hilang saat dikurangkan, tetapi tetap menaikkan level dasar dan derau. Sinyal diferensial ternormalisasi (selisih dibagi jumlah) karena itu kira-kira sebanding dengan fraksi langsung terhadap total, dan F adalah proksi horizontal untuk fraksi itu (pendekatan: sensor dianggap mendatar, difus isotropik, tanpa pantulan sekat). Saat F menuju 0 (mendung) selisih antar LDR tenggelam dalam derau, sehingga kontrol LDR kehilangan arah.

## 4. Pembahasan tren gain tracker

Hasil model (rata-rata 4 hari): cerah sekitar +22 persen, berawan sebagian (okta 4) sekitar +18 persen, mendung tebal (okta 8) sekitar -7 persen. Urutan turun dari cerah ke berawan ke mendung sama dengan Saputra (28.4, 19.7, sekitar 10 persen), dan nilai cerah serta berawan sebagian berada dalam kisaran yang sama (17.9 sampai 26.7 persen untuk cerah, 14.2 sampai 21.5 persen untuk okta 4). Perbedaannya ada pada mendung tebal: model ini memberi gain NEGATIF, Saputra positif sekitar 10 persen.

Penyebab perbedaan, jujur:
1. Pada mendung penuh model Erbs memberi fraksi difus hampir 98 persen (kt rendah), jadi DNI praktis nol. Irradiance langit difus isotropik paling banyak diterima bidang yang mendatar (melihat seluruh langit). Bidang "ideal tegak lurus matahari" pada pagi dan sore miring besar, melihat sebagian langit dan sebagian tanah (albedo 0.25), sehingga menerima LEBIH SEDIKIT dari panel hampir datar 7 deg. Maka tracker ideal yang mengarah ke matahari bukan lagi arah optimum pada langit mendung. Ini artefak definisi tracker "selalu menghadap matahari" ditambah model langit isotropik.
2. Paper melaporkan pengukuran nyata: awan mendung tebal di dunia nyata tidak seragam, masih ada DNI sisa, kecerahan langit tidak isotropik (zenit lebih terang), dan kontrol hibrid mereka berbasis SPA yang tetap bisa menempatkan panel di sudut yang masuk akal. Kita tidak mereplikasi itu di sini.
3. Peningkatan Saputra juga mencakup efek pengukuran (daya panel) yang tidak sama dengan irradiance bidang ideal; paper berasal dari Palembang dan konfigurasi panel statis mereka tidak diketahui sama dengan miring 7 deg utara.

Implikasi untuk tesis: pada skenario mendung tebal sebaiknya kontrol hibrid tidak mengandalkan LDR (F kecil) dan tidak perlu mengejar matahari, cukup tahan di sudut dekat datar, atau pakai posisi hasil pvlib dengan batas kemiringan. Gain tracker pada mendung tebal sebaiknya dilaporkan sebagai sekitar 0 atau negatif pada model ini, bukan dipaksakan menyamai 10 persen.

## 5. Sensitivitas terhadap model dekomposisi

Dibandingkan: Erbs, Boland, Orgill-Hollands, dan DISC (DHI = GHI - DNI cos z) pada GHI yang sama dari Kasten-Czeplak. Kolom "vs Erbs" memakai rata-rata 4 hari.

| okta | model | F 04 Okt | gain % 04 Okt | F rata 4 hari | gain % rata 4 hari | GHI 04 Okt kWh/m2 | dF vs Erbs | dgain vs Erbs (poin %) |
|---|---|---|---|---|---|---|---|
| 4 | erbs | 0.665 | 16.211 | 0.645 | 17.981 | 6.716 | 0.000 | 0.000 |
| 4 | boland | 0.583 | 12.834 | 0.562 | 14.154 | 6.716 | -0.083 | -3.828 |
| 4 | orgill_hollands | 0.642 | 15.988 | 0.623 | 17.734 | 6.716 | -0.022 | -0.248 |
| 4 | disc | 0.561 | 17.928 | 0.563 | 19.634 | 6.716 | -0.082 | 1.652 |
| 6 | erbs | 0.353 | 4.355 | 0.337 | 4.545 | 5.191 | 0.000 | 0.000 |
| 6 | boland | 0.286 | 2.406 | 0.273 | 2.338 | 5.191 | -0.065 | -2.207 |
| 6 | orgill_hollands | 0.371 | 5.401 | 0.356 | 5.743 | 5.191 | 0.019 | 1.198 |
| 6 | disc | 0.282 | 5.097 | 0.279 | 5.216 | 5.191 | -0.058 | 0.671 |
| 8 | erbs | 0.016 | -6.261 | 0.016 | -7.514 | 1.807 | 0.000 | 0.000 |
| 8 | boland | 0.022 | -6.019 | 0.022 | -7.237 | 1.807 | 0.006 | 0.276 |
| 8 | orgill_hollands | 0.044 | -5.107 | 0.043 | -6.193 | 1.807 | 0.027 | 1.320 |
| 8 | disc | 0.028 | -6.605 | 0.024 | -7.895 | 1.807 | 0.009 | -0.381 |


Temuan:
- Okta 4 (berawan sebagian): F berubah dari 0.645 (Erbs) menjadi 0.562 (Boland) dan 0.563 (DISC), turun sekitar 0.08 (sekitar 13 persen relatif); Orgill-Hollands hampir sama dengan Erbs (turun 0.02). Gain tracker bergeser dari 18.0 persen menjadi 14.2 (Boland), 17.7 (Orgill-Hollands), 19.6 (DISC): rentang sekitar 5 poin persen.
- Okta 6: F berkisar 0.27 sampai 0.36 dan gain 2.3 sampai 5.7 persen, jadi sangat bergantung pada model. Wilayah kt menengah adalah yang paling tidak pasti.
- Okta 8 (mendung): F kecil di semua model (0.016 sampai 0.043), gain tracker tetap negatif (-5.1 sampai -7.9 persen). Kesimpulan kualitatif (sinyal LDR diferensial hampir hilang, gain tracker ideal tidak positif) kokoh terhadap pilihan model, tetapi besarnya F bisa berbeda sampai dua kali lipat (Orgill-Hollands 0.043 vs Erbs 0.016).
- Karena F menentukan kekuatan sinyal LDR, ambang "LDR dipercaya" pada kontrol hibrid sebaiknya diberi margin (misalnya F dari bacaan sensor nyata, bukan dari model) dan dikalibrasi dari data, bukan dari satu model dekomposisi.

## 6. Keterbatasan model awan dan saran lanjutan

Keterbatasan:
- Kasten-Czeplak adalah relasi empiris rata-rata klimatologis (GHI/GHI cerah sebagai fungsi okta), bukan fisika transfer radiasi. Tidak ada bedanya jenis awan (cirrus tipis vs stratus tebal) pada okta sama.
- Tutupan awan seragam sepanjang hari: tidak ada awan bergerak, tidak ada transisi cerah-berawan, tidak ada kedipan DNI menit-an yang menjadi tantangan terbesar kontrol LDR.
- Tidak ada cloud enhancement (GHI sesaat melebihi langit cerah akibat pantulan tepi awan) dan tidak ada kenaikan DNI sebagian saat matahari muncul di celah awan.
- Dekomposisi (Erbs, Boland, dll.) dikalibrasi pada GHI terukur di lintang menengah; di khatulistiwa lembap berbeda, dan model deterministik kt ke fraksi difus memberi DNI hampir nol pada okta 8, padahal nyata masih ada DNI sisa.
- Langit difus diasumsikan isotropik pada bidang miring (pvlib `isotropic`); langit mendung nyata lebih terang di zenit, yang justru menguntungkan bidang datar dan memperburuk gain negatif.
- Langit cerah memakai Ineichen dengan turbiditas klimatologi (lookup), bukan aerosol terukur, sehingga DNI cerah (sekitar 6.5 sampai 6.9 kWh/m2/hari) mungkin terlalu tinggi untuk Surabaya lembap.
- Albedo 0.25 dan ketinggian 5 m adalah asumsi. Gain dihitung pada irradiance bidang, bukan daya listrik panel (tanpa efek suhu dan sudut datang).

Saran data nyata (langkah lanjutan, belum dikerjakan): ukur GHI dan DHI (atau pyranometer dengan shadow band) di lokasi sendiri, atau pakai reanalisis dan satelit (ERA5, CAMS/Himawari) untuk DNI/GHI/DHI per jam atau 10 menit; rekam bacaan 4 LDR bersama pyranometer pada hari cerah, berawan, dan mendung untuk memverifikasi F sebagai prediktor selisih LDR; gunakan deret awan bergerak (clear-sky index nyata) sebagai masukan uji kontrol, bukan okta seragam.

## Verifikasi

Skrip menjalankan self-check (assert): GHI okta 8 = 0.25 x cerah, DNI cos z + DHI = GHI, 0 <= F <= 1, F cerah > F mendung, GHI cerah > mendung, tracker ideal > tetap saat cerah. Semua lolos pada pelaksanaan terakhir.
