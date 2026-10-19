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

{tab_main}

Ringkasan rata-rata 4 hari dan pembanding Saputra 2026:

{tab_mean}

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

{tab_sens}

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
