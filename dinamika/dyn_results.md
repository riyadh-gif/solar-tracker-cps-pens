# Fase 4: dinamika satu sumbu (stepper, poros fleksibel, angin)

Skrip: `dyn_model.py` (`python dyn_model.py` hanya self-check, `python dyn_model.py --full` menjalankan semua eksperimen, sekitar 70 detik). Tabel lengkap: `dyn_results_tables.md`. Gambar: `dyn_step.png`, `dyn_rate.png`, `dyn_wind.png`.

## Definisi dan keputusan

- **Loop terbuka (open-loop) pada motor stepper:** driver mengirim pulsa langkah dan motor dianggap mengikutinya, tanpa sensor posisi pada poros motor dan tanpa umpan balik ke driver. Posisi sebenarnya tidak diukur, sehingga **lepas langkah** (torsi beban melampaui torsi tahan atau pull-out, atau terpicu resonansi) tidak terdeteksi dan galat posisi menumpuk karena program tetap mengira posisinya sesuai perintah.
- **Pembeda dengan loop kontrol di tingkat sistem:** motor boleh loop terbuka sementara pelacak secara keseluruhan tetap memakai umpan balik cahaya (LDR) untuk mengoreksi arah. Umpan balik LDR itu mengoreksi galat pointing, termasuk akibat lepas langkah, tetapi hanya selama LDR dipercaya. Pada mode astronomi saja (mendung atau LDR tidak terpercaya) lepas langkah tidak terkoreksi, kecuali ada acuan mutlak (saklar posisi rumah, saklar batas, atau inklinometer).
- **Keputusan (dikonfirmasi pengguna): driver motor 57HSE3.0N berjenis loop terbuka.** Maka model stepper buka-loop di `dyn_model.py` berlaku, dan risiko lepas langkah akibat angin kencang (sekitar 20 m/s menurut model) harus diperhitungkan di desain. Konsekuensi ke depan: perlu acuan posisi mutlak (rumah atau saklar batas) untuk menyetel ulang posisi, dan hibrid dengan memori koreksi LDR sebaiknya diuji terhadap lepas langkah.

## Model

Rotor Jm digerakkan hukum torsi-sudut stepper hibrid, Tm = -Th sin(Nr (theta_m - cmd)) - bm w_m, dengan kekakuan tahan stepper Kh = Th Nr = 150 N.m/rad. Rotor dihubungkan ke beban JL lewat poros (kekakuan k, redaman c), beban punya redaman viskos dan gesekan Coulomb (dihaluskan tanh), plus momen gravitasi tetap dan momen angin. Integrasi RK4 (dt 0.1 ms), vektor atas banyak kasus. Langkah buka-loop, driver sumber arus ideal.

| Parameter | Nilai | Sumber |
|---|---|---|
| Th, Nr | 3.0 N.m, 50 gigi | nama motor 57HSE3.0N, stepper 1.8 deg (nominal) |
| Jm | 1.1e-4 kg.m2 | asumsi |
| JL elevasi, azimuth | 0.0353, 0.0488 kg.m2 | CAD (`inersia/`), dengan massa motor 1.2 kg dan panel 1.5 kg yang ASUMSI (rentang 0.026 sampai 0.085) |
| k elevasi | 317 N.m/rad | geometri poros baja 8 mm sepanjang 100 mm (`inersia/`) |
| k azimuth | 1600 N.m/rad | asumsi: poros motor 8 mm sepanjang 20 mm |
| bl, fc, zeta | 0.2 N.m.s/rad, 0.05 N.m, 0.02 | asumsi (sangat menentukan dering) |
| Momen gravitasi elevasi | 0.64 N.m | CAD, kasus terburuk dalam +-45 deg |
| Momen angin | 0.5 rho v2 Cn A arm, Cn 1.2, A 0.132 m2, arm 0.066 (elevasi) dan 0.10 m (azimuth) | asumsi (lengan pusat tekanan, tanpa faktor hembusan) |

## Temuan

1. **Frekuensi alami terendah sekitar 8.5 Hz untuk kedua sumbu** (8.54 elevasi, 8.43 azimuth). Kekakuan yang menentukan bukan hanya poros: kekakuan tahan stepper (150 N.m/rad) sebanding dengan poros 8 mm (317 N.m/rad). Bila poros dibuat kaku tak hingga, frekuensi elevasi hanya naik ke sekitar 10.4 Hz (hitungan analitik dari angka di atas), jadi memperkaku poros paling banyak menambah sekitar 20%; azimuth (poros diasumsikan 1600 N.m/rad) sudah hampir di batasnya, sekitar 8.8 Hz. Batasnya adalah kekakuan motor buka-loop itu sendiri.
2. **Respons satu microstep:** elevasi 1/16 overshoot 38% dari besar langkah dan menetap dalam 0.57 s; 1/8 overshoot 61%; 1/32 tanpa overshoot (elevasi) atau 18% (azimuth). Redaman viskos beban mengubahnya besar (elevasi 1/16: overshoot 47% pada bl 0.05 turun ke 7% pada bl 1), jadi redaman nyata perlu diukur.
3. **Riak saat bergerak kontinu:** pada laju rendah riak kira-kira satu sampai dua kali besar langkah (1/16: 0.14 sampai 0.21 deg puncak-ke-puncak); puncak di frekuensi alami: 1/8 mencapai 0.51 deg (elevasi) dan 0.72 deg (azimuth), 1/16 azimuth 0.21 deg pada 7.8 langkah/detik. Di atas sekitar 30 langkah/detik riak turun di bawah 0.03 deg lalu naik lagi pada laju sangat tinggi.
4. **Pada laju keyhole** (0.5 sampai 2 deg/s, dari `analisis_zenith`): microstep 1/16 memberi riak 0.02 sampai 0.19 deg (terburuk azimuth 1 deg/s dekat resonansi: 0.19 deg); 1/32 selalu di bawah 0.032 deg untuk laju >= 0.5 deg/s; 1/8 sampai 0.7 deg dan sebaiknya dihindari. Rekomendasi dari sini: microstep 1/16 atau 1/32.
5. **Angin (hembusan naik 0.3 s):** defleksi sudut beban elevasi 0.09, 0.37, 0.91 deg pada 5, 10, 15 m/s; azimuth 0.10, 0.41, 1.06 deg. **Lepas sinkron (buka-loop) pada 20 m/s untuk kedua sumbu** (torsi 3.15 N.m elevasi dengan gravitasi dan 3.8 N.m azimuth melebihi torsi tahan 3.0 N.m). Driver dikonfirmasi loop terbuka (lihat bagian Definisi dan keputusan), jadi risiko lepas langkah ini berlaku dan tidak terdeteksi oleh motor sendiri.
6. **Dinamika tidak membatasi loop LDR.** Konstanta waktu loop LDR sekitar 13 detik sedangkan mekanik menetap dalam sekitar 0.5 detik, dan riak 0.1 sampai 0.2 deg jauh di bawah lantai galat LDR (4 deg) dan setara derau sensor (0.44 deg). Hal yang relevan untuk akurasi justru defleksi angin 0.4 sampai 1 deg pada 10 sampai 15 m/s, sebanding dengan galat kontrol hibrid (0.5 sampai 3 deg).

## Keterbatasan dan asumsi (penting)

- Hampir semua parameter mekanik adalah asumsi atau tebakan massa (lihat tabel). Kesimpulan 1 sampai 3 bergantung pada kekakuan, redaman, dan gesekan nyata yang belum diukur.
- Gesekan Coulomb dihaluskan dengan tanh (lebar 2 mrad/s sekitar 0.11 deg/s). Untuk gerak sangat lambat (pelacakan matahari biasa, 0.004 deg/s) gesekan ini berperilaku seperti redaman viskos besar, dan stick-slip pada microstep halus (1/32) belum tertangkap. Respons satu langkah kecil karena itu tidak boleh dibaca sebagai bukti.
- Driver diasumsikan sumber arus ideal tanpa dinamika arus dan tanpa penurunan torsi terhadap kecepatan (tidak relevan pada laju di bawah 100 langkah/detik, tapi belum diuji). Tanpa backlash, tanpa kelenturan pillow block atau kopling, tanpa fleksibilitas panel.
- Angin hanya satu hembusan naik linear, tanpa spektrum turbulensi, tanpa faktor hembusan, dan koefisien gaya serta lengan adalah asumsi. Gravitasi dianggap tetap pada nilai terburuk 0.64 N.m untuk elevasi dan 0 untuk azimuth. J azimuth dipakai pada kemiringan 0 (turun sekitar 27% pada 45 deg).
- Kekakuan tahan stepper memakai Th nominal pada arus nominal; pada arus berkurang (mode hemat daya) kekakuannya turun.
- Pemeriksaan mandiri: kesetimbangan statis, frekuensi dering cocok dengan nilai eigen (kasus tanpa gesekan Coulomb), posisi akhir tercapai, ambang lepas sinkron di sekitar Th, dan beban gravitasi tetap tidak menimbulkan defleksi sendiri (pemeriksaan ini ditambah setelah ditemukan bug pada putaran pertama).

## Riwayat koreksi

Putaran pertama menghasilkan defleksi angin elevasi yang salah karena beban gravitasi hanya dipakai di kondisi awal, bukan di persamaan gerak. Ditemukan lewat pola yang tidak wajar (defleksi 0.616 deg sama untuk 5, 10, 15 m/s), diperbaiki, dan ditambahi pemeriksaan otomatis.

## Langkah lanjut

1. Ukur di alat: redaman dan gesekan beban (uji ayunan atau jatuh bebas) dan kekakuan puntir sendi (beban uji kecil). Tipe driver sudah diputuskan: loop terbuka.
2. Gabungkan dengan `kontrol_hibrid`: beri sudut aktual panel = perintah + defleksi angin dan riak, lalu lihat dampaknya ke RMSE dan aktivasi.
3. Tambahkan model stick-slip dan spektrum angin bila data tersedia.
