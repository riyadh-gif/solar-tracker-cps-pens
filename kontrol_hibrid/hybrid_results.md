# Fase 3: model LDR 4 kuadran dan kontrol hibrid (kinematik, kuasi-statis)

Skrip: `hybrid_sim.py` (jalankan ulang: `python hybrid_sim.py`, sekitar 1 menit; `--quick` untuk uji cepat). Tabel lengkap: `hybrid_results_tables.md`, data mentah E1: `hybrid_runs_e1.csv`.
Titik panel PENS (-7.276006, 112.794327). Hari: 2026-10-04 (matahari lewat 2.9 deg dari zenith) dan 2026-06-21. Langkah 2 detik, hanya elevasi matahari >= 5 deg. Galat pointing = sudut antara normal panel dan matahari, dihitung hanya saat matahari terjangkau (zenith <= 45 deg).

## Model

- **Sensor:** 4 LDR di tengah sisi panel, sekat silang di tengah. Bayangan sekat pada LDR di sisi lawan: fraksi = min(1, r tan(galat)), r = tinggi sekat / lebar LDR. Cahaya tiap LDR = berkas langsung x cos(sudut datang) x (1 - bayangan) + difus (sama untuk keempatnya). Pembagi tegangan jenuh V = Vcc x/(1+x), x = (lux/L0)^gamma, lalu derau ADC 2 mV dan kuantisasi 12 bit. Ketidakcocokan antar LDR sigma 3% (tetap per seed). Sinyal e = (V1 - V2)/(V1 + V2) per pasang.
- **Kinematik dan batas:** IK kontinu dari `tracker.py`, kemiringan dibatasi 45 deg (asumsi), laju azimuth 0.5 deg/s (hasil `analisis_zenith`), kuantisasi 1.8/16 deg per microstep, azimuth tidak dikoreksi LDR bila |q2| < 5 deg (dekat zenith).
- **Cuaca:** awan sintetis per seed. Cerah (faktor 1), sebagian (awan datang dan pergi, lama cerah rata-rata 10 menit, lama berawan 5 menit, faktor 0.15 sampai 0.5, transisi 40 detik), mendung (AR(1) di sekitar 0.25). GHI ke DNI dan DHI dengan Erbs.
- **Salah pasang mekanik:** panel sebenarnya berada pada (perintah + bias). Astronomi tidak tahu bias, LDR melihatnya. Tiga kasus: 0/0, 3/2, 6/4 deg (azimuth/kemiringan).
- **Kontroler:** `astro` (pvlib saja); `ldr` (LDR saja, mulai dari posisi rumah menghadap timur); `saputra` (hibrid ala Saputra 2026: LDR dipakai bila cahaya sensor melewati ambang tetap, selain itu astronomi, tanpa memori); `bias` (usulan: astronomi sebagai feedforward, LDR hanya mengoreksi memori offset saat indeks kecerahan sensor terhadap langit cerah pvlib >= 0.75, dengan anti-windup dan batas 10 deg).

## Temuan (rata-rata antar seed, tabel E1, E2, E3)

1. **Galat LDR dibatasi sensor, bukan algoritma.** Kemiringan sinyal hanya sekitar 0.0013 per derajat saat cerah (pembagi jenuh) dan derau e 0.0006. Dengan sensor default (r=1, ketidakcocokan 3%), semua kontroler berbasis LDR berhenti di RMSE sekitar 4 sampai 4.5 deg di hari cerah, hampir tidak bergantung pada salah pasang. Astronomi saja: 0.04 deg tanpa salah pasang, 2.4 deg (3/2), 4.8 deg (6/4).
2. **LDR baru menguntungkan bila salah pasang lebih besar dari lantai galat sensor.** Pada sensor default itu terjadi hanya di kasus 6/4 (cerah: astro 4.84, bias 3.93). Dengan sensor lebih baik (E2, salah pasang 3/2, 4 Okt): r=2 dan ketidakcocokan 1%: hibrid bias RMSE 0.72 (cerah) dan 0.54 (sebagian) deg vs astro 2.4; ketidakcocokan 0: 0.13 deg. Jadi kualitas dan kalibrasi sensor menentukan, dan itu harus diukur di alat nyata.
3. **LDR saja gagal saat berawan.** Awan sebagian: RMSE 17 sampai 19 deg (P95 sekitar 40 deg) dan aktivasi motor 3 sampai 5 kali lipat. Mendung: 34 sampai 42 deg, dan energi lebih buruk dari astronomi (4 Okt, tanpa salah pasang: -7.4% vs -4.5% terhadap panel tetap).
4. **Hibrid ala Saputra memperbaiki sebagian besar, hibrid bias lebih tahan.** Awan sebagian (4 Okt, sensor default): saputra 3.8 sampai 5.3 deg (satu kasus 10.4 deg pada seed 0), bias 2.3 sampai 2.9 deg. Mendung: keduanya kembali ke astronomi tanpa kerugian. Di awan sebagian tanpa salah pasang, hibrid bias bisa sedikit lebih buruk dari astronomi murni (2.9 vs 0.04 deg) karena offset sensor ikut masuk memori.
5. **Sensitivitas deadzone (E3).** Hibrid ala Saputra sangat bergantung pada deadzone: cerah, RMSE 4.3 deg (e=0.002) naik ke 12.5 deg (e=0.02) sementara aktivasi turun dari 1330 ke 319. Hibrid bias tidak: RMSE 3.85 turun ke 2.40 deg (sama dengan astronomi, karena LDR hampir tidak mengoreksi). Feedforward astronomi membuat pilihan deadzone tidak kritis.
6. **Pengaruh ke energi kecil.** Di hari cerah semua kontroler berada dalam 19.3 sampai 19.7% terhadap panel tetap (galat 4 deg hanya menghilangkan sekitar 0.25%, cos 4 deg = 0.9976). Perbedaan nyata hanya di awan sebagian (LDR saja sekitar 1.3 sampai 1.5 poin lebih rendah) dan mendung (3 poin lebih rendah). Manfaat utama LDR bukan energi, melainkan mengoreksi salah pasang dan (bagi kontroler tanpa feedforward) menghemat aktivasi motor, dengan bayaran galat beberapa derajat.

## Asumsi dan keterbatasan (penting)

- **Semua parameter sensor adalah asumsi**: r, ketidakcocokan 3%, L0 20 klux, gamma 0.7, derau 2 mV, ADC 12 bit. Kesimpulan nomor 1 dan 2 bergantung langsung pada r dan ketidakcocokan (lihat E2). Perlu diukur dari alat: kurva e terhadap sudut dan selisih antar LDR.
- Awan sintetis (proses Markov), bukan data nyata. Seed sedikit (sebagian 4, mendung 2), hanya rata-rata, tanpa interval kepercayaan.
- Kuasi-statis: tidak ada dinamika stepper, kekakuan poros, atau angin (Fase 4). Motor dianggap mencapai perintah.
- Batas kemiringan 45 deg, arah hadap +x alat (kini Timur, HEAD = 90 derajat), dan ukuran panel masih asumsi. Laju kemiringan 1 deg/s adalah asumsi.
- Gain energi memakai irradiance bidang (albedo 0.25, langit isotropik), bukan daya listrik panel.
- Indeks kecerahan hibrid bias memakai model langit cerah pvlib (Ineichen, turbiditas klimatologi) yang mungkin terlalu tinggi untuk Surabaya, jadi ambang 0.75 perlu dikalibrasi dari data.
- Pemeriksaan mandiri hanya memastikan tanda koreksi LDR benar (galat mengecil), bukan kebenaran fisik model sensor.

## Langkah lanjut yang disarankan

1. Ukur sensor nyata: e terhadap sudut, offset antar LDR, dan ada tidaknya sekat (menentukan r).
2. Kalibrasi offset LDR otomatis (belum dibuat). Batasannya: offset sensor adalah konstanta di frame panel, sedangkan salah pasang azimuth muncul sebagai galat sumbu y sebesar bias x sin(q2) yang berubah sepanjang hari, jadi hanya salah pasang azimuth yang bisa dipisahkan lewat regresi. Salah pasang kemiringan dan offset sumbu x sama-sama konstan di frame panel, sehingga tidak bisa dipisahkan oleh geometri saja. Pemisahannya butuh acuan mutlak, misalnya inklinometer (MPU6050 seperti di paper Wondafrash) atau saklar posisi rumah.
3. Ganti awan sintetis dengan deret clear-sky index nyata dari PENS.
4. Lanjut ke Fase 4 (dinamika) memakai kontroler terbaik sebagai beban uji.
