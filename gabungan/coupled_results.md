# Penggabungan dinamika dan kontrol hibrid (kuasi-statis, model tereduksi)

Skrip: `coupled_sim.py` (`--check` self-check, tanpa argumen menjalankan semua eksperimen, sekitar 35 detik dengan 14 proses, `--table` mencetak tabel lepas langkah). Tabel lengkap: `coupled_results_tables.md`, data mentah: `coupled_runs.csv` (336 simulasi), tabel ODE: `slip_table_*.json`. Hari 2026-10-04, salah pasang 3/2 deg, dua kelas sensor: default (r=1, ketidakcocokan 3%) dan baik (r=2, 1%).

## Cara penggabungan (model tereduksi)

ODE dinamika (dt 0.1 ms) tidak bisa dijalankan 12 jam bersama simulasi kontrol (dt 2 s), jadi tiga hasil dinamika dipakai sebagai model tereduksi:
1. **Defleksi kuasi-statis:** asin(tau/Th)/Nr + tau/k per sumbu, dari momen angin dan gravitasi (gravitasi elevasi 0.895 sin q2 N.m dari CAD). Dicek terhadap ODE di `self_check` (selisih kurang dari 0.06 deg).
2. **Kejadian melewati torsi tahan:** dihitung dari deret torsi angin per 0.25 s. Dampaknya dilihat dari tabel ODE (lihat di bawah). Kejadian hanya DICATAT, tidak diterapkan ke posisi.
3. **Uji pemulihan:** lepas langkah kecil (1, 3, 10 siklus listrik = 7.2, 21.6, 72 deg) disuntikkan pada kedua sumbu di sepertiga hari.
Angin sintetis (proses OU + hembusan Poisson) di tiga skenario: tenang (rata-rata 3 m/s), berangin (7 m/s), badai (11 m/s, uji tekan). Posisi fisik = perintah terkuantisasi + salah pasang + defleksi + lepas langkah. Tanpa angin, gravitasi, dan suntikan, hasilnya identik dengan simulasi tanpa dinamika (diuji di `self_check`).

## Temuan

1. **Lepas langkah berarti kehilangan sumbu, bukan galat kecil.** Tabel ODE: satu kejadian melewati torsi tahan menghasilkan sekitar 15 sampai 915 siklus (azimuth) dan 167 sampai 959 siklus (elevasi), yaitu ratusan sampai ribuan derajat. Beban lari karena redaman viskos yang diasumsikan (0.2 N.m.s/rad) tidak mampu menangkapnya kembali; pada elevasi gravitasi 0.64 N.m terus mendorong. Besar angkanya bergantung pada asumsi redaman, tetapi kesimpulan "bukan galat beberapa derajat" kokoh.
2. **Kecepatan angin yang melewati torsi tahan (asumsi Cn 1.2 dan lengan 0.066 dan 0.10 m): sekitar 17.8 m/s (azimuth) dan 19.4 m/s (elevasi, dengan gravitasi terburuk).** Frekuensi kejadian per hari siang (10 seed): tenang 0, berangin sekitar 5 (azimuth) dan 0.7 (elevasi), badai lebih dari 1000. Skenario badai adalah uji tekan yang tidak realistis, bukan perkiraan iklim PENS.
3. **Di bawah torsi tahan, dinamika menambah galat kecil.** Astronomi saja (salah pasang 3/2): RMSE 2.40 deg tanpa dinamika, 2.58 deg dengan gravitasi (defleksi elevasi bergantung kemiringan, sekitar 0.2 sampai 0.36 deg), 2.58 (tenang), 2.59 (berangin). Hibrid bias dengan sensor baik: 0.72 (tanpa dinamika), 0.72 (gravitasi), 0.74 (berangin). Dengan sensor default semua kontroler berbasis LDR tidak terpengaruh (RMSE 3.8 sampai 4.5 deg). Defleksi angin tidak mengubah kesimpulan Fase 3.
4. **Baris badai di tabel E1 tidak valid sebagai galat.** Kejadian melewati torsi tahan di sana hanya dicatat, sehingga RMSE (2.66 untuk astro) mengabaikan kehilangan sumbu yang sebenarnya akan terjadi.
5. **Pemulihan dari lepas langkah (E2, sensor baik, cerah):**
   - 7.2 deg (1 siklus): LDR-saja, ala Saputra, dan hibrid bias pulih dalam sekitar 8 sampai 10 detik (hibrid bias paling halus, RMSE setelah kejadian 0.71 deg).
   - 21.6 deg (3 siklus): LDR-saja pulih dalam sekitar 0.6 menit, ala Saputra dalam sekitar 42 menit, **hibrid bias tidak pulih** (galat akhir 17.8 deg) karena memori koreksi dibatasi 10 deg per sumbu (dmax), padahal lepas langkah 21.6 deg lebih besar dari batas itu.
   - 72 deg (10 siklus): **tidak ada kontroler yang pulih**; matahari keluar dari bidang pandang sensor bersekat (sinyal jenuh pada sekitar 45 deg) dan astronomi saja tidak punya cara mengoreksi.
   - Astronomi saja tidak pernah pulih pada ukuran lepas langkah berapa pun.
   - Dengan sensor default (lantai galat sekitar 4 deg) ambang pemulihan 3 deg sendiri lebih kecil dari lantai sensor, jadi "tidak pulih" pada beberapa baris cerah default adalah artefak ambang, bukan kegagalan kontroler.
   - Pada awan sebagian pemulihan jauh lebih lambat (puluhan menit) atau gagal, karena LDR tidak dipercaya saat berawan.

## Implikasi desain (turunan dari model, bergantung pada asumsi)

- **Parkir saat angin kencang (stow):** ambang pemicu sebaiknya jauh di bawah sekitar 17.8 m/s (misalnya sekitar 12 sampai 15 m/s, perlu margin hembusan), karena satu kejadian melewati torsi tahan sudah menghilangkan sumbu.
- **Acuan posisi mutlak** (saklar posisi rumah atau saklar batas) wajib ada untuk menyetel ulang setelah kejadian, karena LDR hanya memulihkan lepas langkah kecil dan hanya saat cerah.
- **Batas memori koreksi hibrid (dmax)** harus ditetapkan terhadap besar lepas langkah yang ingin dipulihkan, atau diganti logika "kembali ke rumah lalu cari matahari" untuk galat besar.
- **Margin torsi:** reduksi roda gigi (misalnya 5:1) mengalikan torsi di poros dan menurunkan torsi yang terlihat motor, sehingga kecepatan angin lepas sinkron naik sekitar akar N kali. Itu hitungan sederhana, belum disimulasikan, dan perlu diperiksa terhadap laju keyhole (3.6 deg/s) dan resolusi.
- Alternatif lain: driver loop tertutup (encoder) menghilangkan risiko lepas langkah, tetapi diputuskan loop terbuka, jadi desain di atas yang berlaku.

## Keterbatasan (penting)

- Angin sintetis dan seluruh koefisien (Cn 1.2, lengan 0.066 dan 0.10 m, tanpa faktor hembusan, arah sama untuk dua sumbu) adalah asumsi. Frekuensi kejadian bergantung kuadrat pada kecepatan angin dan pada Cn dan lengan.
- Dampak lepas langkah bergantung pada redaman dan gesekan beban yang belum diukur, dan model tidak memasukkan perubahan momen aerodinamika saat panel berputar. Angka ratusan siklus adalah batas atas kasar, bukan prediksi.
- Penyuntikan lepas langkah hanya searah di dua sumbu, pada satu waktu, satu hari, dan awan sintetis (cerah 1 seed, sebagian 2). Tidak ada interval kepercayaan. Hasil LDR-saja pada awan sebagian sangat peka terhadap kondisi awal (RMSE berubah-ubah antar skenario angin tanpa penyebab fisik), jadi bandingkan hanya tren.
- Model kuasi-statis: tidak ada getaran, resonansi, atau backlash dalam simulasi 12 jam. Gravitasi memakai model momen 0.895 sin(q2) dari CAD (massa panel dan motor asumsi).
- Parameter kontroler (kp, dead, kcthr, dmax) tidak dituning ulang untuk kehadiran dinamika.

## Perubahan pada `hybrid_sim.py`

Ditambah kait opsional `ext` pada `simulate` (offset posisi fisik) dan pengembalian deret galat bila `ext` diberikan. Perilaku bawaan tanpa `ext` tidak berubah (diuji ulang: angka acuan identik).
