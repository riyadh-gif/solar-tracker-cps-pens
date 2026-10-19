# Solar Tracker Dua Sumbu PENS: Simulasi dan Cyber-Physical System

Digital Model tervalidasi untuk solar tracker dua sumbu (azimuth dan kemiringan) di Politeknik Elektronika
Negeri Surabaya, titik `-7.276006, 112.794327`. Panel MOS C-50-18-M 50 W, dua motor stepper 57HSE3.0N
terhubung langsung ke poros tanpa gearbox, empat LDR bersekat di tepi panel.

Tesis Magister Terapan, mata kuliah Cyber-Physical Systems.

![Panel kontrol web: mode, matahari, sendi manual, stepper dan angin](image.png)

Panel kontrol berbasis web (`solar_tracker/control_web.py`, buka `http://localhost:8765`). Semua parameter
simulasi dapat digeser saat Gazebo berjalan: tanggal, jam, kecepatan waktu, tutupan awan, microstep,
pengali angin nyata, dan torsi gangguan.

![Gazebo: panel miring mengikuti matahari, bola matahari dan bayangan](image%20copy.png)

Model di Gazebo. Panel berguling ke kanan dan kiri terhadap rel (bukan jungkat-jungkit), bola merah adalah
penanda arah matahari dari pvlib, dan bayangan menunjukkan sudut datang yang sebenarnya.

![Rangka alat nyata: dua stepper 57HSE3.0N bertumpuk, rel aluminium ekstrusi](WhatsApp%20Image%202026-09-28%20at%2011.38.49.jpeg)

Alat nyata, 28 September 2026. Dua motor 57HSE3.0N bertumpuk: yang bawah memutar azimuth, yang atas
memiringkan panel terhadap rel. Keduanya terhubung langsung ke poros tanpa gearbox, dan belum ada enkoder.
Panel, LDR, dan kabel daya belum terpasang. Inilah yang harus diukur untuk mengisi parameter asumsi.

---

## Apa yang sudah ada

| Lapisan | Isi | Status |
|---|---|---|
| Kinematika | vektor normal panel, kinematika balik, Jacobian, efek keyhole | tervalidasi terhadap Gazebo |
| Dinamika | Euler-Lagrange dua sendi, inersia dari CAD, gravitasi, gesekan | tervalidasi terhadap Gazebo |
| Aktuator | stepper loop terbuka, torsi-sudut sinusoidal, kuantisasi microstep | model Python dan Gazebo |
| Kontrol | umpan maju pvlib SPA, profil trapesium, input shaper ZVD | terukur di Gazebo |
| Sensor | LDR empat kuadran, model dioda tunggal panel, arus sebagai sensor | model, belum ada alat nyata |
| Penaksir | EKF menaksir simpangan sudut, pengganti enkoder | terukur, loop tertutup opsional |
| Gangguan | angin nyata Open-Meteo 11 tahun, berarah | tervalidasi terhadap Gazebo |
| Pemantauan | panel web, RViz, grafik waktu, kompas angin | berjalan |

### Hasil terukur

| Besaran | Nilai | Cara diperoleh |
|---|---|---|
| Posisi matahari | selisih < 0,011 derajat | pvlib SPA vs astropy, SolTrack, Pysolar |
| Kinematika normal panel | selisih 0,14 sampai 0,34 derajat | TF Gazebo vs `tracker.fk` |
| Defleksi gravitasi pada q2 = 0 | 0,161 terukur vs 0,155 prediksi | Gazebo |
| Frekuensi alami azimuth | 4,79 prediksi, 5,19 terukur Hz | nilai eigen vs Gazebo 1 kHz |
| Lewatan langkah 1 derajat | 93,3 % teori, 90 % terukur | analitik vs Gazebo |
| ZVD menekan lewatan terburuk | 19,35 menjadi 2,30 derajat | Gazebo, lima ulangan |
| Rugi energi dari osilasi | 0,007 % | model dua sumbu 1 kHz |
| Energi harian 7 Okt | 431,0 Wh tracker vs 362,3 Wh panel datar | model dioda tunggal |
| Margin angin terburuk 11 tahun | 9,2 % torsi tahan | Open-Meteo + model berarah |
| EKF dengan LDR terkalibrasi | 2,52 menjadi 0,18 derajat | simulasi hari penuh |
| Pemulihan lepas langkah 7,2 derajat | 350 detik | simulasi hari penuh |

---

## Dokumen yang harus dibaca lebih dulu

| Berkas | Isi |
|---|---|
| **`sistem_cps_tracker_pens.pdf`** | **Mulai dari sini.** 19 halaman, seluruh persamaan fisika dengan nama modelnya, arsitektur CPS tiga lapis, dan posisi terhadap Digital Twin. Sumber LaTeX ada di `.tex`, versi markdown di `.md`. |
| `parameter_kinematik.md` | Dokumen induk, 18 bagian. Setiap parameter, asalnya, dan riwayat keputusan. |
| `parameter_asumsi_dan_pengukuran.md` | Daftar ~31 parameter yang masih diasumsikan, dan cara mengukurnya di alat nyata. |
| `solar_tracker/README_gazebo.md` | Rincian model Gazebo, topik, dan perintah. |

---

## Menjalankan simulasi

Lingkungan: **Windows + WSL (Ubuntu)**, ROS 2 Lyrical, Gazebo (gz sim 10.5.0).

### Persiapan sekali saja

```bash
# symlink agar path tanpa spasi (Gazebo dan ROS tidak suka spasi)
ln -s "/mnt/c/Magister PENS/Semester 2/cyberphysicalsystem/modelkinematik" ~/mk

# venv untuk studi luring (tanpa ROS)
python3 -m venv ~/.venvs/solar
~/.venvs/solar/bin/pip install pvlib scipy trimesh pysolar numpy pandas

# venv untuk node ROS (butuh akses rclpy dari paket sistem)
python3 -m venv --system-site-packages ~/.venvs/solar-ros
~/.venvs/solar-ros/bin/pip install pvlib

source /opt/ros/lyrical/setup.bash
```

### Membangun ulang model dari CAD

```bash
cd ~/mk && ~/.venvs/solar/bin/python build_model.py
```

Membaca `New folder (2)/solar_stl/parts_v2/*.stl`, menghasilkan `solar_tracker/solar_tracker.urdf` dan tiga
mesh di `solar_tracker/meshes/`. Massa dan inersia diambil dari `inersia/inersia_link.json`.

### Menjalankan

```bash
# dasar: Gazebo + jembatan ros_gz + robot_state_publisher
ros2 launch ~/mk/solar_tracker/launch/sim.launch.py

# lengkap: matahari pvlib, sensor, EKF, RViz, panel kontrol web
ros2 launch ~/mk/solar_tracker/launch/sim.launch.py \
    sun_node:=true ekf:=true rviz:=true panel:=true \
    date:=2026-10-07 speed:=600 kalibrasi:=true

# model stepper loop terbuka (bisa lepas langkah) dan angin nyata 3x
ros2 launch ~/mk/solar_tracker/launch/sim.launch.py \
    actuator:=stepper sun_node:=true panel:=true skala_angin:=3.0

# loop tertutup: perintah matahari DIKOREKSI oleh taksiran EKF
ros2 launch ~/mk/solar_tracker/launch/sim.launch.py \
    sun_node:=true ekf:=true pakai_ekf:=true panel:=true
```

Panel kontrol terbuka sendiri di browser Windows pada `http://localhost:8765`.

### Argumen launch

| Argumen | Bawaan | Arti |
|---|---|---|
| `actuator` | `pid` | `pid` = kontrol posisi Gazebo disetel ke kekakuan seri stepper; `stepper` = model stepper loop terbuka, bisa lepas langkah |
| `sun_node` | `false` | node perintah matahari pvlib |
| `date`, `speed`, `okta` | `2026-10-04`, `600`, `0` | tanggal, detik simulasi per detik nyata, tutupan awan |
| `skala_angin` | `1.0` | pengali data angin NYATA. `3.0` memberi sekitar 83 % torsi tahan |
| `alpha_angin` | `0.30` | eksponen profil angin (0,16 lapangan terbuka, 0,35 kampus rapat) |
| `tanggal_angin` | `2026-10-07` | hari angin nyata yang diputar ulang (data 2015 sampai 2026) |
| `mism` | `0.03` | ketidakcocokan antar LDR. `0.03` = belum dikalibrasi |
| `kalibrasi` | `false` | `true` = ketidakcocokan LDR dihapus (prosedur A sempurna) |
| `ekf` | `false` | jalankan `ekf_node`, terbitkan `/ekf/state` |
| `pakai_ekf` | `false` | `sun_command_node` MEMAKAI koreksi EKF (loop tertutup). Butuh `ekf:=true` |
| `rviz`, `panel`, `headless` | `false` | tampilan dan mode tanpa GUI |

### Jebakan yang sudah ditemukan

Ini nyata, bukan teori. Menghemat berjam-jam.

- **Gazebo mati bila sesi `wsl.exe` yang memulainya selesai.** Jalankan `ros2 launch` di terminal yang tetap terbuka.
- **Jangan mematikan satu node yang dikelola `ros2 launch`.** `pkill -9 -f control_web.py` menjatuhkan seluruh launch.
- Proses server Gazebo bernama `gz-sim-main`, bukan `gz sim`. `pkill -f "gz sim"` tidak membunuhnya.
- `ros2 param set` dengan tanggal butuh kutip ganda lalu tunggal: `"'2026-06-21'"`. Tanpa itu CLI membacanya sebagai tanggal YAML dan crash.
- **`use_sim_time` wajib pada semua node.** Tanpa itu profil gerak dan input shaper berjalan di jam dinding sedangkan plant di jam simulasi (Gazebo sekitar 85 % real-time), dan lewatan menjadi tidak dapat diulang.
- **WSL di sini tidak punya akses internet, Windows punya.** Unduh data lewat PowerShell, olah di WSL.
- Panel web membaca ulang `control_web.html` tiap permintaan, jadi menyunting HTML cukup disegarkan di browser.

---

## Peta berkas

### Inti model

| Berkas | Isi |
|---|---|
| `sun.py` | posisi matahari dan irradiance PENS (pvlib SPA, Ineichen, Kasten-Czeplak, Erbs) |
| `tracker.py` | `fk`, `sun_vec`, `ik_track`. `HEAD = 90 derajat`, `Q2_MAX = 45 derajat` |
| `build_model.py` | parts_v2 CAD ke tiga mesh link dan URDF |
| `inersia/inersia_cad.py` | massa, pusat massa, tensor inersia tiap link dari mesh. Keluaran `inersia_link.json` |
| `dinamika/dyn_model.py` | dinamika satu sumbu, model stepper, kekakuan poros |
| `dinamika/osilasi_energi.py` | osilasi dua sumbu 1 kHz dan dampaknya pada energi |

### Kontrol hibrid dan sensor

| Berkas | Isi |
|---|---|
| `kontrol_hibrid/hybrid_sim.py` | model LDR empat kuadran dan kontrol hibrid |
| `kontrol_hibrid/panel_listrik.py` | model dioda tunggal lima parameter, dicocokkan ke pelat nama |
| `kontrol_hibrid/sensor_arus.py` | rantai pengukuran arus dan dua batas mendasarnya |
| `kontrol_hibrid/fusi_sensor.py` | ketidakpastian tiap sensor, pemisahan derau dan bias, bobot |
| `kontrol_hibrid/ekf.py` | tapis Kalman diperluas, penaksir simpangan sudut |
| `kontrol_hibrid/kalibrasi_ldr.py` | prosedur kalibrasi A (cahaya baur) dan B (regresi harian) |

### Angin nyata

| Berkas | Isi |
|---|---|
| `angin/angin_openmeteo.py` | muat, koreksi tinggi, model torsi berarah |
| `angin/angin_pens_2015-2026.csv` | 102.984 jam data Open-Meteo untuk titik PENS |

Kolom CSV: `waktu`, `v10_ms` (kecepatan rata-rata satu jam pada 10 m), `hembusan10_ms` (puncak sesaat dalam
jam itu), `arah_deg` (bearing kompas **dari mana** angin datang; bertiup menuju arah itu + 180).

### Simulasi Gazebo dan node ROS 2

| Berkas | Isi |
|---|---|
| `solar_tracker/world.sdf`, `world_stepper.sdf` | dunia Gazebo, gravitasi 10 m/s2 |
| `solar_tracker/launch/sim.launch.py` | launch utama, semua argumen di atas |
| `solar_tracker/sun_command_node.py` | perintah dari pvlib, profil trapesium, input shaper ZVD |
| `solar_tracker/stepper_node.py` | model stepper loop terbuka memberi torsi ke sendi |
| `solar_tracker/wind_node.py` | angin nyata berarah, terbitkan `/wind/state` |
| `solar_tracker/sensor_node.py` | LDR, dioda tunggal, energi, uraian galat ke `/tracker/diag` |
| `solar_tracker/ekf_node.py` | penaksir simpangan ke `/ekf/state` dan `/ekf/koreksi` |
| `solar_tracker/control_web.py` + `.html` | panel kontrol browser |
| `solar_tracker/rec_*.py` | skrip perekam untuk percobaan tertentu |

---

## Status: Digital Model, belum Digital Twin

Menurut taksonomi Kritzinger:

| Tingkat | Aliran data | Posisi |
|---|---|---|
| **Digital Model** | tidak ada, manual | **di sini** |
| Digital Shadow | otomatis, fisik ke digital | belum |
| Digital Twin | otomatis, dua arah | belum |

Yang ada adalah Digital Model **tervalidasi silang tiga jalur**: nilai eigen analitik, model Python dua
sumbu, dan Gazebo saling cocok. Yang belum ada adalah hubungan ke alat fisik.

**Jembatannya pendek.** `sensor_node` menghasilkan data sensor dan menerbitkannya ke `/tracker/diag`;
`ekf_node` memakannya dari topik itu. Mengganti `sensor_node` dengan node pembaca perangkat keras yang
menerbitkan pesan sama membuat seluruh rantai di belakangnya berjalan tanpa diubah.

---

## Yang harus dikerjakan selanjutnya

Diurutkan dari paling bernilai.

### 1. Uji peluruhan bebas pada alat nyata

Percobaan tunggal paling bernilai. Dorong panel satu langkah, rekam ayunannya, lalu dari satu rekaman itu
diperoleh sekaligus:

- frekuensi alami, dari periode ayunan
- redaman sendi `b`, dari rasio puncak berurutan lewat `zeta = -ln(rasio) / 2 pi`
- gesekan Coulomb `tau_c`, dari simpangan sisa yang tidak kembali ke nol
- kekakuan poros `k`, dari frekuensi dan inersia yang sudah diketahui dari CAD

Empat parameter asumsi terbesar hilang sekaligus. Semua hasil dinamika di repo ini bergantung padanya.

### 2. Kurva LDR dan kalibrasinya

Ukur kurva lux terhadap tegangan di meja untuk mendapat `gam` dan `L0`, lalu jalankan prosedur A saat
mendung tebal. **Ini bukan pilihan.** LDR yang tidak terkalibrasi membuat EKF **lebih buruk** daripada tidak
memakai LDR sama sekali (5,63 derajat berbanding 2,52 derajat), karena bias LDR tidak dapat dibedakan dari
salah pasang mekanik.

### 3. Instrumentasi menuju Digital Shadow

1. Mikrokontroler membaca empat LDR dan arus panel, kirim ke serial.
2. Node pembaca serial menerbitkan ke `/tracker/diag` dengan format yang sama seperti `sensor_node`.
3. Tutup loop ke driver stepper. **Ini Digital Twin.**

### 4. Ukur yang lain

Lihat `parameter_asumsi_dan_pengukuran.md`. Yang paling menentukan setelah tiga di atas: batas kemiringan
maksimum (asumsi 45 derajat), arah hadap rel dengan kompas (asumsi Utara), albedo tanah di lokasi (asumsi
0,25, menggeser puncak arus sampai 2,75 derajat), dan massa motor (asumsi 1,2 kg).

### 5. Yang belum dimodelkan

Suku Coriolis (besarnya sudah dihitung: 0,0000 % saat menjejak, 3,4 % saat slew 90 derajat/s), latensi dan
jitter jaringan, event-triggered control, identifikasi parameter daring, indeks kesehatan dari hanyutan
gesekan, biaya kelelahan struktur, backlash kopling, dan mode getar rangka.

---

## Peringatan untuk yang melanjutkan

**Hasil yang sudah ditarik.** Jangan dipakai lagi:

1. Batas angin tarik-lepas azimuth 9,6 m/s. Bergantung lengan momen yang ditebak; rentang sebenarnya 6,5
   sampai 39,4 m/s.
2. Pergeseran puncak arus +2,6 derajat. Memakai irradiance yang tidak konsisten. Yang benar: -2,75 sampai
   +2,75 derajat menurut albedo.
3. Skenario angin sintetis (`tenang`, `berangin`, `badai`). Dua sampai tiga kali terlalu berat, terbukti dari
   data 11 tahun. Sudah dibuang dari jalur langsung, diganti pengali terhadap data nyata.

**Hasil yang kedaluwarsa.** Dihitung dengan panel 1,5 kg berukuran 0,44 x 0,30 m, g = 9,81, dan sumbu engsel
lama. Jalankan ulang skripnya sebelum dikutip: `dinamika/dyn_results.md`, `gabungan/coupled_results.md`,
`gabungan/stow_results.md`, `kontrol_hibrid/hybrid_results.md`, isi `robustness/` dan `skenario_cuaca/`.
Dua angka stale juga ada di `solar_tracker/README_gazebo.md` (panel 1,5 kg dan sumbu elevasi `+y`); yang
berlaku adalah panel 2,55 kg dan sumbu `(1,0,0)` searah rel.

`gabungan/coupled_sim.wind_series` ditandai **USANG** tetapi dipertahankan agar studi luring lama tetap jalan.

**Yang otoritatif** adalah `sistem_cps_tracker_pens.pdf` dan `parameter_kinematik.md`.

---

## Tidak ada di repo ini

- `jurnalrandom/` berisi PDF jurnal pihak ketiga. Tidak diunggah karena berhak cipta dan repo ini publik.
  Pemetaan isinya terhadap sistem ini ada di `sistem_cps_tracker_pens.pdf`.
- Lingkungan Python (`.venv/`), log percobaan mentah, dan dump CAD besar yang sudah digantikan `parts_v2`.
  Lihat `.gitignore`.

## Lisensi

Belum ditentukan. Karya tesis, hubungi penulis sebelum dipakai ulang.
