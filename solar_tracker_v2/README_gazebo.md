# Model tracker di Gazebo

Dibangkitkan oleh `build_model.py` (`python build_model.py` di WSL, venv `~/.venvs/solar`). Berkas: `solar_tracker.urdf` (model), `world.sdf` (dunia), `meshes/` (tiga mesh link).

## Isi model
- Rantai: `world` -> `base_link` -> (`azimuth_joint`, kontinu, sumbu -z, positif = searah jarum jam dari atas) -> `azimuth_link` -> (`elevation_joint`, sumbu +y, batas +-45 deg ASUMSI) -> `elevation_link` -> `panel`, empat `ldr_*`.
- Massa dan inersia dari CAD (`inersia/inersia_link.json`): base 2.13 kg, azimuth_link 1.44 kg, elevation_link 0.51 kg, panel 1.5 kg. Massa motor 1.2 kg dan panel 1.5 kg adalah ASUMSI.
- Sendi punya redaman 0.2 N.m.s/rad dan gesekan 0.05 N.m (asumsi, dari `dinamika/`). Tidak ada kelenturan poros atau model stepper di Gazebo: sendi kaku.
- Plugin: `JointStatePublisher` dan dua `JointPositionController` disetel ke kekakuan seri stepper dan poros (P 137.14 azimuth, 101.82 elevasi N.m/rad, I = D = 0, batas torsi 3 N.m; dari `stiffness_gains()` di `build_model.py`). Linier, tanpa tundaan, tanpa lepas langkah.
- Hanya visual, tanpa geometri tabrakan. Matahari di dunia hanyalah cahaya visual; irradiance fisis berasal dari pvlib.

## Menjalankan (di terminal WSL)
```
gz sim -r ~/mk/solar_tracker/world.sdf          # dengan GUI (WSLg), atau tambah -s untuk server saja
# di terminal lain, gerakkan sendi (radian):
gz topic -t /model/solar_tracker/joint/azimuth_joint/0/cmd_pos   -m gz.msgs.Double -p "data: 0.5"
gz topic -t /model/solar_tracker/joint/elevation_joint/0/cmd_pos -m gz.msgs.Double -p "data: 0.3"
gz topic -e -t /world/solar_world/model/solar_tracker/joint_state -n 1   # baca posisi sendi
```

## Yang sudah diuji (tanpa GUI)
- `check_urdf` dan `gz sdf -k world.sdf`: valid. Empat blok `<inertial>`.
- Server berjalan, topik kontrol dan `joint_state` ada. Perintah azimuth 0.5 dan elevasi 0.3 rad tercapai: 0.5034 dan 0.3053 rad dalam 14 detik.
- Normal panel dari pose Gazebo cocok dengan `tracker.fk` pada dua pose (q1=0.5, q2=0.3: selisih 0.31 deg; q1=2.0, q2=-0.4: selisih 0.85 deg). Jadi arah sumbu dan tanda putaran sama dengan kinematik yang dipakai di simulasi lain.
- **Belum diuji:** GUI Gazebo di WSLg (RViz sebelumnya butuh `QT_QPA_PLATFORM=xcb`, sudah ada di `~/.bashrc`), dan jembatan ROS 2.

## Jembatan ros_gz ke ROS 2
Berkas: `launch/sim.launch.py` (menjalankan Gazebo, jembatan, dan `robot_state_publisher`) dan `launch/bridge.yaml` (daftar topik).
```
source /opt/ros/lyrical/setup.bash
ros2 launch ~/mk/solar_tracker/launch/sim.launch.py              # dengan GUI Gazebo
ros2 launch ~/mk/solar_tracker/launch/sim.launch.py headless:=true   # server saja
# perintah sendi (rad), dari terminal lain:
ros2 topic pub --once /solar_tracker/azimuth_cmd   std_msgs/msg/Float64 "{data: 0.5}"
ros2 topic pub --once /solar_tracker/elevation_cmd std_msgs/msg/Float64 "{data: 0.3}"
ros2 topic echo /joint_states --once
```
| Topik ROS 2 | Jenis | Arah | Topik Gazebo |
|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | Gazebo ke ROS | `/clock` |
| `/joint_states` | sensor_msgs/JointState | Gazebo ke ROS | `/world/solar_world/model/solar_tracker/joint_state` |
| `/solar_tracker/azimuth_cmd` | std_msgs/Float64 | ROS ke Gazebo | `/model/solar_tracker/joint/azimuth_joint/0/cmd_pos` |
| `/solar_tracker/elevation_cmd` | std_msgs/Float64 | ROS ke Gazebo | `/model/solar_tracker/joint/elevation_joint/0/cmd_pos` |

`robot_state_publisher` memakai `/joint_states` dan jam simulasi (`use_sim_time`), sehingga TF (`/tf`) mengikuti gerak di Gazebo. Untuk melihatnya di RViz: tambahkan RobotModel dengan Description Topic `/robot_description`, Fixed Frame `world`.

**Teruji (tanpa GUI, `headless:=true`):** semua topik muncul; `/clock`, `/joint_states` mengalir; perintah azimuth 0.5 dan elevasi 0.3 rad lewat ROS tercapai (0.5027 dan 0.3045 rad dalam 14 detik); TF `base_link` ke `elevation_link` dari `robot_state_publisher` memberi normal panel (0.263, -0.145, 0.954), sama dengan `tracker.fk` pada sudut itu; log launch tanpa galat atau peringatan.
**Belum diuji:** GUI Gazebo dan RViz di WSLg, dan peningkatan ke kontrol sendi `ros2_control`.

## Node matahari pvlib (opsional)
`sun_command_node.py` memerintahkan sendi dari posisi matahari PENS pada tanggal pilihan, dengan waktu dipercepat. **Mati secara bawaan.**
```
ros2 launch ~/mk/solar_tracker/launch/sim.launch.py headless:=true sun_node:=true date:=2026-03-20 speed:=600
ros2 param set /sun_command_node enabled false        # jeda (sendi bisa digerakkan manual); true = lanjut
ros2 param set /sun_command_node date "'2026-06-21'"  # ganti tanggal saat berjalan; KUTIP wajib, tanpa kutip CLI membacanya sebagai tanggal YAML dan crash
```
Parameter: `date`, `speed`, `enabled`, `okta`, `loop`, `q2max`, `az_rate`, `tilt_rate`, `marker`. Keluaran: `/solar_tracker/azimuth_cmd`, `/solar_tracker/elevation_cmd`, `/sun/state`, `/sun_marker`. Node memakai venv `~/.venvs/solar-ros` (pvlib dan akses rclpy). Tanpa ROS: `python sun_command_node.py --selftest` mencetak tabel 13 tanggal 2026.
**Teruji:** tanpa opsi node tidak ada; selftest lolos (elevasi puncak 59.3 deg pada 21 Jun sampai 89.9 deg pada 2 Mar, matahari di selatan zenith Nov sampai Jan dan di utara Mar sampai Okt); jeda dan lanjut bekerja; ganti tanggal saat berjalan bekerja; tanggal salah ditolak (`date harus YYYY-MM-DD`); pada speed 1 selisih perintah dan sendi Gazebo (aktuator pid) 0.056 deg dan menyusut ke 0.006 deg. Pada speed tinggi dekat zenith panel tertinggal (efek keyhole dibesarkan oleh percepatan waktu).

## Aktuator stepper (`actuator:=stepper`)
`stepper_node.py` (torsi-sudut sinus seri poros, rotor tak bermassa, loop terbuka) memberi torsi ke sendi lewat `ApplyJointForce`; opsi angin: `wind:=tenang|berangin|badai`.
**Teruji dan benar:** selftest model torsi; defleksi gravitasi elevasi pada 40 deg: 40.33 deg vs analitik 40.32 deg; tanda torsi azimuth dan tanda kecepatan benar; langkah kecil dan besar.
**Koreksi:** kesimpulan lama ("lead dan damping memperburuk, tidak deterministik") ditarik. Itu artefak uji: server Gazebo (`gz-sim-main`) kebal SIGTERM sehingga simulasi menumpuk. Dengan SIGKILL dan simulasi baru per konfigurasi (laju pesan 1000 Hz), hasilnya deterministik.
**Hasil bersih (langkah 1 deg dan langkah besar az 28.65 deg, el 40 deg):**
- `lead=0`, `damping=0`: azimuth berosilasi terus (siklus batas) 2.8 deg pada 6.5 Hz, elevasi menetap 0.7 sampai 2.4 s. Penyebab: tundaan jembatan ROS (beberapa ms) pada pegas hampir tanpa redaman.
- `lead=0.005` s (bawaan sekarang): kedua sumbu menetap, azimuth 0.2 s (1 deg) dan 1.0 s (28.65 deg), lewatan 0.1 sampai 0.4 deg, osilasi akhir 0.
- `damping` 0.5 sampai 2 N.m.s/rad (tanpa lead) dan damping 1 + lead 0.005 juga menetap.
Lead dan damping adalah kompensasi numerik atas tundaan jembatan, bukan redaman fisik motor. Selisih akhir elevasi 0.36 deg pada 40 deg adalah defleksi gravitasi yang sah.
**Belum diuji:** angin, kecepatan matahari tinggi, dan jalan penuh sehari dengan aktuator stepper.
**Pelajaran uji:** simulasi baru untuk tiap konfigurasi (posisi beban menyimpan lepas langkah), satu pekerjaan simulasi pada satu waktu, matikan dengan `kill -9` pada `gz-sim-main` dan pastikan nol proses tersisa, verifikasi laju pesan sekitar 1000 Hz.

## Berikutnya
Uji angin dan pelacakan matahari sehari penuh dengan `actuator:=stepper`; GUI Gazebo dan RViz di WSLg; ganti parameter asumsi dengan ukuran alat nyata.

## Uji angin dan pelacakan sehari (aktuator stepper)
Alat: `rec_wind.py` (perintah 0, rekam `/stepper/state`, bandingkan dengan defleksi kuasi-statis dari kecepatan angin) dan `rec_day.py` (node matahari + stepper, galat pointing dari sudut beban lewat `tracker.fk`). Launch: `wind_seed:=N`, `loop:=false`, `rviz:=true`.
**Angin (170 s, pose datar, benih tetap):** badai seed 2 (v maks 14.7 m/s, tau az 2.06 N.m < 3): tanpa lepas langkah, korelasi 0.995 dengan prediksi, galat RMS 0.02 deg. Badai seed 4 (v maks 21.6 m/s, tau az 4.43 N.m > 3): lepas langkah azimuth diprediksi t=33.3 s dan teramati di Gazebo t=33.5 s, beban lari ratusan siklus; elevasi (tau maks 2.92 < 3) tetap aman. Hasil tenang dan berangin hilang karena keluaran skrip tertimpa, belum diulang.
**Sehari (speed 300x, tanpa angin, 5 tanggal 2026):** di dalam jangkauan sendi (zenith matahari <= 45 deg, batas kemiringan ASUMSI) galat perintah 0.000 deg dan galat beban rata 0.19 sampai 0.29 deg, maks 0.45 deg, yaitu defleksi gravitasi elevasi (loop terbuka tanpa koreksi). Di luar jangkauan (sekitar 46 sampai 57% sampel) galat besar karena batas kemiringan, bukan karena aktuator: rata-rata seluruh hari sekitar 9 deg. 2 Mar (zenith 0.1 deg): galat perintah maks 1.17 deg, beban 4.4 deg, azimuth tertinggal sampai 112 deg saat melewati zenith (efek keyhole, diperbesar kecepatan 300x), akhirnya menyusul tanpa lepas langkah.
**Lepas langkah:** 21 Des muncul sekali (selisih akhir beban terhadap perintah -14.4 deg di azimuth dan elevasi, kira-kira 2 siklus 7.2 deg), tetapi dua ulangan berikutnya bersih (selisih maks azimuth 0.12 deg, elevasi 0.42 deg). Penyebab kejadian tunggal belum diketahui (dugaan: transien awal saat Gazebo belum siap, tidak terbukti). Empat tanggal lain tanpa lepas langkah.

## RViz dan panel kontrol manual (untuk demo)
```
ros2 launch ~/mk/solar_tracker/launch/sim.launch.py headless:=true actuator:=stepper sun_node:=true rviz:=true panel:=true date:=2026-06-21 speed:=600
```
RViz (`launch/tracker.rviz`): model tracker, panah kuning arah matahari, garis oranye lintasan matahari hari itu, teks tanggal dan jam. `control_panel.py` (Tkinter, jendela terpisah) berisi slider dan pembacaan langsung:
1. **Mode:** otomatis (ikuti matahari) atau manual (geser sendi sendiri, matahari tetap ditampilkan).
2. **Matahari dan waktu:** tanggal 2026 (hari ke-1 sampai 365), jam WIB (geser untuk membekukan waktu), centang "waktu jalan otomatis" dan kecepatan, tutupan awan.
3. **Sendi manual:** azimuth q1 dan kemiringan q2.
4. **Stepper dan gangguan:** microstep, angin dan seed, torsi tambahan azimuth dan elevasi (lewati 3 N.m untuk memperlihatkan lepas langkah), tombol reset.
5. **Pembacaan:** jam, az dan el matahari, DNI GHI DHI, perintah sendi, sudut beban, galat pointing berwarna (hijau < 0.6 deg), selisih stepper dan penanda LEPAS LANGKAH, kecepatan angin.
Node matahari mendapat parameter baru `hour` (jam beku, -1 = jalan otomatis) dan `manual` (tidak mengirim perintah sendi). `QT_QPA_PLATFORM=xcb` diatur oleh launch.
**Teruji:** RViz tampil (tangkapan layar pengguna). Panel: uji otomatis `python control_panel.py --selftest` menggeser tanggal, jam, dan mode lewat layanan parameter dan node matahari bereaksi (tanggal berubah, jam beku di 9.50). Tampilan jendela panel belum saya lihat sendiri.

## Catatan tampilan (WSLg)
- **Panel:** `panel:=true` membuka panel web di `http://localhost:8765` (browser Windows); `panel:=tk` memakai jendela Tkinter, yang pernah tidak tampil di WSLg. Panel web diuji: halaman, data langsung, dan slider berfungsi di komputer pengguna.
- **RViz tidak tampil** (jendela terdaftar di Windows, tidak hung, tetapi transparan, tidak bisa dimunculkan lewat taskbar atau Alt+Tab): sembuh setelah `wsl --shutdown` di PowerShell Windows lalu menjalankan ulang. Dugaan penyebab: lapisan jendela WSLg yang lama hidup dan banyak menerima kill-paksa (belum terbukti). Bila terulang, ulangi `wsl --shutdown`.
- Beban RViz dikurangi: marker matahari 5 Hz, jejak lintasan tiap 2 detik, frame rate 15.
- `tata_layar_demo.ps1` memindahkan jendela browser panel ke separuh kiri layar (berhasil); untuk RViz tidak efektif, atur sendiri dengan Win+Panah.

## Energi pada posisi panel (panel web, bagian 6)
`control_web.py` menghitung irradiance di bidang panel dari sudut BEBAN (posisi nyata, termasuk lepas langkah dan defleksi): `POA = DNI cos(theta) + DHI (1 + cos q2)/2 + GHI albedo (1 - cos q2)/2` (model isotropik, albedo 0.25, sama dengan `sun.poa`). Daya = POA x luas 0.44 x 0.30 m x efisiensi 18% (ASUMSI). Ditampilkan: irradiance, cos sudut datang, daya sekarang, acuan panel datar dan tracker ideal (batas atas), serta energi terkumpul hari itu (Wh) dan keuntungan terhadap panel datar. Tombol Reset energi dan Unduh CSV (`/energi.csv`: tanggal, jam, az dan el matahari, q1 dan q2 beban, POA, daya, energi, acuan datar), satu baris per 10 menit simulasi. Energi terintegrasi hanya saat jam simulasi maju.
**Teruji:** POA identik dengan `pvlib get_total_irradiance` (selisih maks 0.0000 W/m2, 54 titik, 21 Jun); tracker ideal >= tracker >= 0. Tampilan di browser belum saya lihat sendiri.

## Lampu matahari di Gazebo (GUI)
Dengan `sun_node:=true headless:=false`, `sun_command_node` mengirim arah, warna, dan intensitas lampu `sun` ke Gazebo lewat layanan `/world/solar_world/light_config` (pesan `gz.msgs.Light`, perintah `gz service`, thread terpisah, sekitar 1 pembaruan per detik nyata). Arah cahaya = -sun_vec (frame dunia: x = Utara, y = Barat, z = atas), warna lebih hangat saat matahari rendah, intensitas 0.08 sampai 1 menurut elevasi. Hanya visual: irradiance tetap dari pvlib. Matikan dengan `ros2 param set /sun_command_node gz_light false`.
**Perhatian:** `intensity` wajib diisi; nilai bawaan protobuf 0 membuat dunia gelap gulita. Server Gazebo dan GUI mati bila sesi `wsl.exe` yang memulainya selesai, jadi jalankan `ros2 launch` di terminal yang tetap terbuka.
**Teruji:** tangkapan layar pada 7 Okt 2026 jam 08:00 dan 15:00: bayangan panel berpindah dari satu sisi ke sisi lain dan panel berganti sisi miring; selftest `sun_command_node` lolos (arah cahaya, intensitas malam).

## Lingkungan Gazebo (dunia `world.sdf` dan `world_stepper.sdf`)
Visual saja (tidak memengaruhi fisika selain tanah). Frame dunia = frame dasar tracker: +x = Utara (ASUMSI arah rel), +y = Barat, +z = atas.
- Langit biru dan tanah hijau 40 x 40 m, landasan abu-abu 1.6 x 1.6 m di bawah tracker.
- Penanda kompas 3.2 m dari pusat: Utara = kerucut merah (+x), Timur = kotak hijau (-y), Selatan = kotak putih (-x), Barat = kotak kuning (+y); garis abu-abu = sumbu Utara-Selatan dan Timur-Barat.
- Bola kuning `sun_ball` mengikuti posisi matahari (jarak 3.2 m), diperbarui `sun_command_node` lewat layanan `/world/solar_world/set_pose` bersama lampu (bila `sun_node:=true`); di bawah horizon bola masuk ke tanah.
- Kamera GUI awal diatur `launch` (hanya `headless:=false`): dari Selatan menghadap Utara, sejajar sumbu engsel, jadi Timur di kanan layar dan panel tampak berguling kanan-kiri.
**Teruji:** tangkapan layar 7 Okt 2026 jam 09:00 dan 15:00: bola dan bayangan berpindah sisi, panel miring ke sisi matahari. Posisi kamera akhir (x = -1.1, z = 0.5) belum dilihat (hanya x = -1.5 dan -2.4 yang dilihat).
Belum: awan (okta) belum memengaruhi lampu, lintasan matahari belum digambar di Gazebo, panah galat belum ada.

## Angin pada aktuator pid (`wind_node.py`)
Mode `pid` kini selalu menjalankan `wind_node.py`: perintah `/solar_tracker/*_cmd` (dari node matahari, panel, atau skrip) DITERUSKAN ke `/solar_tracker/*_cmd_gz` (yang dijembatani ke Gazebo, lihat `bridge.yaml`) setelah ditambah pergeseran angin. Tanpa node ini sendi tidak bergerak di mode pid.
- Dasar: gaya kontrol posisi `F = K (cmd - q)` linier dengan `K` = kekakuan seri stepper dan poros, jadi torsi luar `tau` setara persis dengan menggeser perintah `delta = tau / K` (azimuth 137.14, elevasi 101.82 N.m/rad). Cara lain (ApplyJointForce bersama pengendali posisi) tidak berefek: Gazebo tidak menjumlahkannya (diuji, selisih 0.0000).
- Angin sama dengan `stepper_node` (`coupled_sim.wind_series`, `dyn_model.wind_torque`; Cn dan lengan ASUMSI): `wind:=none|tenang|berangin|badai`, `wind_seed:=N`, dapat diubah saat berjalan (`ros2 param set /wind_node wind badai`) atau dari panel web (bagian 4). Torsi uji tambahan: `extra_torque_az` dan `extra_torque_el`.
- Batas: kejenuhan 3 N.m dan lepas langkah TIDAK dimodelkan di mode pid (pakai `actuator:=stepper`, eksperimental). `/wind/state` = [t, v, tau_az, tau_el, delta_az_deg, delta_el_deg].
**Uji hari 7 Okt 2026** (`speed:=600`, `loop:=false`, jendela rekam sekitar 08.9 sampai 17.0, hanya saat matahari terjangkau zenith <= 45 deg dan elevasi > 5 deg; 5 Hz):
| angin | v rata-rata, maks (m/s) | pergeseran maks az, el (deg) | galat pointing rata-rata, p95, maks (deg) | rugi cos rata-rata |
|---|---|---|---|---|
| none | 0, 0 | 0, 0 | 0.282, 0.628, 1.040 | 0.0017 % |
| berangin | 8.0, 11.9 | 0.559, 0.497 | 0.279, 0.585, 0.920 | 0.0017 % |
| badai | 13.0, 19.9 | 1.574, 1.400 | 0.461, 1.026, 1.213 | 0.0044 % |
Galat dasar 0.28 deg berasal dari defleksi gravitasi dan ketertinggalan perintah pada waktu dipercepat 600x. Kerugian energi karena angin sangat kecil di mode pid (tanpa lepas langkah). Panel web (bagian 4) mengubah angin dan menampilkan v serta pergeseran; diuji lewat HTTP (`wind=badai` memberi v 11.2 m/s, `none` kembali 0). Tampilan halaman di browser belum saya lihat.

## Peringatan: `wind_node` mode pid MEREMEHKAN risiko angin
`wind_node.py` menyuntikkan angin sebagai pergeseran perintah `delta = tau / K`, **tanpa kejenuhan torsi tahan 3 N.m**, jadi sendi selalu mengikuti dan tidak pernah lepas sinkron. Model Python `dinamika/osilasi_energi.py` memakai kejenuhan yang benar dan menunjukkan bahwa pada angin `berangin` (puncak 11.9 m/s) momen angin azimuth melewati 3 N.m selama 10.8 % waktu, dan azimuth lepas sinkron di 4 dari 4 jendela uji. Untuk menilai risiko angin, pakai `osilasi_energi.py` atau `actuator:=stepper`, bukan angka dari mode `pid`.

## Peredam osilasi di `sun_command_node` (profil trapesium + input shaper)
Perintah sendi kini melewati dua tahap sebelum diterbitkan: **profil trapesium** (`az_rate`/`tilt_rate` dan `az_accel`/`tilt_accel`, bawaan 90 deg/s dan 300 deg/s2) lalu **input shaper** (`shaper`: `none`, `zv`, atau `zvd`; bawaan `zvd`). Impuls shaper dihitung dari frekuensi dan redaman tiap sumbu di `dinamika/dyn_model.py` (azimuth 4.79 Hz, elevasi 6.89 Hz). Node berdetak 100 Hz (`RATE_HZ`), jadi jeda shaper dibulatkan ke 10 ms; ZVD tahan terhadap galat sebesar itu, ZV kurang.
Ubah saat berjalan: `ros2 param set /sun_command_node shaper none|zv|zvd`.
**Catatan penting:** shaper TIDAK berguna saat menjejak pelan. Satu microstep tidak bisa dipecah jadi dua setengah langkah, jadi perintahnya sama saja setelah dibulatkan. Gunanya saat gerak besar: lompat jam, ganti tanggal, nyala awal, parkir.

### Validasi model Python terhadap Gazebo (`rec_shaped.py`)
`rec_shaped.py` menyuntikkan lintasan perintah berbentuk langsung ke `*_cmd_gz` memakai jam simulasi, lalu membandingkan dengan `dinamika/osilasi_energi.py`. Matikan `wind_node` dulu (`pkill -f wind_node.py`) supaya tidak ada dua penerbit.

| Uji | langsung (Gazebo / model) | scurve | zv | zvd |
|---|---|---|---|---|
| Azimuth 90 deg, lewatan % | 52.43 / 52.41 | 0.88 / 0.77 | 3.07 / 0.00 | 0.26 / 0.00 |
| Azimuth 5 deg, lewatan % | 82.60 / 82.78 | 15.46 / 15.77 | 0.32 / 0.40 | 0.97 / 0.85 |
| Elevasi 20 deg, lewatan % | 5.98 / 66.03 | 1.47 / 1.43 | 0.61 / 0.01 | 0.00 / 0.00 |

Azimuth cocok sangat baik (52.43 vs 52.41 persen). **Elevasi `langsung` tidak cocok** karena URDF membatasi kecepatan sendi elevasi `velocity="1"` (1 rad/s) sedangkan model Python tidak: perintah mentah membuat kecepatan melewati 4 rad/s, lalu Gazebo mengeremnya secara sempurna. Stepper nyata tidak punya rem itu, ia akan lepas langkah. Untuk strategi yang menghormati batas laju (scurve, zv, zvd) kecepatan tetap di bawah 0.52 rad/s dan keduanya cocok dalam 0.3 poin persen.

### Uji lompat jam 09:00 ke 15:00 (azimuth berputar 181 deg), 5 ulangan
| shaper | lewatan median | lewatan terburuk |
|---|---|---|
| `none` (profil saja) | 0.58 % (1.05 deg) | **10.71 % (19.35 deg)** |
| `zvd` | 0.11 % (0.20 deg) | 1.27 % (2.30 deg) |

ZVD menekan kasus terburuk sekitar 8 kali dan membuatnya **dapat diramalkan**. Tanpa shaper, sisa getaran sangat bergantung pada kapan tepatnya ramp berakhir terhadap fase ayunan, sehingga sebarannya lebar (0.43 sampai 10.71 persen pada uji yang sama).

### Perbaikan: `use_sim_time` untuk node matahari
`sun_command_node` sebelumnya berjalan di jam dinding sedangkan Gazebo di jam simulasi (sekitar 85 persen real-time), sehingga profil gerak dan shaper tidak sinkron dengan plant dan hasilnya tidak dapat diulang. Sekarang `sim.launch.py` memberinya `use_sim_time:=true`, sama seperti `stepper_node` dan `wind_node`. Sisa sebaran yang masih ada berasal dari jitter timer Python 100 Hz saat beban tinggi.

## Sensor dan diagnostik dalam simulasi (`sensor_node.py`)
Sebelumnya simulasi hanya menerbitkan sudut sendi dan posisi matahari, padahal model LDR, model listrik panel, dan hasil kalibrasi sudah ada di `kontrol_hibrid/`. `sensor_node.py` membawa semuanya masuk. Node ini jalan otomatis bila `sun_node:=true`.

**Masukan:** `/joint_states` (sudut BEBAN sebenarnya dari Gazebo) dan `/sun/state`.
**Keluaran:** `/tracker/diag` (`std_msgs/String` berisi JSON, 5 Hz) dan `/ldr/state` (`Float64MultiArray`: v1, v2, v3, v4, ex, ey) untuk perekaman laju tinggi.

JSON dipilih supaya menjelaskan dirinya sendiri. Deret `Float64MultiArray` yang panjang mudah salah urutan, dan itu pernah terjadi di proyek ini pada `/stepper/state`.

Isi `/tracker/diag`:
| Kelompok | Isi |
|---|---|
| `matahari` | az, el, zen, dni, dhi, ghi |
| `sendi` | cmd_q1, cmd_q2, beban_q1, beban_q2, dan selisihnya |
| `galat` | `perintah` (batas jangkauan 45 deg + ketertinggalan), `beban` (ditambah defleksi, ayunan, lepas langkah), `dinamik` (selisih keduanya), `terjangkau` |
| `ldr` | empat tegangan ADC, ex, ey, dan ketidakcocokan yang dipakai |
| `listrik` | POA efektif (dengan IAM), suhu sel, Isc, Imp, Vmp, Pmp, Voc dari model dioda tunggal MOS C-50-18-M |
| `energi` | Wh tracker, panel datar, tracker ideal, dan keuntungan persen |

**Argumen launch baru:** `mism` (ketidakcocokan antar LDR, bawaan 0.03 = belum dikalibrasi) dan `kalibrasi` (true = ketidakcocokan dihapus, mewakili prosedur A yang sempurna; lihat `kontrol_hibrid/kalibrasi_ldr_results.md`). Keduanya memungkinkan memperagakan langsung temuan bahwa LDR tak terkalibrasi lebih buruk daripada tanpa LDR.

**Panel web bagian 7** menampilkan semuanya: batang tegangan keempat LDR, nilai listrik, uraian galat, dan energi dari model dioda. Bagian 6 yang lama tetap ada dan memakai perkiraan POA kali luas kali efisiensi; bagian 7 lebih teliti karena memakai kurva I-V dan suhu sel.

**Teruji:** jalan 7 Okt 2026 jam 07:39, topik `/tracker/diag` dan `/ldr/state` terbit, panel menerima JSON lengkap. Contoh: matahari el 35.8 deg (zenith 54.2, DI LUAR jangkauan), galat perintah dan beban sama sama 8.96 deg dengan galat dinamik -0.0001 deg, LDR [2.43, 2.33, 2.40, 2.42] V dengan ex +0.0207 (matahari di sisi +x, benar karena kemiringan terpotong 45 deg), POA 777 W/m2, suhu sel 54.3 C, Imp 2.11 A pada 15.6 V, Pmp 33.1 W. Tampilan halaman di browser belum saya lihat sendiri.

### Pengukuran ayunan (osilasi) pada laju penuh
`/tracker/diag` terbit 5 Hz, sedangkan dering sendi 4.79 Hz (azimuth) dan 6.89 Hz (elevasi). **Mencuplik 5 Hz sinyal 4.8 Hz adalah aliasing total**, jadi sebelum ini kolom `galat.dinamik` dan `sendi.d_q1` hanya memperlihatkan defleksi dan ketertinggalan, BUKAN ayunannya.
Sekarang `sensor_node` menyimpan `/joint_states` pada laju penuh (1000 Hz, jendela 4 detik) dan menghitung statistik ayunan di sana, lalu menerbitkan RINGKASANNYA saja pada 5 Hz: `osilasi.az` dan `osilasi.el` berisi `pp` (puncak ke puncak), `rms`, dan `f` (frekuensi utama), ditambah `hz` dan `n` sebagai bukti laju cuplik yang benar-benar terpakai. Gerak jejak yang lambat dibuang dengan rata-rata bergerak (penapis lolos tinggi di atas 2 Hz), jadi tidak perlu menyamakan waktu dengan topik perintah.
**Teruji (selftest):** dering buatan 4.79 Hz beramplitudo 0.8 deg puncak ke puncak di atas jejak 0.05 deg/s terukur frekuensinya dalam 5 persen dan amplitudonya dalam 15 persen; tanpa dering, amplitudo terbaca nol.
**Teruji (Gazebo, 7 Okt, speed 600):** saat menjejak dekat transit zenith az pp 1.90 deg rms 0.51 deg pada 5.19 Hz, el pp 0.72 deg pada 6.57 Hz (prediksi nilai eigen 4.79 dan 6.89 Hz). Setelah lompat jam 11 ke 15 (azimuth berputar 181 deg): az pp 4.53 deg dan el pp 5.44 deg, lalu meluruh ke 1.26 deg dan akhirnya 0 dalam sekitar 9 detik. Laju cuplik terpakai 992 sampai 1000 Hz dari 4000 cuplikan.
Panel web bagian 7 menampilkannya, lengkap dengan peringatan merah bila laju cuplik turun di bawah 30 Hz, dan bagian 9 menambah grafik ayunan puncak ke puncak terhadap waktu.

## Angin nyata berarah (Open-Meteo) menggantikan angin sintetis
`wind_node.py` bawaannya kini `sumber:=openmeteo`: data angin NYATA Surabaya per jam dari `angin/angin_openmeteo.py`, lengkap dengan ARAH. Pilihan lain `sintetis` (model lama yang buta arah, untuk pembanding) dan `none`.

**Preset sintetis sudah dibuang.** Data kini 11 tahun (2015 sampai 2026, 102984 jam). Maksimum hembusan tahunan Surabaya sangat stabil: rata 16.28 m/s pada 10 m, simpangan baku 1.29, rentang 13.81 sampai 17.81. Maksimum 11 tahun hanya 1.06 kali maksimum 1 tahun, jadi data nyata sudah mencakup rentang penuh dan preset karangan tidak diperlukan. Untuk menguji DI LUAR rekaman, pakai pengali `skala`.

| Parameter `wind_node` | Arti |
|---|---|
| `tanggal` | hari yang diputar ulang dari data, mis. `2026-01-15` |
| `alpha` | eksponen profil angin (0.16 lapangan terbuka, 0.30 bawaan, 0.35 kampus rapat) |
| `pakai_hembusan` | `true` memakai nilai hembusan, bukan rata-rata jam |
| `skala` | pengali uji tekan terhadap angin nyata (1.0 apa adanya) |

Argumen launch: `skala_angin`, `alpha_angin`, `tanggal_angin`. `stepper_node` memakai angin yang sama lewat `angin:=true`.

**Teruji (13:00, 7 Okt 2026, angin dari 105 deg Timur menuju 285 deg Barat):**
| Pengali | v di panel | Torsi az, el | % torsi tahan |
|---|---|---|---|
| 1.0 | 1.48 m/s | -0.0018, -0.0135 N.m | 0.45 % |
| 2.0 | 2.97 m/s | -0.0071, -0.0539 N.m | 1.80 % |
| 3.0 | 4.45 m/s | -0.0158, -0.1211 N.m | 4.04 % |
| 3.0 + hembusan | | -0.0910, -0.7037 N.m | 23.46 % |

Torsi sebanding kuadrat kecepatan (rasio 4 dan 9 terbaca persis), jadi pengalinya berperilaku benar.

`/wind/state` = `[t, v_panel, tau_az, tau_el, geser_az_deg, geser_el_deg, arah_DARI_deg, v10, hembusan10, hembusan_panel]`. `arah_DARI_deg` adalah bearing tempat angin DATANG; angin bertiup MENUJU arah itu ditambah 180 derajat.

**Model torsi berarah.** Gaya pada pelat datar bekerja tegak lurus permukaan, `F = 0.5 rho v^2 A Cn (w_hat . n_hat)`. Dengan pergeseran pusat tekanan `e_c` searah chord dan `e_s` searah span:
```
tau_elevasi = -e_c * F
tau_azimuth = -e_s * F * sin(q2)
```
Jadi **torsi azimuth NOL saat panel datar** dan tumbuh dengan kemiringan. Itu benar secara fisika: gaya tegak lurus panel datar tidak dapat memuntir sumbu tegak. Model lama tidak punya sifat ini, dan lengan azimuth 0.152 m yang dipakainya adalah tebakan tanpa dasar.

**Panel web bagian 10** menampilkan kompas dengan panah arah tiupan, teks DARI dan MENUJU (derajat dan nama mata angin), kecepatan di 10 m dan di panel, hembusan, torsi tiap sumbu, pergeseran perintah, dan persentase terhadap torsi tahan 3 N.m.

**Teruji di Gazebo (7 Okt 2026):**
| Jam | Angin | Kecepatan | Kemiringan | Torsi az, el | % torsi tahan |
|---|---|---|---|---|---|
| 08:00 | dari 94 deg (Timur) menuju 274 deg (Barat) | 1.44 m/s di 10 m, 0.50 di panel | +45.0 deg | +0.00055, +0.00257 N.m | 0.09 % |
| 13:00 | dari 109 deg (Timur) menuju 289 deg (Barat) | 3.92 m/s di 10 m, 1.37 di panel | +25.9 deg | -0.00147, -0.01124 N.m | 0.37 % |

Perhatikan tandanya berbalik antara pagi dan siang karena orientasi panel berubah terhadap arah angin. Model lama tidak bisa menghasilkan itu.
