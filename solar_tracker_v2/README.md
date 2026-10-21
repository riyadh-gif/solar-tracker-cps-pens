# Cara menjalankan simulasi (desain baru, solar_tracker_v2)

Desain lama (`solar_tracker/`) sudah dihapus. Semua ada di sini.

Isi desain ini: reduksi gigi 5:1 pada kemiringan, motor 57HSE1.5N loop tertutup, azimuth berbatas
(bukan kontinu lagi), angin nyata Open-Meteo 11 tahun, hujan nyata Open-Meteo 11 tahun dengan parkir
datar otomatis, dan model kaca basah serta pendinginan.

---

## 0. Sekali saja: siapkan lingkungan

```bash
# symlink supaya path tanpa spasi (Gazebo dan ROS tidak suka spasi)
ln -s "/mnt/c/Magister PENS/Semester 2/cyberphysicalsystem/modelkinematik" ~/mk

# venv studi luring (tanpa ROS)
python3 -m venv ~/.venvs/solar
~/.venvs/solar/bin/pip install pvlib scipy trimesh pysolar numpy pandas

# venv node ROS (butuh rclpy dari paket sistem)
python3 -m venv --system-site-packages ~/.venvs/solar-ros
~/.venvs/solar-ros/bin/pip install pvlib
```

## 1. Bangun model dari mesh CAD

**Wajib setelah clone baru.** Mesh tampilan 27 MB sengaja tidak masuk git, jadi harus dibangkitkan:

```bash
cd ~/mk && ~/.venvs/solar/bin/python build_model_v2.py
```

Membaca `desainbaru/extracted/.../meshes/*.stl`, menulis `solar_tracker_v2/solar_tracker_v2.urdf`
dan tiga mesh link. Kalau folder `extracted/` belum ada, ekstrak dulu
`desainbaru/Solar2_URDF_ROS2.rar` ke sana.

Data hujan juga dibangkitkan sekali (CSV-nya sudah ikut git, jadi normalnya tidak perlu):

```bash
~/.venvs/solar/bin/python cuaca/hujan.py --impor
```

## 2. BERSIHKAN PROSES SISA. Jangan dilewati.

Ini jebakan paling sering. `pkill gz-sim-main` **tidak** membunuh node Python-nya. Kalau ada sisa,
beberapa node menerbitkan perintah ke topik yang sama dan hasilnya kacau tanpa pesan galat apa pun.

```bash
ps -eo pid,args | grep -E "[w]ind_node|[s]un_command_node|[s]ensor_node|[e]kf_node|[g]z-sim-main|[p]arameter_bridge|[r]obot_state_publisher" \
  | awk '{print $1}' | xargs -r kill -9
```

Periksa setelahnya: tiap topik harus punya **tepat satu** penerbit.

```bash
ros2 topic info /sun/state | grep -i "publisher count"
```

## 3. Jalankan

Selalu di terminal yang **tetap terbuka**: Gazebo mati kalau sesi `wsl.exe` yang memulainya selesai.

```bash
source /opt/ros/lyrical/setup.bash
cd ~/mk
```

**Paling sederhana, hanya plant:**
```bash
ros2 launch solar_tracker_v2/launch/sim_v2.launch.py
```

**Lengkap dengan matahari, sensor, EKF dan panel web:**
```bash
ros2 launch solar_tracker_v2/launch/sim_v2.launch.py \
    sun_node:=true ekf:=true panel:=true rviz:=true \
    date:=2026-10-07 speed:=600 kalibrasi:=true
```

**Hari hujan, untuk melihat parkir datar bekerja:**
```bash
ros2 launch solar_tracker_v2/launch/sim_v2.launch.py \
    headless:=true sun_node:=true ekf:=true \
    date:=2026-02-18 okta:=8 speed:=600
```
18 Feb 2026 adalah hari siang terbasah tahun itu (18,1 mm). Panel akan rata saat hujan turun.

**Angin kuat:**
```bash
ros2 launch solar_tracker_v2/launch/sim_v2.launch.py \
    sun_node:=true skala_angin:=3.0 tanggal_angin:=2026-10-07
```

## 4. Argumen launch

| Argumen | Bawaan | Arti |
|---|---|---|
| `headless` | `false` | `true` = server Gazebo tanpa GUI |
| `sun_node` | `false` | node matahari pvlib + sensor_node |
| `date` | `2026-10-04` | tanggal; dipakai matahari DAN deteksi hujan |
| `speed` | `600` | detik simulasi per detik nyata |
| `okta` | `0` | tutupan awan 0..8 (8 = mendung tebal, seperti hujan) |
| `skala_angin` | `1.0` | pengali angin NYATA; 3.0 sekitar 83 % torsi tahan |
| `alpha_angin` | `0.30` | eksponen profil angin |
| `tanggal_angin` | `2026-10-07` | hari angin nyata yang diputar ulang |
| `mism` | `0.03` | ketidakcocokan antar LDR; 0.03 = belum dikalibrasi |
| `kalibrasi` | `false` | `true` = ketidakcocokan LDR dihapus |
| `ekf` | `false` | jalankan `ekf_node` |
| `pakai_ekf` | `false` | `sun_command_node` MEMAKAI koreksi EKF (loop tertutup) |
| `rviz`, `panel` | `false` | tampilan; `panel:=true` buka `http://localhost:8765` |

## 5. Melihat hasilnya

```bash
ros2 topic echo --once --field data /sun/state       # jam, az, el, ghi, dni, dhi, q1_cmd, q2_cmd
ros2 topic echo --once --field position /joint_states
ros2 topic echo --once --field data /tracker/diag    # JSON: LDR, listrik, galat, osilasi, basah
ros2 topic echo --once --field data /wind/state
ros2 topic echo --once /ekf/state
```

Perintah sendi manual (lewat wind_node, satuan radian):
```bash
ros2 topic pub --once /solar_tracker/azimuth_cmd   std_msgs/msg/Float64 "{data: 0.5}"
ros2 topic pub --once /solar_tracker/elevation_cmd std_msgs/msg/Float64 "{data: 0.3}"
```

Ubah parameter saat berjalan (tanggal WAJIB dikutip ganda lalu tunggal):
```bash
ros2 param set /sun_command_node date "'2026-06-21'"
ros2 param set /sun_command_node parkir_hujan false
ros2 param set /wind_node skala 3.0
```

## 6. Jebakan yang sudah terbukti

- **Jangan `pkill` satu node** yang dikelola `ros2 launch`: seluruh launch ikut mati.
- **Jangan `pkill -f "ros2 launch"` atau `pkill -f sim_v2.launch`** dari dalam skrip: polanya cocok
  dengan baris perintahnya sendiri dan membunuh skrip itu juga.
- Proses server Gazebo bernama `gz-sim-main`, bukan `gz sim`.
- `use_sim_time` wajib di semua node, dan launch sudah mengurusnya.
- WSL di sini tidak punya internet; unduh data lewat PowerShell di Windows.
- Panel web membaca ulang `control_web.html` tiap permintaan, jadi cukup segarkan browser.

## 7. Aktuator

Hanya mode `pid` (kontrol posisi Gazebo disetel ke `K_ser`). Mode `stepper` tidak disediakan untuk
desain ini: reduksi gigi membuat pegas magnet yang dirujuk ke beban (1875 N·m/rad) jauh lebih kaku
daripada poros (163,8), sehingga kurva torsi kuasi-statis bernilai ganda dan model sinus loop terbuka
di `stepper_node.py` tidak sah pada sumbu kemiringan. Lihat `stepper_node.sumbu_sah()`.
