#!/usr/bin/env bash
# Pasang proyek ini di Raspberry Pi 5 pada /home/pi/projekssf, untuk ROS 2.
#
# PRASYARAT OS. Skrip ini mengandaikan Ubuntu Server 24.04 LTS ARM64, BUKAN Raspberry Pi OS.
# Alasannya: ROS 2 Jazzy hanya punya paket apt resmi untuk Ubuntu 24.04. Raspberry Pi OS adalah
# Debian bookworm dan tidak punya binari ROS 2 resmi, jadi pilihannya tinggal bangun dari sumber
# (berjam-jam di Pi) atau Docker (menambah lapisan di perangkat kecil).
#
# Tulis kartu SD dengan Raspberry Pi Imager:
#   Pilih OS -> Other general purpose OS -> Ubuntu -> Ubuntu Server 24.04 LTS (64-bit)
#   Di pengaturan lanjutan, SET NAMA PENGGUNA KE  pi  supaya /home/pi benar-benar ada.
#   Nyalakan SSH sekalian.
#
# Jalankan DI RASPBERRY PI:
#   curl -fsSL https://raw.githubusercontent.com/riyadh-gif/solar-tracker-cps-pens/main/pasang_di_raspi.sh | bash
set -euo pipefail

TUJUAN="${TUJUAN:-/home/pi/projekssf}"
REPO="https://github.com/riyadh-gif/solar-tracker-cps-pens.git"
ROS_DISTRO_WANTED="${ROS_DISTRO_WANTED:-jazzy}"

echo "== 1. Periksa lingkungan =="
echo "  arsitektur : $(uname -m)"
. /etc/os-release 2>/dev/null || true
echo "  OS         : ${PRETTY_NAME:-tidak diketahui}"
if [ "${ID:-}" != "ubuntu" ]; then
  echo
  echo "  PERINGATAN: ini bukan Ubuntu. Paket ROS 2 $ROS_DISTRO_WANTED tidak akan tersedia lewat apt."
  echo "  Tulis ulang kartu dengan Ubuntu Server 24.04 LTS ARM64 (lihat catatan di kepala skrip ini),"
  echo "  atau jalankan ROS 2 dalam Docker."
  echo
  read -r -p "  Lanjutkan tanpa ROS 2 (hanya inti model Python)? [y/N] " j
  [ "${j:-N}" = "y" ] || exit 1
  TANPA_ROS=1
fi

echo "== 2. Paket dasar =="
sudo apt update
sudo apt install -y git python3-venv python3-dev python3-pip i2c-tools

echo "== 3. Aktifkan I2C untuk ADS1115 dan INA226 =="
# Pi 5 di Ubuntu: I2C biasanya sudah ada di /boot/firmware/config.txt
if ! grep -q '^dtparam=i2c_arm=on' /boot/firmware/config.txt 2>/dev/null; then
  echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt >/dev/null
  echo "  I2C dinyalakan, PERLU REBOOT setelah skrip ini selesai"
fi
sudo usermod -aG i2c,dialout "$USER" || true
echo "  (keluar-masuk sesi atau reboot agar keanggotaan grup berlaku)"

if [ -z "${TANPA_ROS:-}" ]; then
  echo "== 4. ROS 2 $ROS_DISTRO_WANTED =="
  if [ ! -d "/opt/ros/$ROS_DISTRO_WANTED" ]; then
    sudo apt install -y software-properties-common curl
    sudo add-apt-repository -y universe
    sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt update
    # ros-base: tanpa RViz dan Gazebo, karena Pi tidak menjalankan simulasi
    sudo apt install -y "ros-$ROS_DISTRO_WANTED-ros-base" python3-colcon-common-extensions
  else
    echo "  sudah terpasang"
  fi
fi

echo "== 5. Ambil kode ke $TUJUAN =="
if [ -d "$TUJUAN/.git" ]; then
  git -C "$TUJUAN" pull --ff-only && echo "  diperbarui"
else
  mkdir -p "$(dirname "$TUJUAN")"
  git clone --depth 1 "$REPO" "$TUJUAN"
fi

echo "== 6. Lingkungan Python =="
# --system-site-packages WAJIB agar rclpy dari apt terlihat venv
VENV="$HOME/.venvs/ssf"
python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/pip" install --upgrade pip >/dev/null
"$VENV/bin/pip" install pvlib numpy pandas scipy smbus2

echo "== 7. Uji inti model tanpa perangkat keras =="
cd "$TUJUAN"
"$VENV/bin/python" - <<'PY'
import sys
for s in (".", "dinamika", "kontrol_hibrid", "cuaca"):
    sys.path.insert(0, s)
import sun, tracker, dyn_model as D, panel_listrik as PL, hujan as HJ
df = sun.sun_day("2026-10-07", freq="1h")
i = int(df["el"].values.argmax())
ax = D.AXES["elevasi"]
print(f"  matahari puncak 7 Okt : el {df['el'].values[i]:.1f} deg")
print(f"  kemiringan            : N={ax.N:.2f} K_ser={ax.K_ser():.2f} Th_L={ax.Th_L:.2f} N.m")
print(f"  transmisi kaca basah  : {PL.transmisi_basah():.4f}")
print(f"  jam hujan terekam     : {len(HJ.muat())}")
print("  OK: inti model jalan")
PY

echo "== 8. Periksa I2C (abaikan bila sensor belum tersambung) =="
sudo i2cdetect -y 1 2>/dev/null | sed 's/^/  /' || echo "  i2cdetect gagal, mungkin perlu reboot dulu"

cat <<EOF

== Selesai ==
  kode : $TUJUAN
  venv : $VENV

MENJALANKAN DI ALAT NYATA:
  source /opt/ros/$ROS_DISTRO_WANTED/setup.bash
  cd $TUJUAN/solar_tracker_v2
  \$VENV/bin/python sun_command_node.py --ros-args -p date:=2026-10-07 &
  \$VENV/bin/python sensor_hw_node.py  --ros-args -p tiruan:=false &
  \$VENV/bin/python ekf_node.py &

  sensor_hw_node.py menggantikan sensor_node.py: ia membaca 4 LDR lewat ADS1115 (0x48) dan
  arus panel lewat INA226 (0x40), lalu menerbitkan /tracker/diag dengan format yang SAMA.
  Karena formatnya sama, ekf_node dan panel web jalan tanpa diubah.
  Dengan -p tiruan:=true node tetap jalan tanpa perangkat keras, berguna untuk menguji di meja.

MASIH HARUS DIKERJAKAN:
  1. Node penggerak: mengirim pulsa langkah dan arah ke driver loop tertutup lewat GPIO.
     Belum ada. stepper_node.py adalah model simulasi, bukan pengemudi perangkat keras.
  2. Sensor hujan sungguhan. Sekarang kebijakan parkir memakai data historis Open-Meteo;
     di alat, pemicunya harus sensor.
  3. Kalibrasi LDR prosedur A saat mendung tebal. Tanpa ini, LDR membuat EKF LEBIH BURUK
     daripada tidak memakai LDR sama sekali (5,63 deg berbanding 2,52 deg).

YANG TIDAK DIPERLUKAN DI PI: Gazebo, URDF, mesh, build_model_v2.py, dokumen LaTeX dan CAD.
Semua itu simulasi. Pi hanya perlu sun.py, tracker.py, dinamika/, kontrol_hibrid/, cuaca/, angin/.
EOF
