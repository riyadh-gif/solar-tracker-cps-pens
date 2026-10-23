#!/usr/bin/env bash
# Jalankan perintah di dalam kontainer ROS 2 pada Raspberry Pi.
#
#   bash raspi/jalan.sh bangun                 bangun citra (sekali, atau setelah Dockerfile berubah)
#   bash raspi/jalan.sh                        masuk shell interaktif
#   bash raspi/jalan.sh python3 uji_pi.py      jalankan satu perintah
#   bash raspi/jalan.sh ros2 topic list
#
# --network host  : DDS butuh multicast; tanpa ini node di kontainer tidak saling melihat
#                   dengan node di host atau di kontainer lain
# --ipc host      : DDS memakai memori bersama untuk transport lokal
# --device        : lewatkan I2C dan GPIO. /dev/i2c-1 baru ada SETELAH dtparam=i2c_arm=on
#                   ditambahkan ke /boot/firmware/config.txt dan Pi di-reboot.
set -euo pipefail

AKAR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CITRA="${CITRA:-ssf-ros:jazzy}"

if [ "${1:-}" = "bangun" ]; then
  exec docker build -t "$CITRA" -f "$AKAR/raspi/Dockerfile" "$AKAR/raspi"
fi

ARGS=(--rm -it
      --network host --ipc host
      -v "$AKAR:/proyek"
      -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}")

# Lewatkan perangkat hanya bila ada, supaya tetap jalan sebelum I2C diaktifkan
for d in /dev/i2c-1 /dev/gpiochip0 /dev/gpiochip4; do
  [ -e "$d" ] && ARGS+=(--device "$d")
done

if [ $# -eq 0 ]; then
  exec docker run "${ARGS[@]}" "$CITRA" bash
fi
exec docker run "${ARGS[@]}" "$CITRA" "$@"
