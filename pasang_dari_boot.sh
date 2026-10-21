#!/usr/bin/env bash
# Jalankan DI RASPBERRY PI setelah kartu di-boot:
#   sudo bash /boot/firmware/pasang_dari_boot.sh
set -euo pipefail
SRC="${SRC:-/boot/firmware/bekal_raspi.tar.gz}"
TUJUAN="${TUJUAN:-/home/pi/projekssf}"
PENGGUNA="${PENGGUNA:-pi}"
[ -f "$SRC" ] || { echo "tidak ada $SRC"; exit 1; }
mkdir -p "$(dirname "$TUJUAN")"
tar xzf "$SRC" -C "$(dirname "$TUJUAN")"
chown -R "$PENGGUNA:$PENGGUNA" "$TUJUAN"
echo "terpasang di $TUJUAN, pemilik $PENGGUNA"
echo "lanjut:  bash $TUJUAN/pasang_di_raspi.sh   (memasang ROS 2 dan venv)"
