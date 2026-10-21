#!/usr/bin/env bash
# Bungkus HANYA bagian yang dibutuhkan Raspberry Pi jadi satu berkas, untuk dipindahkan LEWAT
# PARTISI BOOT kartu SD. Berguna bila Pi belum punya jaringan, atau bila Anda tidak ingin
# bergantung pada git clone.
#
# Kenapa lewat partisi boot: partisi itu FAT32 dan TERBACA WINDOWS (muncul sebagai drive letter),
# sedangkan partisi root ext4 tempat /home/pi TIDAK terbaca Windows tanpa hak admin. Linux di Pi
# memasang partisi boot itu di /boot/firmware, jadi berkas yang Anda taruh di situ dari Windows
# langsung terlihat oleh Pi.
#
# Jalankan di WSL:  bash buat_bekal_raspi.sh
# Hasil:            bekal_raspi.tar.gz  (sekitar 7 MB)
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/bekal_raspi.tar.gz"
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

DIR="$STAGE/projekssf"
mkdir -p "$DIR"/{dinamika,kontrol_hibrid,cuaca,angin,solar_tracker_v2}

cd "$HERE"
# Inti model: tidak satupun butuh ROS
cp sun.py tracker.py pasang_di_raspi.sh "$DIR"/
cp dinamika/dyn_model.py "$DIR"/dinamika/
cp kontrol_hibrid/*.py "$DIR"/kontrol_hibrid/
cp cuaca/hujan.py cuaca/hujan_pens_2015-2026.csv "$DIR"/cuaca/
cp angin/angin_openmeteo.py angin/angin_pens_2015-2026.csv "$DIR"/angin/
# Node yang benar-benar dipakai di alat nyata
cp solar_tracker_v2/sun_command_node.py \
   solar_tracker_v2/sensor_hw_node.py \
   solar_tracker_v2/ekf_node.py \
   solar_tracker_v2/control_web.py \
   solar_tracker_v2/control_panel.py \
   solar_tracker_v2/control_web.html "$DIR"/solar_tracker_v2/

# Skrip pembongkar, dijalankan DI PI
cat > "$DIR/../pasang_dari_boot.sh" <<'EOS'
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
EOS
chmod +x "$DIR/../pasang_dari_boot.sh"

tar czf "$OUT" -C "$STAGE" projekssf
BYTE=$(stat -c%s "$OUT")
echo "dibuat: $OUT  ($(( BYTE / 1024 / 1024 )) MB)"
echo
echo "LANGKAHNYA:"
echo "  1. Tulis kartu SD dengan Ubuntu Server 24.04 LTS ARM64 (Raspberry Pi Imager),"
echo "     SET NAMA PENGGUNA ke 'pi' dan nyalakan SSH."
echo "  2. Colok lagi kartunya; Windows akan menampilkan partisi boot sebagai drive letter."
echo "  3. Salin DUA berkas ini ke akar drive itu:"
echo "       $OUT"
echo "       $STAGE/pasang_dari_boot.sh   (sudah ikut di dalam tar, juga disalin terpisah di bawah)"
echo "  4. Boot Pi, lalu:  sudo bash /boot/firmware/pasang_dari_boot.sh"
cp "$STAGE/pasang_dari_boot.sh" "$HERE/pasang_dari_boot.sh"
echo
echo "pasang_dari_boot.sh juga ditaruh di $HERE untuk disalin ke partisi boot."
