#!/usr/bin/env bash
# Uji tiga fungsi Raspi bersama, semua mode tiruan, di dalam kontainer ROS 2.
# set -u dilepas: setup.bash ROS memakai variabel tak terdefinisi.
# Log ditulis ke /proyek (volume terpasang), BUKAN /tmp: /tmp mati bersama kontainer.
set +u
source /opt/ros/jazzy/setup.bash
cd /proyek/solar_tracker_v2
LOG=/proyek/log_uji; mkdir -p "$LOG"; rm -f "$LOG"/*.log

echo "== Jalankan node (jam beku ${JAM:-12.0}) =="
python3 sun_command_node.py --ros-args -p date:=2026-10-07 -p speed:=1 -p marker:=false -p gz_light:=false -p hour:=${JAM:-12.0} > "$LOG/sun.log" 2>&1 &
python3 motor_node.py      --ros-args -p tiruan:=true > "$LOG/mot.log" 2>&1 &
python3 sensor_hw_node.py  --ros-args -p tiruan:=true > "$LOG/sen.log" 2>&1 &
python3 ekf_node.py        --ros-args                 > "$LOG/ekf.log" 2>&1 &
sleep 20

python3 /proyek/raspi/dengar.py
RC=$?

echo
echo "== Baris pertama tiap log =="
for f in sun mot sen ekf; do printf '   %-4s %s\n' "$f" "$(head -1 "$LOG/$f.log" 2>/dev/null)"; done

echo
echo "== Galat =="
for f in sun mot sen ekf; do
  printf '   %-4s %s baris galat\n' "$f" "$(grep -cE "Traceback|ERROR|Error" "$LOG/$f.log" 2>/dev/null || echo 0)"
done

pkill -f sun_command_node; pkill -f motor_node; pkill -f sensor_hw_node; pkill -f ekf_node
sleep 1
echo "== selesai rc=$RC =="
