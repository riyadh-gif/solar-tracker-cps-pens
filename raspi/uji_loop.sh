#!/usr/bin/env bash
# Uji FUNGSI 2 sampai ujung: salah pasang mekanik 2 deg disuntikkan, LDR melihatnya,
# EKF menaksirnya, sun_command_node menggeser perintah, motor_node menggerakkan.
set +u
source /opt/ros/jazzy/setup.bash
cd /proyek/solar_tracker_v2
LOG=/proyek/log_uji; mkdir -p "$LOG"; rm -f "$LOG"/*.log

SP1=${SP1:-2.0}
SP2=${SP2:-2.0}
EKF=${EKF:-true}
echo "== salah pasang q1=$SP1 deg, q2=$SP2 deg; pakai_ekf=$EKF; jam beku ${JAM:-12.0} =="

python3 sun_command_node.py --ros-args -p date:=2026-10-07 -p speed:=1 -p marker:=false \
        -p gz_light:=false -p pakai_ekf:=$EKF -p hour:=${JAM:-12.0}            > "$LOG/sun.log" 2>&1 &
python3 motor_node.py      --ros-args -p tiruan:=true    > "$LOG/mot.log" 2>&1 &
python3 sensor_hw_node.py  --ros-args -p tiruan:=true \
        -p salah_pasang_q1:=$SP1 -p salah_pasang_q2:=$SP2 > "$LOG/sen.log" 2>&1 &
python3 ekf_node.py        --ros-args                     > "$LOG/ekf.log" 2>&1 &
sleep 8

python3 /proyek/raspi/dengar_loop.py "${DURASI:-45}"

echo
echo "== Galat log =="
for f in sun mot sen ekf; do
  c=$(grep -cE "Traceback|ERROR" "$LOG/$f.log" 2>/dev/null || echo 0)
  printf '   %-4s %s\n' "$f" "$c"
done
pkill -f sun_command_node; pkill -f motor_node; pkill -f sensor_hw_node; pkill -f ekf_node
sleep 1
echo "== selesai =="
