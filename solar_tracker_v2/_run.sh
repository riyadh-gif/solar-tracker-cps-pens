source /opt/ros/lyrical/setup.bash
ps -eo pid,args | grep -E "gz-sim|parameter_bridge|ros2 launch|sun_command|wind_node|sensor_node|ekf_node|control_web" | grep -v grep | awk '{print $1}' | xargs -r kill -9; sleep 3
exec ros2 launch ~/mk/solar_tracker/launch/sim.launch.py headless:=true sun_node:=true date:=2026-10-07 speed:=600 panel:=true ekf:=true kalibrasi:=true
