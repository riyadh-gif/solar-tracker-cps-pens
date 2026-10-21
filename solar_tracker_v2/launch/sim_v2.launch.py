"""Gazebo + jembatan ros_gz + robot_state_publisher untuk DESAIN BARU (solar_tracker_v2).

Beda dengan desain lama (solar_tracker/):
  - reduksi gigi 5.25:1 pada sumbu kemiringan (gear 8 dan 42 gigi, modul 2)
  - azimuth TIDAK kontinu lagi: ada pangkon limit dan empat saklar, dibatasi +-170 deg (ASUMSI)
  - panel hampir seimbang terhadap engsel: gravitasi pada q2 = 0 hanya 0.0088 N.m
  - LDR memakai teropong dan lensa, bukan sekat datar (model penginderaan BELUM disesuaikan)

Pakai (di WSL):
    source /opt/ros/lyrical/setup.bash
    ros2 launch ~/mk/solar_tracker_v2/launch/sim_v2.launch.py [headless:=true] [rviz:=true]

Gerakkan sendi (radian, langsung ke Gazebo tanpa node perantara):
    ros2 topic pub --once /solar_tracker_v2/azimuth_cmd   std_msgs/msg/Float64 "{data: 0.5}"
    ros2 topic pub --once /solar_tracker_v2/elevation_cmd std_msgs/msg/Float64 "{data: 0.3}"
    ros2 topic echo /joint_states --once

Bangun ulang model dari mesh CAD lebih dulu bila perlu:
    ~/.venvs/solar/bin/python ~/mk/build_model_v2.py

CATATAN: node matahari, sensor, EKF dan angin BELUM dipindahkan ke desain ini. Semuanya membaca
dyn_model.AXES (desain lama, N = 1) dan menerbitkan ke /solar_tracker/*, bukan /solar_tracker_v2/*.
Memakainya di sini akan memberi gain dan lewatan yang salah.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path.home() / "mk" / "solar_tracker_v2"
ARGS = ["headless", "rviz"]


def setup(context):
    c = {k: LaunchConfiguration(k).perform(context).lower() for k in ARGS}
    urdf = ROOT / "solar_tracker_v2.urdf"
    if not urdf.exists():
        raise RuntimeError(f"{urdf} tidak ada. Jalankan dulu: python ~/mk/build_model_v2.py")
    acts = [
        ExecuteProcess(cmd=["gz", "sim", "-r"] + (["-s"] if c["headless"] == "true" else [])
                           + [str(ROOT / "world_v2.sdf")], output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": str(ROOT / "launch" / "bridge_v2.yaml")}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf.read_text(), "use_sim_time": True}]),
    ]
    if c["headless"] != "true":
        # kamera GUI awal: dari Selatan menghadap Utara (sejajar sumbu engsel), Timur di kanan layar
        cam = ('for i in $(seq 1 30); do sleep 2; gz service -s /gui/move_to/pose '
               '--reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 1500 '
               "--req 'pose {position {x: -1.4 y: 0.0 z: 0.65} orientation {x: 0 y: 0.0749 z: 0 w: 0.9972}}' "
               '2>/dev/null | grep -q true && break; done')
        acts.append(ExecuteProcess(cmd=["bash", "-c", cam], output="log"))
    if c["rviz"] == "true":
        acts.append(ExecuteProcess(cmd=["rviz2", "--ros-args", "-p", "use_sim_time:=true"],
                                   output="screen", additional_env={"QT_QPA_PLATFORM": "xcb"}))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", description="true = server Gazebo tanpa GUI"),
        DeclareLaunchArgument("rviz", default_value="false", description="true = buka RViz (butuh WSLg)"),
        OpaqueFunction(function=setup),
    ])
