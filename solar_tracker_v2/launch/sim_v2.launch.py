"""Gazebo + jembatan ros_gz + node ROS 2 untuk DESAIN BARU (solar_tracker_v2).

Menggantikan solar_tracker/launch/sim.launch.py sepenuhnya. Desain lama sudah dihapus.

Beda terhadap desain lama:
  - reduksi gigi 5.25:1 (gear 8 dan 42 gigi, modul 2) pada sumbu KEMIRINGAN; azimuth tetap langsung
  - poros kemiringan TERUKUR 194 mm di CAD, jadi k turun 317 -> 163.8 N.m/rad
  - panel hampir seimbang terhadap engsel: gravitasi pada q2 = 0 hanya 0.0088 N.m (dulu 0.276)
  - azimuth TIDAK kontinu lagi: ada pangkon limit dan empat saklar, dibatasi +-170 deg (ASUMSI)
  - LDR memakai teropong dan lensa, bukan sekat datar (model penginderaan BELUM disesuaikan)

Nama topik ROS tidak berubah (/solar_tracker/*, /sun/state, /tracker/diag, /ekf/state, /wind/state),
jadi seluruh perkakas lama tetap berlaku.

Pakai (di WSL):
  ros2 launch ~/mk/solar_tracker_v2/launch/sim_v2.launch.py [headless:=true]
  ros2 launch ~/mk/solar_tracker_v2/launch/sim_v2.launch.py sun_node:=true ekf:=true panel:=true rviz:=true

Bangun ulang model dari mesh CAD lebih dulu bila perlu:
  ~/.venvs/solar/bin/python ~/mk/build_model_v2.py

AKTUATOR: hanya mode `pid` (kontrol posisi Gazebo disetel ke K_ser). Mode `stepper` TIDAK disediakan
untuk desain ini: reduksi gigi membuat pegas magnet yang dirujuk ke beban (2067 N.m/rad) jauh lebih
kaku daripada poros (163.8), sehingga kurva torsi kuasi-statis bernilai ganda dan model sinus
loop terbuka di stepper_node.py tidak sah pada sumbu kemiringan. Lihat stepper_node.sumbu_sah().
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path.home() / "mk" / "solar_tracker_v2"
PY = str(Path.home() / ".venvs" / "solar-ros" / "bin" / "python")
ARGS = ["headless", "sun_node", "date", "speed", "okta", "skala_angin", "alpha_angin", "tanggal_angin",
        "rviz", "loop", "panel", "mism", "kalibrasi", "ekf", "pakai_ekf"]


def setup(context):
    c = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    urdf = ROOT / "solar_tracker_v2.urdf"
    if not urdf.exists():
        raise RuntimeError(f"{urdf} tidak ada. Jalankan dulu: python ~/mk/build_model_v2.py")
    acts = [
        ExecuteProcess(cmd=["gz", "sim", "-r"] + (["-s"] if c["headless"].lower() == "true" else [])
                           + [str(ROOT / "world_v2.sdf")], output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": str(ROOT / "launch" / "bridge_v2.yaml")}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf.read_text(), "use_sim_time": True}]),
        # wind_node meneruskan /solar_tracker/*_cmd ke *_cmd_gz (ditambah pergeseran angin NYATA)
        ExecuteProcess(cmd=[PY, str(ROOT / "wind_node.py"), "--ros-args", "-p", "use_sim_time:=true",
                            "-p", f"skala:={c['skala_angin']}", "-p", f"alpha:={c['alpha_angin']}",
                            "-p", f"tanggal:={c['tanggal_angin']}"], output="screen"),
    ]
    if c["headless"].lower() != "true":
        cam = ('for i in $(seq 1 30); do sleep 2; gz service -s /gui/move_to/pose '
               '--reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 1500 '
               "--req 'pose {position {x: -1.4 y: 0.0 z: 0.65} orientation {x: 0 y: 0.0749 z: 0 w: 0.9972}}' "
               '2>/dev/null | grep -q true && break; done')
        acts.append(ExecuteProcess(cmd=["bash", "-c", cam], output="log"))
    if c["sun_node"].lower() == "true":
        # use_sim_time WAJIB: tanpa itu profil gerak dan input shaper berjalan di jam dinding sedangkan
        # plant di jam simulasi, sehingga lewatan jadi tidak dapat diulang.
        acts.append(ExecuteProcess(cmd=[PY, str(ROOT / "sun_command_node.py"), "--ros-args", "-p", "use_sim_time:=true",
                                        "-p", f"date:={c['date']}", "-p", f"speed:={c['speed']}",
                                        "-p", f"okta:={c['okta']}", "-p", f"loop:={c['loop']}",
                                        "-p", f"pakai_ekf:={c['pakai_ekf']}"], output="screen"))
        acts.append(ExecuteProcess(cmd=[PY, str(ROOT / "sensor_node.py"), "--ros-args", "-p", "use_sim_time:=true",
                                        "-p", f"mism:={c['mism']}", "-p", f"kalibrasi:={c['kalibrasi']}",
                                        "-p", f"tanggal:={c['date']}"], output="screen"))   # tanggal sama dengan node matahari, supaya deteksi hujan sejalan
        if c["ekf"].lower() == "true":
            acts.append(ExecuteProcess(cmd=[PY, str(ROOT / "ekf_node.py"), "--ros-args", "-p", "use_sim_time:=true"], output="screen"))
    if c["rviz"].lower() == "true":
        acts.append(ExecuteProcess(cmd=["rviz2", "-d", str(ROOT / "launch" / "tracker.rviz"), "--ros-args",
                                        "-p", "use_sim_time:=true"], output="screen",
                                   additional_env={"QT_QPA_PLATFORM": "xcb"}))
    if c["panel"].lower() in ("true", "tk"):
        acts.append(ExecuteProcess(cmd=[PY, str(ROOT / ("control_panel.py" if c["panel"].lower() == "tk" else "control_web.py"))],
                                   output="screen"))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", description="true = server Gazebo tanpa GUI"),
        DeclareLaunchArgument("sun_node", default_value="false", description="true = node perintah matahari pvlib + sensor"),
        DeclareLaunchArgument("date", default_value="2026-10-04", description="tanggal YYYY-MM-DD untuk node matahari"),
        DeclareLaunchArgument("speed", default_value="600", description="detik simulasi per detik nyata"),
        DeclareLaunchArgument("okta", default_value="0", description="tutupan awan 0..8"),
        DeclareLaunchArgument("skala_angin", default_value="1.0", description="pengali angin NYATA Open-Meteo"),
        DeclareLaunchArgument("alpha_angin", default_value="0.30", description="eksponen profil angin"),
        DeclareLaunchArgument("tanggal_angin", default_value="2026-10-07", description="hari angin nyata yang diputar ulang"),
        DeclareLaunchArgument("rviz", default_value="false", description="true = buka RViz (butuh WSLg)"),
        DeclareLaunchArgument("loop", default_value="true", description="node matahari: ulangi hari"),
        DeclareLaunchArgument("panel", default_value="false", description="true = panel web; tk = panel Tkinter"),
        DeclareLaunchArgument("mism", default_value="0.03", description="ketidakcocokan antar LDR"),
        DeclareLaunchArgument("kalibrasi", default_value="false", description="true = ketidakcocokan LDR dihapus"),
        DeclareLaunchArgument("ekf", default_value="false", description="true = jalankan ekf_node"),
        DeclareLaunchArgument("pakai_ekf", default_value="false", description="true = sun_command_node memakai koreksi EKF"),
        OpaqueFunction(function=setup),
    ])
