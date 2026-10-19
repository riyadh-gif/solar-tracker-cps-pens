"""Gazebo + jembatan ros_gz + robot_state_publisher untuk solar_tracker, dengan pilihan aktuator dan node matahari OPSIONAL.
Pakai (di WSL): ros2 launch ~/mk/solar_tracker/launch/sim.launch.py [headless:=true] [actuator:=pid|stepper]
  actuator:=pid      (bawaan) kontrol posisi Gazebo disetel ke kekakuan seri stepper dan poros (linier, tanpa lepas langkah)
  actuator:=stepper  model stepper loop terbuka memberi torsi ke sendi (stepper_node.py), angin NYATA selalu aktif, atur dengan skala_angin
Matahari pvlib (opsional, mati bawaan): sun_node:=true date:=2026-03-02 speed:=600 [okta:=0]
Jeda atau lanjut saat berjalan: ros2 param set /sun_command_node enabled false|true
Ganti tanggal saat berjalan (kutip agar dibaca teks): ros2 param set /sun_command_node date "'2026-06-21'"
Perintah sendi manual (rad): ros2 topic pub --once /solar_tracker/azimuth_cmd std_msgs/msg/Float64 "{data: 0.5}"
Tampilan RViz (butuh WSLg): tambah rviz:=true, mis. sun_node:=true rviz:=true speed:=600
Panel manual berbasis web (slider tanggal, jam, sendi, stepper, angin): tambah panel:=true, lalu buka http://localhost:8765 (terbuka sendiri)
Keadaan sendi: ros2 topic echo /joint_states
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path.home() / "mk" / "solar_tracker"   # symlink ke folder proyek (path tanpa spasi)
PY = str(Path.home() / ".venvs" / "solar-ros" / "bin" / "python")   # venv dengan pvlib dan akses rclpy (paket sistem)
ARGS = ["headless", "actuator", "sun_node", "date", "speed", "okta", "skala_angin", "alpha_angin", "tanggal_angin", "rviz", "loop", "panel", "mism", "kalibrasi", "ekf", "pakai_ekf"]


def setup(context):
    c = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    stepper = c["actuator"] == "stepper"
    world = str(ROOT / ("world_stepper.sdf" if stepper else "world.sdf"))
    bridge = str(ROOT / "launch" / ("bridge_stepper.yaml" if stepper else "bridge.yaml"))
    acts = [
        ExecuteProcess(cmd=["gz", "sim", "-r"] + (["-s"] if c["headless"].lower() == "true" else []) + [world], output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen", parameters=[{"config_file": bridge}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": (ROOT / "solar_tracker.urdf").read_text(), "use_sim_time": True}]),
    ]
    if c["headless"].lower() != "true":   # kamera GUI awal: dari Selatan menghadap Utara (sejajar sumbu engsel), Timur di kanan layar
        cam = ('for i in $(seq 1 30); do sleep 2; gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 1500 '
               "--req 'pose {position {x: -1.1 y: 0.0 z: 0.5} orientation {x: 0 y: 0.0749 z: 0 w: 0.9972}}' 2>/dev/null | grep -q true && break; done")
        acts.append(ExecuteProcess(cmd=["bash", "-c", cam], output="log"))
    if not stepper:   # mode pid: perintah /solar_tracker/*_cmd diteruskan ke Gazebo lewat wind_node (ditambah angin bila diminta)
        acts.append(ExecuteProcess(cmd=[PY, str(ROOT / "wind_node.py"), "--ros-args", "-p", "use_sim_time:=true",
                                        "-p", f"skala:={c['skala_angin']}", "-p", f"alpha:={c['alpha_angin']}",
                                        "-p", f"tanggal:={c['tanggal_angin']}"], output="screen"))
    if stepper:
        acts.append(ExecuteProcess(cmd=[PY, str(ROOT / "stepper_node.py"), "--ros-args", "-p", "use_sim_time:=true", "-p", "angin:=true", "-p", f"skala:={c['skala_angin']}", "-p", f"alpha:={c['alpha_angin']}",
                                        "-p", f"tanggal:={c['tanggal_angin']}"], output="screen"))
    if c["sun_node"].lower() == "true":
        # use_sim_time WAJIB: tanpa itu profil gerak dan input shaper berjalan di jam dinding sedangkan plant di jam simulasi
        # (Gazebo sekitar 85 persen real-time), sehingga lewatan jadi tidak dapat diulang (terukur 7.6 % sampai 0.35 % pada uji yang sama).
        acts.append(ExecuteProcess(cmd=[PY, str(ROOT / "sun_command_node.py"), "--ros-args", "-p", "use_sim_time:=true",
                                        "-p", f"date:={c['date']}", "-p", f"speed:={c['speed']}",
                                        "-p", f"okta:={c['okta']}", "-p", f"loop:={c['loop']}",
                                        "-p", f"pakai_ekf:={c['pakai_ekf']}"], output="screen"))
        # sensor dan diagnostik (LDR, dioda tunggal panel, energi, uraian galat) -> /tracker/diag
        acts.append(ExecuteProcess(cmd=[PY, str(ROOT / "sensor_node.py"), "--ros-args", "-p", "use_sim_time:=true",
                                        "-p", f"mism:={c['mism']}", "-p", f"kalibrasi:={c['kalibrasi']}"], output="screen"))
        if c["ekf"].lower() == "true":   # penaksir simpangan sudut -> /ekf/state dan /ekf/koreksi
            acts.append(ExecuteProcess(cmd=[PY, str(ROOT / "ekf_node.py"), "--ros-args", "-p", "use_sim_time:=true"], output="screen"))
    if c["rviz"].lower() == "true":
        acts.append(ExecuteProcess(cmd=["rviz2", "-d", str(ROOT / "launch" / "tracker.rviz"), "--ros-args", "-p", "use_sim_time:=true"], output="screen", additional_env={"QT_QPA_PLATFORM": "xcb"}))
    if c["panel"].lower() in ("true", "tk"):
        acts.append(ExecuteProcess(cmd=[PY, str(ROOT / ("control_panel.py" if c["panel"].lower() == "tk" else "control_web.py"))], output="screen"))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", description="true = server Gazebo tanpa GUI"),
        DeclareLaunchArgument("actuator", default_value="pid", description="pid (kekakuan seri stepper) atau stepper (model stepper loop terbuka)"),
        DeclareLaunchArgument("sun_node", default_value="false", description="true = nyalakan node perintah matahari pvlib (opsional)"),
        DeclareLaunchArgument("date", default_value="2026-10-04", description="tanggal YYYY-MM-DD untuk node matahari"),
        DeclareLaunchArgument("speed", default_value="600", description="detik simulasi per detik nyata untuk node matahari"),
        DeclareLaunchArgument("okta", default_value="0", description="tutupan awan 0..8 untuk irradiance di /sun/state"),
        DeclareLaunchArgument("skala_angin", default_value="1.0", description="pengali angin NYATA (1.0 apa adanya; 3.0 sekitar 83 persen torsi tahan). Preset sintetis sudah dibuang"),
        DeclareLaunchArgument("alpha_angin", default_value="0.30", description="eksponen profil angin untuk koreksi tinggi (0.16 lapangan terbuka, 0.35 kampus rapat)"),
        DeclareLaunchArgument("tanggal_angin", default_value="2026-10-07", description="hari angin NYATA yang diputar ulang (YYYY-MM-DD, data 2015 sampai 2026)"),
        DeclareLaunchArgument("rviz", default_value="false", description="true = buka RViz (model, TF, panah matahari)"),
        DeclareLaunchArgument("loop", default_value="true", description="node matahari: ulangi hari (true) atau berhenti di akhir hari (false)"),
        DeclareLaunchArgument("panel", default_value="false", description="true = panel kontrol manual berbasis web (browser, http://localhost:8765); tk = panel Tkinter (jendela WSLg)"),
        DeclareLaunchArgument("mism", default_value="0.03", description="ketidakcocokan antar LDR (simpangan baku relatif; 0.03 = belum dikalibrasi)"),
        DeclareLaunchArgument("kalibrasi", default_value="false", description="true = ketidakcocokan LDR dihapus (prosedur A sempurna, lihat kontrol_hibrid/kalibrasi_ldr.py)"),
        DeclareLaunchArgument("ekf", default_value="false", description="true = jalankan ekf_node (penaksir simpangan sudut) dan terbitkan /ekf/state"),
        DeclareLaunchArgument("pakai_ekf", default_value="false", description="true = sun_command_node MEMAKAI koreksi EKF (loop tertutup); butuh ekf:=true"),
        OpaqueFunction(function=setup),
    ])
