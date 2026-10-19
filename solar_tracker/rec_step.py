"""Alat ukur respons langkah di dalam proses ROS (tanpa CLI): rekam /joint_states, terbitkan perintah pada waktu yang diketahui, lalu analisis.
Pakai: python rec_step.py <nama> <az_rad> <el_rad> [pra_s=4] [pasca_s=10]
Keluaran: laju pesan (pesan hilang terlihat), nilai akhir, lewatan, frekuensi ayun (persilangan), waktu menetap, ayunan 1 detik terakhir.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def analyse(name, T, X, t_cmd, tgt_deg, f_pred):
    pre = [x for t, x in zip(T, X) if t < t_cmd]
    if len(pre) < 5:
        print(f"  {name}: data sebelum perintah kurang"); return
    x0 = sum(pre[-20:]) / len(pre[-20:])
    post = [(t, x) for t, x in zip(T, X) if t >= t_cmd]
    end = post[-1][0]
    fin = sum(x for t, x in post if t > end - 1.0) / max(1, sum(1 for t, x in post if t > end - 1.0))
    sg = 1 if tgt_deg > x0 else -1
    over = max(sg * (x - fin) for t, x in post)
    cross = [post[i][0] for i in range(1, len(post)) if post[i][0] > t_cmd + 0.3 and (post[i - 1][1] - fin) < 0 <= (post[i][1] - fin)]
    f = 1 / ((cross[-1] - cross[0]) / (len(cross) - 1)) if len(cross) >= 3 else float("nan")
    tset = next((t - t_cmd for i, (t, x) in enumerate(post) if all(abs(y - fin) < 0.2 for _, y in post[i:])), float("nan"))
    amp = max(abs(x - fin) for t, x in post if t > end - 1.0)
    print(f"  {name:8s} awal={x0:7.3f} perintah={tgt_deg:7.3f} akhir={fin:8.3f} selisih={fin - tgt_deg:6.3f} | lewatan={over:6.2f}  f_ayun={f:5.2f} Hz (prediksi {f_pred})  menetap={tset:5.2f} s  ayunan akhir={amp:5.3f} deg")


def main():
    name, az, el = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    pre_s = float(sys.argv[4]) if len(sys.argv) > 4 else 4.0
    post_s = float(sys.argv[5]) if len(sys.argv) > 5 else 10.0
    rclpy.init()
    node = Node("rec_step")
    rec = {"T": [], "A": [], "E": []}

    def cb(m):
        if "azimuth_joint" in m.name and "elevation_joint" in m.name:
            rec["T"].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            rec["A"].append(math.degrees(m.position[m.name.index("azimuth_joint")]))
            rec["E"].append(math.degrees(m.position[m.name.index("elevation_joint")]))

    node.create_subscription(JointState, "/joint_states", cb, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=20000))
    pa = node.create_publisher(Float64, "/solar_tracker/azimuth_cmd", 10)
    pe = node.create_publisher(Float64, "/solar_tracker/elevation_cmd", 10)
    t0, sent, t_cmd = time.time(), False, None
    while time.time() - t0 < pre_s + post_s:
        rclpy.spin_once(node, timeout_sec=0.01)
        if not sent and time.time() - t0 >= pre_s and rec["T"]:
            t_cmd = rec["T"][-1]
            pa.publish(Float64(data=az)); pe.publish(Float64(data=el)); sent = True
    T = rec["T"]
    dur = (T[-1] - T[0]) if len(T) > 1 else 0.0
    print(f"[{name}] pesan diterima: {len(T)} dalam {dur:.1f} s simulasi ({len(T) / max(dur, 1e-9):.0f} Hz; harapan sekitar 1000 Hz bila tak ada yang hilang)")
    if t_cmd is not None:
        analyse("azimuth", T, rec["A"], t_cmd, math.degrees(az), "4.79")
        analyse("elevasi", T, rec["E"], t_cmd, math.degrees(el), "6.89")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
