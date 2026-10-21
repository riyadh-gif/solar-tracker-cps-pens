"""Validasi di Gazebo: bandingkan strategi perintah (langsung, scurve, zv, zvd) terhadap model Python.

Menyuntikkan lintasan perintah berbentuk LANGSUNG ke topik yang dijembatani (`*_cmd_gz`), memakai JAM SIMULASI,
lalu merekam `/joint_states` dan menghitung lewatan serta waktu menetap. Angkanya dibandingkan dengan
`dinamika/osilasi_energi.py` pada konfigurasi yang sama.

Jalankan Gazebo dulu (mode pid, tanpa node matahari), lalu MATIKAN wind_node supaya tidak ikut menerbitkan
ke `*_cmd_gz` (dua penerbit pada satu topik akan saling menimpa):
    ros2 launch ~/mk/solar_tracker/launch/sim.launch.py headless:=true
    pkill -f wind_node.py
    python rec_shaped.py <az|el> <langkah_deg> [microstep=16] [T=8]
"""
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "dinamika"))
import dyn_model as D          # noqa: E402
import osilasi_energi as O     # noqa: E402

RATE = 500.0                   # Hz, laju terbit perintah (resolusi 2 ms; jeda ZVD azimuth 104 ms)
STRATEGIES = ("langsung", "scurve", "zv", "zvd")


def analyse(t, q, t_cmd, q0, qf, band=0.02):
    """Lewatan (%) dan waktu menetap (s) ke pita band x langkah, diukur dari saat perintah dikirim."""
    m = t >= t_cmd
    tt, qq = t[m], q[m]
    rng = qf - q0
    if abs(rng) < 1e-9:
        return float("nan"), float("nan")
    norm = (qq - q0) / rng
    over = max(0.0, float(norm.max() - 1.0)) * 100
    bad = np.flatnonzero(np.abs(norm - 1.0) > band)
    settle = float(tt[bad[-1]] - t_cmd) if len(bad) and bad[-1] + 1 < len(tt) else float("nan")
    return over, settle


def main():
    axis = sys.argv[1] if len(sys.argv) > 1 else "az"
    step_deg = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
    micro = float(sys.argv[3]) if len(sys.argv) > 3 else 16.0
    T = float(sys.argv[4]) if len(sys.argv) > 4 else 8.0
    ax = D.AXES["azimuth" if axis == "az" else "elevasi"]
    joint = "azimuth_joint" if axis == "az" else "elevation_joint"

    rclpy.init()
    node = Node("rec_shaped")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
    rec = {"t": [], "q": []}

    def on_js(m):
        if joint in m.name:
            rec["t"].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            rec["q"].append(math.degrees(m.position[m.name.index(joint)]))

    node.create_subscription(JointState, "/joint_states", on_js, qos)
    pub = {a: node.create_publisher(Float64, f"/solar_tracker/{a}_cmd_gz", 10) for a in ("azimuth", "elevation")}
    other = "elevation" if axis == "az" else "azimuth"

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def send(v):
        pub["azimuth" if axis == "az" else "elevation"].publish(Float64(data=float(v)))
        pub[other].publish(Float64(data=0.0))

    def wait(dur, hold):
        t0 = now()
        while now() - t0 < dur:
            send(hold)
            rclpy.spin_once(node, timeout_sec=0.002)

    # tunggu jam simulasi hidup
    while now() < 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    dt = 1.0 / RATE
    n = int(T / dt)
    i0 = int(1.0 / dt)
    raw = np.concatenate([np.zeros(i0), np.full(n - i0, math.radians(step_deg))])
    vmax, amax = math.radians(30.0), math.radians(300.0)

    print(f"sumbu={axis} langkah={step_deg} deg microstep=1/{micro:.0f} laju terbit={RATE:.0f} Hz")
    print(f"{'strategi':10s} {'lewatan%':>9s} {'menetap s':>10s} | {'model lewatan%':>14s} {'model menetap s':>15s}")
    out = []
    for st in STRATEGIES:
        cmd = O.build_cmd(st, raw, dt, ax, micro, vmax, amax)
        wait(7.0, 0.0)                      # kembali ke nol dan tenang (elevasi perlu lama karena gravitasi)
        rec["t"].clear(); rec["q"].clear()
        t0 = now(); k = -1
        while True:
            tn = now() - t0
            if tn >= T:
                break
            j = min(n - 1, int(tn / dt))
            if j != k:
                send(cmd[j]); k = j
            rclpy.spin_once(node, timeout_sec=0.001)
        t = np.array(rec["t"]); q = np.array(rec["q"])
        if len(t) < 100:
            print(f"{st:10s} data kurang ({len(t)} pesan)"); continue
        t = t - t[0]
        q0 = float(np.mean(q[t < 0.8]))
        qf = float(np.mean(q[t > t[-1] - 1.0]))
        over, settle = analyse(t, q, 1.0, q0, qf)
        m = O.settle_after_step(ax, step_deg, strategy=st, micro=micro, T=max(6.0, T))
        print(f"{st:10s} {over:9.2f} {settle:10.2f} | {m['over_pct']:14.2f} {m['settle_s']:15.2f}")
        out.append((st, over, settle, m["over_pct"], m["settle_s"], len(t) / t[-1]))
    wait(3.0, 0.0)
    print("laju pesan joint_states:", " ".join(f"{o[0]}={o[5]:.0f}Hz" for o in out))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
