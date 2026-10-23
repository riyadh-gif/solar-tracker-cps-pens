"""Pantau rantai LDR -> EKF -> perintah -> motor selama beberapa detik.

Membuktikan FUNGSI 2: apakah ketidakrataan cahaya pada LDR benar-benar menjadi
keputusan sudut motor, bukan hanya angka yang diterbitkan lalu diabaikan.
"""
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

DURASI = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
JEDA = 5.0


def main():
    rclpy.init()
    n = Node("pemantau")
    k = dict(diag=None, kor=None, mot=None)
    n.create_subscription(String, "/tracker/diag", lambda m: k.__setitem__("diag", m.data), 10)
    n.create_subscription(String, "/motor/state", lambda m: k.__setitem__("mot", m.data), 10)
    n.create_subscription(Float64MultiArray, "/ekf/koreksi",
                          lambda m: k.__setitem__("kor", list(m.data)), 10)

    t0 = n.get_clock().now().nanoseconds / 1e9
    print(f"{'t':>5} | {'ex':>9} {'ey':>9} | {'b_az est':>9} {'b_q2 est':>9} "
          f"| {'tgt az':>8} {'tgt el':>8} | {'g perintah':>10} {'g SEBENARNYA':>12}")
    print("-" * 100)
    baris = []
    berikut = 0.0
    while True:
        rclpy.spin_once(n, timeout_sec=0.2)
        t = n.get_clock().now().nanoseconds / 1e9 - t0
        if t >= berikut and k["diag"] and k["mot"]:
            d, m = json.loads(k["diag"]), json.loads(k["mot"])
            kor = k["kor"] or [float("nan"), float("nan")]
            r = (t, d["ldr"]["ex"], d["ldr"]["ey"], kor[0], kor[1],
                 m["azimuth"]["tgt_deg"], m["elevasi"]["tgt_deg"], d["galat"]["perintah"], d["galat"].get("tiruan_sebenarnya"))
            baris.append(r)
            print(f"{r[0]:5.1f} | {r[1]:+9.5f} {r[2]:+9.5f} | {r[3]:+9.4f} {r[4]:+9.4f} "
                  f"| {r[5]:8.3f} {r[6]:8.3f} | {r[7]:10.4f} "
                  f"{(f'{r[8]:12.4f}' if r[8] is not None else 'n/a'.rjust(12))}")
            berikut += JEDA
        if t > DURASI:
            break

    d = json.loads(k["diag"])
    benar = d.get("tiruan") or {}
    print("-" * 100)
    print(f"kebenaran dasar salah pasang: q1={benar.get('salah_pasang_q1')} deg, "
          f"q2={benar.get('salah_pasang_q2')} deg")
    if baris and baris[-1][3] == baris[-1][3]:
        print(f"taksiran EKF akhir        : b_az={baris[-1][3]:+.4f} deg, b_q2={baris[-1][4]:+.4f} deg")
    print(f"ex,ey awal -> akhir       : {baris[0][1]:+.5f},{baris[0][2]:+.5f} "
          f"-> {baris[-1][1]:+.5f},{baris[-1][2]:+.5f}")
    sb = [b[8] for b in baris if b[8] is not None]
    if sb:
        print(f"galat pointing SEBENARNYA : awal {sb[0]:.4f} deg -> akhir {sb[-1]:.4f} deg"
              f"   (purata 5 terakhir {sum(sb[-5:]) / len(sb[-5:]):.4f} deg)")
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
