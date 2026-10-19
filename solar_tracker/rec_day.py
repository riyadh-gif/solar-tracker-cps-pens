"""Uji pelacakan sehari: rekam /stepper/state (10 Hz, sudut beban) dan /sun/state (arah matahari, perintah), hitung galat pointing.
Pakai: python rec_day.py <nama> <durasi_s>. Jalankan bersama sim.launch.py actuator:=stepper sun_node:=true loop:=false.
Galat pointing = arccos(normal(sudut beban) . vektor matahari), tracker.fk dan sun_vec. Galat perintah memakai sudut perintah node matahari.
Lepas langkah: |beban - perintah| terakhir dalam kelipatan 360/Nr = 7.2 deg (siklus listrik), dan puncak |selisih| (beban - perintah stepper).
/stepper/state: [t, elevasi: cmd,beban,selisih,torsi (1-4), azimuth: cmd,beban,selisih,torsi (5-8), v, usia]; /sun/state: [jam, az, el, ghi, dni, dhi, q1_cmd, q2_cmd] (deg)."""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, "/home/riyadh/mk")
import tracker  # noqa: E402


def err(q1, q2, az, el):
    n = tracker.fk(math.radians(q1), math.radians(q2))
    return math.degrees(math.acos(max(-1.0, min(1.0, float(n @ tracker.sun_vec(az, el))))))


def main():
    name, dur = sys.argv[1], float(sys.argv[2])
    rclpy.init()
    node = Node("rec_day")
    sun, rows = [None], []
    node.create_subscription(Float64MultiArray, "/sun/state", lambda m: sun.__setitem__(0, (list(m.data), time.time())), 100)
    node.create_subscription(Float64MultiArray, "/stepper/state", lambda m: rows.append((list(m.data), sun[0][0] if sun[0] and time.time() - sun[0][1] < 0.5 else None)), 1000)
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.05)
    R = [(a, s) for a, s in rows if s is not None]
    if not R:
        print(f"[{name}] tidak ada data"); return
    jam = np.array([s[0] for a, s in R])
    e_load = np.array([err(a[6], a[2], s[1], s[2]) for a, s in R])
    e_cmd = np.array([err(s[6], s[7], s[1], s[2]) for a, s in R])
    d_az = np.array([a[6] - s[6] for a, s in R])      # beban azimuth - perintah node matahari
    d_el = np.array([a[2] - s[7] for a, s in R])
    dl_az = np.array([a[7] for a, s in R]); dl_el = np.array([a[3] for a, s in R])   # selisih stepper (perintah terkuantisasi - beban)... tanda: perintah - beban
    cyc = lambda x: abs(x[-1]) / 7.2
    ok = np.array([90 - s[2] <= 45.0 for a, s in R])      # terjangkau: zenith matahari <= batas kemiringan 45 deg (ASUMSI)
    w = np.arange(len(R)) > 150                          # lewati 15 s pertama (slew awal dari 0)
    f = lambda x, m: f"rata {x[m].mean():.3f} maks {x[m].max():.3f} p95 {np.percentile(x[m], 95):.3f}" if m.any() else "tak ada"
    print(f"[{name}] sampel {len(R)}, jam {jam[0]:.2f} sampai {jam[-1]:.2f}")
    print(f"  galat pointing BEBAN  (semua)      : {f(e_load, np.ones(len(R), bool))} deg")
    print(f"  galat pointing BEBAN  (zenith<=45) : {f(e_load, ok & w)} deg")
    print(f"  galat pointing PERINTAH (zenith<=45): {f(e_cmd, ok & w)} deg")
    print(f"  tertinggal beban-perintah: azimuth maks {np.abs(d_az[w]).max():.2f} deg, elevasi maks {np.abs(d_el[w]).max():.2f} deg; akhir az {d_az[-1]:.3f}, el {d_el[-1]:.3f}")
    print(f"  selisih stepper maks |perintah-beban|: azimuth {np.abs(dl_az).max():.2f} deg, elevasi {np.abs(dl_el).max():.2f} deg (lepas langkah bila > 3.6 deg menetap)")
    print(f"  zenith<=45 deg (dalam jangkauan sendi): {100 * (ok.mean()):.0f}% sampel; di luar jangkauan galat perintah = jarak matahari ke batas kemiringan")
    for nm, col in (("elevasi", 3), ("azimuth", 7)):
        dl = np.array([abs(a[col]) for a, s in R]); i = np.nonzero(dl > 3.6)[0]
        if len(i):
            a, s = R[i[0]]
            print(f"  LEPAS {nm}: pertama sampel {i[0]}, jam {jam[i[0]]:.2f}, perintah stepper az {a[5]:.1f} el {a[1]:.1f}, beban az {a[6]:.1f} el {a[2]:.1f}, matahari az {s[1]:.0f} el {s[2]:.0f}")
        else:
            print(f"  tidak ada lepas langkah {nm}")
    node.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
