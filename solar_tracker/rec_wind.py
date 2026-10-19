"""Uji angin aktuator stepper: rekam /stepper/state (10 Hz) selama D detik, bandingkan sudut beban dengan prediksi kuasi-statis dari kecepatan angin.
Pakai: python rec_wind.py <nama> [durasi_s=170]. Perintah tetap 0 (sendi diam di pose awal, elevasi 0 = datar, gravitasi nol).
Prediksi: tau = momen angin(v), defleksi = asin(tau/Th)/Nr + tau/k; lepas langkah diprediksi bila tau > Th.
Indeks /stepper/state: [t, elevasi: cmd,beban,selisih,torsi (1-4), azimuth: cmd,beban,selisih,torsi (5-8), v, usia]."""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, "/home/riyadh/mk/dinamika")
import dyn_model as D  # noqa: E402

SLIP_DEG = 360 / 50


def main():
    name, dur = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 170.0
    rclpy.init()
    node = Node("rec_wind")
    rows = []
    node.create_subscription(Float64MultiArray, "/stepper/state", lambda m: rows.append(list(m.data)), 1000)
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.05)
    A = np.array(rows)
    print(f"[{name}] sampel {len(A)}, t node {A[0,0]:.1f} sampai {A[-1,0]:.1f} s, v maks {A[:,9].max():.1f} m/s, v rata {A[:,9].mean():.1f}")
    for nm, ax, col in (("azimuth", D.AXES["azimuth"], 6), ("elevasi", D.AXES["elevasi"], 2)):
        v, th = A[:, 9], A[:, col]
        tau = np.array([D.wind_torque(ax, x) for x in v])
        pred = np.degrees(np.arcsin(np.minimum(tau / ax.Th, 1)) / ax.Nr + tau / ax.k)
        ps = np.nonzero(tau > ax.Th)[0]
        os_ = np.nonzero(np.abs(th) > SLIP_DEG / 2)[0]
        print(f"  {nm}: tau maks {tau.max():.2f} N.m (Th {ax.Th}); |beban| maks {np.abs(th).max():.3f} deg; prediksi kuasi-statis maks {pred.max():.3f} deg; "
              f"lepas langkah: prediksi {'t=%.1f s' % A[ps[0],0] if len(ps) else 'tidak'}, teramati {'t=%.1f s' % A[os_[0],0] if len(os_) else 'tidak'}; sudut akhir {th[-1]:.3f} deg")
        k = len(os_) and os_[0] or len(A)
        k = min(k, ps[0] if len(ps) else len(A)) if True else k
        if k > 20:
            x, y = np.abs(th[:k]), pred[:k]
            # nilai tengah dikurangi offset awal (gravitasi) agar adil
            print(f"     sebelum lepas langkah (n={k}): korelasi |beban| vs prediksi {np.corrcoef(x, y)[0,1]:.3f}, galat RMS {np.sqrt(np.mean((x-y)**2)):.3f} deg, selisih maks {np.abs(x-y).max():.3f} deg")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
