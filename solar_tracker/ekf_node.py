"""Node ROS 2: EKF penaksir simpangan sudut, berjalan di dalam simulasi Gazebo.

Memakai kembali seluruh model dari `kontrol_hibrid/ekf.py`. Keadaan yang ditaksir adalah SIMPANGAN
    b = [b_azimuth, b_kemiringan] = sudut beban - sudut perintah
yaitu salah pasang mekanik, lepas langkah terkumpul, dan sisa defleksi yang tidak dimodelkan.

Masukan: /tracker/diag (dari sensor_node: pembacaan LDR, sudut perintah, keadaan langit).
Keluaran:
    /ekf/state    std_msgs/String berisi JSON: b taksiran, sigma (akar diagonal P), inovasi, inovasi
                  ternormalkan (NIS), kepekaan pengukuran, dan penanda keteramatan azimuth.
    /ekf/koreksi  Float64MultiArray [b_az, b_q2] (deg). `sun_command_node` memakainya HANYA bila
                  parameternya `pakai_ekf` disetel true, jadi loop tertutup bersifat opt-in.

Yang menarik dipantau: sigma azimuth MEMBESAR SENDIRI saat matahari tinggi, karena kolom azimuth pada
Jacobian pengukuran mengecil sebanding sin(q2). Efek keyhole tampil sebagai angka, bukan aturan tangan.

Parameter: enabled, q_drift (deg per akar detik), p0 (deg, ketidakpastian awal), nis_batas (ambang uji
pencilan inovasi; pengukuran yang melewatinya dibuang, berguna saat awan lewat atau lepas langkah mendadak).

Pakai: otomatis dijalankan sim.launch.py bila ekf:=true. Tanpa ROS: python ekf_node.py --selftest
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "kontrol_hibrid"))
import ekf as EK             # noqa: E402
import hybrid_sim as H       # noqa: E402

DEFAULTS = dict(enabled=True, q_drift=EK.Q_DRIFT, p0=EK.B0_SIGMA, nis_batas=25.0)
HZ = 5.0


TOLAK_MAKS = 10     # berturut turut; melewati ini, P digembungkan agar gerbang pencilan terbuka lagi


def langkah(f, diag, p, q_drift, dt, nis_batas):
    """Satu langkah prediksi dan koreksi dari isi /tracker/diag. Kembalikan dict untuk /ekf/state.

    Dua pengaman yang WAJIB, keduanya ditemukan saat loop tertutup menyimpang di Gazebo:

    1. LDR hanya dipakai bila matahari TERJANGKAU. Saat zenith melewati 45 derajat, kemiringan terpotong dan
       LDR membaca galat besar yang BUKAN berasal dari simpangan sendi. Menafsirkannya sebagai simpangan
       membuat tapis salah besar; di loop tertutup kesalahan itu ikut menggeser perintah dan memperparah diri.
    2. Anti-penyimpangan pada gerbang pencilan. Bila taksiran sudah telanjur salah, inovasi menjadi besar,
       gerbang menolak semua pengukuran, dan tapis TERKUNCI selamanya (terukur: NIS tetap 82 sampai 90,
       taksiran kemiringan macet di -6.47 derajat). Setelah TOLAK_MAKS penolakan berturut turut, P digembungkan
       sehingga gerbang terbuka dan tapis dapat memperbaiki diri."""
    sky = (diag["matahari"]["az"], diag["matahari"]["el"], diag["matahari"]["dni"],
           diag["matahari"]["dhi"], diag["matahari"]["ghi"])
    c1, c2 = diag["sendi"]["cmd_q1"], diag["sendi"]["cmd_q2"]
    terjangkau = bool(diag.get("galat", {}).get("terjangkau", True))
    f.prediksi(dt=dt, q_drift=q_drift)
    pl1, pl2 = c1 + f.b[0], c2 + f.b[1]                 # sudut beban yang DIRAMALKAN tapis
    Hm, e_pred = EK.jacobian_ldr(pl1, pl2, sky, p)
    z = np.array([diag["ldr"]["ex"], diag["ldr"]["ey"]])
    se = EK.sigma_e(sky, pl1, pl2, p, seed=0, n=20)
    R = np.diag(np.maximum(se, 1e-6) ** 2)
    inov = z - e_pred
    S = Hm @ f.P @ Hm.T + R
    nis = float(inov @ np.linalg.solve(S, inov))        # inovasi ternormalkan: uji pencilan
    tolak = getattr(f, "tolak", 0)
    dipakai = terjangkau and nis <= nis_batas
    if dipakai:
        f.koreksi(z, e_pred, Hm, R)
        tolak = 0
    elif terjangkau:
        tolak += 1
        if tolak >= TOLAK_MAKS:                          # tapis mungkin menyimpang: buka gerbang
            f.P = f.P * 4.0
            tolak = 0
    f.tolak = tolak
    sg = f.sigma()
    return dict(b_az=round(float(f.b[0]), 4), b_q2=round(float(f.b[1]), 4),
                sig_az=round(float(sg[0]), 4), sig_q2=round(float(sg[1]), 4),
                inov_ex=round(float(inov[0]), 6), inov_ey=round(float(inov[1]), 6),
                nis=round(nis, 3), dipakai=bool(dipakai), terjangkau=terjangkau, tolak=int(tolak),
                k_az=round(float(Hm[1, 0]), 6), k_q2=round(float(Hm[0, 1]), 6),
                teramati_az=bool(abs(Hm[1, 0]) > 0.2 * abs(Hm[0, 1])),
                jam=diag["jam"])


def selftest():
    p = H.P()
    f = EK.EKF()
    diag = dict(jam=10.0, matahari=dict(az=90.0, el=45.0, dni=700.0, dhi=170.0, ghi=665.0),
                sendi=dict(cmd_q1=0.0, cmd_q2=45.0), ldr=dict(ex=0.0, ey=0.0), galat=dict(terjangkau=True))
    r = langkah(f, diag, p, EK.Q_DRIFT, 0.2, 25.0)
    assert r["dipakai"] and r["sig_az"] < EK.B0_SIGMA and r["sig_q2"] < EK.B0_SIGMA, r
    assert json.loads(json.dumps(r)) == r
    # keteramatan azimuth: tinggi saat matahari rendah, hilang saat matahari di dekat zenith
    tinggi = dict(diag, matahari=dict(az=90.0, el=85.0, dni=700.0, dhi=170.0, ghi=665.0),
                  sendi=dict(cmd_q1=0.0, cmd_q2=5.0))
    r2 = langkah(EK.EKF(), tinggi, p, EK.Q_DRIFT, 0.2, 25.0)
    assert abs(r2["k_az"]) < abs(r["k_az"]) / 3, (r["k_az"], r2["k_az"])
    assert r["teramati_az"] and not r2["teramati_az"], (r, r2)
    assert r2["sig_az"] > r["sig_az"], (r["sig_az"], r2["sig_az"])
    # uji pencilan: inovasi besar ditolak, sehingga b tidak melompat
    f3 = EK.EKF(); langkah(f3, diag, p, EK.Q_DRIFT, 0.2, 25.0)
    b_sebelum = f3.b.copy()
    liar = dict(diag, ldr=dict(ex=0.5, ey=0.5))
    r3 = langkah(f3, liar, p, EK.Q_DRIFT, 0.2, 25.0)
    assert not r3["dipakai"] and np.allclose(f3.b, b_sebelum), (r3, f3.b, b_sebelum)
    # anti-penyimpangan: penolakan beruntun menggembungkan P sehingga gerbang terbuka lagi
    p_awal = f3.P[0, 0]
    for _ in range(TOLAK_MAKS):
        langkah(f3, liar, p, EK.Q_DRIFT, 0.2, 25.0)
    assert f3.P[0, 0] > p_awal, (p_awal, f3.P[0, 0])
    # matahari di luar jangkauan: LDR TIDAK dipakai (galatnya dari batas 45 deg, bukan dari simpangan)
    f5 = EK.EKF(); b5 = f5.b.copy()
    jauh = dict(diag, galat=dict(terjangkau=False), ldr=dict(ex=0.3, ey=0.0))
    r5 = langkah(f5, jauh, p, EK.Q_DRIFT, 0.2, 1e9)
    assert not r5["dipakai"] and not r5["terjangkau"] and np.allclose(f5.b, b5), r5
    assert f5.sigma()[0] > EK.B0_SIGMA, "tanpa pengukuran, ketidakpastian harus membesar"
    # tapis memang menyatu ke simpangan yang benar bila LDR bersih
    f4, b_benar = EK.EKF(), 2.0
    for _ in range(40):
        Hm, _e = EK.jacobian_ldr(b_benar, 45.0, (90.0, 45.0, 700.0, 170.0, 665.0), p)
        d = dict(diag, ldr=dict(ex=0.0, ey=0.0))
        d["sendi"] = dict(cmd_q1=-f4.b[0], cmd_q2=45.0 - f4.b[1])
        import fusi_sensor as FS
        rng = __import__("random").Random(0)
        d["ldr"] = dict(zip(("ex", "ey"), FS.ldr_baca(d["sendi"]["cmd_q1"] + b_benar, d["sendi"]["cmd_q2"],
                                                      (90.0, 45.0, 700.0, 170.0, 665.0), p, rng, [1.0] * 4)[:2]))
        langkah(f4, d, p, EK.Q_DRIFT, 0.2, 1e9)
    assert abs(f4.b[0] - b_benar) < 0.5, (f4.b, b_benar)
    print("selftest ekf_node: lolos")
    print(f"  matahari 45 deg: k_az {r['k_az']:+.6f}, sigma az {r['sig_az']:.3f} deg, teramati {r['teramati_az']}")
    print(f"  matahari 85 deg: k_az {r2['k_az']:+.6f}, sigma az {r2['sig_az']:.3f} deg, teramati {r2['teramati_az']}")
    print(f"  uji pencilan: inovasi liar NIS {r3['nis']:.0f} ditolak, taksiran tidak melompat")
    print(f"  menyatu ke simpangan sebenarnya {b_benar} deg: taksiran {f4.b[0]:.3f} deg")


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    import rclpy
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, String

    class EkfNode(Node):
        def __init__(self):
            super().__init__("ekf_node")
            for k, v in DEFAULTS.items():
                self.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
            self.p = H.P()
            self.f = EK.EKF(p0=float(self.p_("p0")))
            self.t_lalu = None
            self.pub = self.create_publisher(String, "/ekf/state", 10)
            self.pub_kor = self.create_publisher(Float64MultiArray, "/ekf/koreksi", 10)
            self.create_subscription(String, "/tracker/diag", self.on_diag, 10)
            self.get_logger().info("ekf_node siap (menaksir simpangan sudut; koreksi dipakai hanya bila "
                                   "sun_command_node pakai_ekf:=true)")

        def p_(self, k):
            return self.get_parameter(k).value

        def on_diag(self, m):
            if not self.p_("enabled"):
                return
            d = json.loads(m.data)
            t = self.get_clock().now().nanoseconds * 1e-9
            dt = 1.0 / HZ if self.t_lalu is None else max(1e-3, min(2.0, t - self.t_lalu))
            self.t_lalu = t
            r = langkah(self.f, d, self.p, float(self.p_("q_drift")), dt, float(self.p_("nis_batas")))
            self.pub.publish(String(data=json.dumps(r)))
            k = Float64MultiArray()
            k.data = [float(self.f.b[0]), float(self.f.b[1])]
            self.pub_kor.publish(k)

    rclpy.init(args=sys.argv)
    node = EkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
