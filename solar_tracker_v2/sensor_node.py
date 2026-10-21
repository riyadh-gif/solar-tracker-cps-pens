"""Node ROS 2: SENSOR dan DIAGNOSTIK untuk simulasi Gazebo.

Sampai sekarang simulasi hanya menerbitkan sudut sendi dan posisi matahari. Node ini menambahkan seluruh model
yang sudah dibangun dan diuji di `kontrol_hibrid/`, supaya yang terpantau sama lengkapnya dengan yang dianalisis:

  LDR      empat pembacaan dari model `hybrid_sim` (sekat, pembagi tegangan jenuh, derau ADC, ketidakcocokan
           antar sensor), dihitung dari sudut BEBAN yang sebenarnya di Gazebo.
  Listrik  model dioda tunggal `panel_listrik` (MOS C-50-18-M): POA, suhu sel, Isc, Imp, Vmp, Pmp.
  Energi   terkumpul sehari, dibandingkan panel datar dan tracker ideal.
  Galat    diuraikan: galat PERINTAH (batas jangkauan dan ketertinggalan) berbanding galat BEBAN (ditambah
           defleksi, ayunan, lepas langkah). Selisihnya adalah galat DINAMIK.

Masukan: /joint_states (sudut beban dari Gazebo), /sun/state (dari sun_command_node).
Keluaran: /tracker/diag (std_msgs/String berisi JSON, 5 Hz) dan /ldr/state (Float64MultiArray:
          v1, v2, v3, v4, ex, ey) untuk perekaman laju tinggi.

JSON dipilih supaya menjelaskan dirinya sendiri: deret Float64MultiArray yang panjang mudah salah urutan
(pernah terjadi di proyek ini pada /stepper/state).

Parameter: enabled, mism (ketidakcocokan antar LDR, bawaan 0.03 = belum dikalibrasi), kalibrasi (true =
terapkan faktor koreksi prosedur A, lihat kontrol_hibrid/kalibrasi_ldr.py), tamb (suhu udara C), albedo.

Pakai: otomatis dijalankan sim.launch.py bila sun_node:=true. Tanpa ROS: python sensor_node.py --selftest
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "kontrol_hibrid"))
import hybrid_sim as H       # noqa: E402
import panel_listrik as PL   # noqa: E402
sys.path.insert(0, str(ROOT / "cuaca"))
import hujan as HJ   # noqa: E402  (hujan NYATA untuk menandai kaca basah)
import tracker               # noqa: E402

DEFAULTS = dict(enabled=True, mism=0.03, kalibrasi=False, tamb=30.0, albedo=0.25, seed=0)
HZ = 5.0


def ldr_baca(q1_deg, q2_deg, az, el, dni, dhi, p, rng, mism):
    """Empat tegangan LDR (V) dan (ex, ey) pada sudut beban. Sama dengan kontrol_hibrid/hybrid_sim.read."""
    n, xp, yp = H.basis(q1_deg, q2_deg)
    s = tracker.sun_vec(az, el)
    c = H.dot(n, s)
    ex_a, ey_a = math.atan2(H.dot(s, xp), c), math.atan2(H.dot(s, yp), c)
    beam, dif = dni * max(c, 0.0), dhi * (1 + n[2]) / 2
    sh = (H.shade(-ex_a, p.r), H.shade(ex_a, p.r), H.shade(-ey_a, p.r), H.shade(ey_a, p.r))
    v = [H.volt((beam * (1 - sh[i]) + dif) * mism[i], p, rng) for i in range(4)]
    ex = (v[0] - v[1]) / (v[0] + v[1] + 1e-9)
    ey = (v[2] - v[3]) / (v[2] + v[3] + 1e-9)
    return v, ex, ey


def osilasi(t, q, f_min=2.0):
    """Statistik AYUNAN dari deret sudut beban laju tinggi.

    Gerak jejak itu lambat (0.05 sampai 0.5 deg/s) sedangkan deringnya 4.8 Hz (azimuth) dan 6.9 Hz (elevasi).
    Keduanya dipisah dengan membuang rata-rata bergerak: sisa AC itulah ayunannya. Tidak perlu menyamakan
    waktu dengan topik perintah, cukup sudut beban sendiri.

    PENTING: /tracker/diag terbit 5 Hz, dan mencuplik dering 4.8 Hz pada 5 Hz adalah aliasing total. Karena itu
    statistik dihitung di sini pada laju /joint_states (1000 Hz), lalu RINGKASANNYA saja yang diterbitkan."""
    t, q = np.asarray(t, float), np.asarray(q, float)
    if len(t) < 32 or t[-1] - t[0] < 0.2:
        return dict(pp=0.0, rms=0.0, f=0.0, n=int(len(t)), hz=0.0)
    dt = (t[-1] - t[0]) / max(1, len(t) - 1)
    w = max(3, int(round(1.0 / (f_min * dt))) | 1)          # jendela rata-rata bergerak, ganjil
    if w >= len(q):
        return dict(pp=0.0, rms=0.0, f=0.0, n=int(len(t)), hz=float(1.0 / dt))
    ker = np.ones(w) / w
    lambat = np.convolve(q, ker, mode="valid")
    ac = q[w // 2: w // 2 + len(lambat)] - lambat
    cr = np.flatnonzero((ac[:-1] < 0) & (ac[1:] >= 0))      # persilangan nol naik -> frekuensi utama
    f = float(1.0 / (np.mean(np.diff(cr)) * dt)) if len(cr) >= 3 else 0.0
    return dict(pp=float(ac.max() - ac.min()), rms=float(np.sqrt((ac ** 2).mean())),
                f=f, n=int(len(t)), hz=float(1.0 / dt))


def galat_pointing(q1_deg, q2_deg, az, el):
    n = tracker.fk(math.radians(q1_deg), math.radians(q2_deg))
    return math.degrees(math.acos(max(-1.0, min(1.0, float(n @ tracker.sun_vec(az, el))))))


def listrik(q1_deg, q2_deg, az, el, dni, dhi, ghi, tamb, albedo, basah=False):
    """basah=True: kaca tertutup air (tetesan dan lapisan) dan sel didinginkan air hujan."""
    poa = PL.poa_efektif(math.radians(q1_deg), math.radians(q2_deg), az, el, dni, dhi, ghi, albedo, basah)
    tc = PL.suhu_sel(poa, tamb, basah=basah)
    r = PL.listrik(poa, tc)
    return poa, tc, r


def selftest():
    p = H.P()
    rng = __import__("random").Random(0)
    az, el, dni, dhi, ghi = 90.0, 45.0, 700.0, 170.0, 665.0
    q1, q2 = 90.0 - math.degrees(tracker.HEAD), 45.0          # tepat ke matahari
    assert galat_pointing(q1, q2, az, el) < 1e-6
    v, ex, ey = ldr_baca(q1, q2, az, el, dni, dhi, p, rng, [1.0] * 4)
    assert max(abs(ex), abs(ey)) < 5e-3, (ex, ey)             # LDR seimbang saat tepat sasaran
    v2, ex2, _ = ldr_baca(q1, q2 + 5.0, az, el, dni, dhi, p, rng, [1.0] * 4)
    assert ex2 < ex - 1e-3, (ex, ex2)                         # miring ke +q2: sisi x- lebih terang
    poa, tc, r = listrik(q1, q2, az, el, dni, dhi, ghi, 30.0, 0.25)
    assert 800 < poa < 1000 and 40 < tc < 70 and 1.5 < r["i_mp"] < 3.0, (poa, tc, r["i_mp"])
    d = diag_dict(12.0, az, el, dni, dhi, ghi, q1, q2, q1 + 0.3, q2 - 0.2, p, rng, [1.0] * 4, 30.0, 0.25, 0.0, 0.0, 0.0)
    assert json.loads(json.dumps(d)) == d, "diag harus dapat diserialkan ke JSON"
    assert d["galat"]["beban"] > d["galat"]["perintah"], d["galat"]
    # osilasi: dering buatan 4.8 Hz di atas gerak jejak lambat harus terukur amplitudo dan frekuensinya
    tt = np.arange(0, 2.0, 1e-3)
    lambat = 0.05 * tt                                   # jejak 0.05 deg/s
    dering = 0.4 * np.sin(2 * np.pi * 4.79 * tt)         # puncak ke puncak 0.8 deg
    o = osilasi(tt, lambat + dering)
    assert abs(o["f"] - 4.79) / 4.79 < 0.05, o
    assert abs(o["pp"] - 0.8) / 0.8 < 0.15, o
    o0 = osilasi(tt, lambat)                             # tanpa dering: amplitudo harus nol
    assert o0["pp"] < 1e-6, o0
    assert d["osilasi"]["az"]["pp"] == 0.0
    print("selftest sensor_node: lolos")
    print(f"  tepat sasaran: ex {ex:+.5f} ey {ey:+.5f}; miring 5 deg: ex {ex2:+.5f}")
    print(f"  listrik: POA {poa:.0f} W/m2, suhu sel {tc:.1f} C, Imp {r['i_mp']:.3f} A, Pmp {r['p_mp']:.2f} W")
    print(f"  galat: perintah {d['galat']['perintah']:.3f} deg, beban {d['galat']['beban']:.3f} deg, "
          f"dinamik {d['galat']['dinamik']:+.3f} deg")


def diag_dict(jam, az, el, dni, dhi, ghi, c1, c2, b1, b2, p, rng, mism, tamb, albedo, wh, wh_flat, wh_ideal,
              osil=None, basah=False):
    """Susun seluruh diagnostik jadi satu dict JSON."""
    v, ex, ey = ldr_baca(b1, b2, az, el, dni, dhi, p, rng, mism)
    poa, tc, r = listrik(b1, b2, az, el, dni, dhi, ghi, tamb, albedo, basah)
    g_cmd, g_load = galat_pointing(c1, c2, az, el), galat_pointing(b1, b2, az, el)
    return dict(
        jam=round(jam, 4),
        matahari=dict(az=round(az, 3), el=round(el, 3), zen=round(90 - el, 3),
                      dni=round(dni, 1), dhi=round(dhi, 1), ghi=round(ghi, 1)),
        sendi=dict(cmd_q1=round(c1, 4), cmd_q2=round(c2, 4), beban_q1=round(b1, 4), beban_q2=round(b2, 4),
                   d_q1=round(b1 - c1, 4), d_q2=round(b2 - c2, 4)),
        galat=dict(perintah=round(g_cmd, 4), beban=round(g_load, 4), dinamik=round(g_load - g_cmd, 4),
                   terjangkau=bool((90 - el) <= 45.0)),
        ldr=dict(v=[round(x, 4) for x in v], ex=round(ex, 6), ey=round(ey, 6),
                 mism=[round(x, 4) for x in mism]),
        listrik=dict(poa=round(poa, 1), tcell=round(tc, 2), isc=round(float(r["i_sc"]), 4),
                     imp=round(float(r["i_mp"]), 4), vmp=round(float(r["v_mp"]), 3),
                     pmp=round(float(r["p_mp"]), 3), voc=round(float(r["v_oc"]), 3),
                     basah=bool(basah), t_basah=round(PL.transmisi_basah(), 4) if basah else 1.0),
        energi=dict(wh=round(wh, 3), wh_datar=round(wh_flat, 3), wh_ideal=round(wh_ideal, 3),
                    gain_pct=round((wh / wh_flat - 1) * 100, 2) if wh_flat > 1e-6 else 0.0),
        osilasi=osil or dict(az=dict(pp=0.0, rms=0.0, f=0.0), el=dict(pp=0.0, rms=0.0, f=0.0), hz=0.0, n=0))


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    import rclpy
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, String

    class Sensor(Node):
        def __init__(self):
            super().__init__("sensor_node")
            for k, v in DEFAULTS.items():
                self.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
            self.p = H.P()
            self.rng = __import__("random").Random(int(self.p_("seed")))
            self.mism = self._mism()
            self.sun, self.load = None, [0.0, 0.0]
            self.wh = [0.0, 0.0, 0.0]      # beban, panel datar, tracker ideal
            self.jam_lalu = None
            # antrean dalam: /joint_states terbit 1000 Hz dan statistik ayunan butuh laju penuh (dering 4.8 Hz)
            from collections import deque
            self.buf = deque(maxlen=4000)      # (t, q1, q2) sekitar 4 detik terakhir
            self.create_subscription(JointState, "/joint_states", self.on_js, 4000)
            self.create_subscription(Float64MultiArray, "/sun/state", lambda m: setattr(self, "sun", list(m.data)), 10)
            self.create_subscription(Float64MultiArray, "/hujan/state", lambda m: setattr(self, "hujan", list(m.data)), 10)
            self.pub = self.create_publisher(String, "/tracker/diag", 10)
            self.pub_ldr = self.create_publisher(Float64MultiArray, "/ldr/state", 10)
            self.create_timer(1.0 / HZ, self.tick)
            self.get_logger().info("sensor_node siap (LDR, dioda tunggal panel, energi, uraian galat)")

        def p_(self, k):
            return self.get_parameter(k).value

        def _mism(self):
            m = float(self.p_("mism"))
            mm = [1.0 + self.rng.gauss(0.0, m) for _ in range(4)] if m > 0 else [1.0] * 4
            if bool(self.p_("kalibrasi")):     # prosedur A sempurna: ketidakcocokan dihapus
                mm = [1.0] * 4
            return mm

        def on_js(self, m):
            if "azimuth_joint" in m.name and "elevation_joint" in m.name:
                self.load = [math.degrees(m.position[m.name.index("azimuth_joint")]),
                             math.degrees(m.position[m.name.index("elevation_joint")])]
                self.buf.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, self.load[0], self.load[1]))

        def tick(self):
            if not self.p_("enabled") or self.sun is None:
                return
            s = self.sun
            jam, az, el, ghi, dni, dhi, c1, c2 = s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]
            tamb, alb = float(self.p_("tamb")), float(self.p_("albedo"))
            b1, b2 = self.load
            # kaca BASAH: diputuskan sun_command_node dan disiarkan di /hujan/state, supaya
            # tidak ada dua sumber tanggal yang bisa berbeda diam-diam.
            basah = bool(getattr(self, "hujan", [0.0])[0] > 0.5) if getattr(self, "hujan", None) else False
            # energi: integrasi terhadap JAM SIMULASI, hanya saat waktu maju (sama dengan control_web)
            if self.jam_lalu is not None and 0 < jam - self.jam_lalu < 0.25:
                dh = jam - self.jam_lalu
                poa_b = PL.poa_efektif(math.radians(b1), math.radians(b2), az, el, dni, dhi, ghi, alb)
                cz = math.sin(math.radians(el))
                poa_f = dni * max(0.0, cz) + dhi
                poa_i = dni + dhi * (1 + cz) / 2 + ghi * alb * (1 - cz) / 2
                if basah:
                    poa_b, poa_f, poa_i = (x * PL.transmisi_basah() for x in (poa_b, poa_f, poa_i))
                for i, poa in enumerate((poa_b, poa_f, poa_i)):
                    self.wh[i] += float(PL.listrik(poa, PL.suhu_sel(poa, tamb, basah=basah))["p_mp"]) * dh
            elif self.jam_lalu is not None and jam < self.jam_lalu - 0.5:
                self.wh = [0.0, 0.0, 0.0]      # hari berganti
            self.jam_lalu = jam
            osil = None
            if len(self.buf) > 32:
                bt = list(self.buf)
                tt = [x[0] for x in bt]
                oa, oe = osilasi(tt, [x[1] for x in bt]), osilasi(tt, [x[2] for x in bt])
                osil = dict(az={k: round(v, 5) for k, v in oa.items() if k in ("pp", "rms", "f")},
                            el={k: round(v, 5) for k, v in oe.items() if k in ("pp", "rms", "f")},
                            hz=round(oa["hz"], 1), n=oa["n"])
            d = diag_dict(jam, az, el, dni, dhi, ghi, c1, c2, b1, b2, self.p, self.rng, self.mism,
                          tamb, alb, self.wh[0], self.wh[1], self.wh[2], osil, basah=basah)
            self.pub.publish(String(data=json.dumps(d)))
            m = Float64MultiArray()
            m.data = [float(x) for x in d["ldr"]["v"]] + [d["ldr"]["ex"], d["ldr"]["ey"]]
            self.pub_ldr.publish(m)

    rclpy.init(args=sys.argv)
    node = Sensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
