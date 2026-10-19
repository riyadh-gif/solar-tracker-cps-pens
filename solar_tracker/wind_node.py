"""Node ROS 2: angin dan torsi gangguan untuk aktuator `pid` (kekakuan seri stepper), tanpa stepper_node.
Mode pid memakai kontrol posisi bergain K = kekakuan seri stepper dan poros (build_model.stiffness_gains). Gaya kontrol F = K (cmd - q) bersifat linier,
jadi torsi luar tau setara persis dengan pergeseran perintah delta = tau / K (M q'' = K (cmd - q) + tau = K (cmd + tau/K - q)); kejenuhan Th diabaikan
(lepas langkah tidak dimodelkan di mode pid, itu urusan actuator:=stepper). Gazebo tidak menjumlahkan ApplyJointForce dengan pengendali posisi (diuji: tak berefek),
sebab itu angin disuntikkan lewat perintah.
Masukan: /solar_tracker/azimuth_cmd dan /elevation_cmd (Float64, rad; dari sun_command_node, panel, atau skrip), /joint_states.
Keluaran: /solar_tracker/azimuth_cmd_gz dan /elevation_cmd_gz (Float64, rad, dijembatani ke Gazebo; = cmd + delta, diterbitkan 100 Hz), /wind/state.
/wind/state = [t, v_panel, tau_az, tau_el, delta_az_deg, delta_el_deg, arah_DARI_deg, v10, hembusan10, hembusan_panel].
  `arah_DARI_deg` adalah bearing kompas tempat angin DATANG; angin BERTIUP MENUJU arah itu + 180 derajat.

ANGIN SELALU DARI DATA NYATA. Sumbernya `angin/angin_openmeteo.py`: reanalisis Open-Meteo per jam untuk PENS,
11 tahun (2015 sampai 2026), pada 10 m, dikoreksi ke tinggi panel dengan profil pangkat, LENGKAP DENGAN ARAH.
Torsinya bergantung arah angin DAN orientasi panel, sehingga torsi azimuth NOL saat panel datar.

Preset sintetis lama (`tenang`, `berangin`, `badai`) SUDAH DIBUANG dari jalur langsung. Alasannya: maksimum
hembusan tahunan di Surabaya sangat stabil (rata 16.28 m/s pada 10 m, sb 1.29 atas 11 tahun), jadi data nyata
sudah mencakup rentang penuh; dan preset itu salah kalibrasi, `badai` 11 m/s adalah dua kali apa pun yang
pernah tercatat di tinggi panel. Untuk menguji DI LUAR rekaman, pakai `skala`: mengali data nyata dapat
dipertanggungjawabkan, angka karangan tidak. (`gabungan/coupled_sim.wind_series` masih ada untuk studi luring
lama, tetapi tidak lagi dipakai node mana pun.)

Parameter (ros2 param set /wind_node <nama> <nilai>): enabled, tanggal (YYYY-MM-DD, hari yang diputar ulang),
  alpha (eksponen profil angin, bawaan 0.30; 0.16 lapangan terbuka, 0.35 kampus rapat), pakai_hembusan
  (true: pakai nilai hembusan, bukan rata-rata jam), skala (pengali uji tekan, 1.0 = apa adanya;
  3.0 memberi sekitar 83 persen torsi tahan), extra_torque_az, extra_torque_el (N.m, torsi uji tambahan).
Tanpa node ini, mode pid tidak menggerakkan sendi (topik _cmd tidak sampai ke Gazebo): launch menyalakannya otomatis bila actuator:=pid.
Tanpa ROS: python wind_node.py --selftest"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "dinamika"))
sys.path.insert(0, str(ROOT / "angin"))
import angin_openmeteo as AO   # noqa: E402  (angin nyata berarah)
import dyn_model as D          # noqa: E402

DEFAULTS = dict(enabled=True, tanggal="2026-10-07", alpha=AO.ALPHA, pakai_hembusan=False, skala=1.0,
                extra_torque_az=0.0, extra_torque_el=0.0)
JOINT = {"azimuth": "azimuth_joint", "elevasi": "elevation_joint"}
DTW = 0.25   # resolusi deret angin (s), sama dengan gabungan/coupled_sim.py


def kseri(ax):
    return ax.K_ser()   # satu sumber: dyn_model.Axis.K_ser (ikut Kp loop tertutup)


def delta_cmd(ax, tau):
    """Pergeseran perintah (rad) yang setara dengan torsi luar tau (N.m)."""
    return tau / kseri(ax)


def selftest():
    for nm, ax in D.AXES.items():
        assert abs(delta_cmd(ax, kseri(ax)) - 1.0) < 1e-12
    # nilai yang dipakai build_model.stiffness_gains. Tripwire: SENGAJA dipatok supaya perubahan Th atau k gagal berisik.
    # Th 3.0 -> 1.5 (57HSE1.5N) menurunkan ini dari 137.14 / 101.82.
    assert abs(kseri(D.AXES["azimuth"]) - 71.64) < 0.01 and abs(kseri(D.AXES["elevasi"]) - 60.65) < 0.01
    print("selftest wind_node: lolos")


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    import numpy as np   # noqa: F401
    import rclpy
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64, Float64MultiArray

    class Wind(Node):
        def __init__(self):
            super().__init__("wind_node")
            for k, v in DEFAULTS.items():
                self.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
            self.cmd, self.q2, self.t0, self.n, self.cache = {"azimuth": 0.0, "elevasi": 0.0}, 0.0, None, 0, None
            self.q1 = 0.0            # sudut beban azimuth: torsi angin bergantung orientasi panel
            self.jam = None          # jam lokal dari /sun/state, untuk memutar ulang data angin per jam
            self.hari_cache = None
            self.pub = {"azimuth": self.create_publisher(Float64, "/solar_tracker/azimuth_cmd_gz", 10),
                        "elevasi": self.create_publisher(Float64, "/solar_tracker/elevation_cmd_gz", 10)}
            self.create_subscription(Float64, "/solar_tracker/azimuth_cmd", lambda m: self.cmd.__setitem__("azimuth", m.data), 10)
            self.create_subscription(Float64, "/solar_tracker/elevation_cmd", lambda m: self.cmd.__setitem__("elevasi", m.data), 10)
            self.create_subscription(JointState, "/joint_states", self.on_js, 1)
            self.create_subscription(Float64MultiArray, "/sun/state",
                                     lambda m: setattr(self, "jam", float(m.data[0])), 10)
            self.pub_state = self.create_publisher(Float64MultiArray, "/wind/state", 10)
            self.create_timer(0.01, self.tick)
            self.get_logger().info("wind_node siap (angin sebagai pergeseran perintah, mode pid)")

        def p(self, k):
            return self.get_parameter(k).value

        def on_js(self, m):
            if "elevation_joint" in m.name:
                self.q2 = m.position[m.name.index("elevation_joint")]
            if "azimuth_joint" in m.name:
                self.q1 = m.position[m.name.index("azimuth_joint")]

        def angin_nyata(self):
            """(v_panel, hembusan_panel, arah_dari_deg, v10, hembusan10) dari Open-Meteo pada jam simulasi sekarang."""
            tgl = str(self.p("tanggal"))
            if self.hari_cache is None or self.hari_cache[0] != tgl:
                self.hari_cache = (tgl, AO.hari(tgl))
            d = self.hari_cache[1]
            j = int(self.jam if self.jam is not None else 12.0) % 24
            i = int(d["jam"].tolist().index(j)) if j in d["jam"].tolist() else 0
            a = float(self.p("alpha"))
            return (float(AO.v_pada_panel(d["v10"][i], a)), float(AO.v_pada_panel(d["hembusan10"][i], a)),
                    float(d["arah"][i]), float(d["v10"][i]), float(d["hembusan10"][i]))

        def tick(self):
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.t0 is None:
                self.t0 = now
            on = bool(self.p("enabled"))
            extra = {"azimuth": float(self.p("extra_torque_az")), "elevasi": float(self.p("extra_torque_el"))}
            v_panel = hembus = v10 = hembus10 = arah = 0.0
            tau = {"azimuth": 0.0, "elevasi": 0.0}
            if on:
                sk = max(0.0, float(self.p("skala")))
                v_panel, hembus, arah, v10, hembus10 = self.angin_nyata()
                v_panel, hembus = v_panel * sk, hembus * sk
                v = hembus if bool(self.p("pakai_hembusan")) else v_panel
                ta, te = AO.torsi(v, arah, math.degrees(self.q1), math.degrees(self.q2))
                tau = {"azimuth": ta, "elevasi": te}
            dls = {}
            for nm, ax in D.AXES.items():
                t_tot = tau[nm] + extra[nm]
                d = delta_cmd(ax, t_tot)
                self.pub[nm].publish(Float64(data=float(self.cmd[nm] + d)))
                tau[nm], dls[nm] = t_tot, math.degrees(d)
            self.n += 1
            if self.n % 10 == 0:
                st = Float64MultiArray()
                st.data = [now - self.t0, v_panel, tau["azimuth"], tau["elevasi"], dls["azimuth"], dls["elevasi"],
                           arah, v10, hembus10, hembus]
                self.pub_state.publish(st)

    rclpy.init(args=sys.argv)
    node = Wind()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
