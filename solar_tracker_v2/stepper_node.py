"""Node ROS 2: model stepper (loop terbuka) yang memberi TORSI ke sendi Gazebo, menggantikan kontrol posisi demo.
Dipakai dengan varian aktuator stepper: ros2 launch ~/mk/solar_tracker/launch/sim.launch.py actuator:=stepper
Masukan: /solar_tracker/azimuth_cmd dan /solar_tracker/elevation_cmd (Float64, rad, sama dengan mode demo, jadi sun_command_node tetap bisa dipakai),
  /joint_states (sudut beban dari Gazebo). Keluaran: /solar_tracker/azimuth_torque dan /elevation_torque (Float64, N.m, dijembatani ke ApplyJointForce).

Model per sumbu (rotor tak bermassa, driver arus ideal): torsi penggerak tau pada beban memenuhi
    tau = Th sin(Nr (theta_cmd - theta_beban - tau / k))        (stepper dengan torsi-sudut sinus seri dengan poros kaku-puntir k)
Bila k > Th Nr (150 N.m/rad) penyelesaiannya unik untuk selisih apa pun dan periodik 360/Nr deg: lepas langkah muncul sendiri
(beban lari bila torsi luar melewati Th) tanpa pelacakan cabang. Perintah posisi dijadikan tangga microstep dengan pembatas laju.
Gravitasi, inersia (CAD), gesekan dan redaman sendi dihitung oleh Gazebo. Rotor Jm diabaikan (mode rotor-poros 270 Hz tidak bisa
disimulasikan lewat jembatan ROS); dinamika lebih lambat dari sekitar 30 Hz sesuai dinamika/dyn_model.py.
Parameter (ros2 param set /stepper_node <nama> <nilai>): enabled, microstep (1/N), rate (deg/s laju perintah maks), accel (deg/s2 percepatan perintah maks,
  profil trapesium: tanpa batas percepatan stepper kehilangan sinkron saat perintah melompat), damping (N.m.s/rad redaman tambahan pada kecepatan beban, bawaan 0),
  lead (s, tambahan kompensasi tundaan: sudut beban diekstrapolasi dengan kecepatan sepanjang usia data + lead),
  angin (true/false: angin NYATA berarah dari angin/angin_openmeteo.py), tanggal (YYYY-MM-DD),
  alpha (eksponen profil angin), pakai_hembusan, skala (pengali uji tekan), extra_torque_az, extra_torque_el (N.m).
  Preset sintetis lama (tenang/berangin/badai) sudah dibuang: data nyata 11 tahun sudah mencakup rentang penuh,
  dan torsinya kini bergantung arah angin serta orientasi panel (torsi azimuth NOL saat panel datar).
Catatan: rotor tak bermassa dan umpan balik lewat jembatan ROS punya tundaan beberapa milidetik; pegas tanpa redaman dengan umpan balik tertunda
  dapat berosilasi atau tidak stabil, sebab itu ada lead dan damping. Keduanya adalah kompensasi numerik, bukan redaman fisik motor.
Keadaan: /stepper/state = [t, az: perintah_deg, beban_deg, selisih_deg, torsi, el: perintah_deg, beban_deg, selisih_deg, torsi, angin].
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "dinamika"))
sys.path.insert(0, str(ROOT / "angin"))
import angin_openmeteo as AO   # noqa: E402  (angin nyata berarah)
import dyn_model as D          # noqa: E402

DEFAULTS = dict(enabled=True, microstep=16.0, rate=30.0, accel=300.0, damping=0.0, lead=0.005,
                angin=False, tanggal="2026-10-07", alpha=AO.ALPHA, pakai_hembusan=False, skala=1.0,
                extra_torque_az=0.0, extra_torque_el=0.0)
JOINT = {"azimuth": "azimuth_joint", "elevasi": "elevation_joint"}
DTW = 0.25   # resolusi deret angin (s), sama dengan gabungan/coupled_sim.py


def stepper_torque(delta, ax):
    """Selesaikan tau = sat(Th sin(Nr e) + Kp e, +-Th) dengan e = delta - tau/k, lewat bisection.
    Kp = 0 (loop terbuka) membuat clip tidak aktif, jadi identik dengan model lama. delta dalam rad."""
    Th, Nr, k, Kp = ax.Th, ax.Nr, ax.k, ax.Kp
    lo, hi = -Th, Th
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        e = delta - mid / k
        if mid - max(-Th, min(Th, Th * math.sin(Nr * e) + Kp * e)) > 0:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def sumbu_sah(ax):
    """Model torsi-sudut ini hanya SAH bila poros lebih kaku daripada pegas magnet yang dirujuk ke beban.

    Syaratnya k > Th_L * Nr_L. Dengan penggerak langsung (N = 1) itu 317 > 75, aman. Dengan reduksi
    gigi, pegas magnet yang dirujuk ke beban naik N kuadrat: pada kemiringan desain baru menjadi
    1.5*5.25 * 50*5.25 = 2067 N.m/rad melawan poros 163.8, sehingga syaratnya GAGAL.

    Akibatnya kurva torsi kuasi-statis menjadi BERNILAI GANDA (ada snap-through) dan bisection di
    stepper_torque tidak lagi menemukan cabang yang benar. Ini keterbatasan model, bukan keterbatasan
    alat: alat nyata memang bisa snap, dan memodelkannya butuh pelacakan cabang dengan histeresis.

    Mode aktuator `pid` TIDAK terpengaruh: ia memakai K_ser linier dan itulah yang dipakai seluruh
    validasi Gazebo sejauh ini.
    """
    return ax.k > ax.Th_L * ax.Nr_L


def selftest():
    """Tanpa ROS: sifat model torsi. Sumbu bergigi dilewati, lihat sumbu_sah()."""
    for nm, ax in D.AXES.items():
        if not sumbu_sah(ax):
            print(f"  ({nm} DILEWATI: k={ax.k:.1f} < Th_L*Nr_L={ax.Th_L * ax.Nr_L:.0f}, "
                  f"reduksi N={ax.N:.2f} membuat torsi bernilai ganda; pakai aktuator pid)")
            continue
        s0 = stepper_torque(0.0, ax)
        assert abs(s0) < 1e-9
        # defleksi kecil: tau ~ K_seri * delta, K_seri = 1/(1/(Th Nr) + 1/k)
        kser = ax.K_ser()
        d = 1e-4
        assert abs(stepper_torque(d, ax) / d - kser) / kser < 0.01, (nm, kser)
        # periodik dalam 2 pi / Nr dan terbatas oleh Th
        for dd in (0.013, 0.05, 0.31):
            t1 = stepper_torque(dd, ax)
            t2 = stepper_torque(dd + 2 * math.pi / ax.Nr, ax)
            assert abs(t1 - t2) < 1e-6 and abs(t1) <= ax.Th + 1e-9
        # defleksi statis untuk torsi beban sama dengan rumus dyn_model/coupled_sim
        tau = 0.9
        delta = float(D.rotor_offset(ax, tau)) + tau / ax.k
        assert abs(stepper_torque(delta, ax) - tau) < 1e-6
    print("selftest stepper_node: lolos")


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    import rclpy
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64, Float64MultiArray

    class Stepper(Node):
        def __init__(self):
            super().__init__("stepper_node")
            for k, v in DEFAULTS.items():
                self.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
            self.ax = D.AXES
            self.tgt = {nm: 0.0 for nm in self.ax}      # rad
            self.cmd = {nm: 0.0 for nm in self.ax}      # rad, perintah terkuantisasi
            self.cmd_f = {nm: 0.0 for nm in self.ax}    # rad, perintah kontinu (sebelum kuantisasi)
            self.th = {nm: 0.0 for nm in self.ax}       # rad, sudut beban dari Gazebo
            self.w = {nm: 0.0 for nm in self.ax}        # rad/s, kecepatan beban dari Gazebo
            self.v = {nm: 0.0 for nm in self.ax}        # rad/s, kecepatan perintah (profil trapesium)
            self.t0 = None
            self.stamp = None       # waktu simulasi (s) pesan joint_states terakhir
            self.wind_cache = None
            self.jam = None          # jam lokal dari /sun/state, untuk memutar ulang angin nyata per jam
            self.create_subscription(Float64MultiArray, "/sun/state",
                                     lambda m: setattr(self, "jam", float(m.data[0])), 10)
            self.n = 0
            self.pub = {"azimuth": self.create_publisher(Float64, "/solar_tracker/azimuth_torque", 10),
                        "elevasi": self.create_publisher(Float64, "/solar_tracker/elevation_torque", 10)}
            self.pub_state = self.create_publisher(Float64MultiArray, "/stepper/state", 10)
            self.create_subscription(Float64, "/solar_tracker/azimuth_cmd", lambda m: self.tgt.__setitem__("azimuth", m.data), 10)
            self.create_subscription(Float64, "/solar_tracker/elevation_cmd", lambda m: self.tgt.__setitem__("elevasi", m.data), 10)
            self.create_subscription(JointState, "/joint_states", self.on_js, 1)
            self.dt = 1 / 200.0
            self.create_timer(self.dt, self.tick)
            buruk = [nm for nm, ax in self.ax.items() if not sumbu_sah(ax)]
            if buruk:
                self.get_logger().error(
                    f"sumbu {buruk} memakai reduksi gigi sehingga k < Th_L*Nr_L: kurva torsi BERNILAI GANDA "
                    f"dan hasil node ini TIDAK SAH untuk sumbu itu. Pakai aktuator pid. Lihat sumbu_sah().")
            self.get_logger().info("stepper_node siap (rotor tak bermassa, torsi-sudut sinus, loop terbuka)")

        def p(self, k):
            return self.get_parameter(k).value

        def on_js(self, m):
            self.stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            for nm, jn in JOINT.items():
                if jn in m.name:
                    i = m.name.index(jn)
                    self.th[nm] = m.position[i]
                    if len(m.velocity) > i:
                        self.w[nm] = m.velocity[i]

        def angin_nyata(self):
            """(v_pakai, arah_dari_deg) dari data nyata pada jam simulasi, sudah dikali `skala`."""
            tgl = str(self.p("tanggal"))
            if self.wind_cache is None or self.wind_cache[0] != tgl:
                self.wind_cache = (tgl, AO.hari(tgl))
            d = self.wind_cache[1]
            j = int(self.jam if self.jam is not None else 12.0) % 24
            lst = d["jam"].tolist()
            i = lst.index(j) if j in lst else 0
            a, sk = float(self.p("alpha")), max(0.0, float(self.p("skala")))
            kol = "hembusan10" if bool(self.p("pakai_hembusan")) else "v10"
            return float(AO.v_pada_panel(d[kol][i], a)) * sk, float(d["arah"][i])

        def tick(self):
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.t0 is None:
                self.t0 = now
            t = now - self.t0
            if not self.p("enabled"):
                return
            step = math.radians(1.8 / float(self.p("microstep")))
            vmax, acc = math.radians(float(self.p("rate"))), math.radians(float(self.p("accel")))
            damp = float(self.p("damping"))
            stale = max(0.0, min(0.05, now - self.stamp)) if self.stamp is not None else 0.0     # usia data sudut beban (s)
            horizon = stale + float(self.p("lead"))
            v, arah = self.angin_nyata() if bool(self.p("angin")) else (0.0, 0.0)
            tw = dict(zip(("azimuth", "elevasi"),
                          AO.torsi(v, arah, math.degrees(self.th["azimuth"]), math.degrees(self.th["elevasi"]))
                          if v > 0 else (0.0, 0.0)))
            extra = {"azimuth": float(self.p("extra_torque_az")), "elevasi": float(self.p("extra_torque_el"))}
            row = [t]
            for nm, ax in self.ax.items():
                e = self.tgt[nm] - self.cmd_f[nm]
                vdes = math.copysign(min(vmax, math.sqrt(2 * acc * abs(e))), e)     # profil trapesium: berhenti tepat di sasaran
                self.v[nm] += max(-acc * self.dt, min(acc * self.dt, vdes - self.v[nm]))
                self.cmd_f[nm] += self.v[nm] * self.dt
                if e * (self.tgt[nm] - self.cmd_f[nm]) < 0:        # melewati sasaran: berhenti di sasaran
                    self.cmd_f[nm], self.v[nm] = self.tgt[nm], 0.0
                self.cmd[nm] = round(self.cmd_f[nm] / step) * step     # tangga microstep
                th_pred = self.th[nm] + self.w[nm] * horizon      # kompensasi tundaan: sudut beban diekstrapolasi
                delta = self.cmd[nm] - th_pred
                tau = stepper_torque(delta, ax) - damp * self.w[nm] + tw[nm] + extra[nm]
                self.pub[nm].publish(Float64(data=tau))
                row += [math.degrees(self.cmd[nm]), math.degrees(self.th[nm]), math.degrees(delta), tau]
            self.n += 1
            if self.n % 20 == 0:
                st = Float64MultiArray()
                st.data = row + [v, stale]
                self.pub_state.publish(st)

    rclpy.init(args=sys.argv)
    node = Stepper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
