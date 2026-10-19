"""Node ROS 2 OPSIONAL: perintah sendi dari pvlib untuk tanggal pilihan dengan waktu dipercepat.
Mati secara bawaan di launch (sun_node:=false). Tanpa ROS: python sun_command_node.py --selftest
Parameter (bisa diubah saat berjalan: ros2 param set /sun_command_node <nama> <nilai>):
  date      tanggal YYYY-MM-DD (bawaan 2026-10-04), lintasan matahari berbeda tiap tanggal
  speed     detik simulasi per detik nyata (bawaan 600: satu hari siang sekitar 72 detik)
  enabled   true/false: jeda atau lanjut (false = tidak mengirim perintah, sendi bisa digerakkan manual)
  okta      0..8, tutupan awan untuk irradiance pada /sun/state (tidak memengaruhi arah panel)
  loop      ulangi hari bila selesai (bawaan true)
  q2max     batas kemiringan (deg, bawaan 45, asumsi)
  az_rate, tilt_rate   pembatas laju perintah (deg/s waktu nyata, bawaan 90)
  az_accel, tilt_accel pembatas percepatan perintah (deg/s2, bawaan 300): profil trapesium, menekan ayunan sekitar 50 kali
  shaper    none|zv|zvd peredam ayunan pada perintah (bawaan zvd). Impuls dihitung dari frekuensi dan redaman tiap sumbu
            (dinamika/dyn_model.py): azimuth 4.79 Hz, elevasi 6.89 Hz. Tidak berpengaruh saat menjejak pelan
            (satu microstep tidak bisa dipecah), sangat berpengaruh saat lompat jam, ganti tanggal, nyala awal, dan parkir
  marker    terbitkan panah arah matahari di /sun_marker (frame world)
  hour      jam lokal tetap (5..19): waktu dibekukan di jam itu (untuk geser jam manual); -1 = waktu berjalan otomatis
  gz_light  true: arah dan warna lampu 'sun' di Gazebo mengikuti posisi matahari (layanan gz light_config, sekitar 1 kali per detik nyata, hanya visual)
  manual    true: perintah sendi TIDAK dikirim (panel manual yang mengirim), keadaan matahari dan marker tetap terbit
Topik keluar: /solar_tracker/azimuth_cmd, /solar_tracker/elevation_cmd (Float64, rad),
  /sun/state (Float64MultiArray: jam_lokal, azimuth_deg, elevasi_deg, ghi, dni, dhi, q1_deg, q2_deg), /sun_marker.
Catatan: pada speed tinggi, lintasan dekat zenith menuntut laju azimuth sebesar speed kali laju nyata, jadi panel tertinggal
(efek keyhole yang dibesar-besarkan oleh percepatan waktu); dengan speed 1 geraknya realistis.
"""
import datetime as dt
import math
import sys
import threading
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent   # folder proyek (berisi sun.py dan tracker.py)
sys.path.insert(0, str(ROOT))
import sun       # noqa: E402
import tracker   # noqa: E402
sys.path.insert(0, str(ROOT / "dinamika"))
import dyn_model as D   # noqa: E402  (frekuensi dan redaman sumbu untuk input shaper)

DEFAULTS = dict(date="2026-10-04", speed=600.0, enabled=True, okta=0.0, loop=True, q2max=45.0, az_rate=90.0, tilt_rate=90.0, marker=True, hour=-1.0, manual=False, gz_light=True,
                az_accel=300.0, tilt_accel=300.0, shaper="zvd", pakai_ekf=False)
RATE_HZ = 100.0   # laju tick: jeda shaper ZVD azimuth 104 ms = 10.4 cuplikan, galat pembulatan sekitar 4 persen (ZVD tahan sebesar ini)


def trajectory(date, okta=0.0, q2max=45.0, step="10s", elmin=5.0):
    """Lintasan siang (elevasi >= elmin) satu tanggal: waktu (s sejak awal), sudut sendi kontinu (rad), irradiance, az dan el matahari."""
    d = sun.sun_day(date, freq=step, okta=float(okta))
    d = d[d.el >= elmin]
    if d.empty:
        raise ValueError(f"tidak ada siang (elevasi >= {elmin} deg) pada {date}")
    q1, q2 = tracker.ik_track(d.az.values, d.el.values, q2_max=math.radians(90))   # cabang kontinu tanpa batas, lalu dipotong
    q2 = np.clip(q2, -math.radians(q2max), math.radians(q2max))
    return dict(t=(d.index - d.index[0]).total_seconds().values, q1=q1, q2=q2, az=d.az.values, el=d.el.values,
                ghi=d.ghi.values, dni=d.dni.values, dhi=d.dhi.values, t0=d.index[0], t1=d.index[-1],
                hour0=d.index[0].hour + d.index[0].minute / 60 + d.index[0].second / 3600)


def selftest():
    """Tanpa ROS: bandingkan lintasan beberapa tanggal 2026 dan periksa FK(IK) = arah matahari."""
    dates = ["2026-01-01", "2026-02-01", "2026-03-02", "2026-03-20", "2026-04-01", "2026-05-01", "2026-06-21",
             "2026-08-01", "2026-09-23", "2026-10-04", "2026-10-10", "2026-11-01", "2026-12-21"]
    r = light_request(90.0, 45.0)   # matahari di Timur (-y): cahaya bergerak ke Barat (+y) dan ke bawah
    assert 'y: 0.7071 z: -0.7071' in r and 'x: -0.0000' in r.replace('x: 0.0000', 'x: -0.0000'), r
    assert ball_request(90.0, 90.0) == 'name: "sun_ball" position {x: 0.000 y: -0.000 z: 3.200}' or 'z: 3.200' in ball_request(90.0, 90.0)
    assert light_request(90.0, -10.0).count('intensity: 0.080') == 1
    print(f"{'tanggal':11s} {'terbit':>6s} {'terbenam':>8s} {'siang(j)':>8s} {'el maks':>7s} {'jam':>5s} {'matahari di':>12s} {'terjangkau(j)':>13s} {'rentang q1':>10s}")
    rows = []
    for ds in dates:
        tr = trajectory(ds)
        i = int(np.argmax(tr["el"]))
        zen = 90 - tr["el"]
        reach = float((zen <= 45).sum() * 10 / 3600)
        side = "utara" if (tr["az"][i] < 90 or tr["az"][i] > 270) else "selatan"
        rows.append((ds, tr["el"].max()))
        hour_max = tr["hour0"] + tr["t"][i] / 3600
        print(f"{ds:11s} {tr['t0']:%H:%M} {tr['t1']:%H:%M} {len(tr['t']) * 10 / 3600:8.1f} {tr['el'].max():7.1f} {hour_max:5.2f} "
              f"{side:>12s} {reach:13.1f} {math.degrees(np.ptp(tr['q1'])):10.0f}")
        ok = zen <= 45
        for k in np.flatnonzero(ok)[::50]:
            assert np.allclose(tracker.fk(tr["q1"][k], tr["q2"][k]), tracker.sun_vec(tr["az"][k], tr["el"][k]), atol=1e-9), (ds, k)
    emax = dict(rows)
    assert abs(emax["2026-06-21"] - emax["2026-03-02"]) > 5.0, "lintasan harus berbeda antar tanggal"
    assert len({round(v, 1) for v in emax.values()}) > 6, "elevasi puncak harus bervariasi sepanjang tahun"
    print("selftest: lolos (FK(IK) = arah matahari saat terjangkau; lintasan berbeda antar tanggal)")


def light_request(az, el):
    """Isi pesan gz.msgs.Light untuk lampu 'sun': arah = dari matahari ke tanah (-sun_vec), intensitas dan warna menurut elevasi.
    Visual saja (irradiance fisis tetap dari pvlib). Intensitas HARUS diisi: nilai bawaan protobuf 0 membuat dunia gelap."""
    v = -tracker.sun_vec(az, el)
    c = float(np.clip(el / 30.0, 0.0, 1.0))                  # warna lebih hangat saat matahari rendah
    inten = 0.08 + 0.92 * float(np.clip(el / 20.0, 0.0, 1.0))  # redup saat malam, bukan nol
    return (f'name: "sun" type: DIRECTIONAL direction {{x: {v[0]:.4f} y: {v[1]:.4f} z: {v[2]:.4f}}} '
            f'diffuse {{r: 1 g: {0.75 + 0.25 * c:.3f} b: {0.55 + 0.45 * c:.3f} a: 1}} specular {{r: 0.3 g: 0.3 b: 0.3 a: 1}} '
            f'intensity: {inten:.3f} cast_shadows: true')


def shaper_taps(kind, ax, dt):
    """Impuls input shaper -> (amplitudo, jeda dalam cuplikan) untuk satu sumbu. Lihat dinamika/osilasi_energi.py."""
    if kind not in ("zv", "zvd"):
        return [(1.0, 0)]
    K_ser = ax.K_ser()   # ikut Kp: shaper MERETUNE DIRI saat loop ditutup
    zeta = ax.bl / (2 * math.sqrt(K_ser * ax.JL))
    f = math.sqrt(K_ser / ax.JL) / (2 * math.pi)
    K = math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2))
    d = 1.0 / (2 * f * math.sqrt(1 - zeta ** 2))
    n = max(1, int(round(d / dt)))
    if kind == "zv":
        return [(1 / (1 + K), 0), (K / (1 + K), n)]
    sq = (1 + K) ** 2
    return [(1 / sq, 0), (2 * K / sq, n), (K * K / sq, 2 * n)]


def profile_step(q, v, tgt, vmax, amax, dt):
    """Satu langkah profil trapesium (laju dan percepatan dibatasi, berhenti tepat di sasaran)."""
    e = tgt - q
    vdes = math.copysign(min(vmax, math.sqrt(2 * amax * abs(e))), e) if e else 0.0
    v += max(-amax * dt, min(amax * dt, vdes - v))
    q += v * dt
    if e * (tgt - q) < 0:
        q, v = tgt, 0.0
    return q, v


def ball_request(az, el, r=3.2):
    """Posisi bola matahari (model 'sun_ball') pada jarak r dari pusat tracker, menuju matahari; di bawah horizon bola masuk ke tanah (tak terlihat)."""
    v = r * tracker.sun_vec(az, el)
    return f'name: "sun_ball" position {{x: {v[0]:.3f} y: {v[1]:.3f} z: {v[2]:.3f}}}'


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    import rclpy
    from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
    from rclpy.node import Node
    from std_msgs.msg import Float64, Float64MultiArray
    from visualization_msgs.msg import Marker

    class SunCommand(Node):
        def __init__(self):
            super().__init__("sun_command_node")
            for k, v in DEFAULTS.items():
                self.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
            self.add_on_set_parameters_callback(self.on_params)
            self.pub_az = self.create_publisher(Float64, "/solar_tracker/azimuth_cmd", 10)
            self.pub_el = self.create_publisher(Float64, "/solar_tracker/elevation_cmd", 10)
            self.pub_state = self.create_publisher(Float64MultiArray, "/sun/state", 10)
            self.pub_marker = self.create_publisher(Marker, "/sun_marker", 10)
            self.ekf_kor = [0.0, 0.0]   # taksiran simpangan dari ekf_node; dipakai HANYA bila pakai_ekf true
            self.create_subscription(Float64MultiArray, "/ekf/koreksi",
                                     lambda m: setattr(self, "ekf_kor", list(m.data)[:2]), 10)
            self.light_req, self.light_last = None, None
            threading.Thread(target=self.light_worker, daemon=True).start()
            self.dt0 = 1.0 / RATE_HZ
            self.taps = [shaper_taps(str(self.p("shaper")), D.AXES[a], self.dt0) for a in ("azimuth", "elevasi")]
            self.prof = [0.0, 0.0]      # keluaran profil trapesium (sebelum shaper)
            self.pv = [0.0, 0.0]        # laju profil
            self.hist = [[0.0] * 64, [0.0] * 64]   # riwayat profil untuk shaper (64 cuplikan = 0.64 s pada 100 Hz)
            self.dt, self.sim_t, self.prev, self.tr, self.reload_flag, self.n, self.done = 0.05, 0.0, [0.0, 0.0], None, True, 0, False
            self.create_timer(self.dt0, self.tick)

        def light_worker(self):
            """Kirim arah lampu ke Gazebo di thread sendiri (satu panggilan gz sekitar 1 detik). Berhenti bila gz tidak ada atau layanan tidak merespons 3 kali."""
            import subprocess, time
            fails = 0
            calls = (("light_config", "gz.msgs.Light"), ("set_pose", "gz.msgs.Pose"))
            while fails < 3:
                req, self.light_req = self.light_req, None
                if req is None:
                    time.sleep(0.2); continue
                try:   # dua panggilan gz serentak (lampu dan bola matahari), tiap panggilan sekitar 1 detik
                    ps = [subprocess.Popen(["gz", "service", "-s", f"/world/solar_world/{svc}", "--reqtype", typ, "--reptype", "gz.msgs.Boolean",
                                            "--timeout", "1500", "--req", r], stdout=subprocess.PIPE, text=True) for (svc, typ), r in zip(calls, req)]
                    ok = all("true" in p.communicate(timeout=5)[0] for p in ps)
                    fails = 0 if ok else fails + 1
                except (OSError, subprocess.TimeoutExpired):
                    fails += 1
            self.get_logger().warning("lampu Gazebo: layanan light_config tidak tersedia, pembaruan lampu dihentikan (visual saja)")

        def p(self, k):
            return self.get_parameter(k).value

        def on_params(self, params):
            for q in params:
                if q.name == "date":
                    try:
                        dt.date.fromisoformat(str(q.value))
                    except ValueError:
                        return SetParametersResult(successful=False, reason="date harus YYYY-MM-DD")
                if q.name == "shaper":
                    self.taps = [shaper_taps(str(q.value), D.AXES[a], self.dt0) for a in ("azimuth", "elevasi")]
                if q.name in ("date", "okta", "q2max"):
                    self.reload_flag = True
            return SetParametersResult(successful=True)

        def reload(self):
            ds = str(self.p("date"))
            if not ds.startswith("2026"):
                self.get_logger().warn(f"tanggal {ds} di luar 2026 (tetap dihitung)")
            self.tr = trajectory(ds, self.p("okta"), float(self.p("q2max")))
            self.sim_t, self.reload_flag, self.done = 0.0, False, False
            tr = self.tr
            self.get_logger().info(f"{ds}: terbit {tr['t0']:%H:%M}, terbenam {tr['t1']:%H:%M}, elevasi maks {tr['el'].max():.1f} deg, "
                                   f"zenith minimum {90 - tr['el'].max():.1f} deg, kecepatan {float(self.p('speed')):g}x")

        def tick(self):
            if self.reload_flag:
                self.reload()
            tr = self.tr
            hr = float(self.p("hour"))
            if hr >= 0:     # jam dibekukan (geser jam manual)
                self.sim_t, self.done = float(np.clip((hr - tr["hour0"]) * 3600, 0, tr["t"][-1])), False
            elif not self.p("enabled") or self.done:
                return
            else:
                self.sim_t += float(self.p("speed")) * self.dt
            if hr < 0 and self.sim_t > tr["t"][-1]:
                if self.p("loop"):
                    self.sim_t = 0.0
                else:
                    self.done = True
                    self.get_logger().info("hari selesai (loop=false), perintah terakhir dipertahankan")
                    return
            g = lambda k: float(np.interp(self.sim_t, tr["t"], tr[k]))
            tgt = [g("q1"), g("q2")]
            if self.p("pakai_ekf"):   # loop tertutup: perintah digeser melawan simpangan yang ditaksir EKF
                tgt = [t - math.radians(k) for t, k in zip(tgt, self.ekf_kor)]
            # profil trapesium lalu input shaper: perintah mentah membuat ayunan sampai 137 deg saat lompat besar
            vmax = [math.radians(float(self.p("az_rate"))), math.radians(float(self.p("tilt_rate")))]
            amax = [math.radians(float(self.p("az_accel"))), math.radians(float(self.p("tilt_accel")))]
            for i in (0, 1):
                self.prof[i], self.pv[i] = profile_step(self.prof[i], self.pv[i], tgt[i], vmax[i], amax[i], self.dt0)
                self.hist[i].append(self.prof[i])
                del self.hist[i][0]
                self.prev[i] = sum(a * self.hist[i][-1 - n] for a, n in self.taps[i])
            if not self.p("manual"):
                self.pub_az.publish(Float64(data=self.prev[0]))
                self.pub_el.publish(Float64(data=self.prev[1]))
            az, el = g("az"), g("el")
            st = Float64MultiArray()
            st.data = [tr["hour0"] + self.sim_t / 3600, az, el, g("ghi"), g("dni"), g("dhi"), math.degrees(self.prev[0]), math.degrees(self.prev[1])]
            self.pub_state.publish(st)
            if self.p("gz_light") and self.n % 50 == 0:
                self.light_req = (light_request(az, el), ball_request(az, el))   # thread worker mengirim yang terbaru saja
            if self.p("marker") and self.n % 20 == 0:     # marker 5 Hz (20 Hz membuat RViz berat di render perangkat lunak WSLg)
                s = tracker.sun_vec(az, el)
                m = Marker()
                m.header.frame_id, m.ns, m.id, m.type, m.action = "world", "sun", 0, Marker.ARROW, Marker.ADD
                from geometry_msgs.msg import Point
                m.points = [Point(x=0.0, y=0.0, z=0.26), Point(x=0.7 * s[0], y=0.7 * s[1], z=0.26 + 0.7 * s[2])]
                m.scale.x, m.scale.y, m.scale.z = 0.02, 0.04, 0.06
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.0, 1.0
                self.pub_marker.publish(m)
                h = st.data[0]
                tx = Marker()
                tx.header.frame_id, tx.ns, tx.id, tx.type, tx.action = "world", "sun_text", 1, Marker.TEXT_VIEW_FACING, Marker.ADD
                tx.pose.position.z = 0.85
                tx.scale.z = 0.05
                tx.color.r = tx.color.g = tx.color.b = tx.color.a = 1.0
                tx.text = f"{self.p('date')}  {int(h):02d}:{int((h % 1) * 60):02d} WIB  el {el:.0f} deg"
                self.pub_marker.publish(tx)
                if self.n % int(2 / self.dt) == 0:     # jejak lintasan matahari seharian (ujung panah tiap 1 menit), diperbarui 1 kali per detik
                    ln = Marker()
                    ln.header.frame_id, ln.ns, ln.id, ln.type, ln.action = "world", "sun_path", 2, Marker.LINE_STRIP, Marker.ADD
                    ln.scale.x = 0.006
                    ln.color.r, ln.color.g, ln.color.b, ln.color.a = 1.0, 0.5, 0.0, 0.8
                    ln.points = [Point(x=0.7 * v[0], y=0.7 * v[1], z=0.26 + 0.7 * v[2]) for v in (tracker.sun_vec(a_, e_) for a_, e_ in zip(tr["az"][::6], tr["el"][::6]))]
                    self.pub_marker.publish(ln)
            self.n += 1
            if self.n % int(5 / self.dt) == 0:
                self.get_logger().info(f"jam {st.data[0]:5.2f}  az {az:6.1f}  el {el:5.1f}  perintah q1 {st.data[6]:7.1f}  q2 {st.data[7]:5.1f} deg")

    rclpy.init(args=sys.argv)
    node = SunCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
