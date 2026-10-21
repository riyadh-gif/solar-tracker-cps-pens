"""Panel kontrol manual berbasis WEB (dibuka di browser Windows, tanpa jendela WSLg): http://localhost:8765
Isi sama dengan control_panel.py (Tkinter): mode otomatis/manual, tanggal, jam, kecepatan, awan, sendi manual, microstep, angin, torsi gangguan, pembacaan langsung.
Dipakai lewat: ros2 launch ~/mk/solar_tracker/launch/sim.launch.py sun_node:=true rviz:=true panel:=true [actuator:=stepper]
Opsi: --port N, --no-open (jangan buka browser Windows otomatis)."""
import datetime as dt
import json
import math
import queue
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import tracker  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "angin"))
import angin_openmeteo as AO  # noqa: E402  (nama mata angin untuk tampilan)
from control_panel import Panel  # noqa: E402  (node ROS yang sama dengan panel Tkinter)

import rclpy  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402

PORT = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 8765
HTML = Path(__file__).parent / "control_web.html"


def page():
    """Dibaca ULANG tiap permintaan: menyunting control_web.html cukup disegarkan di browser, tanpa menjalankan
    ulang node. Dulu dibaca sekali saat impor, dan itu membuat suntingan tampak tidak berpengaruh."""
    return HTML.read_text(encoding="utf-8")
Q = queue.Queue()
S = dict(mode="auto", run=True, hour=12.0, doy=277, q1=0.0, q2=0.0, hujan_mode="auto")   # keadaan kontrol (dibaca halaman)
AREA, EFF, ALB = 0.670 * 0.445, 50.0 / (0.670 * 0.445 * 1000.0), 0.25   # MOS C-50-18-M: luas 0.2982 m2, efisiensi = Pmax 50 W / (luas x 1000 W/m2) = 16.8%, albedo (sama dengan sun.poa)
E = dict(wh=0.0, flat=0.0, ideal=0.0, last_h=None, last_doy=277, log=[], t_log=0.0)   # energi terkumpul (Wh) hari ini


def date_of(d):
    return (dt.date(2026, 1, 1) + dt.timedelta(days=int(d) - 1)).isoformat()


def apply(node, name, v):
    """Dijalankan di utas utama (aman untuk rclpy)."""
    sun = lambda n, x: node.set_param("sun_command_node", n, x)
    stp = lambda n, x: node.set_param("stepper_node", n, x) | node.set_param("wind_node", n, x)   # mode stepper: stepper_node, mode pid: wind_node
    if name == "mode":
        S["mode"] = v
        sun("manual", v == "manual")
        if v == "manual":
            S["q1"], S["q2"] = round(node.load[0], 1), round(node.load[1], 1)
    elif name == "doy":
        S["doy"] = int(v); sun("date", date_of(v))
    elif name == "hour":
        S["hour"], S["run"] = float(v), False; sun("hour", float(v))
    elif name == "run":
        S["run"] = bool(v); sun("hour", -1.0 if v else S["hour"])
    elif name == "speed":
        sun("speed", float(v))
    elif name == "okta":
        sun("okta", float(v))
    elif name == "hujan_mode":
        S["hujan_mode"] = str(v); sun("hujan_mode", str(v))
    elif name in ("q1", "q2"):
        S[name] = float(v)
    elif name == "zero":
        S["q1"] = S["q2"] = 0.0
    elif name == "microstep":
        stp("microstep", float(v))
    elif name == "skala":
        stp("skala", float(v))
    elif name == "alpha":
        stp("alpha", float(v))
    elif name == "hembus":
        stp("pakai_hembusan", bool(v))
    elif name == "xaz":
        stp("extra_torque_az", float(v))
    elif name == "xel":
        stp("extra_torque_el", float(v))
    elif name == "reset_energi":
        E.update(wh=0.0, flat=0.0, ideal=0.0, last_h=None, log=[], t_log=0.0)
    elif name == "reset":
        stp("extra_torque_az", 0.0); stp("extra_torque_el", 0.0); node.slipped = False


def irradiance(node):
    """Irradiance bidang (W/m2) pada panel dari sudut BEBAN, panel datar, dan tracker ideal (normal ke matahari), model isotropik seperti sun.poa."""
    sn = node.sun
    if not sn:
        return None
    az, el, ghi, dni, dhi = sn[1], sn[2], sn[3], sn[4], sn[5]
    n = tracker.fk(math.radians(node.load[0]), math.radians(node.load[1]))
    cth = max(0.0, float(n @ tracker.sun_vec(az, el)))
    ct = float(n[2])                     # cos kemiringan panel
    poa = dni * cth + dhi * (1 + ct) / 2 + ghi * ALB * (1 - ct) / 2
    flat = dni * max(0.0, math.sin(math.radians(el))) + dhi
    cz = math.sin(math.radians(el))      # tracker ideal: kemiringan = zenith matahari, normal tepat ke matahari
    ideal = dni + dhi * (1 + cz) / 2 + ghi * ALB * (1 - cz) / 2
    return dict(poa=poa, flat=flat, ideal=ideal, cth=cth)


def accumulate(node):
    """Integrasi energi terhadap jam simulasi (hanya saat waktu maju), log tiap sekitar 10 menit simulasi."""
    sn, ir = node.sun, irradiance(node)
    if not ir or sn[2] <= 0:
        return
    h = sn[0]
    if S["doy"] != E["last_doy"] or (E["last_h"] is not None and h < E["last_h"] - 0.5):
        E.update(wh=0.0, flat=0.0, ideal=0.0, last_h=None, log=[], t_log=0.0, last_doy=S["doy"])
    if E["last_h"] is not None and 0 < h - E["last_h"] < 0.25:
        dh = h - E["last_h"]
        E["wh"] += ir["poa"] * AREA * EFF * dh
        E["flat"] += ir["flat"] * AREA * EFF * dh
        E["ideal"] += ir["ideal"] * AREA * EFF * dh
    E["last_h"] = h
    if h - E["t_log"] >= 1 / 6 or h < E["t_log"]:
        E["t_log"] = h
        if len(E["log"]) < 5000:
            E["log"].append((date_of(S["doy"]), round(h, 3), round(sn[1], 2), round(sn[2], 2), round(node.load[0], 2), round(node.load[1], 2),
                             round(ir["poa"], 1), round(ir["poa"] * AREA * EFF, 2), round(E["wh"], 3), round(ir["flat"] * AREA * EFF, 2), round(E["flat"], 3)))


def state(node):
    s = node.sun
    a = node.stp if node.stp and time.time() - node.stp_t < 1.0 else None
    d = dict(S, date=date_of(S["doy"]), ok_sun=s is not None, ok_stepper=a is not None, slipped=node.slipped,
             load_q1=node.load[0], load_q2=node.load[1])
    if s:
        h = s[0]
        err = math.degrees(math.acos(max(-1.0, min(1.0, float(tracker.fk(math.radians(node.load[0]), math.radians(node.load[1])) @ tracker.sun_vec(s[1], s[2]))))))
        d.update(jam=f"{int(h):02d}:{int((h % 1) * 60):02d}", h=h, az=s[1], el=s[2], ghi=s[3], dni=s[4], dhi=s[5], cmd_q1=s[6], cmd_q2=s[7], err=err)
    if a:
        d.update(d_az=a[7], d_el=a[3], wind_v=a[9])
    elif node.wind_st:   # mode pid: wind_node (angin = pergeseran perintah, tanpa lepas langkah)
        w = node.wind_st
        d.update(ok_wind=True, wind_v=w[1], tau_az=w[2], tau_el=w[3], wd_az=w[4], wd_el=w[5])
        if len(w) >= 10:     # sumber openmeteo: arah dan nilai pada 10 m ikut dikirim
            dari = w[6] % 360.0
            d.update(w_dari=dari, w_menuju=(dari + 180.0) % 360.0, w_dari_nama=AO.nama_arah(dari),
                     w_menuju_nama=AO.nama_arah(dari + 180.0), w_v10=w[7], w_hembusan10=w[8], w_hembusan=w[9])
    ir = irradiance(node)
    if ir:
        d.update(poa=ir["poa"], p_w=ir["poa"] * AREA * EFF, flat_w=ir["flat"] * AREA * EFF, ideal_w=ir["ideal"] * AREA * EFF, cth=ir["cth"],
                 e_wh=E["wh"], e_flat=E["flat"], e_ideal=E["ideal"], area=AREA, eff=EFF)
    if node.diag:        # sensor_node: LDR, dioda tunggal panel, energi, uraian galat (jauh lebih teliti daripada di atas)
        d["diag"] = node.diag
    if node.ekf:         # ekf_node: taksiran simpangan sudut dan ketidakpastiannya
        d["ekf"] = node.ekf
    if node.hujan:       # sun_command_node: satu sumber kebenaran untuk hujan
        h = node.hujan
        d.update(hujan_basah=h[0] > 0.5, hujan_mm=round(h[1], 2),
                 hujan_mode=["auto", "on", "off"][int(h[2]) if int(h[2]) in (0, 1, 2) else 0])
    return d


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype):
        b = body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(b))); self.send_header("Cache-Control", "no-store")
        self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        if self.path.startswith("/state"):
            self._send(json.dumps(state(self.server.node)), "application/json")
        elif self.path.startswith("/energi.csv"):
            head = "tanggal,jam,az_matahari,el_matahari,q1_beban,q2_beban,poa_W_m2,daya_W,energi_Wh,daya_datar_W,energi_datar_Wh\n"
            self._send(head + "\n".join(",".join(map(str, r)) for r in E["log"]) + "\n", "text/csv; charset=utf-8")
        else:
            self._send(page(), "text/html; charset=utf-8")

    def do_POST(self):
        try:
            m = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
            Q.put((m["name"], m.get("value")))
        except (ValueError, KeyError):
            pass
        self._send("{}", "application/json")


def main():
    rclpy.init()
    node = Panel()
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), H)
    srv.node = node
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f"http://localhost:{PORT}"
    print(f"Panel kontrol web: {url}", flush=True)
    if "--no-open" not in sys.argv:
        try:
            subprocess.Popen(["cmd.exe", "/c", "start", "", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError:
            print("(browser tidak dibuka otomatis, buka alamat di atas sendiri)", flush=True)
    n = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            while not Q.empty():
                apply(node, *Q.get())
            n += 1
            if S["mode"] == "manual" and n % 2 == 0:
                node.pa.publish(Float64(data=math.radians(S["q1"])))
                node.pe.publish(Float64(data=math.radians(S["q2"])))
            if S["run"] and node.sun:
                S["hour"] = round(node.sun[0], 2)
            if node.sun:
                accumulate(node)
    except KeyboardInterrupt:
        pass
    srv.shutdown()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
