"""Panel kontrol manual untuk demo (Tkinter, berdampingan dengan RViz). Semua parameter bisa digeser langsung, pembacaan tampil langsung.
Dipakai lewat: ros2 launch ~/mk/solar_tracker/launch/sim.launch.py sun_node:=true rviz:=true panel:=true [actuator:=stepper]
Bagian: Mode (otomatis ikuti matahari atau manual geser sendi), Waktu (tanggal 2026, jam, jalan otomatis dan kecepatan, awan),
Sendi manual (azimuth q1, kemiringan q2), Stepper (microstep, angin, torsi gangguan tambahan untuk memperlihatkan lepas langkah), Pembacaan langsung.
Perintah ke node lewat service parameter (/sun_command_node, /stepper_node) dan topik /solar_tracker/*_cmd (rad)."""
import datetime as dt
import math
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import tracker  # noqa: E402

import rclpy  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64, Float64MultiArray  # noqa: E402

SLIP_DEG = 3.6      # setengah siklus listrik stepper (360/50/2): selisih menetap lebih besar berarti lepas langkah


class Panel(Node):
    def __init__(self):
        super().__init__("control_panel")
        self.cli = {n: self.create_client(SetParameters, f"/{n}/set_parameters") for n in ("sun_command_node", "stepper_node", "wind_node")}
        self.pa = self.create_publisher(Float64, "/solar_tracker/azimuth_cmd", 10)
        self.pe = self.create_publisher(Float64, "/solar_tracker/elevation_cmd", 10)
        self.sun, self.load, self.stp, self.stp_t = None, [0.0, 0.0], None, 0.0
        self.create_subscription(Float64MultiArray, "/sun/state", lambda m: setattr(self, "sun", list(m.data)), 10)
        self.create_subscription(Float64MultiArray, "/stepper/state", self.on_stp, 10)
        self.create_subscription(JointState, "/joint_states", self.on_js, 1)
        self.slipped = False
        self.wind_st = None
        self.diag = None      # JSON diagnostik lengkap dari sensor_node (LDR, listrik, energi, uraian galat)
        self.create_subscription(Float64MultiArray, "/wind/state", lambda m: setattr(self, "wind_st", list(m.data)), 10)
        from std_msgs.msg import String as _Str
        import json as _json
        self.create_subscription(_Str, "/tracker/diag", lambda m: setattr(self, "diag", _json.loads(m.data)), 10)
        self.ekf = None
        self.create_subscription(_Str, "/ekf/state", lambda m: setattr(self, "ekf", _json.loads(m.data)), 10)

    def on_stp(self, m):
        self.stp, self.stp_t = list(m.data), time.time()
        if max(abs(m.data[3]), abs(m.data[7])) > SLIP_DEG:
            self.slipped = True

    def on_js(self, m):
        if "azimuth_joint" in m.name and "elevation_joint" in m.name:
            self.load = [math.degrees(m.position[m.name.index("azimuth_joint")]), math.degrees(m.position[m.name.index("elevation_joint")])]

    def set_param(self, node, name, value):
        c = self.cli[node]
        if not c.service_is_ready():
            return False
        v = ParameterValue()
        if isinstance(value, bool):
            v.type, v.bool_value = ParameterType.PARAMETER_BOOL, value
        elif isinstance(value, int):
            v.type, v.integer_value = ParameterType.PARAMETER_INTEGER, value
        elif isinstance(value, float):
            v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, value
        else:
            v.type, v.string_value = ParameterType.PARAMETER_STRING, str(value)
        c.call_async(SetParameters.Request(parameters=[Parameter(name=name, value=v)]))
        return True


def main():
    rclpy.init()
    node = Panel()
    root = tk.Tk()
    root.title("Panel Kontrol Solar Tracker (PENS -7.276, 112.794)")
    root.geometry("560x940+20+20")
    root.lift(); root.attributes("-topmost", True)   # selalu di depan RViz (kolom kiri layar); matikan lewat kotak centang di bawah
    F = ("Segoe UI", 11)
    root.option_add("*Font", F)
    mode, run, speed, doy, hour, okta = tk.StringVar(value="auto"), tk.BooleanVar(value=True), tk.StringVar(value="600"), tk.IntVar(value=277), tk.DoubleVar(value=12.0), tk.IntVar(value=0)
    q1, q2 = tk.DoubleVar(value=0.0), tk.DoubleVar(value=0.0)
    mstep, wind, seed, xaz, xel = tk.StringVar(value="16"), tk.StringVar(value="none"), tk.IntVar(value=0), tk.DoubleVar(value=0.0), tk.DoubleVar(value=0.0)
    date_lbl = tk.StringVar()
    rd = {k: tk.StringVar(value="-") for k in ("waktu", "matahari", "irr", "cmd", "load", "err", "stp", "wind")}
    last = {"hour": 0.0}

    def date_of(d):
        return (dt.date(2026, 1, 1) + dt.timedelta(days=int(d) - 1)).isoformat()

    def sun_set(n, v):
        node.set_param("sun_command_node", n, v)

    def apply_time():
        sun_set("hour", -1.0 if run.get() else float(hour.get()))

    def on_mode():
        manual = mode.get() == "manual"
        sun_set("manual", manual)
        if manual:
            q1.set(round(node.load[0], 1)); q2.set(round(node.load[1], 1))
        for w in manual_w:
            w.state(["!disabled"] if manual else ["disabled"])

    def on_date(_=None):
        date_lbl.set(date_of(doy.get()))

    def on_date_release(_=None):
        sun_set("date", date_of(doy.get()))

    def on_hour(_=None):
        run.set(False)
        if time.time() - last["hour"] > 0.1:
            last["hour"] = time.time(); apply_time()

    def lf(text):
        f = ttk.LabelFrame(root, text=text, padding=6); f.pack(fill="x", padx=8, pady=4); return f

    def slider(f, label, var, lo, hi, res, cb=None, rel=None):
        r = ttk.Frame(f); r.pack(fill="x")
        ttk.Label(r, text=label, width=24).pack(side="left")
        s = tk.Scale(r, variable=var, from_=lo, to=hi, resolution=res, orient="horizontal", length=290, command=cb)
        if rel:
            s.bind("<ButtonRelease-1>", rel)
        s.pack(side="left", fill="x", expand=True)
        return s

    f = lf("1. Mode")
    ttk.Radiobutton(f, text="Otomatis: ikuti matahari (IK pvlib)", variable=mode, value="auto", command=on_mode).pack(anchor="w")
    ttk.Radiobutton(f, text="Manual: geser sendi sendiri (matahari tetap ditampilkan)", variable=mode, value="manual", command=on_mode).pack(anchor="w")

    f = lf("2. Matahari dan waktu")
    slider(f, "Tanggal (hari ke- 2026)", doy, 1, 365, 1, on_date, on_date_release)
    ttk.Label(f, textvariable=date_lbl, font=("Segoe UI", 12, "bold")).pack(anchor="e")
    slider(f, "Jam lokal WIB", hour, 5.0, 19.0, 0.05, on_hour)
    r = ttk.Frame(f); r.pack(fill="x")
    ttk.Checkbutton(r, text="Waktu jalan otomatis", variable=run, command=apply_time).pack(side="left")
    ttk.Label(r, text="  kecepatan (x nyata):").pack(side="left")
    cb = ttk.Combobox(r, textvariable=speed, values=("1", "60", "300", "600", "1200"), width=6, state="readonly"); cb.pack(side="left")
    cb.bind("<<ComboboxSelected>>", lambda e: sun_set("speed", float(speed.get())))
    slider(f, "Tutupan awan (okta 0..8)", okta, 0, 8, 1, None, lambda e: sun_set("okta", float(okta.get())))

    f = lf("3. Sendi manual (aktif bila mode Manual)")
    s1 = slider(f, "Azimuth q1 (deg, CW; arah miring = q1 + 90 dari Utara)", q1, -200, 200, 0.5)
    s2 = slider(f, "Kemiringan q2 (deg, 0 = datar)", q2, -45, 45, 0.5)
    manual_w = [s1, s2]
    for w in manual_w:
        w.state = lambda st, w=w: w.config(state="disabled" if "disabled" in st else "normal")
    ttk.Button(f, text="Nol (datar, azimuth 0)", command=lambda: (q1.set(0.0), q2.set(0.0))).pack(anchor="e")

    f = lf("4. Stepper dan gangguan (aktuator stepper)")
    r = ttk.Frame(f); r.pack(fill="x")
    ttk.Label(r, text="Microstep 1/").pack(side="left")
    c1 = ttk.Combobox(r, textvariable=mstep, values=("1", "2", "4", "8", "16", "32"), width=4, state="readonly"); c1.pack(side="left")
    c1.bind("<<ComboboxSelected>>", lambda e: node.set_param("stepper_node", "microstep", float(mstep.get())))
    ttk.Label(r, text="   Angin:").pack(side="left")
    c2 = ttk.Combobox(r, textvariable=wind, values=("none", "tenang", "berangin", "badai"), width=9, state="readonly"); c2.pack(side="left")
    c2.bind("<<ComboboxSelected>>", lambda e: node.set_param("stepper_node", "wind", wind.get()))
    ttk.Label(r, text=" seed").pack(side="left")
    sp = ttk.Spinbox(r, from_=0, to=99, textvariable=seed, width=4, command=lambda: node.set_param("stepper_node", "wind_seed", int(seed.get()))); sp.pack(side="left")
    slider(f, "Torsi tambahan azimuth (N.m)", xaz, -5, 5, 0.1, None, lambda e: node.set_param("stepper_node", "extra_torque_az", float(xaz.get())))
    slider(f, "Torsi tambahan elevasi (N.m)", xel, -5, 5, 0.1, None, lambda e: node.set_param("stepper_node", "extra_torque_el", float(xel.get())))
    ttk.Label(f, text="Torsi tahan stepper 3 N.m: melewati itu, motor lepas langkah.", foreground="#666").pack(anchor="w")

    def reset_slip():
        xaz.set(0.0); xel.set(0.0)
        node.set_param("stepper_node", "extra_torque_az", 0.0); node.set_param("stepper_node", "extra_torque_el", 0.0)
        node.slipped = False
    ttk.Button(f, text="Reset gangguan dan penanda lepas langkah", command=reset_slip).pack(anchor="e")

    f = lf("5. Pembacaan langsung")
    big = ("Consolas", 12)
    for k, t in (("waktu", "Waktu"), ("matahari", "Matahari"), ("irr", "Irradiance"), ("cmd", "Perintah sendi"), ("load", "Sudut beban"), ("err", "Galat pointing"), ("stp", "Stepper"), ("wind", "Angin")):
        r = ttk.Frame(f); r.pack(fill="x")
        ttk.Label(r, text=t, width=15).pack(side="left")
        lab = ttk.Label(r, textvariable=rd[k], font=big); lab.pack(side="left"); rd[k + "_w"] = lab

    top = tk.BooleanVar(value=True)
    ttk.Checkbutton(root, text="Selalu di depan jendela lain", variable=top, command=lambda: root.attributes("-topmost", top.get())).pack(anchor="w", padx=10)
    on_date(); on_mode(); n_pub = [0]

    def tick():
        if not rclpy.ok():      # Ctrl+C pada ros2 launch: tutup jendela agar launch bisa berhenti
            root.destroy()
            return
        rclpy.spin_once(node, timeout_sec=0)
        n_pub[0] += 1
        if mode.get() == "manual" and n_pub[0] % 3 == 0:
            node.pa.publish(Float64(data=math.radians(q1.get()))); node.pe.publish(Float64(data=math.radians(q2.get())))
        if n_pub[0] % 5 == 0:
            s = node.sun
            if s:
                h = s[0]
                rd["waktu"].set(f"{date_lbl.get()}  {int(h):02d}:{int((h % 1) * 60):02d} WIB")
                rd["matahari"].set(f"az {s[1]:6.1f}  el {s[2]:5.1f} deg")
                rd["irr"].set(f"DNI {s[4]:5.0f}  GHI {s[3]:5.0f}  DHI {s[5]:4.0f} W/m2")
                rd["cmd"].set(f"q1 {s[6]:7.1f}  q2 {s[7]:5.1f} deg")
                e = math.degrees(math.acos(max(-1.0, min(1.0, float(tracker.fk(math.radians(node.load[0]), math.radians(node.load[1])) @ tracker.sun_vec(s[1], s[2]))))))
                rd["err"].set(f"{e:6.2f} deg" + ("" if s[2] > 0 else "  (matahari di bawah horizon)"))
                rd["err_w"].config(foreground="#0a8a0a" if e < 0.6 else "#d98000" if e < 3 else "#cc0000")
                if run.get():
                    hour.set(round(h, 2))
            rd["load"].set(f"q1 {node.load[0]:7.1f}  q2 {node.load[1]:5.1f} deg")
            if node.stp and time.time() - node.stp_t < 1.0:
                a = node.stp
                rd["stp"].set(f"selisih az {a[7]:5.2f}  el {a[3]:5.2f} deg" + ("   LEPAS LANGKAH" if node.slipped else ""))
                rd["stp_w"].config(foreground="#cc0000" if node.slipped else "#0a8a0a")
                rd["wind"].set(f"{a[9]:4.1f} m/s")
            else:
                rd["stp"].set("tidak aktif (aktuator pid)"); rd["wind"].set("-")
        root.after(20, tick)

    root.after(300, tick)
    if "--selftest" in sys.argv:    # tanpa tangan: geser kontrol lewat callback, lalu keluar
        def st():
            doy.set(172); on_date(); on_date_release()
            hour.set(9.5); on_hour()
            mode.set("manual"); on_mode(); q1.set(40.0); q2.set(20.0)
            print("selftest panel: kontrol digeser; layanan sun_command_node siap =", node.cli["sun_command_node"].service_is_ready(), flush=True)
            root.after(1500, lambda: (print("selftest panel: selesai tanpa galat", flush=True), root.destroy()))
        root.after(2500, st)
    root.protocol("WM_DELETE_WINDOW", lambda: (root.destroy(), rclpy.try_shutdown()))
    root.mainloop()


if __name__ == "__main__":
    main()
