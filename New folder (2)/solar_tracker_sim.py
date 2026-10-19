#!/usr/bin/env python3
"""Solar tracker 2 sumbu (2 stepper NEMA23 + 4 LDR) dengan log CSV lengkap.

Model (stepper, LDR, matahari, kontrol) dihitung di Python. CoppeliaSim (ZMQ Remote API)
dipakai untuk visualisasi gerak; boleh dimatikan dengan --no-coppelia (jauh lebih cepat,
disarankan untuk run panjang).

Contoh:
  py solar_tracker_sim.py --no-coppelia --days 30               # cepat, tanpa CoppeliaSim
  py solar_tracker_sim.py --realtime --compress 1 --days 1      # 1:1 dengan jam dinding
  py solar_tracker_sim.py --port 23000 --fresh                  # dengan CoppeliaSim yang sedang terbuka

Kerangka koordinat (sama dengan STL): +Z atas, +X = selatan, +Y = timur.
  phi   = azimuth dari +X berlawanan jarum jam (phi>0 ke arah timur, phi<0 ke barat)
  theta = sudut polar dari +Z = 90 - elevasi
  az    = azimuth kompas (utara=0, timur=90, selatan=180, barat=270) = (180 - phi) mod 360
Motor pan  : sudut poros = phi normal panel, TIDAK di-wrap (-100..280, sesuai kabel/batas mekanis); panel_phi_deg = versi wrap.
Motor tilt : sudut poros = elevasi normal panel (0 = horizontal, 90 = tegak lurus ke atas).
Lokasi default PENS Surabaya (-7.2756, 112.7938, WIB). Matahari memakai algoritma NOAA + tanggal (--date), jadi musim ikut berubah.
Posisi motor dalam koordinat bola: r=1, theta=90-tilt_deg, phi=pan_deg (kolom panel_*).
Sisi LDR di bidang panel: E/W = arah +l/-l (arah phi bertambah/berkurang), U/D = atas/bawah.
Pose pada satu baris = pose di AWAL step (yang dipakai LDR); pulses = pulsa yang keluar di step itu.
pps/dps = kecepatan RATA-RATA selama step (pulsa / DT), bukan kecepatan sesaat di awal step.
"""
import argparse, csv, glob, json, math, os, random, shutil, sys, time
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- lokasi: PENS (Politeknik Elektronika Negeri Surabaya), WIB ----
LAT, LON, TZ = -7.2756, 112.7938, 7.0
# ---- stepper NEMA23 57HSE, penggerak langsung ----
STEP_DEG, MICROSTEP = 1.8, 16
V_MAX_PPS, ACCEL_PPS2 = 1500.0, 10000.0
# pan -100..280: di lintang 7 LS matahari siang lewat di langit UTARA (sekitar April-September), jadi phi bisa sampai 270
PAN_LIM_DEG, TILT_LIM_DEG = (-100.0, 280.0), (0.0, 90.0)
HOME_PAN_DEG, HOME_TILT_DEG = 90.0, 0.0            # hadap timur di cakrawala, siap untuk matahari terbit
# ---- waktu ----
DT, CTRL_EVERY = 0.025, 2                          # 40 Hz simulasi, kontrol tiap 0.05 s
# ---- kontrol 4 LDR ----
STEP_INC, DEADBAND, DARK_SUM = 2, 0.02, 60         # mikrostep per tick, ambang error ternormalisasi, ambang gelap (jumlah ADC)
STEP_GAIN = 60                                     # tambahan mikrostep/tick untuk error besar (mengejar matahari dekat zenit)
# ---- LDR (geometri penyekat silang, meter) ----
H_DIV, D_CELL, W_LDR = 0.030, 0.0045, 0.008
ADC_FS, ADC_NOISE, LUX_FULL = 900.0, 1.5, 100000.0  # ADC 10 bit, lux setara matahari penuh
# ---- awan (proses Ornstein-Uhlenbeck pada waktu matahari) ----
CLOUD_MEAN, CLOUD_TAU, CLOUD_SIGMA = 0.25, 900.0, 0.008

QUADS = (("eu", 1, 1), ("wu", -1, 1), ("ed", 1, -1), ("wd", -1, -1))   # (nama, sigma_l, sigma_u)

COLUMNS = ["step", "sim_time_s", "day", "local_time", "local_hour", "state",
           "sun_az_deg", "sun_el_deg", "sun_theta_deg", "sun_phi_deg",
           "pan_steps", "tilt_steps", "pan_deg", "tilt_deg", "pan_pulses", "tilt_pulses",
           "pan_pps", "tilt_pps", "pan_dps", "tilt_dps",
           "panel_r", "panel_theta_deg", "panel_phi_deg", "panel_az_deg", "panel_el_deg",
           "pointing_err_deg",
           "ldr_eu", "ldr_wu", "ldr_ed", "ldr_wd", "lux_eu", "lux_wu", "lux_ed", "lux_wd",
           "cloud", "err_lr", "err_ud"]


def vec(phi_deg, el_deg):
    p, e = math.radians(phi_deg), math.radians(el_deg)
    return (math.cos(e) * math.cos(p), math.cos(e) * math.sin(p), math.sin(e))


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def sun_pos(doy, hour, lat=LAT, lon=LON, tz=TZ):
    """Posisi matahari (algoritma NOAA). doy = hari ke-N dalam tahun, hour = jam lokal. Kembalikan (phi, elevasi) derajat."""
    g = 2 * math.pi / 365.0 * (doy - 1 + (hour - 12.0) / 24.0)
    eqt = 229.18 * (0.000075 + 0.001868 * math.cos(g) - 0.032077 * math.sin(g)
                    - 0.014615 * math.cos(2 * g) - 0.040849 * math.sin(2 * g))
    dec = (0.006918 - 0.399912 * math.cos(g) + 0.070257 * math.sin(g) - 0.006758 * math.cos(2 * g)
           + 0.000907 * math.sin(2 * g) - 0.002697 * math.cos(3 * g) + 0.00148 * math.sin(3 * g))
    ha = math.radians((hour * 60.0 + eqt + 4.0 * lon - 60.0 * tz) / 4.0 - 180.0)
    la = math.radians(lat)
    el = math.degrees(math.asin(max(-1.0, min(1.0, math.sin(la) * math.sin(dec) + math.cos(la) * math.cos(dec) * math.cos(ha)))))
    a_south = math.degrees(math.atan2(math.sin(ha), math.cos(ha) * math.sin(la) - math.tan(dec) * math.cos(la)))
    az = (180.0 + a_south) % 360.0                      # kompas, utara = 0
    return (180.0 - az + 180.0) % 360.0 - 180.0, el     # phi dari +X (selatan) berlawanan jarum jam


class Stepper:
    def __init__(self, lo_deg, hi_deg):
        self.rad = math.radians(STEP_DEG) / MICROSTEP
        self.lo, self.hi = round(math.radians(lo_deg) / self.rad), round(math.radians(hi_deg) / self.rad)
        self.pos = self.target = 0
        self.v = self.frac = 0.0

    def deg(self, steps=None):
        return math.degrees((self.pos if steps is None else steps) * self.rad)

    def set_target(self, t):
        self.target = max(self.lo, min(self.hi, int(t)))

    def update(self, dt):
        """Profil trapesium (akselerasi terbatas, kecepatan maksimum). Mengembalikan jumlah mikrostep di step ini."""
        err, dv = self.target - self.pos, ACCEL_PPS2 * dt
        if err == 0 and abs(self.v) <= dv:
            self.v = self.frac = 0.0
            return 0
        vdes = math.copysign(min(V_MAX_PPS, math.sqrt(2 * ACCEL_PPS2 * abs(err))), err)
        self.v += max(-dv, min(dv, vdes - self.v))
        self.frac += self.v * dt
        n = int(self.frac)
        self.frac -= n
        if n * err > 0 and abs(n) > abs(err):
            n, self.v, self.frac = err, 0.0, 0.0
        self.pos += n
        return n


def ldr_sample(sun, panel_phi, panel_el, cloud, atm):
    """Kembalikan (adc[4], lux[4]) urut eu, wu, ed, wd. Penyekat silang menaungi LDR di sisi berlawanan matahari."""
    p, e = math.radians(panel_phi), math.radians(panel_el)
    n = (math.cos(e) * math.cos(p), math.cos(e) * math.sin(p), math.sin(e))
    l = (-math.sin(p), math.cos(p), 0.0)
    u = (-math.sin(e) * math.cos(p), -math.sin(e) * math.sin(p), math.cos(e))
    sn, sl, su = dot(sun, n), dot(sun, l), dot(sun, u)
    adc, lux = [], []
    for _, gl, gu in QUADS:
        illum = 0.0
        if sn > 0.02:
            cov = 1.0
            for sg, sp in ((gl, sl), (gu, su)):
                f = 0.0
                if sg * sp < 0:
                    f = min(1.0, max(0.0, (H_DIV * abs(sp) / sn - (D_CELL - W_LDR / 2)) / W_LDR))
                cov *= 1.0 - f
            illum = sn * cov * atm * (1.0 - cloud)
        lux.append(LUX_FULL * illum)
        adc.append(int(round(min(1023.0, max(0.0, ADC_FS * illum + random.gauss(0.0, ADC_NOISE))))))
    return adc, lux


# ------------------------------------------------------------------ CoppeliaSim
class Link:
    # solar_stl/parts_v2 (hasil split_stl.py, sudah TEGAK: rangka di bawah z=0, mekanisme di atasnya)
    PAN_IDS = {13, 14, 15, 16, 17, 18, 20, 21}     # kradle, SK8, motor tilt, lengan (ikut berputar pan)
    TILT_IDS = {19}                                # poros 8 mm
    PAN_AXIS, TILT_AXIS = (0.0, 0.0, 0.165), (-0.003, 0.055, 0.204)

    def __init__(self, args):
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        self.client = RemoteAPIClient(host=args.host, port=args.port)
        self.client.initialTimeout = 90          # panggilan pertama bisa lambat
        self.sim = sim = self.client.require("sim")
        if args.fresh:
            sim.closeScene()
        sim.setBoolParam(sim.boolparam_dynamics_handling_enabled, True)    # rigid body respondable; joint tetap kinematik
        self.build(args)
        if args.save_scene:
            try:
                sim.saveScene(os.path.abspath(args.save_scene))
            except Exception as e:      # gagal di instance headless tertentu; simpan manual dari GUI (File > Save scene)
                print(f"PERINGATAN: scene tidak tersimpan ({str(e)[:60]}).")
        sim.setFloatParam(sim.floatparam_simulation_time_step, DT * args.link_every)
        self.client.setStepping(True)
        sim.startSimulation()

    def _color(self, h, rgb, transp=0.0):
        s = self.sim
        s.setShapeColor(h, None, s.colorcomponent_ambient_diffuse, list(rgb))
        if transp:
            s.setShapeColor(h, None, s.colorcomponent_transparency, [transp])

    def _rigid(self, h):
        """Rigid body: respondable + static. Bagian bergerak digerakkan joint kinematik; shape dinamis di bawah joint kinematik jatuh (diuji)."""
        s = self.sim
        s.setObjectInt32Param(h, s.shapeintparam_respondable, 1)
        s.setObjectInt32Param(h, s.shapeintparam_static, 1)

    def _prim(self, kind, size, pos, quat, rgb, name, parent, transp=0.0):
        s = self.sim
        h = s.createPrimitiveShape(kind, size, 0)
        s.setObjectPose(h, list(pos) + list(quat), s.handle_world)
        self._color(h, rgb, transp)
        s.setObjectAlias(h, name)
        s.setObjectParent(h, parent, True)
        self._rigid(h)
        s.setObjectInt32Param(h, s.shapeintparam_respondable, 0)     # panel/LDR ikut rigid tapi tidak menabrak
        return h

    def build(self, args):
        s = self.sim
        files = sorted(glob.glob(os.path.join(args.parts_dir, "*.stl")))
        if not files:
            raise SystemExit(f"tidak ada STL di {args.parts_dir}")
        shapes = {}
        for f in files:
            idx = int(os.path.basename(f).split("_")[0])
            h = s.importShape(4, f, 0, 0.0, 1.0)      # 4 = STL, satuan meter, koordinat asli terjaga
            s.setObjectAlias(h, os.path.splitext(os.path.basename(f))[0])
            shapes[idx] = h
        self.pan = s.createJoint(s.joint_revolute_subtype, s.jointmode_kinematic, 0, [0.04, 0.004])
        self.tilt = s.createJoint(s.joint_revolute_subtype, s.jointmode_kinematic, 0, [0.04, 0.004])
        s.setObjectPose(self.pan, list(self.PAN_AXIS) + [0, 0, 0, 1], s.handle_world)
        r = math.sqrt(0.5)
        s.setObjectPose(self.tilt, list(self.TILT_AXIS) + [-r, 0, 0, r], s.handle_world)   # sumbu Z joint = +Y dunia
        s.setObjectAlias(self.pan, "pan_joint")
        s.setObjectAlias(self.tilt, "tilt_joint")
        s.setJointInterval(self.pan, False, [math.radians(-110), math.radians(400)])
        s.setJointInterval(self.tilt, False, [math.radians(-100), math.radians(110)])
        for idx, h in shapes.items():
            s.setObjectParent(h, self.tilt if idx in self.TILT_IDS else self.pan if idx in self.PAN_IDS else -1, True)
        s.setObjectParent(self.tilt, self.pan, True)
        for idx, h in shapes.items():
            self._rigid(h)
        # panel placeholder + penyekat silang + 4 LDR, menempel di poros tilt
        tx, ty, tz = self.TILT_AXIS
        face = tx + 0.012 + 0.002
        q0, qy = (0, 0, 0, 1), (0, r, 0, r)             # qy: sumbu silinder Z -> +X
        self._prim(s.primitiveshape_cuboid, [0.004, 0.10, 0.10], (tx + 0.012, ty, tz), q0, (0.1, 0.25, 0.6), "panel", self.tilt, 0.35)
        self._prim(s.primitiveshape_cuboid, [H_DIV, 0.002, 0.024], (face + H_DIV / 2, ty, tz), q0, (0.2, 0.2, 0.2), "divider_lr", self.tilt)
        self._prim(s.primitiveshape_cuboid, [H_DIV, 0.024, 0.002], (face + H_DIV / 2, ty, tz), q0, (0.2, 0.2, 0.2), "divider_ud", self.tilt)
        for name, gl, gu in QUADS:
            self._prim(s.primitiveshape_cylinder, [W_LDR, W_LDR, 0.002], (face + 0.001, ty + gl * D_CELL, tz + gu * D_CELL), qy,
                       (0.9, 0.2, 0.1), "ldr_" + name, self.tilt)
        self.sun = s.createPrimitiveShape(s.primitiveshape_spheroid, [0.05, 0.05, 0.05], 0)
        self._color(self.sun, (1.0, 0.9, 0.1))
        s.setObjectAlias(self.sun, "sun")
        s.setObjectInt32Param(self.sun, s.shapeintparam_static, 1)
        s.setObjectInt32Param(self.sun, s.shapeintparam_respondable, 0)
        self.center = self.TILT_AXIS

    def update(self, pan_deg, tilt_deg, sun):
        s = self.sim
        s.setJointPosition(self.pan, math.radians(pan_deg))
        s.setJointPosition(self.tilt, -math.radians(tilt_deg))
        c = self.center
        s.setObjectPosition(self.sun, [c[0] + 0.7 * sun[0], c[1] + 0.7 * sun[1], c[2] + 0.7 * sun[2]], s.handle_world)
        self.client.step()

    def readback_deg(self):
        s = self.sim
        return math.degrees(s.getJointPosition(self.pan)), -math.degrees(s.getJointPosition(self.tilt))

    def close(self):
        try:
            self.client.setStepping(False)
            self.sim.stopSimulation()
        except Exception:
            pass


# ------------------------------------------------------------------ logger
class Logger:
    def __init__(self, out, rows_per_file, max_bytes):
        self.out, self.rpf, self.max_bytes = out, rows_per_file, max_bytes
        self.stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.part, self.rows, self.total_rows, self.total_bytes, self.files = 0, 0, 0, 0, []
        self.f = self.w = None
        self._flush_at = time.time() + 5
        self._open()

    def _open(self):
        if self.f:
            self.total_bytes += self.f.tell()
            self.f.close()
        self.part += 1
        path = os.path.join(self.out, f"solar_log_{self.stamp}_part{self.part:04d}.csv")
        self.f = open(path, "w", newline="", buffering=1 << 20)
        self.w = csv.writer(self.f)
        self.w.writerow(COLUMNS)
        self.files.append(path)
        self.rows = 0

    def full(self):
        return (self.total_bytes + self.f.tell() >= self.max_bytes or
                shutil.disk_usage(self.out).free < 2 * 1024 ** 3)

    def write(self, row):
        self.w.writerow(row)
        self.rows += 1
        self.total_rows += 1
        if self.rows >= self.rpf:
            self._open()
        now = time.time()
        if now >= self._flush_at:
            self.f.flush()
            self._flush_at = now + 5

    def close(self):
        self.total_bytes += self.f.tell()
        self.f.flush()
        self.f.close()


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--days", type=float, default=30, help="lama simulasi (hari matahari)")
    ap.add_argument("--start-hour", type=float, default=5.0)
    ap.add_argument("--compress", type=float, default=120.0, help="detik matahari per detik simulasi")
    ap.add_argument("--realtime", action="store_true", help="tahan kecepatan: 1 detik simulasi = 1 detik dinding")
    ap.add_argument("--wall-hours", type=float, default=0, help="berhenti setelah N jam dinding (0 = tanpa batas)")
    ap.add_argument("--date", default=None, help="tanggal mulai YYYY-MM-DD (default hari ini); deklinasi berubah tiap hari")
    ap.add_argument("--lat", type=float, default=LAT, help="lintang, selatan negatif (default PENS)")
    ap.add_argument("--lon", type=float, default=LON)
    ap.add_argument("--tz", type=float, default=TZ, help="offset UTC jam (WIB = 7)")
    ap.add_argument("--no-clouds", action="store_true")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--out", default=os.path.join(HERE, "logs"))
    ap.add_argument("--rows-per-file", type=int, default=1_000_000)
    ap.add_argument("--max-gb", type=float, default=20.0, help="berhenti rapi bila total CSV melewati ini")
    ap.add_argument("--log-every", type=int, default=1, help="1 = catat setiap step (paling lengkap)")
    ap.add_argument("--no-coppelia", action="store_true")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=23000)
    ap.add_argument("--fresh", action="store_true", help="kosongkan scene CoppeliaSim dulu (MENGHAPUS scene yang terbuka)")
    ap.add_argument("--link-every", type=int, default=5, help="update CoppeliaSim tiap N step model")
    ap.add_argument("--parts-dir", default=os.path.join(HERE, "solar_stl", "parts_v2"))
    ap.add_argument("--save-scene", default=None, help="simpan scene .ttt")
    args = ap.parse_args()
    if args.seed is not None:
        random.seed(args.seed)
    os.makedirs(args.out, exist_ok=True)

    link = None
    if not args.no_coppelia:
        try:
            link = Link(args)
            print("CoppeliaSim terhubung, scene dibangun.")
        except Exception as e:
            print(f"PERINGATAN: CoppeliaSim tidak bisa dipakai ({str(e)[:100]}). Lanjut tanpa visualisasi.")
            link = None

    log = Logger(args.out, args.rows_per_file, int(args.max_gb * 1024 ** 3))
    with open(os.path.join(args.out, f"solar_run_{log.stamp}.json"), "w") as f:
        json.dump({"args": vars(args), "columns": COLUMNS, "step_deg": STEP_DEG, "microstep": MICROSTEP,
                   "dt": DT, "ctrl_every": CTRL_EVERY, "deadband": DEADBAND, "step_inc": STEP_INC,
                   "v_max_pps": V_MAX_PPS, "accel_pps2": ACCEL_PPS2}, f, indent=1)

    pan, tilt = Stepper(*PAN_LIM_DEG), Stepper(*TILT_LIM_DEG)
    home_pan = round(math.radians(HOME_PAN_DEG) / pan.rad)
    home_tilt = round(math.radians(HOME_TILT_DEG) / tilt.rad)
    pan.pos = pan.target = home_pan
    tilt.pos = tilt.target = home_tilt

    start_date = datetime.strptime(args.date, "%Y-%m-%d") if args.date else datetime.combine(datetime.now().date(), datetime.min.time())
    cloud, day, doy = CLOUD_MEAN, -1, 1
    state, e_lr, e_ud = "PARK", 0.0, 0.0
    err_sum = err_n = 0
    t0 = time.time()
    next_report = t0 + 30
    step = 0
    try:
        while True:
            sim_t = step * DT
            solar_s = args.start_hour * 3600.0 + sim_t * args.compress
            if sim_t * args.compress >= args.days * 86400:
                break
            d = int(solar_s // 86400)
            if d != day:
                day = d
                doy = (start_date + timedelta(days=d)).timetuple().tm_yday
            hour = (solar_s / 3600.0) % 24.0
            now_local = start_date + timedelta(seconds=solar_s)
            phi_s, el_s = sun_pos(doy, hour, args.lat, args.lon, args.tz)
            sun = vec(phi_s, el_s)
            if args.no_clouds:
                cloud = 0.0
            else:
                ds = DT * args.compress
                cloud += (CLOUD_MEAN - cloud) * ds / CLOUD_TAU + CLOUD_SIGMA * math.sqrt(ds) * random.gauss(0, 1)
                cloud = min(0.9, max(0.0, cloud))
            atm = max(0.0, math.sin(math.radians(el_s))) ** 0.3

            pan_deg, tilt_deg = pan.deg(), tilt.deg()
            adc, lux = ldr_sample(sun, pan_deg, tilt_deg, cloud, atm)

            if step % CTRL_EVERY == 0:
                tot = sum(adc)
                if tot < DARK_SUM:
                    state, e_lr, e_ud = "PARK", 0.0, 0.0
                    pan.set_target(home_pan)
                    tilt.set_target(home_tilt)
                else:
                    state = "TRACK"
                    eu, wu, ed, wd = adc
                    e_lr = ((eu + ed) - (wu + wd)) / tot
                    e_ud = ((eu + wu) - (ed + wd)) / tot
                    if abs(e_lr) > DEADBAND:
                        pan.set_target(pan.target + math.copysign(STEP_INC + STEP_GAIN * max(0.0, abs(e_lr) - 0.1), e_lr))
                    if abs(e_ud) > DEADBAND:
                        tilt.set_target(tilt.target + math.copysign(STEP_INC + STEP_GAIN * max(0.0, abs(e_ud) - 0.1), e_ud))
            pan_v, tilt_v = pan.v, tilt.v
            pan_p, tilt_p = pan.update(DT), tilt.update(DT)

            n = vec(pan_deg, tilt_deg)
            perr = math.degrees(math.acos(max(-1.0, min(1.0, dot(sun, n)))))
            if state == "TRACK" and el_s > 5.0:
                err_sum += perr
                err_n += 1

            if step % args.log_every == 0:
                log.write([step, f"{sim_t:.3f}", day, now_local.strftime("%Y-%m-%d %H:%M:%S"), f"{hour:.5f}", state,
                           f"{(180.0 - phi_s) % 360.0:.4f}", f"{el_s:.4f}", f"{90.0 - el_s:.4f}", f"{phi_s:.4f}",
                           pan.pos - pan_p, tilt.pos - tilt_p, f"{pan_deg:.4f}", f"{tilt_deg:.4f}", pan_p, tilt_p,
                           f"{pan_p / DT:.1f}", f"{tilt_p / DT:.1f}", f"{pan_p * math.degrees(pan.rad) / DT:.4f}", f"{tilt_p * math.degrees(tilt.rad) / DT:.4f}",
                           1, f"{90.0 - tilt_deg:.4f}", f"{(pan_deg + 180.0) % 360.0 - 180.0:.4f}", f"{(180.0 - pan_deg) % 360.0:.4f}", f"{tilt_deg:.4f}",
                           f"{perr:.4f}",
                           adc[0], adc[1], adc[2], adc[3], f"{lux[0]:.1f}", f"{lux[1]:.1f}", f"{lux[2]:.1f}", f"{lux[3]:.1f}",
                           f"{cloud:.4f}", f"{e_lr:.5f}", f"{e_ud:.5f}"])

            if link and step % args.link_every == 0:
                try:
                    link.update(pan.deg(), tilt.deg(), sun)
                except Exception as e:
                    print(f"PERINGATAN: koneksi CoppeliaSim putus ({str(e)[:80]}). Log tetap jalan.")
                    link = None

            step += 1
            now = time.time()
            if step % 4000 == 0:
                if log.full():
                    print("Batas ukuran/disk tercapai, berhenti rapi.")
                    break
                if args.wall_hours and now - t0 > args.wall_hours * 3600:
                    print("Batas waktu dinding tercapai.")
                    break
            if now >= next_report:
                next_report = now + 30
                print(f"[{now - t0:7.0f}s] hari {day} jam {hour:5.2f} baris {log.total_rows:,} "
                      f"file {log.part} ~{(log.total_bytes + log.f.tell()) / 1e6:.0f} MB", flush=True)
            if args.realtime:
                lag = t0 + step * DT - time.time()
                if lag > 0:
                    time.sleep(lag)
    except KeyboardInterrupt:
        print("Dihentikan (Ctrl+C), menutup file...")
    finally:
        rb = None
        if link:
            try:
                rb = link.readback_deg()
            except Exception:
                pass
            link.close()
        log.close()
        summary = {"steps": step, "rows": log.total_rows, "files": log.files, "bytes": log.total_bytes,
                   "mean_pointing_err_deg_tracking_el_gt_5": (err_sum / err_n) if err_n else None,
                   "coppelia_joint_readback_deg": rb, "final_pan_deg": pan.deg(), "final_tilt_deg": tilt.deg(),
                   "wall_s": time.time() - t0}
        with open(os.path.join(args.out, f"solar_summary_{log.stamp}.json"), "w") as f:
            json.dump(summary, f, indent=1)
        print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
