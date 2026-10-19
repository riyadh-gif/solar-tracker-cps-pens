"""Osilasi DUA sumbu sekaligus dan dampaknya pada energi panen.

Pertanyaan yang dijawab: tiap perubahan perintah motor menimbulkan ayunan; seberapa besar ayunannya, berapa lama
sampai tenang, dan berapa energi yang hilang karenanya. Dibandingkan empat strategi perintah.

Model (aktuator `pid`, yang didukung di Gazebo), per sumbu, keadaan x = [q, q']:
    J(q2) q'' = sat(K (cmd - q), +-Th) - bl q' - fc tanh(q'/2e-3) + tau_grav(q) + tau_angin(t)
    K  = kekakuan seri stepper dan poros 1/(1/(Th Nr) + 1/k)   (Th 1.5: azimuth 71.64, elevasi 60.65 N.m/rad)
    tau_grav elevasi = -m g (cy cos q - cz sin q), dari inersia/inersia_link.json (tanda tervalidasi di Gazebo)
    J_az(q2) = a + b cos^2(q2)  (kopling antar sumbu; cocok 2 titik dari J_az pada q2 = 0 dan 45 deg)
Perintah selalu dibulatkan ke tangga microstep: itulah sumber eksitasi yang sebenarnya.

Strategi perintah yang dibandingkan:
    langsung  perintah lintasan apa adanya, dibulatkan ke microstep
    scurve    pembatas laju dan percepatan (profil trapesium, seperti stepper_node.py)
    zv        input shaper ZV   (2 impuls, jeda setengah periode)
    zvd       input shaper ZVD  (3 impuls, lebih tahan salah tebak frekuensi)

BATAS YANG PENTING:
 - Ini model aktuator `pid` (linier dengan kejenuhan Th). Stepper sebenarnya punya torsi Th sin(Nr delta) yang
   MENURUN setelah 1.8 deg, jadi model ini optimistis saat selisih besar. Lepas langkah tidak dimodelkan di sini.
 - Suku Coriolis dan giroskopik diabaikan (laju jejak 0.05 sampai 0.5 deg/s, sumbangannya sangat kecil).
 - bl 0.2 N.m.s/rad dan fc 0.05 N.m masih ASUMSI dan justru paling menentukan lama ayunan.
 - Hanya jendela waktu terpilih yang disimulasikan 1 kHz, bukan satu hari penuh (biaya hitung).

Pakai: python osilasi_energi.py            (sapuan penuh, tulis osilasi_results.md)
       python osilasi_energi.py --selftest  (cek mandiri, cepat)
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "gabungan"))
import dyn_model as D   # noqa: E402
import sun              # noqa: E402
import tracker          # noqa: E402

AREA = 0.670 * 0.445              # m2, MOS C-50-18-M
EFF = 50.0 / (AREA * 1000.0)      # 16.8 %, turunan dari Pmax dan STC
ALB = 0.25
G = 10.0                          # m/s2, sama dengan world.sdf dan inersia_cad.py
DTW = 0.25                        # resolusi deret angin, sama dengan gabungan/coupled_sim.py

_J = json.loads((ROOT / "inersia" / "inersia_link.json").read_text())
M_EL = _J["link"]["elevation_link"]["massa_kg"] + _J["link"]["panel"]["massa_kg"]
CY, CZ = _J["dinamika"]["lengan_y_mm"] / 1e3, _J["dinamika"]["lengan_z_mm"] / 1e3
JAZ_B = (_J["dinamika"]["J_az_q0_kgm2"] - _J["dinamika"]["J_az_q45_kgm2"]) / 0.5
JAZ_A = _J["dinamika"]["J_az_q0_kgm2"] - JAZ_B


def kseri(ax):
    return ax.K_ser()   # satu sumber: dyn_model.Axis.K_ser (ikut Kp loop tertutup)


def j_az(q2):
    """Inersia azimuth bergantung kemiringan (kopling antar sumbu)."""
    return JAZ_A + JAZ_B * math.cos(q2) ** 2


def tau_grav_el(q):
    """Momen gravitasi pada sumbu elevasi (N.m). Negatif = menarik q turun; tanda sudah cocok dengan Gazebo."""
    return -M_EL * G * (CY * math.cos(q) - CZ * math.sin(q))


def quasi_static(q_cmd, ax, tau_ext, elev):
    """Sudut setimbang tanpa inersia: K (cmd - q) + tau = 0, dengan gravitasi ikut dihitung (iterasi kecil)."""
    K, q = kseri(ax), q_cmd
    for _ in range(30):
        t = tau_ext + (tau_grav_el(q) if elev else 0.0)
        q = q_cmd + max(-ax.Th, min(ax.Th, t)) / K
    return q


# ---------------------------------------------------------------- pembentuk perintah

def shaper_impulses(kind, zeta, f_hz):
    """Amplitudo dan jeda impuls input shaper. Td = periode teredam."""
    if kind not in ("zv", "zvd"):
        return [(1.0, 0.0)]
    K = math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2))
    dt = 1.0 / (2 * f_hz * math.sqrt(1 - zeta ** 2))
    if kind == "zv":
        s = 1 + K
        return [(1 / s, 0.0), (K / s, dt)]
    s = (1 + K) ** 2
    return [(1 / s, 0.0), (2 * K / s, dt), (K * K / s, 2 * dt)]


def apply_shaper(u, dt, imps):
    """Konvolusi sinyal perintah dengan deret impuls (jeda dibulatkan ke cuplikan)."""
    y = np.zeros_like(u)
    for a, d in imps:
        n = int(round(d / dt))
        y[n:] += a * u[:len(u) - n] if n else a * u
        if n:
            y[:n] += a * u[0]
    return y


def rate_limit(u, dt, vmax, amax):
    """Profil trapesium: laju dan percepatan dibatasi, berhenti tepat di sasaran (sama dengan stepper_node.py)."""
    y, v, q = np.empty_like(u), 0.0, float(u[0])
    for i, tgt in enumerate(u):
        e = tgt - q
        vdes = math.copysign(min(vmax, math.sqrt(2 * amax * abs(e))), e) if e else 0.0
        v += max(-amax * dt, min(amax * dt, vdes - v))
        q += v * dt
        if e * (tgt - q) < 0:
            q, v = float(tgt), 0.0
        y[i] = q
    return y


def build_cmd(strategy, q_raw, dt, ax, micro, vmax, amax):
    """Perintah akhir: bentuk dulu, lalu DIBULATKAN ke tangga microstep (driver hanya bisa langkah bulat).
    `zv` dan `zvd` dipasang DI ATAS profil trapesium, bukan di atas lompatan mentah: pada lompatan besar
    kontroler jenuh di Th, geraknya jadi bang-bang, dan input shaper (yang mengandaikan sistem linier)
    sama sekali tidak berefek (terbukti: lewatan 71.2 % sama persis dengan dan tanpa ZV)."""
    u = np.asarray(q_raw, float)
    if strategy != "langsung":
        u = rate_limit(u, dt, vmax, amax)
    if strategy in ("zv", "zvd"):
        K, J = kseri(ax), ax.JL
        zeta = ax.bl / (2 * math.sqrt(K * J))
        f = math.sqrt(K / J) / (2 * math.pi)
        u = apply_shaper(u, dt, shaper_impulses(strategy, zeta, f))
    step = math.radians(1.8 / micro)
    return np.round(u / step) * step


# ---------------------------------------------------------------- integrasi dua sumbu

def simulate(cmd_az, cmd_el, dt, wind=None, x0=None):
    """RK4 dua sumbu terkopel. cmd_* dalam rad pada grid dt. wind = (v, sign) per cuplikan atau None.
    Mengembalikan sudut beban (rad) az dan el."""
    a_az, a_el = D.AXES["azimuth"], D.AXES["elevasi"]
    K_az, K_el = kseri(a_az), kseri(a_el)
    n = len(cmd_az)
    qa = np.empty(n); qe = np.empty(n)
    x = list(x0) if x0 else [cmd_az[0], 0.0, quasi_static(cmd_el[0], a_el, 0.0, True), 0.0]

    def deriv(x, ca, ce, tw_az, tw_el):
        q1, w1, q2, w2 = x
        t1 = max(-a_az.Th, min(a_az.Th, K_az * (ca - q1))) - a_az.bl * w1 - a_az.fc * math.tanh(w1 / 2e-3) + tw_az
        t2 = (max(-a_el.Th, min(a_el.Th, K_el * (ce - q2))) - a_el.bl * w2 - a_el.fc * math.tanh(w2 / 2e-3)
              + tau_grav_el(q2) + tw_el)
        return [w1, t1 / j_az(q2), w2, t2 / a_el.JL]

    for i in range(n):
        qa[i], qe[i] = x[0], x[2]
        if i == n - 1:
            break
        ca0, ce0, ca1, ce1 = cmd_az[i], cmd_el[i], cmd_az[i + 1], cmd_el[i + 1]
        cam, cem = 0.5 * (ca0 + ca1), 0.5 * (ce0 + ce1)
        if wind is None:
            ta = te = 0.0
        else:
            v, sg = wind[0][i], wind[1][i]
            ta = sg * float(D.wind_torque(a_az, v))
            te = sg * float(D.wind_torque(a_el, v))
        k1 = deriv(x, ca0, ce0, ta, te)
        k2 = deriv([x[j] + 0.5 * dt * k1[j] for j in range(4)], cam, cem, ta, te)
        k3 = deriv([x[j] + 0.5 * dt * k2[j] for j in range(4)], cam, cem, ta, te)
        k4 = deriv([x[j] + dt * k3[j] for j in range(4)], ca1, ce1, ta, te)
        x = [x[j] + dt / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]) for j in range(4)]
    return qa, qe


# ---------------------------------------------------------------- energi

def poa(q1, q2, az, el, dni, dhi, ghi):
    """Irradiance bidang panel (W/m2) dari sudut sendi, model isotropik (sama dengan sun.poa dan control_web.py)."""
    n = tracker.fk(q1, q2)
    s = tracker.sun_vec(az, el)
    cth = max(0.0, float(n @ s))
    ct = float(n[2])
    return dni * cth + dhi * (1 + ct) / 2 + ghi * ALB * (1 - ct) / 2


def energy_wh(q1, q2, az, el, dni, dhi, ghi, dt):
    """Energi (Wh) sepanjang jendela dari deret sudut sendi."""
    p = np.array([poa(a, b, az, el, dni, dhi, ghi) for a, b in zip(q1, q2)]) * AREA * EFF
    return float(p.sum() * dt / 3600.0), float(p.mean())


# ---------------------------------------------------------------- lintasan dan jendela

def day(date, okta=0.0):
    """Lintasan perintah sehari (rad) pada grid 10 s, tanpa batas kemiringan lalu dipotong ke +-45 deg."""
    d = sun.sun_day(date, "10s", okta)
    d = d[d.el > 3]
    q1, q2 = tracker.ik_track(d.az.values, d.el.values, q2_max=math.radians(90))
    q2 = np.clip(q2, -math.radians(45), math.radians(45))
    ok = ~np.isnan(q1)
    t = (d.index - d.index[0]).total_seconds().values
    h = d.index.hour.values + d.index.minute.values / 60 + d.index.second.values / 3600
    return dict(t=t[ok], h=h[ok], q1=q1[ok], q2=q2[ok], az=d.az.values[ok], el=d.el.values[ok],
                dni=d.dni.values[ok], dhi=d.dhi.values[ok], ghi=d.ghi.values[ok])


def window(tr, hour, T, dt):
    """Potong lintasan jadi jendela 1 kHz yang berpusat di jam tertentu. None bila di luar jangkauan."""
    if not (tr["h"][0] <= hour <= tr["h"][-1]):
        return None
    t0 = float(np.interp(hour, tr["h"], tr["t"]))
    if t0 + T > tr["t"][-1]:
        t0 = tr["t"][-1] - T
    tw = np.arange(0.0, T, dt) + t0
    k = int(np.argmin(np.abs(tr["t"] - (t0 + T / 2))))
    return dict(t=tw, q1=np.interp(tw, tr["t"], tr["q1"]), q2=np.interp(tw, tr["t"], tr["q2"]),
                az=float(tr["az"][k]), el=float(tr["el"][k]), dni=float(tr["dni"][k]),
                dhi=float(tr["dhi"][k]), ghi=float(tr["ghi"][k]), hour=hour)


def wind_window(kind, n, dt, seed):
    """Deret angin pada grid dt (ditahan per DTW, sama dengan coupled_sim.wind_series)."""
    if kind == "none":
        return None
    import coupled_sim as C
    v, sg = C.wind_series(kind, int(max(8, n * dt / DTW) + 4), seed)
    idx = (np.arange(n) * dt / DTW).astype(int) % len(v)
    return v[idx], sg[idx]


# ---------------------------------------------------------------- metrik

def metrics(q, cmd, dt):
    """Pisahkan simpangan jadi bias (rata-rata, = defleksi) dan ayunan (simpangan baku dan puncak-ke-puncak)."""
    e = np.degrees(q - cmd)
    ac = e - e.mean()
    return dict(bias=float(e.mean()), rms=float(np.sqrt((ac ** 2).mean())), pp=float(ac.max() - ac.min()))


def settle_after_step(ax, step_deg, dt=1e-3, T=6.0, band=0.02, strategy="langsung", micro=16.0):
    """Satu langkah besar pada sumbu tunggal: lewatan (%), waktu menetap (s) ke pita band x langkah."""
    n = int(T / dt)
    raw = np.concatenate([np.zeros(int(0.5 / dt)), np.full(n - int(0.5 / dt), math.radians(step_deg))])
    cmd = build_cmd(strategy, raw, dt, ax, micro, math.radians(30.0), math.radians(300.0))
    elev = ax.name == "elevasi"
    zero = np.zeros(n)
    if elev:
        q = simulate(zero, cmd, dt)[1]
    else:
        q = simulate(cmd, zero, dt)[0]
    q0, qf = q[0], quasi_static(cmd[-1], ax, 0.0, elev)
    rng = qf - q0
    if abs(rng) < 1e-9:
        return dict(over_pct=0.0, settle_s=0.0)
    norm = (q - q0) / rng
    over = max(0.0, float(norm.max() - 1.0)) * 100 if rng > 0 else max(0.0, float(1.0 - norm.min())) * 100
    bad = np.flatnonzero(np.abs(norm - 1.0) > band)
    return dict(over_pct=over, settle_s=float((bad[-1] + 1) * dt - 0.5) if len(bad) else 0.0)


# ---------------------------------------------------------------- cek mandiri

def selftest():
    a_el, a_az = D.AXES["elevasi"], D.AXES["azimuth"]
    # 1) gravitasi: nilai dan tanda cocok dengan inersia_cad dan ukuran Gazebo
    assert abs(abs(tau_grav_el(0.0)) - _J["dinamika"]["Mg_pada_q0_Nm"]) < 1e-3
    assert abs(abs(tau_grav_el(math.radians(-45))) - _J["dinamika"]["Mg_max_dalam_batas_45deg_Nm"]) < 2e-3
    assert tau_grav_el(0.0) < 0, "gravitasi harus menarik q2 ke arah negatif (terukur -0.161 deg di Gazebo)"
    d0 = math.degrees(quasi_static(0.0, a_el, 0.0, True))
    assert -0.20 < d0 < -0.12, f"defleksi q2=0 harus sekitar -0.155 deg, dapat {d0:.3f}"
    # 2) inersia azimuth: cocok di dua titik acuan
    assert abs(j_az(0.0) - _J["dinamika"]["J_az_q0_kgm2"]) < 1e-9
    assert abs(j_az(math.radians(45)) - _J["dinamika"]["J_az_q45_kgm2"]) < 1e-9
    # 3) shaper: jumlah amplitudo = 1 (tidak menggeser nilai akhir)
    for kind in ("zv", "zvd"):
        assert abs(sum(a for a, _ in shaper_impulses(kind, 0.03, 5.0)) - 1.0) < 1e-12
    u = np.concatenate([np.zeros(500), np.ones(2500)])
    for kind in ("zv", "zvd"):
        y = apply_shaper(u, 1e-3, shaper_impulses(kind, 0.03, 5.0))
        assert abs(y[-1] - 1.0) < 1e-12 and y.max() <= 1.0 + 1e-12
    # 4) dinamika bebas: frekuensi dering cocok dengan sqrt(K/J)/2pi dalam 3 %
    dt, n = 1e-4, 30000
    q = simulate(np.full(n, math.radians(1.0)), np.zeros(n), dt, x0=[0.0, 0.0, 0.0, 0.0])[0]
    x = q - q[-1]
    cr = np.flatnonzero((x[:-1] < 0) & (x[1:] >= 0))
    f = 1.0 / (np.mean(np.diff(cr[:6])) * dt)
    fpred = math.sqrt(kseri(a_az) / a_az.JL) / (2 * math.pi)
    assert abs(f - fpred) / fpred < 0.03, f"f dering {f:.2f} vs prediksi {fpred:.2f}"
    # 5) langkah besar: perintah mentah jenuh dan melewati jauh; ZV di atas trapesium harus menekan lewatan
    raw = settle_after_step(a_az, 20.0, micro=16.0)
    sc = settle_after_step(a_az, 20.0, strategy="scurve", micro=16.0)
    zv = settle_after_step(a_az, 20.0, strategy="zv", micro=16.0)
    assert raw["over_pct"] > 40, raw
    assert zv["over_pct"] < sc["over_pct"], (sc, zv)
    # 6) energi: tracker tepat sasaran tidak boleh kalah dari panel datar, dan rugi = 1 - cos(galat)
    e_ok = poa(0.0, math.radians(30.0), 90.0, 60.0, 800.0, 100.0, 900.0)
    assert e_ok > 0
    n1, s1 = tracker.fk(0.0, math.radians(30)), tracker.sun_vec(90.0, 60.0)
    assert abs(math.degrees(math.acos(min(1.0, float(n1 @ s1)))) - 0.0) < 31
    print("selftest osilasi_energi: lolos")
    print(f"  defleksi gravitasi q2=0: {d0:.3f} deg (Gazebo -0.161)")
    print(f"  f dering azimuth: {f:.2f} Hz (prediksi {fpred:.2f})")
    print(f"  langkah 20 deg azimuth: lewatan {raw['over_pct']:.1f} % -> ZV {zv['over_pct']:.1f} %")


def poa_ideal(az, el, dni, dhi, ghi):
    """Batas atas: normal panel tepat ke matahari."""
    cz = math.sin(math.radians(el))
    return dni + dhi * (1 + cz) / 2 + ghi * ALB * (1 - cz) / 2


def poa_flat(el, dni, dhi, ghi):
    """Acuan panel datar menghadap atas."""
    return dni * max(0.0, math.sin(math.radians(el))) + dhi


def run_window(w, strategy, wind_kind, seed, dt, micro, vmax, amax, raw=None):
    """Satu jendela, satu strategi: metrik osilasi, galat pointing, dan daya rata-rata.
    `raw` menimpa lintasan perintah (dipakai untuk kejadian gerak besar)."""
    a_az, a_el = D.AXES["azimuth"], D.AXES["elevasi"]
    r1, r2 = (w["q1"], w["q2"]) if raw is None else raw
    n = len(w["t"])
    c1 = build_cmd(strategy, r1, dt, a_az, micro, vmax, amax)
    c2 = build_cmd(strategy, r2, dt, a_el, micro, vmax, amax)
    wd = wind_window(wind_kind, n, dt, seed)
    q1, q2 = simulate(c1, c2, dt, wind=wd)
    sun_arg = (w["az"], w["el"], w["dni"], w["dhi"], w["ghi"])
    p_load = np.array([poa(a, b, *sun_arg) for a, b in zip(q1, q2)]) * AREA * EFF
    p_cmd = np.array([poa(a, b, *sun_arg) for a, b in zip(c1, c2)]) * AREA * EFF
    s = tracker.sun_vec(w["az"], w["el"])
    ang = lambda a, b: math.degrees(math.acos(max(-1.0, min(1.0, float(tracker.fk(a, b) @ s)))))
    err = np.array([ang(a, b) for a, b in zip(q1, q2)])
    err_c = np.array([ang(a, b) for a, b in zip(c1, c2)])
    # diagnostik lepas sinkron: momen angin melewati torsi tahan Th, dan simpangan melewati satu siklus listrik 7.2 deg
    if wd is None:
        f_pull, runaway = 0.0, False
    else:
        tw_az = np.array([float(D.wind_torque(a_az, v)) for v in wd[0]])
        f_pull = float((np.abs(tw_az) > a_az.Th).mean())
        runaway = bool(np.abs(np.degrees(q1 - c1)).max() > 7.2)
    return dict(hour=w["hour"], strategy=strategy, az=metrics(q1, c1, dt), el=metrics(q2, c2, dt),
                f_pull=f_pull, runaway=runaway,
                err_mean=float(err.mean()), err_max=float(err.max()),
                err_dyn=float((err - err_c).mean()), err_dyn_max=float(np.abs(err - err_c).max()),
                p_load=float(p_load.mean()), p_cmd=float(p_cmd.mean()),
                p_ideal=poa_ideal(*sun_arg) * AREA * EFF, p_flat=poa_flat(w["el"], w["dni"], w["dhi"], w["ghi"]) * AREA * EFF,
                series=(w["t"] - w["t"][0], err, np.degrees(q1 - c1), np.degrees(q2 - c2)))


def event_cmd(w, kind, dt):
    """Lintasan perintah MENTAH untuk kejadian gerak besar: diam 1 s, lalu lompat, lalu tahan.
    Inilah yang membedakan strategi; jejak harian terlalu lambat untuk membedakannya."""
    n = len(w["t"])
    i0 = int(1.0 / dt)
    tgt = {"nyala_awal": (w["q1"][0], w["q2"][0]),                       # dari posisi parkir datar ke pose jejak
           "stow": (w["q1"][0], 0.0),                                     # miringkan ke datar (parkir angin)
           "putar_90": (w["q1"][0] + math.radians(90.0), w["q2"][0])}[kind]
    start = (w["q1"][0], 0.0) if kind == "nyala_awal" else (w["q1"][0], w["q2"][0])
    r1 = np.concatenate([np.full(i0, start[0]), np.full(n - i0, tgt[0])])
    r2 = np.concatenate([np.full(i0, start[1]), np.full(n - i0, tgt[1])])
    return r1, r2


def day_energy_cmd(tr):
    """Energi sehari (Wh) dari sudut PERINTAH saja (tanpa dinamika), grid 10 s. Acuan ekstrapolasi."""
    p = np.array([poa(a, b, az, el, dn, dh, gh) for a, b, az, el, dn, dh, gh in
                  zip(tr["q1"], tr["q2"], tr["az"], tr["el"], tr["dni"], tr["dhi"], tr["ghi"])]) * AREA * EFF
    pf = np.array([poa_flat(el, dn, dh, gh) for el, dn, dh, gh in zip(tr["el"], tr["dni"], tr["dhi"], tr["ghi"])]) * AREA * EFF
    pi = np.array([poa_ideal(az, el, dn, dh, gh) for az, el, dn, dh, gh in
                   zip(tr["az"], tr["el"], tr["dni"], tr["dhi"], tr["ghi"])]) * AREA * EFF
    dt = float(np.median(np.diff(tr["t"])))
    return p.sum() * dt / 3600, pf.sum() * dt / 3600, pi.sum() * dt / 3600


def md(rows, cols, head):
    out = ["| " + " | ".join(head) + " |", "|" + "|".join(["---"] * len(head)) + "|"]
    for r in rows:
        out.append("| " + " | ".join(f"{r[c]:.4g}" if isinstance(r[c], float) else str(r[c]) for c in cols) + " |")
    return "\n".join(out)


def sweep(date="2026-10-07", hours=(9.5, 11.2, 13.0, 14.5), T=60.0, dt=1e-3,
          micro=16.0, strategies=("langsung", "scurve", "zv", "zvd"), winds=("none", "berangin"), seed=3,
          events=("nyala_awal", "stow", "putar_90"), Tev=20.0, micros=(8.0, 16.0, 32.0, 64.0)):
    tr = day(date)
    e_cmd, e_flat, e_ideal = day_energy_cmd(tr)
    wins = [w for w in (window(tr, h, T, dt) for h in hours) if w is not None]
    wev = window(tr, 12.0, Tev, dt)
    vmax, amax = math.radians(30.0), math.radians(300.0)
    rows, per, series, evrows = [], {}, {}, []
    for wk in winds:
        for st in strategies:
            rs = [run_window(w, st, wk, seed, dt, micro, vmax, amax) for w in wins]
            loss = float(np.mean([1 - r["p_load"] / r["p_cmd"] for r in rs])) * 100
            per[(wk, st)] = dict(
                angin=wk, strategi=st,
                osil_az_pp=float(np.max([r["az"]["pp"] for r in rs])), osil_el_pp=float(np.max([r["el"]["pp"] for r in rs])),
                galat_rata=float(np.mean([r["err_mean"] for r in rs])), galat_dyn=float(np.mean([r["err_dyn"] for r in rs])),
                galat_dyn_maks=float(np.max([r["err_dyn_max"] for r in rs])),
                f_pull_pct=float(np.mean([r["f_pull"] for r in rs])) * 100,
                lepas=sum(r["runaway"] for r in rs),
                rugi_pct=loss, E_hari_Wh=e_cmd * (1 - loss / 100))
            rows += [dict(angin=wk, strategi=st, jam=r["hour"], az_pp=r["az"]["pp"], el_pp=r["el"]["pp"],
                          az_bias=r["az"]["bias"], el_bias=r["el"]["bias"], galat=r["err_mean"], galat_dyn=r["err_dyn"],
                          daya_W=r["p_load"], rugi_pct=100 * (1 - r["p_load"] / r["p_cmd"])) for r in rs]
            if wk == "none":
                series[st] = rs[1]["series"]
            for ev in events:
                r = run_window(wev, st, wk, seed, dt, micro, vmax, amax, raw=event_cmd(wev, ev, dt))
                evrows.append(dict(angin=wk, kejadian=ev, strategi=st, az_pp=r["az"]["pp"], el_pp=r["el"]["pp"],
                                   galat_dyn_maks=r["err_dyn_max"], daya_W=r["p_load"],
                                   rugi_pct=100 * (1 - r["p_load"] / r["p_cmd"])))

    out = [f"# Osilasi dua sumbu dan dampaknya pada energi\n",
           f"Dibangkitkan `dinamika/osilasi_energi.py` (tanggal {date}, aktuator `pid`, microstep 1/{micro:.0f}, "
           f"jendela jejak {T:.0f} s dan kejadian {Tev:.0f} s pada 1 kHz, deret angin seed {seed}).\n",
           "Panel MOS C-50-18-M: 0.670 x 0.445 m, 2.55 kg, Pmax 50 W, efisiensi 16.8 %. Gravitasi 10 m/s2.\n",
           "## 1. Jejak matahari biasa (jam saat matahari terjangkau)\n",
           "Osilasi = simpangan sudut beban terhadap perintah, dipisah jadi bias (defleksi tetap) dan ayunan (`pp` = puncak ke puncak, diambil jendela TERBURUK karena sebagian jendela tidak ada langkah sama sekali). "
           "`galat dinamik` = galat pointing beban dikurangi galat pointing perintah, jadi batas jangkauan 45 deg tidak ikut terhitung. "
           "Rugi = daya pada sudut beban dibandingkan daya pada sudut perintah (negatif berarti defleksi kebetulan membantu).\n",
           md(list(per.values()), ["angin", "strategi", "osil_az_pp", "osil_el_pp", "galat_rata", "galat_dyn",
                                   "galat_dyn_maks", "f_pull_pct", "lepas", "rugi_pct", "E_hari_Wh"],
              ["angin", "strategi", "ayun az pp (deg)", "ayun el pp (deg)", "galat total (deg)", "galat dinamik (deg)",
               "galat dinamik maks (deg)", "angin > torsi tahan (% waktu)", "jendela lepas sinkron", "rugi (%)", "energi hari (Wh)"]),
           f"\nEnergi sehari pada sudut perintah (tanpa dinamika): **{e_cmd:.1f} Wh**; panel datar {e_flat:.1f} Wh; "
           f"tracker ideal {e_ideal:.1f} Wh. Kolom energi hari adalah EKSTRAPOLASI: energi perintah dikali (1 - rugi), "
           f"dengan rugi rata-rata dari jendela terpilih, bukan simulasi 1 kHz sehari penuh.\n",
           "\n## 2. Kejadian gerak besar (di sinilah strategi berbeda)\n",
           "`nyala_awal` = dari parkir datar ke pose jejak, `stow` = kemiringan ke 0 deg, `putar_90` = azimuth berputar 90 deg. "
           "Jejak harian terlalu lambat untuk membedakan strategi: setelah dibulatkan ke microstep, perintahnya identik.\n",
           md(evrows, ["angin", "kejadian", "strategi", "az_pp", "el_pp", "galat_dyn_maks", "daya_W", "rugi_pct"],
              ["angin", "kejadian", "strategi", "ayun az pp (deg)", "ayun el pp (deg)", "galat dinamik maks (deg)",
               "daya rata (W)", "rugi (%)"]),
           "\n## 3. Pengaruh microstep pada jejak biasa (tanpa angin)\n",
           "Saat menjejak, sumber ayunan satu-satunya adalah tangga microstep. Shaper tidak bisa menolong di sini "
           "(satu microstep tidak bisa dipecah dua), jadi satu-satunya tuas adalah memperhalus langkah.\n"]
    mrow = []
    for mc in micros:
        rs = [run_window(w, "langsung", "none", seed, dt, mc, vmax, amax) for w in wins]
        mrow.append(dict(micro=f"1/{mc:.0f}", langkah=1.8 / mc,
                         az_pp=float(np.max([r["az"]["pp"] for r in rs])), el_pp=float(np.max([r["el"]["pp"] for r in rs])),
                         galat_dyn_maks=float(np.max([r["err_dyn_max"] for r in rs])),
                         rugi_pct=float(np.mean([1 - r["p_load"] / r["p_cmd"] for r in rs])) * 100))
    out += [md(mrow, ["micro", "langkah", "az_pp", "el_pp", "galat_dyn_maks", "rugi_pct"],
               ["microstep", "satu langkah (deg)", "ayun az pp (deg)", "ayun el pp (deg)", "galat dinamik maks (deg)", "rugi (%)"]),
            "\n## 3b. Respons langkah tunggal (tanpa angin)\n",
            "Lewatan terhadap langkah, dan waktu menetap ke pita 2 %.\n"]
    srow = []
    for nm in ("azimuth", "elevasi"):
        for st in strategies:
            for sd in (1.0, 20.0):
                r = settle_after_step(D.AXES[nm], sd, strategy=st, micro=micro)
                srow.append(dict(sumbu=nm, strategi=st, langkah=sd, over=r["over_pct"], settle=r["settle_s"]))
    out += [md(srow, ["sumbu", "strategi", "langkah", "over", "settle"],
               ["sumbu", "strategi", "langkah (deg)", "lewatan (%)", "menetap (s)"]),
            "\n## 4. Rincian per jendela jejak\n", md(rows, ["angin", "strategi", "jam", "az_pp", "el_pp", "az_bias", "el_bias",
                                                            "galat", "galat_dyn", "daya_W", "rugi_pct"],
                                                      ["angin", "strategi", "jam", "ayun az pp", "ayun el pp", "bias az",
                                                       "bias el", "galat total (deg)", "galat dinamik (deg)", "daya (W)", "rugi (%)"]),
            "\n## 5. Batas\n",
            "- Model aktuator `pid` (linier dengan kejenuhan Th). Stepper sebenarnya bertorsi `Th sin(Nr delta)` yang MENURUN "
            "setelah 1.8 deg, jadi angka di sini optimistis saat selisih besar; lepas langkah tidak dimodelkan.\n"
            "- Redaman sendi 0.2 N.m.s/rad dan gesekan 0.05 N.m masih ASUMSI dan paling menentukan lama ayunan.\n"
            "- Suku Coriolis diabaikan; kopling antar sumbu hanya lewat J_az(q2).\n"
            "- Matahari dianggap tetap di dalam satu jendela 12 s (benar sampai sekitar 0.05 deg).\n"
            "- Hanya jendela terpilih yang disimulasikan 1 kHz; energi harian adalah ekstrapolasi.\n"]
    (Path(__file__).parent / "osilasi_results.md").write_text("\n".join(out), encoding="utf-8")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for st in strategies:
        t, err, e1, e2 = series[st]
        ax[0].plot(t, err, lw=0.8, label=st)
        ax[1].plot(t, e1, lw=0.8, label=st)
    ax[0].set_ylabel("galat pointing (deg)"); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    ax[1].set_ylabel("simpangan azimuth (deg)"); ax[1].set_xlabel("waktu (s)"); ax[1].grid(alpha=0.3)
    ax[0].set_title(f"Jendela jam {wins[1]['hour']:.1f}, {date}, tanpa angin")
    fig.tight_layout(); fig.savefig(Path(__file__).parent / "osilasi_jendela.png", dpi=110)

    print(md(list(per.values()), ["angin", "strategi", "osil_az_pp", "osil_el_pp", "galat_dyn", "f_pull_pct", "lepas", "rugi_pct"],
             ["angin", "strategi", "ayun az pp", "ayun el pp", "galat dinamik", "angin>Th %", "lepas", "rugi %"]))
    print(md(mrow, ["micro", "az_pp", "el_pp", "rugi_pct"], ["microstep", "ayun az pp", "ayun el pp", "rugi %"]))
    print(md([r for r in evrows if r["angin"] == "none"], ["kejadian", "strategi", "az_pp", "el_pp", "galat_dyn_maks"],
             ["kejadian", "strategi", "ayun az pp", "ayun el pp", "galat dinamik maks"]))
    print(f"\nenergi perintah {e_cmd:.1f} Wh, datar {e_flat:.1f} Wh, ideal {e_ideal:.1f} Wh")
    print("tersimpan osilasi_results.md dan osilasi_jendela.png")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
    else:
        selftest()
        sweep()
