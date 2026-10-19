"""Simulasi kontrol tracker 2 sumbu: astronomi (pvlib), LDR 4 kuadran bersekat, dan hibrid.
Kinematik saja (kuasi-statis): motor dianggap mencapai perintah, dibatasi laju dan dikuantisasi per microstep.
Frame dunia: x=Utara, y=Barat, z=atas. q1 azimuth sendi (CW, arah miring kompas = q1 + HEAD, lihat tracker.py), q2 kemiringan (0 = horizontal), seperti tracker.py.
Kontroler: astro (A), ldr (B), saputra (C: hibrid ambang tetap, tanpa memori), bias (D: hibrid indeks kecerahan + memori koreksi).
"""
import math
import random
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import pandas as pd
import pvlib

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
import sun       # noqa: E402  (modul proyek, tidak diubah)
import tracker   # noqa: E402

LUX = 110.0  # lm/W, sama dengan sun.lux
CTRLS = ["astro", "ldr", "saputra", "bias"]


@dataclass
class P:
    dt: float = 2.0
    step: float = 1.8 / 16        # deg per microstep (poros langsung, microstep 1/16 default)
    rate1: float = 0.5            # deg/s batas laju azimuth (hasil analisis_zenith)
    rate2: float = 1.0            # deg/s batas laju kemiringan (asumsi)
    q2max: float = 45.0           # deg, asumsi (belum diukur)
    bias: tuple = (0.0, 0.0)      # (azimuth, kemiringan) salah pasang mekanik, deg
    r: float = 1.0                # tinggi sekat / lebar LDR (asumsi: bersekat silang)
    mism: float = 0.03            # simpangan baku ketidakcocokan antar LDR (asumsi)
    L0: float = 20000.0           # lux, setengah jenuh pembagi tegangan (asumsi)
    gam: float = 0.7              # eksponen LDR (asumsi)
    vcc: float = 3.3
    bits: int = 12
    vn: float = 0.002             # V, derau ADC (asumsi)
    kp: float = 60.0              # deg/s per satuan e (kemiringan e sekitar 0.0013 per deg saat cerah, jadi tau sekitar 13 s)
    dead: float = 0.002           # deadzone pada e (sekitar 1.5 deg saat cerah, di atas 3x derau e 0.0006)
    wthr: float = 400.0           # W/m2 ekuivalen sensor, ambang hibrid ala Saputra (asumsi)
    kcthr: float = 0.75           # ambang indeks kecerahan hibrid bias
    guard: float = 5.0            # deg, |q2| di bawah ini azimuth tidak dikoreksi LDR
    dmax: float = 10.0            # deg, batas memori koreksi
    elmin: float = 5.0            # deg, di bawah ini siaga
    albedo: float = 0.25


def basis(q1, q2):
    """Normal panel n, sumbu x panel (arah rel, arah kemiringan) dan sumbu y panel di frame base."""
    a, b = math.radians(q1) + tracker.HEAD, math.radians(q2)   # arah miring kompas = q1 + HEAD (lihat tracker.py)
    n = (math.sin(b) * math.cos(a), -math.sin(b) * math.sin(a), math.cos(b))
    xp = (math.cos(b) * math.cos(a), -math.cos(b) * math.sin(a), -math.sin(b))
    yp = (math.sin(a), math.cos(a), 0.0)
    return n, xp, yp


def dot(u, v):
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]


def shade(eps, r):
    """Fraksi LDR yang tertutup bayangan sekat bila matahari miring eps (rad) ke sisi LDR lawan."""
    return min(1.0, max(0.0, r * math.tan(eps))) if 0.0 < eps < math.pi / 2 else (1.0 if eps >= math.pi / 2 else 0.0)


def volt(w, p, rng):
    """W/m2 pada LDR -> tegangan ADC (pembagi tegangan jenuh, derau, kuantisasi)."""
    x = (max(w, 0.0) * LUX / p.L0) ** p.gam
    v = p.vcc * x / (1 + x) + p.vn * rng.gauss(0, 1)
    lsb = p.vcc / (2 ** p.bits - 1)
    return min(p.vcc, max(0.0, round(v / lsb) * lsb))


def read(n, xp, yp, s, dni, dhi, mism, p, rng):
    """Empat LDR (x+, x-, y+, y-): kembalikan ex, ey, V rata-rata."""
    c = dot(n, s)
    ex_a, ey_a = math.atan2(dot(s, xp), c), math.atan2(dot(s, yp), c)
    beam, dif = dni * max(c, 0.0), dhi * (1 + n[2]) / 2
    sh = (shade(-ex_a, p.r), shade(ex_a, p.r), shade(-ey_a, p.r), shade(ey_a, p.r))  # tiap LDR dibayangi sekat dari sisi lawan
    v = [volt((beam * (1 - sh[i]) + dif) * mism[i], p, rng) for i in range(4)]
    return (v[0] - v[1]) / (v[0] + v[1] + 1e-9), (v[2] - v[3]) / (v[2] + v[3] + 1e-9), sum(v) / 4


def ldr_step(q1, q2, ex, ey, p):
    """Koreksi proporsional dengan deadzone. ex>0: matahari di sisi +x panel (naikkan q2). ey>0: di sisi +y panel."""
    m1, m2 = p.rate1 * p.dt, p.rate2 * p.dt
    if abs(ex) > p.dead:
        q2 += max(-m2, min(m2, p.kp * ex * p.dt))
    if abs(ey) > p.dead and abs(q2) >= p.guard:
        q1 += max(-m1, min(m1, -p.kp * ey * p.dt / math.sin(math.radians(q2))))  # dn/dq1 = -sin(q2) yp
    return q1, q2


def kt_series(kind, n, dt, rng):
    """Faktor GHI/GHI_cerah: cerah, mendung (AR(1) di sekitar 0.25), atau sebagian (awan datang-pergi)."""
    if kind == "cerah":
        return np.ones(n)
    if kind == "mendung":
        a, e = np.zeros(n), rng.normal(0, 0.004, n)
        for i in range(1, n):
            a[i] = 0.995 * a[i - 1] + e[i]
        return np.clip(0.25 + a, 0.1, 0.45)
    out, i, cloudy = np.ones(n), 0, False
    while i < n:
        dur = int(rng.exponential(300 if cloudy else 600) / dt) + 1
        out[i:i + dur] = rng.uniform(0.15, 0.5) if cloudy else 1.0
        i += dur
        cloudy = not cloudy
    k = max(1, int(40 / dt))
    return np.convolve(out, np.ones(k) / k, mode="same")


def day_data(day, kind, seed, p):
    """Siapkan deret matahari, irradiance (dengan awan), dan referensi astronomi untuk satu hari."""
    t = pd.date_range(f"{day} 04:30", f"{day} 19:30", freq=f"{int(p.dt)}s", tz=sun.TZ)
    sp, cs = sun.loc.get_solarposition(t), sun.loc.get_clearsky(t)
    kt = kt_series(kind, len(t), p.dt, np.random.default_rng(seed))
    ghi = cs.ghi * kt
    er = pvlib.irradiance.erbs(ghi, sp.zenith, t)
    m = (sp.apparent_elevation.values >= p.elmin)
    az, el = sp.azimuth.values[m], sp.apparent_elevation.values[m]
    q1r, q2r = tracker.ik_track(az, el, q2_max=np.radians(90))   # cabang kontinu, lalu dipotong ke batas kemiringan
    q1r, q2r = np.degrees(q1r), np.clip(np.degrees(q2r), -p.q2max, p.q2max)
    sv = np.array([tracker.sun_vec(a, e) for a, e in zip(az, el)])
    d = dict(n=int(m.sum()), zen=(90 - el).tolist(), s=sv.tolist(), q1r=q1r.tolist(), q2r=q2r.tolist(),
             dni=er.dni.fillna(0).values[m].tolist(), dhi=er.dhi.fillna(0).values[m].tolist(), ghi=ghi.values[m].tolist(),
             dnic=cs.dni.values[m].tolist(), dhic=cs.dhi.values[m].tolist())
    bz = math.cos(math.radians(7.0))  # panel tetap miring 7 deg menghadap utara
    nf = (math.sin(math.radians(7.0)), 0.0, bz)
    d["e_fixed"] = sum(poa_w(nf, s, a, b, g, p) for s, a, b, g in zip(d["s"], d["dni"], d["dhi"], d["ghi"])) * p.dt / 3.6e6
    return d


def poa_w(n, s, dni, dhi, ghi, p):
    return dni * max(dot(n, s), 0.0) + dhi * (1 + n[2]) / 2 + ghi * p.albedo * (1 - n[2]) / 2


def simulate(ctrl, d, p, seed, ext=None, ov=None):
    """ext(k, q2_fisik) -> (offset_azimuth, offset_kemiringan) deg: defleksi dan lepas langkah dari model dinamika (opsional).
    ov(k) -> None atau (target_q1 atau None, target_q2): menimpa perintah (parkir/stow), mode STOW."""
    ci = CTRLS.index(ctrl)
    rng = random.Random(seed * 1009 + ci)
    mrng = random.Random(seed + 77)
    mism = [1 + mrng.gauss(0, p.mism) for _ in range(4)]   # sama untuk semua kontroler dalam satu seed
    q1c, q2c = (90.0 - math.degrees(tracker.HEAD), 45.0) if ctrl == "ldr" else (d["q1r"][0], d["q2r"][0])
    q1q, q2q = round(q1c / p.step) * p.step, round(q2c / p.step) * p.step
    d1 = d2 = 0.0   # memori koreksi (kontroler bias)
    kc, modes, errs, energy, acts, travel, kl, pw = 1.0, {}, [], 0.0, 0, 0.0, [], []
    b1, b2 = p.bias
    for k in range(d["n"]):
        o1, o2 = ext(k, q2q + b2) if ext else (0.0, 0.0)
        n, xp, yp = basis(q1q + b1 + o1, q2q + b2 + o2)
        s, dni, dhi = d["s"][k], d["dni"][k], d["dhi"][k]
        c = dot(n, s)
        wk = poa_w(n, s, dni, dhi, d["ghi"][k], p)
        energy += wk
        if ext:
            pw.append(wk)
        if d["zen"][k] <= p.q2max:
            errs.append(math.degrees(math.acos(min(1.0, max(-1.0, c)))))
            kl.append(k)
        q1r, q2r = d["q1r"][k], d["q2r"][k]
        mode = "ASTRO"
        stow = ov(k) if ov else None
        if stow is not None:
            mode = "STOW"
            t1, t2 = (q1c if stow[0] is None else stow[0]), stow[1]
        elif ctrl == "astro":
            t1, t2 = q1r, q2r
        else:
            ex, ey, vm = read(n, xp, yp, s, dni, dhi, mism, p, rng)
            x = vm / max(p.vcc - vm, 1e-6)
            w_est = p.L0 * x ** (1 / p.gam) / LUX
            kc += 0.2 * (w_est / max(d["dnic"][k] + d["dhic"][k] * (1 + math.cos(math.radians(q2r))) / 2, 1e-6) - kc)
            t1, t2 = q1c, q2c
            if ctrl == "ldr":
                mode = "LDR"
                t1, t2 = ldr_step(q1c, q2c, ex, ey, p)
                if abs(q2c) < p.guard:
                    t1 = q1c   # dekat zenith: azimuth ditahan
            elif ctrl == "saputra":
                if w_est >= p.wthr:
                    mode = "LDR"
                    t1, t2 = ldr_step(q1c, q2c, ex, ey, p)
                    if abs(q2c) < p.guard:
                        t1 = q1r
                else:
                    t1, t2 = q1r, q2r
            else:  # bias: astro sebagai feedforward, LDR hanya mengoreksi memori d1,d2 saat sensor dipercaya
                if kc >= p.kcthr:
                    mode = "STABIL"
                    if abs(ex) > p.dead or abs(ey) > p.dead:
                        mode = "KOREKSI"
                        if abs(q2r + d2) < p.q2max - 1e-9:   # anti-windup saat kemiringan menyentuh batas
                            d2 += max(-p.rate2 * p.dt, min(p.rate2 * p.dt, p.kp * ex * p.dt)) if abs(ex) > p.dead else 0.0
                        if abs(ey) > p.dead and abs(q2r) >= p.guard:
                            d1 += max(-p.rate1 * p.dt, min(p.rate1 * p.dt, -p.kp * ey * p.dt / math.sin(math.radians(q2r))))
                        d1, d2 = max(-p.dmax, min(p.dmax, d1)), max(-p.dmax, min(p.dmax, d2))
                t1, t2 = q1r + d1, q2r + d2
        modes[mode] = modes.get(mode, 0) + 1
        q1c += max(-p.rate1 * p.dt, min(p.rate1 * p.dt, t1 - q1c))
        q2c += max(-p.rate2 * p.dt, min(p.rate2 * p.dt, t2 - q2c))
        q2c = max(-p.q2max, min(p.q2max, q2c))
        n1, n2 = round(q1c / p.step) * p.step, round(q2c / p.step) * p.step
        acts += (n1 != q1q) + (n2 != q2q)
        travel += abs(n1 - q1q) + abs(n2 - q2q)
        q1q, q2q = n1, n2
    e = np.array(errs)
    kwh = energy * p.dt / 3.6e6
    f = lambda m: sum(v for kk, v in modes.items() if kk in m) / d["n"]
    return dict(rmse=float(np.sqrt((e ** 2).mean())), tae=float(e.mean()), p95=float(np.percentile(e, 95)), emax=float(e.max()),
                acts=acts, travel=travel, gain=100 * (kwh / d["e_fixed"] - 1), kwh=kwh,
                f_ldr=f({"LDR", "KOREKSI", "STABIL"}), f_astro=f({"ASTRO"}), f_stow=f({"STOW"}), **(dict(errs=errs, ks=kl, pw=pw) if ext else {}))


def self_check():
    """Tanda koreksi LDR benar: dari pose acak, satu langkah koreksi (tanpa derau, tanpa difus) harus mengecilkan galat."""
    p = replace(P(), vn=0.0, mism=0.0, dead=0.0)
    rng = random.Random(1)
    for q1, q2, ox, oy in [(30, 20, 1.0, -1.5), (200, 35, -2.0, 1.0), (310, -25, 1.5, 1.5), (80, 10, -1.0, -2.0)]:
        n, xp, yp = basis(q1, q2)
        s = [n[i] + math.radians(ox) * xp[i] + math.radians(oy) * yp[i] for i in range(3)]
        nrm = math.sqrt(dot(s, s))
        s = tuple(v / nrm for v in s)
        ex, ey, _ = read(n, xp, yp, s, 800.0, 0.0, [1, 1, 1, 1], p, rng)
        n1, n2 = ldr_step(q1, q2, ex, ey, replace(p, kp=1.0))
        nn = basis(n1, n2)[0]
        before, after = math.acos(min(1, dot(n, s))), math.acos(min(1, dot(nn, s)))
        assert after < before, (q1, q2, ox, oy, before, after)
    print("self-check tanda koreksi LDR: lolos")


def quick():
    self_check()
    for kind in ("cerah", "sebagian", "mendung"):
        for bias in ((0.0, 0.0), (3.0, 2.0)):
            p = replace(P(), bias=bias)
            d = day_data("2026-10-04", kind, 0, p)
            print(f"--- {kind}, salah pasang (az, tilt) = {bias}")
            for c in CTRLS:
                r = simulate(c, d, p, 0)
                print(f"{c:8s} rmse={r['rmse']:.3f} tae={r['tae']:.3f} p95={r['p95']:.3f} acts={r['acts']} gain={r['gain']:.2f}% ldr={r['f_ldr']:.2f}")


_CACHE = {}


def data(day, kind, seed, p):
    key = (day, kind, seed, p.dt, p.q2max, p.elmin)
    if key not in _CACHE:
        _CACHE[key] = day_data(day, kind, seed, p)
    return _CACHE[key]


WEATHER = {"cerah": [0], "sebagian": [1, 2, 3, 4], "mendung": [1, 2]}


def run_grid(days, biases, ctrls, weather=WEATHER, **over):
    rows = []
    for day in days:
        for kind, seeds in weather.items():
            for bias in biases:
                p = replace(P(), bias=bias, **over)
                for seed in seeds:
                    d = data(day, kind, seed, p)
                    for c in ctrls:
                        rows.append(dict(day=day, weather=kind, bias=f"{bias[0]:g}/{bias[1]:g}", ctrl=c, seed=seed, **over, **simulate(c, d, p, seed)))
    return pd.DataFrame(rows)


def md(df, cols):
    s = "| " + " | ".join(cols) + " |\n|" + "---|" * len(cols) + "\n"
    for _, r in df.iterrows():
        s += "| " + " | ".join(f"{r[c]:.3g}" if isinstance(r[c], float) else str(r[c]) for c in cols) + " |\n"
    return s


def main():
    self_check()
    out = ["# Tabel hasil simulasi kontrol hibrid (dibuat otomatis oleh hybrid_sim.py)\n",
           "Rata-rata antar seed (cerah 1 seed, sebagian 4, mendung 2). Galat = sudut antara normal panel dan matahari, hanya saat matahari terjangkau (zenith <= 45 deg).",
           "acts = jumlah perubahan microstep (dua sumbu), gain = energi bidang vs panel tetap miring 7 deg utara, f_ldr = fraksi waktu LDR dipakai.\n"]
    g = ["rmse", "tae", "p95", "acts", "gain", "f_ldr"]
    # E1: matriks utama
    e1 = run_grid(["2026-10-04", "2026-06-21"], [(0.0, 0.0), (3.0, 2.0), (6.0, 4.0)], CTRLS)
    e1.to_csv(HERE / "hybrid_runs_e1.csv", index=False)
    for day in e1.day.unique():
        out.append(f"\n## E1: {day}\n")
        a = e1[e1.day == day].groupby(["weather", "bias", "ctrl"], sort=False)[g].mean().reset_index()
        out.append(md(a, ["weather", "bias", "ctrl"] + g))
    # E2: sensitivitas sensor (r = tinggi sekat/lebar LDR, mism = ketidakcocokan antar LDR)
    rows = []
    for r_, m_ in [(1, 0.03), (1, 0.01), (2, 0.03), (2, 0.01), (2, 0.0)]:
        x = run_grid(["2026-10-04"], [(3.0, 2.0)], ["ldr", "saputra", "bias"], weather={"cerah": [0], "sebagian": [1, 2, 3, 4]}, r=float(r_), mism=m_)
        rows.append(x)
    e2 = pd.concat(rows)
    out.append("\n## E2: sensitivitas sensor, 2026-10-04, salah pasang 3/2\n")
    out.append(md(e2.groupby(["weather", "r", "mism", "ctrl"], sort=False)[["rmse", "p95", "acts", "gain"]].mean().reset_index(), ["weather", "r", "mism", "ctrl", "rmse", "p95", "acts", "gain"]))
    # E3: sapuan deadzone
    rows = [run_grid(["2026-10-04"], [(3.0, 2.0)], ["saputra", "bias"], weather={"cerah": [0], "sebagian": [1, 2, 3, 4]}, dead=dz) for dz in (0.001, 0.002, 0.005, 0.01, 0.02)]
    e3 = pd.concat(rows)
    out.append("\n## E3: sapuan deadzone (satuan e), 2026-10-04, salah pasang 3/2\n")
    out.append(md(e3.groupby(["weather", "dead", "ctrl"], sort=False)[["rmse", "p95", "acts", "gain"]].mean().reset_index(), ["weather", "dead", "ctrl", "rmse", "p95", "acts", "gain"]))
    (HERE / "hybrid_results_tables.md").write_text("\n".join(out) + "\n", encoding="utf-8")
    print("selesai: hybrid_results_tables.md")


if __name__ == "__main__":
    quick() if "--quick" in sys.argv else main()
