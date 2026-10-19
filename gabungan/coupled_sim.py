"""Penggabungan dinamika (dyn_model) dan kontrol hibrid (hybrid_sim).
ODE (dt 0.1 ms) tidak bisa dijalankan 12 jam bersama simulasi kontrol (dt 2 s), jadi dipakai model tereduksi:
  1. Defleksi kuasi-statis akibat beban: asin(tau/Th)/Nr + tau/k (dicek terhadap ODE di self_check).
  2. Kejadian melewati torsi tahan (open-loop): dihitung dari angin, dampaknya dilihat dari tabel ODE (satu kejadian = hilang sumbu, ratusan siklus).
  3. Uji pemulihan: lepas langkah kecil disuntikkan (1, 3, 10 siklus listrik) pada sepertiga hari.
  4. Angin sintetis: proses OU + hembusan Poisson, momen = 0.5 rho v2 Cn A arm (semua asumsi).
Konvensi: tau positif = momen penggerak ke arah q positif. Posisi fisik = perintah terkuantisasi + salah pasang + defleksi + lepas langkah.
"""
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "kontrol_hibrid"))
sys.path.insert(0, str(ROOT / "dinamika"))
import hybrid_sim as H   # noqa: E402
import dyn_model as D    # noqa: E402

RATIOS = [1.02, 1.1, 1.25, 1.5, 2.0]   # torsi puncak / torsi tahan
DURS = [0.25, 0.5, 1.0, 2.0, 4.0]      # detik di atas torsi tahan
DTW = 0.25                             # detik, resolusi angin
WINDS = {"tenang": dict(vm=3.0, turb=0.30, gust_h=1.0, gamp=(0.5, 1.0)),
         "berangin": dict(vm=7.0, turb=0.30, gust_h=3.0, gamp=(0.8, 1.5)),
         "badai": dict(vm=11.0, turb=0.35, gust_h=6.0, gamp=(1.0, 1.8))}   # semua skenario adalah asumsi uji tekan
SENSORS = {"default": dict(r=1.0, mism=0.03), "baik": dict(r=2.0, mism=0.01)}
CTRLS = H.CTRLS


def defl(tau, ax):
    """Defleksi kuasi-statis (deg, bertanda) untuk momen penggerak tau (N.m)."""
    a = abs(tau)
    return math.copysign(math.degrees(math.asin(min(a / ax.Th, 0.999)) / ax.Nr + a / ax.k), tau)


def slip_table(ax):
    """|siklus listrik| yang lepas (1 siklus = 360/Nr deg) setelah denyut torsi: baris = RATIOS, kolom = DURS. Dari ODE (cache JSON)."""
    sig = json.dumps([ax.Th, ax.Nr, ax.Jm, ax.k, ax.JL, ax.bl, ax.fc, ax.zeta, ax.bm, ax.grav])
    f = HERE / f"slip_table_{ax.name}.json"
    if f.exists():
        c = json.loads(f.read_text())
        if c["sig"] == sig:
            return np.abs(np.array(c["cycles"]))
    cases = [(r, d) for r in RATIOS for d in DURS]
    amp = np.array([r * ax.Th for r, _ in cases])
    dur = np.array([d for _, d in cases])
    t0, ramp = 0.2, 0.1

    def tau(t):
        shape = np.clip((t - t0) / ramp, 0, 1) - np.clip((t - t0 - dur) / ramp, 0, 1)
        return ax.grav + (amp - ax.grav) * shape

    T = t0 + dur.max() + ramp + 3.0
    t, th, _ = D.integrate(ax, lambda t: np.zeros(len(cases)), tau, T, tau0=ax.grav, M=len(cases))
    cyc = np.round((th[-1] - th[0]) / (2 * math.pi / ax.Nr)).astype(int).reshape(len(RATIOS), len(DURS))
    f.write_text(json.dumps(dict(sig=sig, cycles=cyc.tolist())))
    return np.abs(cyc)


def lookup(tab, ratio, dur):
    if ratio <= 1.0:
        return 0
    rows = [np.interp(np.clip(dur, DURS[0], DURS[-1]), DURS, tab[i]) for i in range(len(RATIOS))]
    return int(round(float(np.interp(np.clip(ratio, RATIOS[0], RATIOS[-1]), RATIOS, rows))))


def wind_series(kind, n_sub, seed):
    """USANG. Kecepatan angin (m/s) dan tanda arah per sub-sampel DTW.

    Tidak lagi dipakai node mana pun. Diganti `angin/angin_openmeteo.py`: data NYATA Open-Meteo 11 tahun untuk
    PENS, lengkap dengan arah, dan torsi yang bergantung orientasi panel. Dua cacat model ini: (a) preset
    `tenang` 3, `berangin` 7, `badai` 11 m/s dimaksudkan sebagai angin DI PANEL padahal hembusan terburuk
    11 tahun di tinggi panel hanya 6.22 m/s, jadi 2 sampai 3 kali terlalu berat; (b) hanya memberi tanda
    plus minus acak, bukan arah mata angin, sehingga torsinya tidak bergantung orientasi panel dan torsi
    azimuth tidak nol saat panel datar. Dipertahankan hanya agar studi luring lama tetap dapat dijalankan."""
    a, rng = WINDS[kind], np.random.default_rng(seed)
    f = math.exp(-DTW / 15.0)
    g = math.sqrt(1 - f * f)
    e, x = rng.normal(size=n_sub), np.zeros(n_sub)
    for i in range(1, n_sub):
        x[i] = f * x[i - 1] + g * e[i]
    v = np.maximum(0.0, a["vm"] * (1 + a["turb"] * x))
    for _ in range(rng.poisson(a["gust_h"] * n_sub * DTW / 3600)):
        t0, dur, amp = rng.uniform(0, n_sub * DTW), rng.uniform(3, 8), rng.uniform(*a["gamp"]) * a["vm"]
        i0, i1 = int(t0 / DTW), min(n_sub, int((t0 + dur) / DTW))
        if i1 > i0:
            v[i0:i1] += amp * np.sin(np.pi * np.arange(i1 - i0) / (i1 - i0)) ** 2
    sign, i = np.empty(n_sub), 0
    while i < n_sub:
        d = int(rng.exponential(60) / DTW) + 1
        sign[i:i + d] = rng.choice([-1.0, 1.0])
        i += d
    return v, sign


class Ext:
    """Kait untuk hybrid_sim.simulate: offset (azimuth, kemiringan) = defleksi kuasi-statis + lepas langkah yang disuntikkan.
    Kejadian melewati torsi tahan hanya DICATAT (events, cycles_would_slip), tidak diterapkan: dampaknya ratusan siklus (lihat tabel)."""

    def __init__(self, wind, seed, p, n_steps, grav_on=True, inject=None, k_inject=0, k_flat=1.0):
        self.ax = D.AXES
        self.tabs = {nm: slip_table(a) for nm, a in self.ax.items()}
        self.grav_on, self.inject, self.k_inject = grav_on, inject, k_inject
        self.k_flat = k_flat   # skala momen angin saat panel datar (1 = tidak ada manfaat), linear terhadap |q2|/45 deg (asumsi)
        self.ev_steps = {nm: [] for nm in self.ax}
        self.m = int(round(p.dt / DTW))
        n = n_steps * self.m
        if wind is None:
            self.v, self.tw = np.zeros(n), {nm: np.zeros(n) for nm in self.ax}
        else:
            self.v, sg = wind_series(wind, n, seed)
            self.tw = {nm: sg * D.wind_torque(a, self.v) for nm, a in self.ax.items()}
        self.slip = {nm: 0.0 for nm in self.ax}
        self.run = {nm: None for nm in self.ax}
        self.events = {nm: 0 for nm in self.ax}
        self.cycles = {nm: 0 for nm in self.ax}

    def _scan(self, nm, taus, k=None):
        ax = self.ax[nm]
        for t in taus:
            if abs(t) > ax.Th:
                if self.run[nm] is None and k is not None:
                    self.ev_steps[nm].append(k)   # langkah awal kejadian
                pk, du = (self.run[nm][0], self.run[nm][1]) if self.run[nm] else (0.0, 0.0)
                self.run[nm] = (max(pk, abs(t)), du + DTW)
            elif self.run[nm] is not None:
                pk, du = self.run[nm]
                self.events[nm] += 1
                self.cycles[nm] += lookup(self.tabs[nm], pk / ax.Th, du)
                self.run[nm] = None

    def __call__(self, k, q2):
        if self.inject and k == self.k_inject:
            for nm in self.slip:
                self.slip[nm] += self.inject
        sl = slice(k * self.m, (k + 1) * self.m)
        g = self.ax["elevasi"].grav / math.sin(math.radians(45)) * math.sin(math.radians(q2)) if self.grav_on else 0.0
        out = []
        sc = self.k_flat + (1.0 - self.k_flat) * min(1.0, abs(q2) / 45.0)
        for nm, gz in (("azimuth", 0.0), ("elevasi", g)):
            tw = self.tw[nm][sl] * sc
            self._scan(nm, gz + tw, k)
            out.append(defl(gz + float(tw.mean()), self.ax[nm]) + self.slip[nm])
        return out[0], out[1]


def make_p(sensor, bias=(3.0, 2.0)):
    return replace(H.P(), bias=bias, **SENSORS[sensor])


def run_job(job):
    """job = (exp, sensor, kind, wseed, wind, ctrl, seed, slip). Kembalikan satu baris hasil."""
    exp, sensor, kind, wseed, wind, ctrl, seed, slip = job
    p = make_p(sensor)
    d = H.data("2026-10-04", kind, seed, p)
    row = dict(exp=exp, sensor=sensor, weather=kind, wind=wind or "tanpa", ctrl=ctrl, seed=seed, wseed=wseed, slip_deg=slip or 0.0)
    if wind == "tanpa_dinamika":
        r = H.simulate(ctrl, d, p, seed)
        ev_az = ev_el = cy_az = cy_el = vmax = 0
    else:
        kinj = d["n"] // 3
        ext = Ext(None if wind in ("gravitasi", None) else wind, wseed, p, d["n"], inject=slip, k_inject=kinj)
        r = H.simulate(ctrl, d, p, seed, ext=ext)
        ev_az, ev_el, cy_az, cy_el, vmax = ext.events["azimuth"], ext.events["elevasi"], ext.cycles["azimuth"], ext.cycles["elevasi"], float(ext.v.max())
        if slip:
            e, ks = np.array(r["errs"]), np.array(r["ks"])
            post = ks >= kinj
            ep = e[post]
            row["rmse_post"] = float(np.sqrt((ep ** 2).mean()))
            row["err_end"] = float(ep[-450:].mean())
            ok = ep < 3.0
            rec = np.nan
            for j in range(len(ep) - 300):
                if ok[j:j + 300].all():
                    rec = (ks[post][j] - kinj) * p.dt / 60.0
                    break
            row["recover_min"] = rec
    row.update(rmse=r["rmse"], p95=r["p95"], emax=r["emax"], gain=r["gain"], acts=r["acts"], f_ldr=r["f_ldr"],
               ev_az=ev_az, ev_el=ev_el, cyc_az=cy_az, cyc_el=cy_el, vmax=vmax)
    return row


def self_check():
    el = D.AXES["elevasi"]
    # (1) defleksi analitik cocok dengan ODE (hembusan 15 m/s pada elevasi dengan gravitasi 0.64 N.m)
    w = D.wind_gust(el, [15.0], tau0=el.grav)
    ana = defl(el.grav + float(D.wind_torque(el, 15.0)), el) - defl(el.grav, el)
    assert abs(ana - float(w["final_deg"][0])) < 0.06, (ana, w["final_deg"])
    # (2) tanpa angin, gravitasi, dan suntikan: sama persis dengan simulasi tanpa dinamika
    p = make_p("default")
    d = H.data("2026-10-04", "cerah", 0, p)
    base = H.simulate("bias", d, p, 0)
    r0 = H.simulate("bias", d, p, 0, ext=Ext(None, 0, p, d["n"], grav_on=False))
    assert abs(base["rmse"] - r0["rmse"]) < 1e-9, (base["rmse"], r0["rmse"])
    # (3) tabel lepas langkah: ada lepas pada denyut kuat dan panjang
    tab = slip_table(el)
    assert tab[-1, -1] >= 1 and all(np.diff(tab[-1]) >= 0), tab
    # (4) suntikan lepas langkah 7.2 deg membuat astro menyimpang >= 5 deg dan tidak pulih
    rj = H.simulate("astro", d, p, 0, ext=Ext(None, 0, p, d["n"], grav_on=False, inject=7.2, k_inject=d["n"] // 3))
    assert np.array(rj["errs"])[-100:].mean() > 5.0, np.array(rj["errs"])[-100:].mean()
    print("self-check gabungan: lolos")


def show_tables():
    for nm, ax in D.AXES.items():
        print(f"\n|siklus| lepas ({360 / ax.Nr:.1f} deg per siklus), {nm}: baris torsi puncak/Th {RATIOS}, kolom durasi {DURS} s\n", slip_table(ax))


def md(df, cols):
    s = "| " + " | ".join(cols) + " |\n|" + "---|" * len(cols) + "\n"
    for _, r in df.iterrows():
        s += "| " + " | ".join(f"{r[c]:.3g}" if isinstance(r[c], float) else str(r[c]) for c in cols) + " |\n"
    return s


def main():
    self_check()
    jobs = []
    for sensor in SENSORS:
        for kind, seeds in (("cerah", [0]), ("sebagian", [1, 2])):
            for seed in seeds:
                for c in CTRLS:
                    jobs.append(("E1", sensor, kind, 0, "tanpa_dinamika", c, seed, None))
                    jobs.append(("E1", sensor, kind, 0, "gravitasi", c, seed, None))
                    for wd in WINDS:
                        for ws in (0, 1, 2):
                            jobs.append(("E1", sensor, kind, ws, wd, c, seed, None))
                    for sl in (7.2, 21.6, 72.0):
                        jobs.append(("E2", sensor, kind, 0, "gravitasi", c, seed, sl))
    with ProcessPoolExecutor(max_workers=14) as ex:
        rows = list(ex.map(run_job, jobs, chunksize=4))
    df = pd.DataFrame(rows)
    df.to_csv(HERE / "coupled_runs.csv", index=False)
    out = ["# Tabel hasil penggabungan dinamika dan kontrol hibrid (dibuat otomatis oleh coupled_sim.py)\n",
           "2026-10-04, salah pasang 3/2 deg, rata-rata antar seed cuaca (cerah 1, sebagian 2) dan seed angin (3). Galat pointing hanya saat zenith <= 45 deg.\n"]
    e1 = df[df.exp == "E1"]
    g = e1.groupby(["sensor", "weather", "wind", "ctrl"], sort=False)[["rmse", "p95", "emax", "gain", "acts", "ev_az", "ev_el", "cyc_el", "vmax"]].mean().reset_index()
    out += ["\n## E1: angin di bawah/di atas torsi tahan, tanpa menerapkan lepas langkah (kejadian hanya dihitung)\n",
            "ev_az dan ev_el = rata-rata kejadian melewati torsi tahan per hari siang (12 jam), vmax = angin maksimum (m/s).\n",
            md(g, ["sensor", "weather", "wind", "ctrl", "rmse", "p95", "emax", "gain", "acts", "ev_az", "ev_el", "vmax"])]
    e2 = df[df.exp == "E2"]
    g2 = e2.groupby(["sensor", "weather", "slip_deg", "ctrl"], sort=False)[["rmse_post", "err_end", "recover_min"]].agg(lambda s: float(np.nanmean(s)) if s.notna().any() else float("nan")).reset_index()
    frac = e2.groupby(["sensor", "weather", "slip_deg", "ctrl"], sort=False)["recover_min"].apply(lambda s: float(s.notna().mean())).reset_index(name="frac_pulih")
    g2 = g2.merge(frac, on=["sensor", "weather", "slip_deg", "ctrl"])
    out += ["\n## E2: pemulihan setelah lepas langkah disuntikkan (kedua sumbu, searah, pada sepertiga hari)\n",
            "rmse_post = RMSE setelah kejadian, err_end = galat rata-rata 15 menit terakhir (jendela terjangkau), recover_min = menit sampai galat < 3 deg bertahan 10 menit (rata-rata hanya seed yang pulih), frac_pulih = fraksi seed yang pulih.\n",
            md(g2, ["sensor", "weather", "slip_deg", "ctrl", "rmse_post", "err_end", "recover_min", "frac_pulih"])]
    # E3: statistik angin dan kecepatan angin lepas sinkron
    p = make_p("default")
    rows3 = []
    for wd in WINDS:
        for s in range(10):
            e = Ext(wd, s, p, 21600)
            # kejadian per hari dari pemindaian torsi penuh (azimuth tanpa gravitasi, elevasi pada gravitasi terburuk 0.64 N.m)
            e._scan("azimuth", e.tw["azimuth"])
            e._scan("elevasi", D.AXES["elevasi"].grav + e.tw["elevasi"])
            rows3.append(dict(wind=wd, seed=s, vmax=float(e.v.max()), frac_v_gt_15=float((e.v > 15).mean()), frac_v_gt_20=float((e.v > 20).mean()),
                              ev_az=e.events["azimuth"], ev_el=e.events["elevasi"]))
    d3 = pd.DataFrame(rows3).groupby("wind", sort=False)[["vmax", "frac_v_gt_15", "frac_v_gt_20", "ev_az", "ev_el"]].mean().reset_index()
    out += ["\n## E3: statistik angin sintetis (10 seed, 12 jam siang) dan kejadian melewati torsi tahan\n", md(d3, ["wind", "vmax", "frac_v_gt_15", "frac_v_gt_20", "ev_az", "ev_el"])]
    po = {nm: math.sqrt((a.Th - (a.grav if nm == "elevasi" else 0)) / (0.5 * 1.2 * 1.2 * a.area * a.arm)) for nm, a in D.AXES.items()}
    out += [f"\nKecepatan angin yang melewati torsi tahan (analitik, Cn 1.2, lengan asumsi): elevasi {po['elevasi']:.1f} m/s (dengan gravitasi terburuk), azimuth {po['azimuth']:.1f} m/s.\n"]
    (HERE / "coupled_results_tables.md").write_text("\n".join(out) + "\n", encoding="utf-8")
    print("selesai: coupled_results_tables.md,", len(df), "baris")


if __name__ == "__main__":
    if "--table" in sys.argv:
        show_tables()
    elif "--check" in sys.argv:
        self_check()
    else:
        main()
