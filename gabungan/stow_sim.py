"""Strategi parkir (stow) dan acuan posisi rumah untuk tracker dengan stepper loop terbuka.
Pertanyaan: seberapa sering sumbu hilang (momen angin melewati torsi tahan), berapa lama panel tidak berguna, dan berapa energi harian,
untuk ambang parkir berbeda dan dengan atau tanpa acuan posisi rumah (saklar batas).
Asumsi (semua perlu diukur): momen angin turun menjadi k_flat kali bila panel datar (linear terhadap |q2|/45 deg), tilt 1 deg/s ke posisi parkir,
anemometer rata-rata 4 detik, lepas parkir setelah angin di bawah 0.7 x ambang selama 10 menit, penyetelan ulang ke rumah 5 menit,
sumbu hilang = energi nol sampai pulih (batas bawah konservatif), dampak kehilangan sumbu tidak disimulasikan fisiknya.
"""
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import coupled_sim as C   # noqa: E402

H, D = C.H, C.D
for v in (3, 5, 7, 9, 11):   # sapuan tingkat angin: rata-rata v, turbulensi 30%, hembusan 3 per jam (0.8 sampai 1.5 x rata-rata)
    C.WINDS[f"vm{v}"] = dict(vm=float(v), turb=0.30, gust_h=3.0, gamp=(0.8, 1.5))
VM = (3, 5, 7, 9, 11)
STOWS = (None, 8.0, 10.0, 12.0, 15.0)
KFLAT = (1.0, 0.5, 0.2)
SEEDS = range(20)
V_RECS = (8.0, 10.0, 12.0)   # m/s, ambang angin untuk menyetel ulang ke rumah setelah kehilangan sumbu, SAMA untuk semua strategi (asumsi, disapu)
HOLD_S, HOME_S = 600.0, 300.0


class Stow:
    """Perintah parkir (azimuth ditahan, kemiringan ke 0) berdasarkan angin hasil rata-rata 4 detik, dengan histeresis dan penahanan."""

    def __init__(self, ext, vstow, p, filt_s=4.0, resume_ratio=0.7):
        self.ext, self.vstow, self.p = ext, vstow, p
        self.nf = max(1, int(filt_s / C.DTW))
        self.vr = resume_ratio * vstow if vstow else 0.0
        self.stowed, self.below = False, 0.0

    def __call__(self, k):
        if self.vstow is None:
            return None
        i1 = k * self.ext.m
        v = float(self.ext.v[max(0, i1 - self.nf):i1].mean()) if i1 > 0 else 0.0
        if not self.stowed:
            if v > self.vstow:
                self.stowed, self.below = True, 0.0
        else:
            self.below = self.below + self.p.dt if v < self.vr else 0.0
            if self.below >= HOLD_S:
                self.stowed = False
        return (None, 0.0) if self.stowed else None


def lost_mask(n, ev, vstep, vres, dt, with_ref):
    """Selang panel tidak berguna: dari kejadian pertama sampai akhir hari (tanpa acuan) atau sampai angin reda 10 menit + penyetelan ulang 5 menit."""
    lost = np.zeros(n, bool)
    if not ev:
        return lost
    if not with_ref:
        lost[min(ev):] = True
        return lost
    hs, hm = int(HOLD_S / dt), int(HOME_S / dt)
    below = vstep < vres
    cs = np.concatenate([[0], np.cumsum(below)])
    ok = (cs[hs:] - cs[:-hs]) == hs
    for e in sorted(ev):
        if lost[e]:
            continue
        j = int(np.argmax(ok[e:])) + e if ok[e:].any() else n
        lost[e:min(n, j + hs + hm)] = True
    return lost


def run_job(job):
    vm, vstow, kf, seed, *rest = job
    p = C.make_p("default")
    if rest:
        from dataclasses import replace
        p = replace(p, rate2=rest[0])   # laju kemiringan (deg/s), sensitivitas
    d = H.data("2026-10-04", "cerah", 0, p)
    ref = H.simulate("astro", d, p, 0, ext=C.Ext(None, 0, p, d["n"], k_flat=kf))["kwh"]
    ext = C.Ext(f"vm{vm}", seed, p, d["n"], k_flat=kf)
    r = H.simulate("astro", d, p, 0, ext=ext, ov=Stow(ext, vstow, p))
    pw = np.array(r["pw"])
    vstep = ext.v.reshape(d["n"], ext.m).mean(axis=1)
    ev = sorted(set(ext.ev_steps["azimuth"] + ext.ev_steps["elevasi"]))
    rows = []
    for with_ref, vrec in [(False, 0.0)] + [(True, v) for v in V_RECS]:
        lm = lost_mask(d["n"], ev, vstep, vrec, p.dt, with_ref)
        rows.append(dict(vm=vm, k_flat=kf, stow=vstow or 0.0, acuan=with_ref, vrec=vrec, seed=seed, n_event=len(ev), ada_event=int(len(ev) > 0),
                         stow_frac=r["f_stow"], lost_h=float(lm.sum() * p.dt / 3600), energi_pct=float(pw[~lm].sum() * p.dt / 3.6e6 / ref * 100),
                         vmax=float(ext.v.max())))
    return rows


def self_check():
    p = C.make_p("default")
    d = H.data("2026-10-04", "cerah", 0, p)
    # angin lemah tanpa parkir: tidak ada kejadian, energi praktis sama dengan acuan (defleksi kecil saja)
    rows = run_job((3, None, 1.0, 0))
    assert rows[0]["n_event"] == 0 and abs(rows[0]["energi_pct"] - 100.0) < 0.05, rows[0]
    # angin kuat, parkir dini dan momen datar rendah: kejadian tidak lebih banyak daripada tanpa parkir dan parkir benar-benar terjadi
    a, b = run_job((11, None, 0.2, 0))[0], run_job((11, 8.0, 0.2, 0))[0]
    assert b["stow_frac"] > 0 and b["n_event"] <= a["n_event"], (a, b)
    # penyetelan ulang tidak lebih buruk daripada tanpa acuan
    ra = run_job((11, None, 1.0, 0))
    assert all(r["energi_pct"] >= ra[0]["energi_pct"] - 1e-9 for r in ra[1:]), ra
    # ambang pulih lebih tinggi tidak membuat panel lebih lama hilang
    rb = run_job((7, 15.0, 0.5, 0))
    assert rb[1]["lost_h"] >= rb[2]["lost_h"] - 1e-9 >= rb[3]["lost_h"] - 2e-9, rb
    print("self-check stow: lolos")


def md(df, cols):
    s = "| " + " | ".join(cols) + " |\n|" + "---|" * len(cols) + "\n"
    for _, r in df.iterrows():
        s += "| " + " | ".join(f"{r[c]:.3g}" if isinstance(r[c], float) else str(r[c]) for c in cols) + " |\n"
    return s


def main():
    self_check()
    jobs = [(vm, vs, kf, s) for vm in VM for kf in KFLAT for vs in STOWS for s in SEEDS]
    with ProcessPoolExecutor(max_workers=14) as ex:
        rows = [r for rr in ex.map(run_job, jobs, chunksize=6) for r in rr]
    df = pd.DataFrame(rows)
    df.to_csv(HERE / "stow_runs.csv", index=False)
    g = df.groupby(["k_flat", "vm", "stow", "acuan", "vrec"], sort=False).agg(p_event=("ada_event", "mean"), n_event=("n_event", "mean"), lost_h=("lost_h", "mean"),
                                                                        energi_pct=("energi_pct", "mean"), stow_frac=("stow_frac", "mean"), vmax=("vmax", "mean")).reset_index()
    out = ["# Tabel hasil strategi parkir dan acuan rumah (dibuat otomatis oleh stow_sim.py)\n",
           "2026-10-04, langit cerah, astronomi saja (sensor tidak berpengaruh pada energi), 20 seed angin per sel. stow = ambang parkir (m/s, 0 = tanpa parkir). acuan = ada saklar posisi rumah. "
           "p_event = fraksi hari dengan minimal satu kejadian melewati torsi tahan, n_event = rata-rata kejadian per hari, lost_h = jam panel tidak berguna, "
           "energi_pct = energi harian terhadap pelacakan ideal tanpa angin pada k_flat yang sama (sumbu hilang dianggap energi nol), stow_frac = fraksi waktu terparkir.\n"]
    for kf in KFLAT:
        out.append(f"\n## k_flat = {kf} (momen angin saat panel datar = {kf} x momen saat miring 45 deg)\n")
        out.append(md(g[g.k_flat == kf], ["vm", "stow", "acuan", "vrec", "p_event", "n_event", "lost_h", "energi_pct", "stow_frac", "vmax"]))
    (HERE / "stow_results_tables.md").write_text("\n".join(out) + "\n", encoding="utf-8")
    print("selesai: stow_results_tables.md,", len(df), "baris")
    rate_sweep()


def rate_sweep():
    """Sensitivitas laju kemiringan (1, 3, 10 deg/s) pada k_flat 0.5: apakah parkir bisa bereaksi terhadap hembusan 3 sampai 8 detik."""
    jobs = [(vm, vs, 0.5, s, r2) for vm in (7, 9) for vs in (10.0, 12.0, 15.0) for r2 in (1.0, 3.0, 10.0) for s in SEEDS]
    with ProcessPoolExecutor(max_workers=14) as ex:
        res = [r for j, rr in zip(jobs, ex.map(run_job, jobs, chunksize=6)) for r in [dict(rr[0], rate2=j[4])]]   # hanya baris tanpa acuan
    g = pd.DataFrame(res).groupby(["vm", "stow", "rate2"], sort=False).agg(p_event=("ada_event", "mean"), n_event=("n_event", "mean"),
                                                                               energi_pct=("energi_pct", "mean"), stow_frac=("stow_frac", "mean")).reset_index()
    f = HERE / "stow_results_tables.md"
    txt = f.read_text(encoding="utf-8").split("\n## Sensitivitas laju kemiringan")[0]
    txt += "\n## Sensitivitas laju kemiringan (k_flat 0.5, tanpa acuan, 20 seed)\n\nrate2 = laju tilt ke posisi parkir (deg/s).\n\n" + md(g, ["vm", "stow", "rate2", "p_event", "n_event", "energi_pct", "stow_frac"])
    f.write_text(txt + "\n", encoding="utf-8")
    print("selesai: sensitivitas laju ditambahkan")


if __name__ == "__main__":
    self_check() if "--check" in sys.argv else (rate_sweep() if "--rate" in sys.argv else main())
