"""Uji ketahanan Monte Carlo: kontroler astro/ldr/saputra/bias terhadap ketidakpastian parameter sensor (asumsi).
Sampel Sobol (scrambled, seed tetap) pada rentang parameter sensor; tiap sampel dijalankan pada 4 Okt 2026,
cerah (seed 0) dan sebagian (seed 1, 2), salah pasang (0,0), (3,2), (6,4) deg. Hasil: robust_runs.csv, robust_results.md, PNG.
Jalankan: python robust_mc.py   (uji mandiri: python robust_mc.py --check)
"""
import math
import random
import sys
import time
from dataclasses import replace
from multiprocessing import get_context
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import qmc, spearmanr

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "kontrol_hibrid"))
import hybrid_sim as H  # noqa: E402

N, QSEED, DAY = 128, 20261004, "2026-10-04"
BIASES = [(0.0, 0.0), (3.0, 2.0), (6.0, 4.0)]
WEATHER = [("cerah", 0), ("sebagian", 1), ("sebagian", 2)]
LO = dict(r=math.log(0.5), mism=0.0, L0=math.log(5000), gam=0.5, vn=math.log(0.001))
HI = dict(r=math.log(3), mism=0.10, L0=math.log(80000), gam=0.9, vn=math.log(0.005))
KEYS = list(LO)
LOGK = {"r", "L0", "vn"}
SAMP = []


def samples():
    u = qmc.Sobol(d=len(KEYS), scramble=True, seed=QSEED).random(N)
    out = []
    for row in u:
        s = {k: LO[k] + x * (HI[k] - LO[k]) for k, x in zip(KEYS, row)}
        out.append({k: (math.exp(v) if k in LOGK else v) for k, v in s.items()})
    return out


def floor_deg(s):
    """Lantai galat teoretis (deg) dari ketidakcocokan dan derau, memakai kemiringan sinyal e per deg (kasus cerah)."""
    p = replace(H.P(), **{**s, "vn": 0.0, "mism": 0.0})
    n, xp, yp = H.basis(30.0, 20.0)

    def e_at(deg):
        v = [n[i] + math.radians(deg) * xp[i] for i in range(3)]
        m = math.sqrt(H.dot(v, v))
        ex, _, vm = H.read(n, xp, yp, tuple(a / m for a in v), 800.0, 100.0, [1] * 4, p, random.Random(0))
        return ex, vm

    (e1, _), (e0, v0) = e_at(1.0), e_at(0.0)
    slope = abs(e1 - e0)                                   # e per deg
    off, noi = s["mism"] / math.sqrt(2) / slope, 0.7071 * s["vn"] / max(v0, 1e-9) / slope
    return math.hypot(off, noi), slope


def work(i):
    s = SAMP[i]
    fl, slope = floor_deg(s)
    rows = []
    for bias in BIASES:
        p = replace(H.P(), bias=bias, **s)
        for kind, seed in WEATHER:
            d = H.data(DAY, kind, seed, p)
            for c in H.CTRLS:
                r = H.simulate(c, d, p, seed)
                rows.append(dict(sample=i, **s, floor=fl, slope=slope, mp=f"{bias[0]:g}/{bias[1]:g}", weather=kind, seed=seed, ctrl=c,
                                 rmse=r["rmse"], p95=r["p95"], acts=r["acts"], gain=r["gain"]))
    return rows


def q(x, a):
    return np.percentile(x, a)


def md(df):
    cols = list(df.columns)
    return "| " + " | ".join(cols) + " |\n|" + "---|" * len(cols) + "\n" + "".join(
        "| " + " | ".join(f"{v:.3g}" if isinstance(v, float) else str(v) for v in r) + " |\n" for r in df.itertuples(index=False))


def best_thr(x, win):
    """Ambang x (menang bila x <= ambang) dengan akurasi klasifikasi tertinggi."""
    cand = np.sort(x.values)
    acc = [((x <= c) == win).mean() for c in cand]
    k = int(np.argmax(acc))
    return float(cand[k]), float(acc[k])


def report(df, t_run):
    g = df.groupby(["sample", "weather", "mp", "ctrl"], as_index=False).agg(rmse=("rmse", "mean"), floor=("floor", "first"), **{k: (k, "first") for k in KEYS})
    out = ["# Uji ketahanan Monte Carlo terhadap parameter sensor (dibuat otomatis oleh robust_mc.py)\n",
           f"{N} sampel parameter sensor (Sobol scrambled, seed {QSEED}), hari {DAY}, cerah (seed 0) dan sebagian (rata-rata seed 1 dan 2), salah pasang 0/0, 3/2, 6/4 deg. "
           f"Total {len(df)} simulasi, waktu jalan {t_run:.0f} s (16 proses). Rentang: r log-seragam [0.5, 3]; mism seragam [0, 0.10]; L0 log-seragam [5000, 80000]; gam seragam [0.5, 0.9]; vn log-seragam [0.001, 0.005] V. "
           "Semua rentang adalah asumsi (sensor belum diukur). RMSE = galat pointing (deg), zenith <= 45 deg. "
           "Lantai galat = hypot(mism/sqrt2, 0.707 vn/V) / kemiringan e per deg, diukur numerik pada kasus cerah.\n"]
    t = g.groupby(["weather", "mp", "ctrl"]).rmse.agg(median="median", P10=lambda x: q(x, 10), P90=lambda x: q(x, 90)).reset_index()
    out += ["\n## (a) Sebaran RMSE (deg) antar sampel sensor\n", md(t)]
    w = g.pivot_table(index=["sample", "weather", "mp"], columns="ctrl", values="rmse").join(g.groupby(["sample", "weather", "mp"]).floor.first()).reset_index()
    w["win"] = w["bias"] < w["astro"]
    w["ldrwin"] = w["ldr"] < w["astro"]
    w["sapwin"] = w["saputra"] < w["astro"]
    fr = w.groupby(["weather", "mp"]).agg(frac_bias_lt_astro=("win", "mean"), frac_saputra_lt_astro=("sapwin", "mean"), frac_ldr_lt_astro=("ldrwin", "mean"),
                                          astro_rmse=("astro", "median"), floor_median=("floor", "median")).reset_index()
    out += ["\n## (b) Fraksi sampel sensor di mana RMSE kontroler < RMSE astro (LDR membantu)\n", md(fr)]
    rows = []
    for (wk, mp), d in w[w.mp != "0/0"].groupby(["weather", "mp"]):
        c, a = best_thr(d.floor, d.win)
        rows.append(dict(weather=wk, mp=mp, floor_thr_deg=c, akurasi=a, n_win=int(d.win.sum()), n=len(d)))
    thr = pd.DataFrame(rows)
    out += ["\n### Titik impas lantai galat per level (ambang lantai galat dengan akurasi pemisahan menang/kalah tertinggi)\n", md(thr)]
    d = w[w.mp != "0/0"].assign(ratio=lambda x: x.floor / x.astro)
    out += ["\n## (c) Aturan desain turunan dari data\n"]
    for wk, dd in d.groupby("weather"):
        c, a = best_thr(dd.ratio, dd.win)
        out.append(f"- Cuaca {wk}: hibrid bias mengalahkan astro bila (lantai galat sensor) / (RMSE astro akibat salah pasang) <= {c:.2f} "
                   f"(akurasi klasifikasi {a:.0%} pada level 3/2 dan 6/4 digabung; LDR menang di {dd.win.mean():.0%} sampel).")
    for wk in ("cerah", "sebagian"):
        s = thr[thr.weather == wk].set_index("mp")
        ar = fr[fr.weather == wk].set_index("mp").astro_rmse
        out.append(f"- Dalam derajat ({wk}): ambang lantai galat {s.floor_thr_deg['3/2']:.2f} deg untuk 3/2 (RMSE astro {ar['3/2']:.2f} deg), "
                   f"{s.floor_thr_deg['6/4']:.2f} deg untuk 6/4 (RMSE astro {ar['6/4']:.2f} deg).")
    f0 = fr[fr.mp == "0/0"].set_index("weather").frac_bias_lt_astro
    out.append(f"- Tanpa salah pasang (0/0): hibrid bias mengalahkan astro hanya di {f0['cerah']:.0%} (cerah) dan {f0['sebagian']:.0%} (sebagian) sampel, jadi LDR tidak menambah apa pun di sana.")
    rows = []
    for (wk, mp), dd in g[g.ctrl == "bias"].groupby(["weather", "mp"]):
        rows.append(dict(weather=wk, mp=mp, **{k: spearmanr(dd[k], dd.rmse)[0] for k in KEYS + ["floor"]}))
    sp = pd.DataFrame(rows)
    rk = sp[sp.mp != "0/0"][KEYS + ["floor"]].abs().mean().sort_values(ascending=False)
    out += ["\n## (d) Korelasi peringkat Spearman antara parameter sensor dan RMSE hibrid bias\n", md(sp),
            "\nPeringkat pengaruh (rata-rata |rho| pada salah pasang 3/2 dan 6/4, cerah dan sebagian): " + ", ".join(f"{k} ({v:.2f})" for k, v in rk.items()) + ".\n"]
    out += ["\n## Keterbatasan\n",
            "- Semua rentang parameter adalah asumsi; kesimpulan hanya berlaku di dalam rentang itu, bukan prediksi untuk sensor nyata. Ukur sensor, lalu bandingkan dengan peta ini.",
            "- Parameter disampel bebas satu sama lain (tanpa korelasi fisik). Jumlah sampel terbatas, jadi ambang adalah taksiran, bukan batas tegas.",
            f"- Satu hari ({DAY}), satu seed cerah, dua seed sebagian (awan sintetis). Realisasi ketidakcocokan antar LDR berasal dari seed simulate() (seed+77), sehingga pola relatif empat LDR sama antar sampel dan hanya skalanya (mism) yang berubah.",
            "- Lantai galat teoretis memakai kemiringan sinyal pada satu pose dan irradiance (DNI 800, DHI 100 W/m2); lantai nyata berubah menurut waktu dan pose. Parameter kontroler (kp, dead, kcthr) tidak ikut diubah.",
            "- Model kuasi-statis, tanpa dinamika motor dan angin. Salah pasang hanya tiga level, tafsiran di antaranya adalah interpolasi.\n"]
    txt = "\n".join(out) + "\n"
    assert "—" not in txt
    (HERE / "robust_results.md").write_text(txt, encoding="utf-8")
    plot(w, fr)
    print(txt)


def plot(w, fr):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 3, figsize=(13, 7), sharex=True, sharey=True)
    for j, mp in enumerate(["0/0", "3/2", "6/4"]):
        for i, wk in enumerate(["cerah", "sebagian"]):
            d = w[(w.mp == mp) & (w.weather == wk)]
            a = ax[i, j]
            a.scatter(d.floor, d.bias, s=14, c="tab:blue", label="bias")
            a.scatter(d.floor, d.ldr, s=10, c="tab:orange", alpha=0.5, label="ldr")
            a.axhline(d.astro.median(), c="k", ls="--", label="astro")
            a.set_xscale("log"); a.set_yscale("log")
            a.set_title(f"{wk}, salah pasang {mp} (bias menang {d.win.mean():.0%})", fontsize=9)
            if j == 0:
                a.set_ylabel("RMSE (deg)")
            if i == 1:
                a.set_xlabel("lantai galat sensor (deg)")
    ax[0, 0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(HERE / "robust_rmse_vs_floor.png", dpi=130)


def main():
    global SAMP
    SAMP = samples()
    for kind, seed in WEATHER:
        H.data(DAY, kind, seed, H.P())      # panaskan cache sebelum fork
    t0 = time.time()
    with get_context("fork").Pool(16) as pool:
        rows = [r for rs in pool.map(work, range(N), chunksize=2) for r in rs]
    t_run = time.time() - t0
    df = pd.DataFrame(rows)
    df.to_csv(HERE / "robust_runs.csv", index=False)
    report(df, t_run)


def self_check():
    """Deterministik; astro tidak bergantung sensor; lantai galat naik bila mism naik."""
    global SAMP
    SAMP = samples()
    assert len(SAMP) == N and all(0.5 <= s["r"] <= 3 and 5000 <= s["L0"] <= 80000 and 0 <= s["mism"] <= 0.1 for s in SAMP)
    a, b, c = work(0), work(0), work(1)
    assert a == b
    ref = {(r["mp"], r["weather"], r["seed"]): r["rmse"] for r in a if r["ctrl"] == "astro"}
    assert all(abs(r["rmse"] - ref[(r["mp"], r["weather"], r["seed"])]) < 1e-9 for r in c if r["ctrl"] == "astro")
    s = SAMP[0]
    assert floor_deg(dict(s, mism=0.1))[0] > floor_deg(dict(s, mism=0.0))[0]
    print("self-check lolos")


if __name__ == "__main__":
    self_check() if "--check" in sys.argv else main()
