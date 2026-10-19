"""Validasi akurasi tool posisi matahari di titik panel PENS, seluruh tahun 2026, langkah 10 menit.
Acuan: pvlib SPA (nrel_numpy). Pembanding: pvlib ephemeris, Pysolar, SolTrack (C), astropy (independen, tanpa refraksi).
Jalankan (di WSL, venv solar): python validate_sun.py   -> validate_sun_results.md + validate_sun.png
"""
import os, subprocess, sys, time
import numpy as np, pandas as pd, pvlib
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from sun import LAT, LON, TZ  # titik panel yang sama dengan sun.py
ALT, P_PA, T_C = 5.0, 101325.0, 12.0  # ketinggian (asumsi), tekanan, suhu (sama untuk semua tool)
PYSOLAR_STEP = 1  # 1 = semua titik; naikkan (mis. 6 = per jam) bila Pysolar terlalu lambat
SOLTRACK_DIR = "/tmp/SolTrack"

t_loc = pd.date_range("2026-01-01", "2026-12-31 23:50", freq="10min", tz=TZ)
t_utc = t_loc.tz_convert("UTC")
N = len(t_utc)
loc = pvlib.location.Location(LAT, LON, TZ, altitude=ALT)


def vec(az, el):
    a, e = np.radians(az), np.radians(el)
    return np.stack([np.cos(e) * np.sin(a), np.cos(e) * np.cos(a), np.sin(e)], -1)


def sep(az1, el1, az2, el2):
    """Sudut pisah (deg) antar dua arah; arctan2(|cross|, dot) stabil untuk sudut kecil."""
    u, v = vec(az1, el1), vec(az2, el2)
    return np.degrees(np.arctan2(np.linalg.norm(np.cross(u, v), axis=-1), (u * v).sum(-1)))


def timed(f):
    t0 = time.perf_counter(); r = f(); return r, time.perf_counter() - t0


# --- tiap tool mengembalikan dict(az, geo, app) (app=None bila tidak ada), panjang N (Pysolar bisa disubsampel)
def run_pvlib(method):
    s = loc.get_solarposition(t_loc, pressure=P_PA, temperature=T_C, method=method)
    return dict(az=s.azimuth.values, geo=s.elevation.values, app=s.apparent_elevation.values)


def run_pysolar():
    from pysolar import solar as ps
    idx = range(0, N, PYSOLAR_STEP)
    az, geo = np.full(N, np.nan), np.full(N, np.nan)
    for i in idx:
        d, h = ps.get_topocentric_position(LAT, LON, t_utc[i].to_pydatetime(), ALT)
        geo[i] = ps.get_topocentric_elevation_angle(LAT, d, h)
        az[i] = ps.get_topocentric_azimuth_angle(h, LAT, d)
    ok = ~np.isnan(geo)
    app = geo.copy(); app[ok] += [ps.get_refraction_correction(P_PA, T_C + 273.15, g) for g in geo[ok]]
    return dict(az=az, geo=geo, app=app)


def run_soltrack():
    exe = os.path.join(HERE, "soltrack_driver")
    if not os.path.exists(SOLTRACK_DIR):
        subprocess.run(["git", "clone", "--depth", "1", "https://github.com/MarcvdSluys/SolTrack", SOLTRACK_DIR], check=True)
    if not os.path.exists(exe):
        subprocess.run(["gcc", "-O2", f"-I{SOLTRACK_DIR}", os.path.join(HERE, "soltrack_driver.c"),
                        f"{SOLTRACK_DIR}/SolTrack.c", "-lm", "-o", exe], check=True)
    inp = "\n".join(f"{t.year} {t.month} {t.day} {t.hour} {t.minute} {t.second}" for t in t_utc)
    out = subprocess.run([exe, str(LAT), str(LON)], input=inp, capture_output=True, text=True, check=True).stdout
    a = np.array(out.split(), float).reshape(-1, 3)
    return dict(az=a[:, 0], geo=a[:, 1], app=a[:, 2])


def run_astropy():
    from astropy.time import Time
    from astropy.coordinates import get_sun, AltAz, EarthLocation
    from astropy.utils import iers
    iers.conf.auto_download = False; iers.conf.iers_degraded_accuracy = "ignore"  # tabel IERS bawaan, tanpa unduh
    t = Time(t_utc.tz_localize(None).values, scale="utc")
    aa = get_sun(t).transform_to(AltAz(obstime=t, location=EarthLocation.from_geodetic(LON, LAT, ALT)))  # pressure=0: tanpa refraksi
    return dict(az=aa.az.deg, geo=aa.alt.deg, app=None)


TOOLS = {"pvlib SPA (acuan)": lambda: run_pvlib("nrel_numpy"), "pvlib ephemeris": lambda: run_pvlib("ephemeris"),
         "Pysolar": run_pysolar, "SolTrack (C)": run_soltrack, "astropy (independen)": run_astropy}
res, bench = {}, {}
for name, f in TOOLS.items():
    res[name], bench[name] = timed(f)
    print(f"{name}: {bench[name]:.2f} s", flush=True)

ref = res["pvlib SPA (acuan)"]
up = ref["geo"] > 0  # hanya matahari di atas horizon (elevasi geometrik acuan)
wrap = lambda d: (d + 180) % 360 - 180


def stats(x):
    x = x[~np.isnan(x)]
    return dict(n=len(x), max=x.max(), mean=x.mean(), p99=np.percentile(x, 99), rmse=np.sqrt((x ** 2).mean()))


def compare(name, key):
    r, o = res[name], ref
    m = up & ~np.isnan(r[key]) if r[key] is not None else None
    if m is None: return None
    s = sep(r["az"][m], r[key][m], o["az"][m], o[key][m])
    return dict(sep=s, **{"sep_" + k: v for k, v in stats(s).items()},
                daz=np.abs(wrap(r["az"][m] - o["az"][m])).max(), del_=np.abs(r[key][m] - o[key][m]).max(), m=m)


cmp = {k: {n: compare(n, k) for n in res if n != "pvlib SPA (acuan)"} for k in ("geo", "app")}

# --- self-check: fungsi sudut pisah + batas kewarasan hasil
assert abs(sep(0, 0, 90, 0) - 90) < 1e-9 and abs(sep(10, 89.99, 190, 89.99) - 0.02) < 1e-6
assert cmp["geo"]["pvlib ephemeris"]["sep_max"] < 0.05 and cmp["geo"]["SolTrack (C)"]["sep_max"] < 0.05
assert cmp["geo"]["astropy (independen)"]["sep_max"] < 0.05, "astropy menyimpang dari SPA > 0.05 deg"


def table(key):
    rows = ["| Tool | N titik | maks (deg) | rata-rata | P99 | RMSE | maks &#124;daz&#124; | maks &#124;del&#124; |", "|---|---|---|---|---|---|---|---|"]
    for n, c in cmp[key].items():
        if c: rows.append(f"| {n} | {c['sep_n']} | {c['sep_max']:.5f} | {c['sep_mean']:.5f} | {c['sep_p99']:.5f} | {c['sep_rmse']:.5f} | {c['daz']:.5f} | {c['del_']:.5f} |")
    return "\n".join(rows)


try:
    cpu = next(l.split(":")[1].strip() for l in open("/proc/cpuinfo") if "model name" in l)
except Exception:
    cpu = "tidak terbaca"
b = "\n".join(f"| {n} | {t:.2f} | {t / N * 1e6:.1f} |" for n, t in bench.items())
ps_note = "semua titik" if PYSOLAR_STEP == 1 else f"disubsampel tiap {PYSOLAR_STEP} titik"
md = f"""# Validasi akurasi tool posisi matahari (langkah 1 peta kerja)

Dibuat otomatis oleh `validate_sun.py`. Titik panel: lintang {LAT}, bujur {LON}, zona {TZ}, ketinggian {ALT:g} m (asumsi).
Rentang: seluruh tahun 2026, langkah 10 menit ({N} titik waktu), hanya titik dengan elevasi geometrik acuan > 0
(n = {int(up.sum())}). Pysolar: {ps_note}. Kondisi atmosfer seragam: {P_PA:.0f} Pa, {T_C:g} C.

Acuan: pvlib SPA (`nrel_numpy`). Ukuran utama: **sudut pisah** antara dua vektor arah matahari (deg).
Kolom daz/del: selisih azimuth (terbungkus ke +-180) dan elevasi terpisah, hanya untuk perbandingan dengan hasil awal satu hari.
Kesalahan azimuth meledak di dekat zenith secara artifisial, sehingga maks daz tidak mencerminkan kesalahan arah sebenarnya.

## Hasil: elevasi geometrik (tanpa refraksi)

{table("geo")}

## Hasil: elevasi tampak (dengan refraksi, astropy tidak termasuk karena dihitung tanpa refraksi)

{table("app")}

SolTrack: azimut hanya tersedia satu (tanpa koreksi refraksi), dipakai di kedua tabel.
Pysolar: refraksi dihitung dengan `get_refraction_correction` dari elevasi geometriknya sendiri.

## Pemeriksaan independen

astropy `get_sun` + `AltAz` (ERFA/IAU, tanpa refraksi, tabel IERS bawaan tanpa unduhan) tidak berbagi kode dengan SPA, jadi
baris astropy pada tabel geometrik adalah pemeriksaan independen terhadap acuan. Keterbatasan: astropy dan SPA sama-sama
efemeris analitik tingkat detik busur, bukan data observasi; tidak ada pembanding berbasis JPL DE yang dijalankan.

## Benchmark waktu hitung (indikasi, satu tahun = {N} titik, semua titik termasuk malam)

Mesin: {cpu} (WSL Ubuntu, satu proses, tanpa paralelisasi). SolTrack diukur dengan proses C terpisah
(termasuk start proses dan pembacaan teks); Pysolar berupa loop Python per titik; pvlib dan astropy vektor.

| Tool | Total (s) | per titik (us) |
|---|---|---|
{b}

## Catatan mikrokontroler dan lisensi (satu kalimat per tool)

- pvlib: pustaka Python/NumPy, tidak cocok langsung untuk mikrokontroler; lisensi BSD-3.
- Pysolar: pustaka Python murni, tidak cocok langsung untuk mikrokontroler; lisensi GPL.
- SolTrack: ditulis dalam C ringan tanpa dependensi selain `libm` (driver di sini hanya dikompilasi dengan gcc di PC); README proyek
  menyebut port Arduino, tetapi port tersebut tidak dijalankan atau diverifikasi di sini; lisensi LGPL-3.
- astropy: pustaka Python besar, hanya dipakai sebagai pemeriksaan di PC; lisensi BSD-3 (tidak diverifikasi ulang di sini).

## Keterbatasan

- Acuan SPA bukan kebenaran mutlak; hanya astropy yang independen, dan ia hanya diuji tanpa refraksi.
- Selisih tampak tergantung model refraksi tiap tool; seluruhnya memakai tekanan dan suhu yang sama.
- Delta T / UT1 mengikuti default tiap tool (pvlib: perkiraan, astropy: tabel IERS bawaan, SolTrack dan Pysolar: model internal).
- Benchmark hanya indikasi (satu mesin, satu kali jalan, tanpa pengulangan).
"""
open(os.path.join(HERE, "validate_sun_results.md"), "w", encoding="utf-8").write(md)

# --- gambar: histogram sudut pisah (geometrik) + maksimum harian
fig, ax = plt.subplots(1, 2, figsize=(11, 4))
cols = dict(zip([n for n in cmp["geo"]], ["#1f77b4", "#d95f02", "#2ca02c", "#7b3294"]))
bins = np.logspace(-6, -1, 60)
days = t_loc.dayofyear.values
for n, c in cmp["geo"].items():
    ax[0].hist(np.clip(c["sep"], 1e-6, None), bins=bins, histtype="step", color=cols[n], label=n)
    dm = pd.Series(c["sep"]).groupby(days[c["m"]]).max()
    ax[1].plot(dm.index, dm.values, color=cols[n], lw=1, label=n)
ax[0].set(xscale="log", xlabel="sudut pisah terhadap pvlib SPA (deg)", ylabel="jumlah titik", title="Distribusi kesalahan, 2026")
ax[1].set(yscale="log", xlabel="hari ke- (2026)", ylabel="maks harian (deg)", title="Kesalahan maksimum harian")
ax[0].legend(fontsize=8); [a.grid(alpha=.3) for a in ax]
fig.tight_layout(); fig.savefig(os.path.join(HERE, "validate_sun.png"), dpi=130)
print("selesai: assert lulus, md dan png ditulis")
