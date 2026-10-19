"""Analisis zenith (keyhole) tracker 2 sumbu di PENS, tahun 2026. Jalankan: python zenith_analysis.py
Keluaran: 3 PNG dan zenith_analysis_results.md di folder ini. Zenith = 90 - elevasi tampak (sama dengan tracker.py)."""
import sys, pathlib
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import sun, tracker
from sun import loc

trapz = getattr(np, "trapezoid", None) or np.trapz
wrap = lambda a: (a + 180) % 360 - 180
THR = (0.1, 0.5, 1.0)                       # ambang laju azimuth (deg/s)
LIMS = (0.02, 0.05, 0.1, 0.5, 1, 2, 5, 10)  # batas laju azimuth sendi (deg/s)
INF = 1e9                                   # "tanpa batas laju"


def pos(day0, off):
    """az, el tampak (deg) pada detik-ke-n sejak tengah malam lokal."""
    sp = loc.get_solarposition(day0 + pd.to_timedelta(off, "s"))
    return sp.azimuth.values, sp.apparent_elevation.values


def rates(off, az, el):
    """Laju azimuth (deg/s) di titik tengah tiap interval, dengan penanda siang dan lebar interval."""
    dt = np.diff(off)
    return np.abs(wrap(np.diff(az))) / dt, (off[1:] + off[:-1]) / 2, el[:-1] > 0, dt


def scan(day0):
    """Satu hari: grid 10 s seharian, 1 s di +-20 menit kulminasi, 0.1 s di +-60 s kulminasi."""
    oc = np.arange(0, 86400, 10.)
    az, el = pos(day0, oc)
    tc = oc[np.argmin(np.where(el > 0, 90 - el, 999))]
    om = np.arange(tc - 1200, tc + 1200, 1.); of = np.arange(tc - 60, tc + 60, 0.1)
    azm, elm = pos(day0, om); azf, elf = pos(day0, of)
    zf = 90 - elf; i = np.argmin(zf)
    vec = tracker.sun_vec(azf, elf)       # laju sudut matahari di kulminasi (deg/s), selang 1 s
    v = np.degrees(np.arccos(np.clip(vec[:, i - 5] @ vec[:, i + 5], -1, 1))) / 1.0
    sels = (lambda m: abs(m - tc) > 1200, lambda m: (abs(m - tc) > 60) & (abs(m - tc) <= 1200), lambda m: abs(m - tc) <= 60)
    R, D = [], []
    for (r, m, u, d), sel in zip((rates(oc, az, el), rates(om, azm, elm), rates(of, azf, elf)), sels):
        s = sel(m) & u; R.append(r[s]); D.append(d[s])
    R, D = np.concatenate(R), np.concatenate(D)
    return dict(zeta=zf[i], t=of[i] / 3600, v=v, rmax=R.max(), tc=tc, dur=[D[R > th].sum() for th in THR])


def simulate(off, az, el, dni, q2max, lim):
    """Tracker dengan laju azimuth sendi dibatasi `lim` (deg/s); q2 optimal dalam [0, q2max] dan cukup cepat.
    Mengembalikan galat pointing (deg), energi DNI*cos (kWh/m2), serta q1, q2 (rad) untuk cek."""
    z, a = np.radians(90 - el), np.radians(az)
    q1 = a[0]; n = len(off)
    Q1, Q2, C = np.empty(n), np.empty(n), np.empty(n)
    for k in range(n):
        if k:
            s = np.radians(lim) * (off[k] - off[k - 1])
            q1 += np.clip((a[k] - q1 + np.pi) % (2 * np.pi) - np.pi, -s, s)
        d = (a[k] - q1 + np.pi) % (2 * np.pi) - np.pi
        q2 = np.clip(np.arctan2(np.sin(z[k]) * np.cos(d), np.cos(z[k])), 0, q2max)
        Q1[k], Q2[k] = q1, q2
        C[k] = np.sin(q2) * np.sin(z[k]) * np.cos(d) + np.cos(q2) * np.cos(z[k])
    err = np.degrees(np.arccos(np.clip(C, -1, 1)))
    e = trapz(dni * np.maximum(C, 0) * (el > 0), off) / 3600 / 1000
    return err, e, Q1, Q2


def keyhole_day(date, tc, q2maxs=(45, 90)):
    """Simulasi batas laju untuk satu hari; grid 1 s seharian dan 0.1 s di +-90 s kulminasi. DNI langit cerah (okta 0)."""
    day0 = pd.Timestamp(date, tz=sun.TZ)
    s = np.arange(0, 86400, 1.)
    off = np.sort(np.concatenate([s[abs(s - tc) > 90], np.arange(tc - 90, tc + 90, 0.1)]))
    az, el = pos(day0, off)
    d1 = sun.sun_day(date, "1s", 0).dni.values
    dni = np.interp(off, np.arange(len(d1)), d1)
    out = {}
    for qm in q2maxs:
        reach = (el > 0) & (90 - el <= qm)
        _, e_ideal, _, _ = simulate(off, az, el, dni, np.radians(qm), INF)
        e_perf = trapz(dni * (el > 0), off) / 3600 / 1000
        for lim in LIMS:
            err, e, Q1, Q2 = simulate(off, az, el, dni, np.radians(qm), lim)
            out[qm, lim] = dict(err_reach=err[reach].max() if reach.any() else 0, loss_kh=100 * (1 - e / e_ideal),
                                loss_perf=100 * (1 - e / e_perf), e_ideal=e_ideal, e_perf=e_perf)
            if lim == 1:  # cek: galat dari fk dan sun_vec tracker.py sama dengan galat simulasi
                k = np.argmax(el); n = tracker.fk(Q1[k] - tracker.HEAD, Q2[k]); sv = tracker.sun_vec(az[k], el[k])
                assert abs(np.degrees(np.arccos(np.clip(n @ sv, -1, 1))) - err[k]) < 1e-6
    return out


def reach_frac(date, qms=(45, 60, 90)):
    d = sun.sun_day(date, "1min", 0); d = d[d.el > 0]
    return [(100 * (d.zen <= q).mean(), 100 * d.dni[d.zen <= q].sum() / d.dni.sum()) for q in qms]


def table(head, rows):
    return "\n".join(["| " + " | ".join(head) + " |", "|" + "---|" * len(head)] + ["| " + " | ".join(map(str, r)) + " |" for r in rows])


def spans(dates):
    """Kelompokkan tanggal berurutan jadi rentang teks."""
    if len(dates) == 0: return "tidak ada"
    out, s, p = [], dates[0], dates[0]
    for d in list(dates[1:]) + [None]:
        if d is None or (d - p).days > 1:
            out.append(f"{s:%d %b}" + (f" sampai {p:%d %b}" if p != s else "")); s = d
        p = d
    return "; ".join(out)


def hhmm(h): return f"{int(h)}:{int(h % 1 * 60):02d}"


def main():
    days = pd.date_range("2026-01-01", "2026-12-31")
    res = pd.DataFrame([scan(pd.Timestamp(d, tz=sun.TZ)) for d in days], index=days)
    res["analitik"] = res.v / np.radians(res.zeta)          # deg/s, v/zeta (zeta dalam rad)
    res["rasio"] = res.rmax / res.analitik
    w = res.zeta.idxmin(); o = pd.Timestamp("2026-10-04")
    assert 2 < res.zeta[o] < 4 and 11.2 < res.t[o] < 11.5, (res.zeta[o], res.t[o])  # cek: sesuai hasil awal
    assert (res.rmax > 0).all() and res.zeta.min() < 5
    # --- plot 1 dan 2
    fig, ax = plt.subplots(figsize=(9, 4)); ax.semilogy(days, res.zeta, c="#1b6ca8")
    for z in (0.5, 1, 2, 5, 10): ax.axhline(z, ls=":", c="gray", lw=.7)
    ax.set(ylabel="jarak zenith minimum (deg)", title="PENS 2026: jarak zenith minimum matahari per hari"); fig.autofmt_xdate(); fig.tight_layout()
    fig.savefig(HERE / "zeta_min_vs_hari.png", dpi=110); plt.close(fig)
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.semilogy(days, res.rmax, c="#1b6ca8", label="numerik (grid 10 s, 1 s, 0.1 s)")
    ax.semilogy(days, res.analitik, c="#d1495b", ls="--", label="analitik v/zeta")
    for t in THR: ax.axhline(t, ls=":", c="gray", lw=.7)
    ax.set(ylabel="laju azimuth maks (deg/s)", title="Laju azimuth sendi maksimum bila mengikuti matahari sempurna"); ax.legend(); fig.autofmt_xdate(); fig.tight_layout()
    fig.savefig(HERE / "laju_azimuth_vs_hari.png", dpi=110); plt.close(fig)
    # --- keyhole
    kd = (("terburuk", w), ("2026-10-04", o))
    kh = {name: keyhole_day(f"{d:%Y-%m-%d}", res.tc[d]) for name, d in kd}
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    for (name, r), c in zip(kh.items(), ("#d1495b", "#1b6ca8")):
        ax[0].loglog(LIMS, [max(r[45, l]["err_reach"], 1e-4) for l in LIMS], "o-", c=c, label=name)
        ax[1].loglog(LIMS, [max(r[45, l]["loss_kh"], 1e-6) for l in LIMS], "o-", c=c, label=name)
    ax[0].set(xlabel="batas laju azimuth (deg/s)", ylabel="galat pointing maks (deg)", title="Galat maks (zenith <= 45 deg)")
    ax[1].set(xlabel="batas laju azimuth (deg/s)", ylabel="rugi energi harian (%)", title="Rugi energi keyhole (q2 maks 45 deg)")
    ax[0].legend(); fig.tight_layout(); fig.savefig(HERE / "galat_rugi_vs_batas_laju.png", dpi=110); plt.close(fig)
    # --- teks hasil
    cnt = [(th, int((res.zeta < th).sum())) for th in (0.5, 1, 2, 5, 10)]
    rep = [("2026-03-20", "ekuinoks Mar"), ("2026-06-21", "solstis Jun"), ("2026-10-04", "4 Okt"), ("2026-12-21", "solstis Des"), (f"{w:%Y-%m-%d}", "hari terburuk")]
    z5 = res.zeta < 5; rw = res.rmax.idxmax()
    L = ["# Analisis zenith (keyhole) tracker 2 sumbu, PENS 2026\n",
         "Dihasilkan otomatis oleh `zenith_analysis.py` (jalankan ulang untuk memperbarui angka).\n",
         "## Asumsi dan metode\n",
         "- Titik panel: lintang -7.276006, bujur 112.794327, Asia/Jakarta, ketinggian 5 m (asumsi). Posisi matahari: pvlib (nrel_numpy) via `sun.loc`; zenith = 90 - elevasi tampak.",
         "- Zeta minimum dan waktunya: kulminasi dicari di grid 10 s, lalu diperhalus pada grid 0.1 s (+-60 s). Resolusi zeta pada grid halus sekitar 0.0002 deg (laju matahari 0.004 deg/s kali 0.05 s).",
         "- Laju azimuth sendi (panel mengikuti matahari sempurna, cabang q2 >= 0, q1 = azimuth matahari): selisih azimuth antar sampel dibagi dt. Grid 10 s seharian, 1 s pada +-20 menit sekitar kulminasi, 0.1 s pada +-60 s. Hanya interval dengan matahari di atas horizon.",
         "- Estimasi analitik: v/zeta dengan v = laju sudut matahari yang diukur di kulminasi (deg/s) dan zeta dalam radian (hampiran lintasan lurus melewati zenith).",
         "- Strategi keyhole: q1 mengejar azimuth matahari dengan laju maks `lim`; q2 dianggap tanpa batas laju dan dipilih optimal dalam [0, q2_maks] (cabang q2 >= 0). Galat = sudut antara normal panel dan arah matahari. Galat maks hanya dilaporkan saat matahari terjangkau (zenith <= q2_maks); saat zenith > q2_maks galat besar bukan akibat keyhole.",
         "- Rugi energi: integral DNI*cos(galat), langit cerah (sun_day okta=0, DNI grid 1 s diinterpolasi ke grid simulasi 1 s, dan 0.1 s dekat kulminasi). Dua pembanding: rugi keyhole = relatif terhadap tracker laju tak terbatas dengan q2_maks sama; rugi total = relatif terhadap pointing sempurna (cos = 1).",
         "- q2_maks = 45 deg adalah asumsi kasar (belum diukur); 60 dan 90 deg hanya sensitivitas. Laju q2 tidak dibatasi. Rasio gigi sendi belum diketahui: laju sendi dianggap sama dengan laju poros motor.\n",
         "## 1. Jarak zenith minimum\n",
         table(["zeta <", "jumlah hari (dari 365)"], [(f"{th} deg", c) for th, c in cnt]), "",
         f"- Hari terdekat zenith: {w:%Y-%m-%d}, zeta = {res.zeta[w]:.4f} deg, pukul {hhmm(res.t[w])} WIB.",
         f"- 2026-10-04: zeta = {res.zeta[o]:.3f} deg, pukul {hhmm(res.t[o])} WIB.",
         f"- Musim zeta < 2 deg: {spans(list(res.index[res.zeta < 2]))}.",
         f"- Musim zeta < 5 deg: {spans(list(res.index[z5]))}.",
         f"- Musim zeta < 10 deg: {spans(list(res.index[res.zeta < 10]))}.\n",
         "![zeta](zeta_min_vs_hari.png)\n",
         "## 2. Laju azimuth sendi maksimum (pointing sempurna)\n",
         f"- Terburuk sepanjang tahun: {res.rmax[rw]:.2f} deg/s pada {rw:%Y-%m-%d} (zeta {res.zeta[rw]:.4f} deg; analitik {res.analitik[rw]:.2f} deg/s).",
         f"- Rasio numerik/analitik pada hari zeta < 5 deg: median {res.rasio[z5].median():.3f}, rentang {res.rasio[z5].min():.3f} sampai {res.rasio[z5].max():.3f}.",
         "- Jumlah hari dengan laju maks melebihi ambang: " + ", ".join(f"> {t} deg/s: {int((res.rmax > t).sum())} hari" for t in THR) + ".",
         "- Durasi laju melebihi ambang per hari, maksimum tahunan: " + ", ".join(f"> {t}: {max(x[i] for x in res.dur):.0f} s" for i, t in enumerate(THR)) + ".\n",
         table(["tanggal", "zeta (deg)", "laju maks numerik (deg/s)", "analitik v/zeta (deg/s)", "detik > 0.1", "detik > 0.5", "detik > 1"],
               [(f"{d:%Y-%m-%d}", f"{res.zeta[d]:.4f}", f"{res.rmax[d]:.3f}", f"{res.analitik[d]:.3f}", *[f"{x:.0f}" for x in res.dur[d]]) for d in (w, o)]), "",
         "Contoh hari lain (laju maks, deg/s): " + ", ".join(f"{d:%m-%d}: {res.rmax[d]:.4f}" for d in pd.to_datetime(["2026-01-15", "2026-03-20", "2026-06-21", "2026-09-23", "2026-12-21"])) + ".\n",
         "![laju](laju_azimuth_vs_hari.png)\n",
         "## 3. Strategi keyhole: batas laju azimuth\n"]
    for name, d in kd:
        r = kh[name]
        L += [f"### {d:%Y-%m-%d} ({'hari terburuk' if name == 'terburuk' else '4 Okt'}), zeta {res.zeta[d]:.4f} deg\n",
              table(["batas laju (deg/s)", "galat maks (deg), q2maks 45", "rugi keyhole % (45)", "rugi total % (45)", "galat maks (deg), q2maks 90", "rugi keyhole % (90)"],
                    [(l, f"{r[45, l]['err_reach']:.2f}", f"{r[45, l]['loss_kh']:.2g}", f"{r[45, l]['loss_perf']:.2f}", f"{r[90, l]['err_reach']:.2f}", f"{r[90, l]['loss_kh']:.2g}") for l in LIMS]), "",
              f"Energi DNI harian (kWh/m2): pointing sempurna {r[45, 1]['e_perf']:.3f}; tracker ideal q2maks 45 {r[45, 1]['e_ideal']:.3f}; q2maks 90 {r[90, 1]['e_ideal']:.3f}.\n"]
    L += ["![keyhole](galat_rugi_vs_batas_laju.png)\n",
          "### Fraksi matahari dapat dijangkau (zenith <= q2_maks), langit cerah, nilai: % waktu siang / % energi DNI\n",
          table(["hari", "q2maks 45", "60 (sensitivitas)", "90 (sensitivitas)"], [(f"{d} ({n})", *[f"{a:.0f} / {b:.0f}" for a, b in reach_frac(d)]) for d, n in rep]), "",
          "Pada 90 deg seluruh waktu siang terjangkau menurut definisi (zenith <= 90 deg).\n"]
    okey = kh["terburuk"]
    ok = lambda r: [l for l in LIMS if r[45, l]["loss_kh"] <= 0.01 and r[45, l]["err_reach"] <= 1.0]
    ok1 = ok(okey); ok1o = ok(kh["2026-10-04"])
    L += ["## 4. Kesimpulan desain\n",
          f"- Laju azimuth teoretis (tanpa batas) di kulminasi: {res.rmax[rw]:.1f} deg/s pada hari terburuk, {res.rmax[o]:.2f} deg/s pada 4 Okt, hanya selama detik hingga puluhan detik. Dari sisi kecepatan langkah laju itu ringan (tabel di bawah: 3.6 deg/s sekitar 32 langkah/detik pada 1/16). Yang belum diketahui adalah percepatan, torsi, dan resonansi saat laju berubah tajam (butuh model dinamika, Fase 4), jadi batas laju (strategi keyhole) dipertahankan sebagai pengaman, bukan karena kecepatan langkah tidak mencukupi.",
          f"- Batas laju terkecil dalam daftar yang menjaga rugi keyhole <= 0.01% energi DNI harian dan galat maks <= 1 deg (langit cerah, q2maks 45): hari terburuk {ok1[0] if ok1 else 'tidak ada dalam daftar'} deg/s, 4 Okt {ok1o[0] if ok1o else 'tidak ada dalam daftar'} deg/s. Rugi pada batas lain ada di tabel bagian 3.",
          "- Rugi energi keyhole dibanding rugi total (kolom rugi total, q2maks 45): bagian besar rugi total berasal dari waktu saat zenith > 45 deg, bukan dari zenith. Keyhole hanya berpengaruh pada hari zeta kecil dan sekitar kulminasi.",
          "- Langkah per detik = laju (deg/s) / (1.8 deg / microstep), poros motor tanpa gearbox (jika ada rasio gigi N, kalikan N):\n",
          table(["laju (deg/s)", "1/8", "1/16", "1/32"], [(f"{r:g}", *[f"{r / (1.8 / m):.3g}" for m in (8, 16, 32)]) for r in (0.004, 0.05, 0.1, 0.5, 1, 2, 5)]), "",
          "- Pelacakan biasa (matahari 0.004 deg/s) hanya perlu sekitar 0.02 sampai 0.07 langkah per detik pada 1/8 sampai 1/32 (baris 0.004), jadi microstep halus dipilih demi resolusi dan kehalusan, bukan kecepatan. Batas laju keyhole 1 sampai 5 deg/s memerlukan sekitar 18 sampai 89 langkah per detik pada 1/32 (tabel), masih ringan bagi driver umum. Kontrol: pasang pembatas laju (rate limiter) pada setpoint q1 dan, dekat zenith, jangan memutar azimuth dari umpan balik LDR, karena arah azimuth dari selisih LDR tidak terdefinisi di sana.",
          "- Belum diverifikasi: batas q2 nyata, rasio gigi, torsi dan kehilangan langkah stepper pada laju dan beban nyata, serta efek awan (hanya langit cerah). Simulasi tidak membatasi laju q2 dan tidak memodelkan LDR."]
    out = HERE / "zenith_analysis_results.md"
    out.write_text("\n".join(L) + "\n", encoding="utf-8")
    assert "—" not in out.read_text(encoding="utf-8")
    print("zeta<:", cnt, "| terburuk", w.date(), f"zeta={res.zeta[w]:.4f}", "| rmax", f"{res.rmax[rw]:.2f} deg/s", rw.date())
    for n, r in kh.items():
        print("kh", n, {l: (round(r[45, l]['err_reach'], 2), round(r[45, l]['loss_kh'], 4), round(r[45, l]['loss_perf'], 2)) for l in LIMS})
    print("OK")


if __name__ == "__main__":
    main()
