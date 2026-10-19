"""Tapis Kalman diperluas (EKF) untuk menaksir SIMPANGAN sudut tracker loop terbuka.

Motor loop terbuka: sudut PERINTAH diketahui persis (langkah dihitung), sudut BEBAN tidak. Yang tidak diketahui
adalah simpangannya:
    q_beban = q_perintah + b,     b = [b_azimuth, b_kemiringan]
`b` memuat salah pasang mekanik, lepas langkah terkumpul, dan sisa defleksi yang tidak dimodelkan. Itulah keadaan
yang ditaksir di sini, bukan sudutnya sendiri.

Model keadaan (jalan acak, karena lepas langkah datang sewaktu-waktu dan tidak dapat diramalkan):
    b_{k+1} = b_k + w,   w ~ N(0, Q)      ->   P_prediksi = P + Q      (F = I, lihat catatan Jacobian di bawah)
Tanpa pengukuran, P membesar terus. Itulah arti loop terbuka secara kuantitatif.

Pengukuran:
  LDR   z = (ex, ey), tak linier terhadap b. EKF melinierkannya dengan Jacobian H = de/db, yang mengandung
        kinematik dn/dq. Di dekat zenith kolom azimuth pada H mengecil sebanding sin(q2), jadi azimuth nyaris
        TIDAK TERAMATI dan P pada azimuth membesar sendiri. Efek keyhole muncul sebagai angka, bukan aturan tangan.
  Arus  z = Imp, juga tak linier. Di dekat puncak turunannya nol, jadi sumbangan informasinya kecil; ia baru
        berguna saat simpangan besar. Dipakai sebagai pengukuran lemah, bukan sebagai pengoreksi utama.

BATAS MENDASAR: bias LDR (ketidakcocokan antar sensor) tidak dapat dibedakan dari salah pasang mekanik. Keduanya
menggeser pembacaan dengan cara yang sama. EKF akan menyerap bias LDR ke dalam taksiran b, lalu MENGARAHKAN panel
keliru sebesar bias itu. Ini bukan kekurangan tapisnya, melainkan sifat susunan sensornya; perbaikannya kalibrasi.

Pakai: python ekf.py [--selftest]
"""
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "kontrol_hibrid"))
import fusi_sensor as F      # noqa: E402
import hybrid_sim as H       # noqa: E402
import panel_listrik as PL   # noqa: E402
import sensor_arus as SA     # noqa: E402
import sun                   # noqa: E402
import tracker               # noqa: E402

DT = 10.0                    # s, selang tapis
Q_DRIFT = 0.01               # deg per akar detik, jalan acak b (ASUMSI: hanyutan mekanik halus)
B0_SIGMA = 3.0               # deg, ketidakpastian awal b (belum tahu apa apa)
SLIP_DEG = 360.0 / 50 / 2    # 3.6 deg, setengah siklus listrik stepper: satuan lepas langkah terkecil
K_MODEL_ARUS = 0.03          # ketidakpastian RELATIF ramalan arus (toleransi pabrik +-3 %; kotor dan suhu menambah)


def jacobian_ldr(q1, q2, sky, p, d=0.25):
    """H = de/db (2x2) dan nilai e tanpa derau pada pose ini. Baris: (ex, ey). Kolom: (b_az, b_q2).
    Kuantisasi dan ketidakcocokan dimatikan: yang dicari kemiringan sebenarnya."""
    q = replace(p, vn=0.0, mism=0.0, bits=30)
    rng = __import__("random").Random(0)
    satu = [1.0] * 4
    e0 = F.ldr_baca(q1, q2, sky, q, rng, satu)[:2]
    h = np.zeros((2, 2))
    for j, (dq1, dq2) in enumerate(((d, 0.0), (0.0, d))):
        a = F.ldr_baca(q1 + dq1, q2 + dq2, sky, q, rng, satu)[:2]
        b = F.ldr_baca(q1 - dq1, q2 - dq2, sky, q, rng, satu)[:2]
        h[:, j] = [(a[0] - b[0]) / (2 * d), (a[1] - b[1]) / (2 * d)]
    return h, np.array(e0)


def sigma_e(sky, q1, q2, p, seed=0, n=200):
    """Simpangan baku derau ACAK pada (ex, ey), dalam satuan sinyal. Ketidakcocokan dipegang tetap (itu bias)."""
    rng = __import__("random").Random(seed)
    satu = [1.0] * 4
    v = np.array([F.ldr_baca(q1, q2, sky, p, rng, satu)[:2] for _ in range(n)])
    return v.std(axis=0)


def arus_dan_jacobian(q1, q2, sky, d=0.5):
    """Arus sebenarnya (A) dan dI/db (2-vektor, A per derajat) pada pose ini."""
    az, el, dni, dhi, ghi = sky
    f = lambda a, b: PL.arus(math.radians(a), math.radians(b), az, el, dni, dhi, ghi)
    i0 = f(q1, q2)
    return i0, np.array([(f(q1 + d, q2) - f(q1 - d, q2)) / (2 * d), (f(q1, q2 + d) - f(q1, q2 - d)) / (2 * d)])


class EKF:
    """Keadaan b = [b_az, b_q2] (deg). F = I, jadi Jacobian dinamika sepele; yang tak linier adalah pengukurannya."""

    def __init__(self, b0=(0.0, 0.0), p0=B0_SIGMA):
        self.b = np.array(b0, float)
        self.P = np.eye(2) * p0 ** 2

    def prediksi(self, dt=DT, q_drift=Q_DRIFT):
        self.P = self.P + np.eye(2) * (q_drift ** 2 * dt)

    def koreksi(self, z, h_pred, Hm, R):
        """Satu pembaruan EKF. z dan h_pred dalam satuan pengukuran, Hm = dh/db, R ragam pengukuran."""
        z, h_pred = np.atleast_1d(z), np.atleast_1d(h_pred)
        Hm = np.atleast_2d(Hm)
        R = np.atleast_2d(R)
        S = Hm @ self.P @ Hm.T + R
        K = self.P @ Hm.T @ np.linalg.inv(S)
        self.b = self.b + K @ (z - h_pred)
        I_KH = np.eye(2) - K @ Hm
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T       # bentuk Joseph: tetap simetris dan positif
        return float(np.sqrt(np.diag(S)[0]))

    def sigma(self):
        return np.sqrt(np.diag(self.P))


def hari(tanggal="2026-10-07", dt=DT, el_min=10.0):
    """Lintasan perintah sehari pada grid dt (deg), hanya saat matahari terjangkau."""
    d = sun.sun_day(tanggal, f"{int(dt)}s")
    d = d[d.el > el_min]
    q1, q2 = tracker.ik_track(d.az.values, d.el.values, q2_max=math.radians(90))
    q2 = np.clip(q2, -math.radians(45), math.radians(45))
    ok = ~np.isnan(q1)
    el = d.el.values[ok]
    return dict(t=(d.index - d.index[0]).total_seconds().values[ok],
                q1=np.degrees(q1[ok]), q2=np.degrees(q2[ok]), az=d.az.values[ok], el=el,
                dni=d.dni.values[ok], dhi=d.dhi.values[ok], ghi=d.ghi.values[ok],
                reach=(90.0 - el) <= 45.0)   # di luar ini kemiringan terpotong 45 deg dan galat didominasi jangkauan


def jalankan(tr, pakai_ldr=True, pakai_arus=False, mism=0.0, slip_pada=None, slip_deg=7.2,
             b_awal=(0.5, -0.3), seed=0, p=None, n_avg_arus=100):
    """Satu hari dengan tapis. Kembalikan galat pointing per langkah dan riwayat sigma.
    `slip_pada` = pecahan hari saat lepas langkah azimuth terjadi. `mism` = ketidakcocokan LDR (bias, tidak dikalibrasi)."""
    p = p or H.P()
    rng = __import__("random").Random(seed)
    mism_tetap = [1.0 + rng.gauss(0.0, mism) for _ in range(4)] if mism else [1.0] * 4
    f = EKF()
    n = len(tr["t"])
    i_slip = int(slip_pada * n) if slip_pada is not None else -1
    b_true = np.array(b_awal, float)
    err, sig, bhat, btrue = [], [], [], []
    for k in range(n):
        sky = (float(tr["az"][k]), float(tr["el"][k]), float(tr["dni"][k]), float(tr["dhi"][k]), float(tr["ghi"][k]))
        if k == i_slip:
            b_true = b_true + np.array([slip_deg, 0.0])          # lepas langkah azimuth: 2 siklus listrik
        b_true = b_true + np.array([rng.gauss(0, Q_DRIFT * math.sqrt(DT)), rng.gauss(0, Q_DRIFT * math.sqrt(DT))])
        # perintah dikoreksi dengan taksiran simpangan: inilah gunanya tapis
        c1 = float(tr["q1"][k]) - f.b[0]
        c2 = float(np.clip(float(tr["q2"][k]) - f.b[1], -45.0, 45.0))
        qb1, qb2 = c1 + b_true[0], c2 + b_true[1]                # sudut beban sebenarnya
        f.prediksi()
        # sudut beban yang DIRAMALKAN tapis = perintah + taksiran simpangan. Di sinilah h dan H harus dihitung.
        pl1, pl2 = c1 + f.b[0], c2 + f.b[1]
        if pakai_ldr:
            Hm, e_pred = jacobian_ldr(pl1, pl2, sky, p)
            z = np.array(F.ldr_baca(qb1, qb2, sky, p, rng, mism_tetap)[:2])
            se = sigma_e(sky, pl1, pl2, p, seed=k, n=40)
            f.koreksi(z, e_pred, Hm, np.diag(np.maximum(se, 1e-6) ** 2))
        if pakai_arus:
            i_pred, Hi = arus_dan_jacobian(pl1, pl2, sky)
            z_i = SA.baca(PL.arus(math.radians(qb1), math.radians(qb2), *sky), rng_np(seed + k), n_avg_arus)
            # R arus HARUS memuat ketidakpastian MODEL, bukan hanya derau ADC. Ramalan arus bergantung toleransi
            # pabrik 3 %, kotor, suhu sel, dan albedo; semuanya jauh lebih besar daripada derau 0.2 mA. Memakai
            # derau ADC saja membuat penguatan Kalman ribuan kali terlalu besar dan tapis rusak (terukur: galat
            # pointing melonjak dari 0.18 ke 20.5 derajat).
            r_i = SA.sigma_baca(n_avg_arus) ** 2 + (K_MODEL_ARUS * max(i_pred, 1e-3)) ** 2
            f.koreksi([z_i], [i_pred], Hi.reshape(1, 2), [[r_i]])
        s = tracker.sun_vec(sky[0], sky[1])
        nb = tracker.fk(math.radians(qb1), math.radians(qb2))
        err.append(math.degrees(math.acos(max(-1.0, min(1.0, float(nb @ s))))))
        sig.append(f.sigma().copy()); bhat.append(f.b.copy()); btrue.append(b_true.copy())
    err, sig, bhat, btrue = map(np.array, (err, sig, bhat, btrue))
    m = tr["reach"]
    return dict(err=err, err_reach=err[m], sig=sig, bhat=bhat, btrue=btrue,
                b_err=np.abs(bhat - btrue), b_err_reach=np.abs(bhat - btrue)[m])


def rng_np(seed):
    return np.random.default_rng(seed)


def selftest():
    tr = hari()
    sky = (float(tr["az"][0]), float(tr["el"][0]), float(tr["dni"][0]), float(tr["dhi"][0]), float(tr["ghi"][0]))
    p = H.P()
    # 1) tanpa pengukuran, P membesar seperti jalan acak: sigma ~ sqrt(t)
    f = EKF(p0=0.5)
    for _ in range(100):
        f.prediksi()
    harap = math.sqrt(0.5 ** 2 + Q_DRIFT ** 2 * DT * 100)
    assert abs(f.sigma()[0] - harap) < 1e-9, (f.sigma(), harap)
    # 2) Jacobian LDR: kolom azimuth mengecil sebanding sin(q2) saat matahari meninggi
    sky45 = F.langit(0.0, el_target=45.0)
    sky80 = F.langit(0.0, el_target=80.0)
    H45 = jacobian_ldr(0.0, 45.0, sky45, p)[0]
    H80 = jacobian_ldr(0.0, 10.0, sky80, p)[0]
    assert abs(H45[1, 0]) > 3 * abs(H80[1, 0]), (H45, H80)
    # 3) koreksi mengecilkan P, dan mengecilkannya lebih banyak pada sumbu yang teramati
    f1, f2 = EKF(p0=3.0), EKF(p0=3.0)
    se = sigma_e(sky45, 0.0, 45.0, p, n=100)
    R = np.diag(np.maximum(se, 1e-6) ** 2)
    f1.koreksi(np.zeros(2), np.zeros(2), H45, R)
    f2.koreksi(np.zeros(2), np.zeros(2), H80, R)
    assert (f1.sigma() < 3.0).all()
    assert f2.sigma()[0] > f1.sigma()[0], (f1.sigma(), f2.sigma())   # azimuth kurang teramati saat matahari tinggi
    # 4) P tetap simetris dan positif setelah banyak koreksi (bentuk Joseph)
    for _ in range(50):
        f1.prediksi(); f1.koreksi(np.zeros(2), np.zeros(2), H45, R)
    assert np.allclose(f1.P, f1.P.T) and np.all(np.linalg.eigvals(f1.P) > 0)
    # 5) tapis dengan LDR bersih memperbaiki galat pointing dibanding astro saja
    a = jalankan(tr, pakai_ldr=False, slip_pada=0.3, seed=2)
    b = jalankan(tr, pakai_ldr=True, slip_pada=0.3, seed=2)
    assert b["err_reach"].mean() < a["err_reach"].mean() / 2, (a["err_reach"].mean(), b["err_reach"].mean())
    assert b["b_err_reach"].mean() < a["b_err_reach"].mean() / 3, (a["b_err_reach"].mean(), b["b_err_reach"].mean())
    # 6) bias LDR yang tidak dikalibrasi diserap jadi taksiran yang keliru (batas mendasar)
    c = jalankan(tr, pakai_ldr=True, mism=0.03, slip_pada=0.3, seed=2)
    assert c["err_reach"].mean() > b["err_reach"].mean(), (b["err_reach"].mean(), c["err_reach"].mean())
    print("selftest ekf: lolos")
    print(f"  galat pointing (saat terjangkau): astro saja {a['err_reach'].mean():.2f} deg, "
          f"EKF+LDR bersih {b['err_reach'].mean():.2f} deg, EKF+LDR 3% tak terkalibrasi {c['err_reach'].mean():.2f} deg")
    print(f"  galat taksiran simpangan b (az, q2): astro {a['b_err_reach'].mean(axis=0).round(2)}, "
          f"EKF {b['b_err_reach'].mean(axis=0).round(2)} deg")
    print(f"  sigma akhir EKF (az, q2): {b['sig'][-1][0]:.3f}, {b['sig'][-1][1]:.3f} deg")
    return dict(tr=tr, a=a, b=b, c=c, H45=H45, H80=H80)


def tabel(r):
    tr = r["tr"]
    out = ["# EKF: menaksir simpangan sudut tracker loop terbuka\n",
           "Dibangkitkan `kontrol_hibrid/ekf.py`. Keadaan yang ditaksir adalah SIMPANGAN `b` antara sudut perintah "
           "dan sudut beban (salah pasang + lepas langkah terkumpul), bukan sudutnya sendiri.\n",
           f"Hari 7 Okt 2026, selang tapis {DT:.0f} s, jalan acak {Q_DRIFT} deg per akar detik (ASUMSI), "
           f"ketidakpastian awal {B0_SIGMA} deg. Lepas langkah azimuth 7.2 deg (dua siklus listrik) disuntikkan "
           "pada 30 persen hari.\n",
           "## 1. Perbandingan galat pointing sehari\n",
           "| Susunan | Galat rata-rata (deg) | Galat p95 (deg) | Galat maks (deg) |\n|---|---|---|---|"]
    kasus = [("astro saja (tanpa tapis)", r["a"]), ("EKF + LDR terkalibrasi", r["b"]),
             ("EKF + LDR 3 % tak terkalibrasi", r["c"])]
    arus = jalankan(tr, pakai_ldr=True, pakai_arus=True, slip_pada=0.3, seed=2)
    kasus.append(("EKF + LDR terkalibrasi + arus", arus))
    buta = jalankan(tr, pakai_ldr=False, pakai_arus=True, slip_pada=0.3, seed=2)
    kasus.append(("EKF + arus saja", buta))
    for nama, d in kasus:
        e = d["err_reach"]
        out.append(f"| {nama} | {e.mean():.3f} | {np.percentile(e, 95):.3f} | {e.max():.3f} |")
    out += ["\nTiga hal yang terbaca:\n"
            "1. **LDR terkalibrasi memperbaiki 14 kali** (2.52 menjadi 0.18 derajat).\n"
            "2. **LDR yang TIDAK dikalibrasi lebih buruk daripada tidak memakai LDR sama sekali** (5.63 berbanding "
            "2.52 derajat). Tapis menyerap bias LDR sebagai salah pasang lalu mengarahkan panel keliru sebesar itu. "
            "Ini menegaskan temuan langkah sebelumnya: kalibrasi adalah syarat, bukan penyempurnaan.\n"
            "3. **Arus hampir tidak menambah apa apa** di atas LDR (0.181 menjadi 0.181), dan sendirian hanya "
            "memperbaiki 11 persen (2.52 menjadi 2.24). Sesuai dugaan dari Jacobian: dI/db hampir nol di dekat puncak.\n",
            "\n**Jebakan yang saya temui:** mula mula R arus saya isi hanya derau ADC (0.2 mA), dan tapis RUSAK, "
            "galat melonjak ke 20.5 derajat. Sebabnya penguatan Kalman K = P H / R menjadi ribuan kali terlalu besar "
            "ketika R jauh lebih kecil daripada galat MODEL ramalan arus. Ketidakpastian ramalan arus didominasi "
            "toleransi pabrik 3 persen, kotor, suhu sel, dan albedo, bukan oleh deraunya ADC. Setelah R memuat suku "
            "model, arus menjadi tidak berbahaya sekaligus hampir tidak berguna. Pelajarannya umum: **dalam tapis "
            "Kalman, R harus mewakili selisih antara pengukuran dan MODEL, bukan hanya derau sensor.**\n",
            "\n## 2. Ketidakpastian yang dilaporkan tapis\n",
            "Akar diagonal P, yaitu seberapa yakin tapis terhadap taksirannya sendiri.\n",
            "| Saat | sigma azimuth (deg) | sigma kemiringan (deg) |\n|---|---|---|"]
    for nama, idx in (("awal", 0), ("sebelum lepas langkah", int(0.29 * len(tr["t"]))),
                      ("sesudah lepas langkah", int(0.31 * len(tr["t"]))), ("tengah hari", len(tr["t"]) // 2),
                      ("akhir hari", -1)):
        s = r["b"]["sig"][idx]
        out.append(f"| {nama} | {s[0]:.3f} | {s[1]:.3f} |")
    m = tr["reach"]
    sig_lap = r["b"]["sig"][m].mean(axis=0)
    sig_nyata = r["b"]["b_err_reach"].std(axis=0) + np.abs(r["b"]["b_err_reach"].mean(axis=0))
    out += ["\n**Pemeriksaan kejujuran tapis:** sigma yang DILAPORKAN harus sepadan dengan galat yang SEBENARNYA. "
            f"Rata-rata sigma yang dilaporkan {sig_lap.round(3).tolist()} derajat, sedangkan sebaran galat taksiran "
            f"sebenarnya {sig_nyata.round(3).tolist()} derajat. Tapis ini **terlalu percaya diri** sekitar "
            f"{float(np.max(sig_nyata / np.maximum(sig_lap, 1e-9))):.1f} kali pada azimuth, terutama karena Q "
            "(jalan acak) tidak mewakili lepas langkah yang datang sebagai lompatan besar sesekali. Memperbesar Q, "
            "atau menambahkan uji pencilan pada inovasi, akan membuat laporannya lebih jujur.\n",
            "\n## 3. Jacobian pengukuran dan efek keyhole\n",
            "H = de/db. Kolom azimuth mengecil sebanding sin(q2), jadi saat matahari tinggi LDR hampir tidak "
            "memberi tahu apa apa tentang azimuth, dan tapis melaporkannya sendiri lewat P yang membesar.\n",
            "| Keadaan | dex/db_az | dex/db_q2 | dey/db_az | dey/db_q2 |\n|---|---|---|---|---|",
            f"| elevasi 45 deg (q2 45) | {r['H45'][0, 0]:+.5f} | {r['H45'][0, 1]:+.5f} | {r['H45'][1, 0]:+.5f} | {r['H45'][1, 1]:+.5f} |",
            f"| elevasi 80 deg (q2 10) | {r['H80'][0, 0]:+.5f} | {r['H80'][0, 1]:+.5f} | {r['H80'][1, 0]:+.5f} | {r['H80'][1, 1]:+.5f} |",
            "\nInilah pertemuan Jacobian dan ketidakpastian yang dibahas sebelumnya: keyhole bukan aturan tangan "
            "`if q2 < 5 derajat`, melainkan akibat langsung dari H yang kehilangan rank.\n",
            "\n## 4. Pemulihan setelah lepas langkah\n"]
    e = r["b"]["err"]
    i_slip = int(0.3 * len(tr["t"]))
    pulih = next((k for k in range(i_slip, len(e)) if all(e[k:k + 10] < 1.0)), None)
    out += [f"Lepas langkah azimuth 7.2 deg disuntikkan pada langkah {i_slip}. Galat memuncak "
            f"{e[i_slip:i_slip + 5].max():.2f} deg, lalu turun di bawah 1 deg setelah "
            f"{'(tidak pulih)' if pulih is None else f'{(pulih - i_slip) * DT:.0f} detik'}. "
            "Tapis tidak perlu diberi tahu bahwa lepas langkah terjadi: selisih pengukuran terhadap ramalan "
            "menanganinya sendiri.\n",
            "\n## 5. Batas mendasar: bias LDR tidak dapat dipisahkan dari salah pasang\n",
            "Keduanya menggeser pembacaan dengan cara yang sama, sehingga tidak teramati secara terpisah. EKF akan "
            "menyerap bias LDR ke dalam taksiran `b` lalu MENGARAHKAN panel keliru sebesar bias itu. Terlihat pada "
            f"baris ketiga tabel 1: galat naik dari {r['b']['err'].mean():.3f} ke {r['c']['err'].mean():.3f} derajat "
            "hanya karena LDR tidak dikalibrasi. Tapis yang lebih pintar tidak menolong; yang menolong kalibrasi, "
            "atau sensor tambahan yang biasnya tidak berkorelasi.\n",
            "\n## 6. Yang masih asumsi\n",
            f"- Jalan acak Q {Q_DRIFT} deg per akar detik, dan ketidakpastian awal {B0_SIGMA} deg.\n"
            "- Lepas langkah dimodelkan sebagai lompatan tunggal 7.2 derajat; nyatanya bergantung angin dan beban.\n"
            "- Seluruh tetapan LDR (sekat, kurva lux, ketidakcocokan) dan rantai pengukuran arus masih ASUMSI.\n"
            "- Tapis bekerja pada SIMPANGAN, bukan pada dinamika 5 Hz. Ayunan tidak diamati dan tidak diredam tapis "
            "(lihat `dinamika/osilasi_results.md`: peredamannya dari sisi perintah).\n"]
    (Path(__file__).parent / "ekf_results.md").write_text("\n".join(out), encoding="utf-8")
    print("tersimpan ekf_results.md")


if __name__ == "__main__":
    r = selftest()
    if "--selftest" not in sys.argv:
        tabel(r)
