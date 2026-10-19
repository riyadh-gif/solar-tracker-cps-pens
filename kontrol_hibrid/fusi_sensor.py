"""Penggabungan LDR dan arus panel dengan bobot dari KETIDAKPASTIAN, bukan setelan tangan.

Dasar pemikiran dari dua langkah sebelumnya:
  - LDR (`hybrid_sim.py`) peka dan BERARAH, karena memakai selisih dua sisi dibagi jumlahnya, jadi tahan
    terhadap perubahan irradiance menyeluruh. Lemah saat irradiance rendah dan saat kemiringan kecil
    (sumbu azimuth hampir tidak teramati, lihat Jacobian: dn/dq1 = -sin q2 yp).
  - Arus (`sensor_arus.py`) mengukur hasil yang sebenarnya dipanen, tetapi BUTA di bawah beberapa derajat,
    tidak memberi arah, dan tertipu awan.
  - Astro (pvlib) selalu ada dan bebas cuaca, tetapi tidak tahu salah pasang, lepas langkah, dan defleksi.

Yang dilakukan berkas ini:
  1. MENGUKUR simpangan baku galat tiap sumber dengan Monte Carlo, sebagai fungsi cuaca dan kemiringan.
  2. Menggabungkan astro dan LDR dengan bobot kebalikan ragam (inverse variance), yaitu bobot optimal
     untuk dua taksiran tak bias yang bebas satu sama lain.
  3. Memakai arus sebagai PEMERIKSA: bila arus terukur jauh di bawah arus yang diramalkan dari sudut beban,
     ada yang salah di luar dugaan model (LDR kotor, lepas langkah besar, bayangan), maka ragam LDR
     DIBESARKAN sehingga penggabungan otomatis lebih percaya astro.

BATAS: ini penggabungan statis per cuplikan (tanpa ingatan). Penggabungan dengan ingatan, yang memakai model
dinamika untuk meramal antar pengukuran, adalah tapis Kalman di langkah berikutnya.

Pakai: python fusi_sensor.py [--selftest]
"""
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "kontrol_hibrid"))
import hybrid_sim as H       # noqa: E402  (model LDR: basis, read, volt, P)
import panel_listrik as PL   # noqa: E402
import sensor_arus as SA     # noqa: E402
import sun                   # noqa: E402
import tracker               # noqa: E402

SIG_ASTRO = 1.5    # deg, simpangan baku galat astro: salah pasang + defleksi + lepas langkah kecil (ASUMSI)
N_MC = 400         # cuplikan Monte Carlo per titik


def langit(okta=0.0, el_target=45.0, tanggal="2026-10-07"):
    """Satu titik langit pvlib yang konsisten (az, el, dni, dhi, ghi)."""
    d = sun.sun_day(tanggal, "5min", okta)
    d = d[d.el > 5]
    k = int(np.argmin(np.abs(d.el.values - el_target)))
    return (90.0, float(d.el.iloc[k]), float(d.dni.iloc[k]), float(d.dhi.iloc[k]), float(d.ghi.iloc[k]))


def ldr_baca(q1_deg, q2_deg, sky, p, rng, mism=None):
    """Satu pembacaan LDR pada sudut BEBAN tertentu. Kembalikan (ex, ey, V_rata)."""
    az, el, dni, dhi, _ = sky
    n, xp, yp = H.basis(q1_deg, q2_deg)
    s = tracker.sun_vec(az, el)
    if mism is None:   # rng adalah random.Random (sama dengan yang dipakai hybrid_sim.volt)
        mism = [1.0 + rng.gauss(0.0, p.mism) for _ in range(4)]
    return H.read(n, xp, yp, s, dni, dhi, mism, p, rng)


def peka_ldr(q1_deg, q2_deg, sky, p, rng, d=0.5):
    """Kepekaan de/dq (per derajat) di sekitar pose ini, dari beda terpusat.
    Derau, ketidakcocokan, DAN kuantisasi dimatikan (bits dinaikkan): yang dicari kemiringan sebenarnya.
    Dengan kuantisasi 12 bit apa adanya, beda 0.5 derajat kadang tidak menggerakkan satu LSB pun sehingga
    kemiringan terbaca nol dan ketidakpastiannya menjadi tak hingga."""
    q = replace(p, vn=0.0, mism=0.0, bits=30)
    satu = np.ones(4)
    e1 = ldr_baca(q1_deg, q2_deg + d, sky, q, rng, satu)
    e2 = ldr_baca(q1_deg, q2_deg - d, sky, q, rng, satu)
    f1 = ldr_baca(q1_deg + d, q2_deg, sky, q, rng, satu)
    f2 = ldr_baca(q1_deg - d, q2_deg, sky, q, rng, satu)
    return (e1[0] - e2[0]) / (2 * d), (f1[1] - f2[1]) / (2 * d)


def pose_jejak(sky):
    """Sudut sendi saat panel TEPAT menunjuk matahari (deg). Kemiringan = zenith matahari."""
    az, el = sky[0], sky[1]
    return az - math.degrees(tracker.HEAD), 90.0 - el


def sigma_ldr(sky, p=None, seed=0, n_dev=40, n_s=20):
    """Ketidakpastian taksiran sudut dari LDR (deg) saat panel TEPAT menunjuk matahari (q2 = 90 - el).

    Dipisah dua, karena sifatnya berbeda:
      sig_acak  derau ADC per pembacaan. BISA ditekan dengan merata-rata (akar n).
      sig_bias  sebaran antar perangkat akibat ketidakcocokan antar LDR. TETAP untuk satu alat, TIDAK hilang
                dengan merata-rata; hanya hilang dengan kalibrasi.
    Ketidakcocokan diundi SEKALI per perangkat lalu dipakai untuk semua cuplikan perangkat itu. Mencampur
    keduanya (mengundi ulang tiap cuplikan) membesar-besarkan derau acak beberapa kali lipat."""
    p = p or H.P()
    rng = __import__("random").Random(seed)
    q1, q2_deg = pose_jejak(sky)
    kx, ky = peka_ldr(q1, q2_deg, sky, p, rng)
    sx_in, sy_in, bx, by = [], [], [], []
    for _ in range(n_dev):
        mism = [1.0 + rng.gauss(0.0, p.mism) for _ in range(4)]      # satu perangkat
        ex, ey = [], []
        for _ in range(n_s):
            a, b, _v = ldr_baca(q1, q2_deg, sky, p, rng, mism)
            ex.append(a); ey.append(b)
        sx_in.append(np.std(ex)); sy_in.append(np.std(ey))
        bx.append(np.mean(ex)); by.append(np.mean(ey))
    f = lambda v, k: float(v / abs(k)) if abs(k) > 1e-12 else float("inf")
    return dict(sig_acak_q2=f(np.mean(sx_in), kx), sig_acak_q1=f(np.mean(sy_in), ky),
                sig_bias_q2=f(np.std(bx), kx), sig_bias_q1=f(np.std(by), ky),
                sig_q2=f(math.hypot(np.mean(sx_in), np.std(bx)), kx),
                sig_q1=f(math.hypot(np.mean(sy_in), np.std(by)), ky),
                k_q2=kx, k_q1=ky)


def sigma_arus(sky, n_avg=100):
    """Ketidakpastian letak optimum menurut arus (deg): jarak dari puncak tempat arus turun 1 sigma pembacaan."""
    az, el, dni, dhi, ghi = sky
    q1 = math.radians(az - math.degrees(tracker.HEAD))
    q2 = math.radians(90 - el)
    return SA.sigma_sudut((q1, q2, az, el, dni, dhi, ghi), n_avg)


def gabung(est_astro, sig_astro, est_ldr, sig_ldr):
    """Gabungan dua taksiran tak bias dengan bobot kebalikan ragam. Kembalikan (taksiran, sigma)."""
    wa, wl = 1.0 / sig_astro ** 2, 1.0 / sig_ldr ** 2
    return (wa * est_astro + wl * est_ldr) / (wa + wl), math.sqrt(1.0 / (wa + wl))


def periksa_arus(i_ukur, i_ramal, ambang=0.05):
    """Faktor pembesar ragam LDR bila arus terukur jauh di bawah ramalan (LDR kotor, bayangan, lepas langkah).
    Mengembalikan faktor >= 1 untuk dikalikan ke ragam LDR."""
    if i_ramal <= 1e-6:
        return 1.0
    kurang = max(0.0, 1.0 - i_ukur / i_ramal)
    return 1.0 if kurang < ambang else 1.0 + (kurang / ambang) ** 2


def selftest():
    sky = langit(0.0, el_target=45.0)
    # 1) kepekaan azimuth mengecil saat matahari makin tinggi (kemiringan mengecil): de/dq1 sebanding sin(q2)
    rng = __import__("random").Random(0)
    sky_tinggi = langit(0.0, el_target=80.0)                 # q2 = 10 deg
    k45 = peka_ldr(*pose_jejak(sky), sky, H.P(), rng)
    k10 = peka_ldr(*pose_jejak(sky_tinggi), sky_tinggi, H.P(), rng)
    rasio = abs(k45[1] / k10[1])
    assert rasio > 2.0, (k45, k10, rasio)
    # kepekaan kemiringan TIDAK ikut mengecil: sumbu itu tetap teramati di mana pun
    assert abs(k45[0] / k10[0]) < 2.0, (k45[0], k10[0])
    # 2) sigma LDR membesar saat langit makin tertutup (irradiance turun, nisbah sinyal ke derau turun)
    s_cerah = sigma_ldr(langit(0.0), seed=1)
    s_mendung = sigma_ldr(langit(6.0), seed=1)
    assert s_mendung["sig_q2"] > s_cerah["sig_q2"], (s_cerah, s_mendung)
    # 3) sigma azimuth membesar saat matahari tinggi (keyhole), untuk cuaca yang sama
    s45 = sigma_ldr(sky, seed=2)
    s10 = sigma_ldr(sky_tinggi, seed=2)
    assert s10["sig_q1"] > s45["sig_q1"], (s45["sig_q1"], s10["sig_q1"])
    # 4) penggabungan: hasilnya tidak pernah lebih buruk daripada sumber terbaik, dan condong ke yang lebih pasti
    e, s = gabung(2.0, 1.5, 0.0, 0.3)
    assert s < 0.3 and abs(e) < 0.2, (e, s)
    e2, s2 = gabung(2.0, 0.3, 0.0, 1.5)
    assert abs(e2 - 2.0) < 0.2 and s2 < 0.3, (e2, s2)
    for sa, sl in ((1.5, 0.3), (0.3, 1.5), (1.0, 1.0), (5.0, 0.1)):
        assert gabung(1.0, sa, 0.0, sl)[1] <= min(sa, sl) + 1e-12
    # 5) pemeriksa arus: diam saat normal, membesar saat arus jauh di bawah ramalan
    assert periksa_arus(1.00, 1.00) == 1.0 and periksa_arus(0.98, 1.00) == 1.0
    assert periksa_arus(0.80, 1.00) > 10.0, periksa_arus(0.80, 1.00)
    assert periksa_arus(0.50, 1.00) > periksa_arus(0.80, 1.00)
    print("selftest fusi_sensor: lolos")
    print(f"  kepekaan LDR saat el 45 deg (q2 45): de/dq2 {k45[0]:.5f}, de/dq1 {k45[1]:.5f} per deg")
    print(f"  kepekaan LDR saat el 80 deg (q2 10): de/dq2 {k10[0]:.5f}, de/dq1 {k10[1]:.5f} per deg "
          f"(azimuth {rasio:.1f} kali lebih tumpul)")
    print(f"  LDR cerah  q2: acak {s_cerah['sig_acak_q2']:.3f} deg + bias antar perangkat {s_cerah['sig_bias_q2']:.3f} deg")
    print(f"  LDR cerah  q1: acak {s_cerah['sig_acak_q1']:.3f} deg + bias antar perangkat {s_cerah['sig_bias_q1']:.3f} deg")
    print(f"  LDR okta 6 q2: acak {s_mendung['sig_acak_q2']:.3f} deg + bias {s_mendung['sig_bias_q2']:.3f} deg")
    print(f"  sigma arus (rata 1 s): {sigma_arus(sky):.2f} deg")
    return dict(k45=k45, k10=k10)


def tabel():
    out = ["# Penggabungan LDR dan arus dengan bobot dari ketidakpastian\n",
           "Dibangkitkan `kontrol_hibrid/fusi_sensor.py`. Model LDR dari `hybrid_sim.py`, model arus dari "
           "`panel_listrik.py` dan `sensor_arus.py`, langit dari pvlib (7 Okt 2026).\n",
           f"Galat astro dianggap {SIG_ASTRO} deg (ASUMSI: salah pasang + defleksi gravitasi + lepas langkah kecil).\n",
           "## 1. Ketidakpastian tiap sumber (Monte Carlo, {} cuplikan)\n".format(N_MC),
           "Simpangan baku taksiran SUDUT, bukan simpangan baku sinyal mentah: derau sinyal dibagi kepekaan de/dq.\n",
           "Panel selalu dianggap TEPAT menunjuk matahari, jadi kemiringan mengikuti elevasi matahari (q2 = 90 - el).\n",
           "LDR dipisah dua: `acak` dari derau ADC (bisa ditekan dengan merata-rata) dan `bias` dari ketidakcocokan "
           "antar LDR (TETAP untuk satu alat, hanya hilang dengan kalibrasi).\n",
           "| Langit | Elevasi (deg) | q2 (deg) | LDR q2 acak | LDR q2 bias | LDR q1 acak | LDR q1 bias | Arus (deg) |\n|---|---|---|---|---|---|---|---|"]
    baris = []
    for okta in (0.0, 2.0, 4.0, 6.0):
        for el in (45.0, 60.0, 70.0, 80.0):
            sky = langit(okta, el_target=el)
            s = sigma_ldr(sky, seed=3)
            sa = sigma_arus(sky)
            q2 = 90.0 - sky[1]
            baris.append((okta, sky[1], q2, s, sa))
            out.append(f"| okta {okta:.0f} | {sky[1]:.0f} | {q2:.0f} | {s['sig_acak_q2']:.3f} | {s['sig_bias_q2']:.3f} | "
                       f"{s['sig_acak_q1']:.3f} | {s['sig_bias_q1']:.3f} | {sa:.2f} |")
    out += ["\nTiga pola terbaca:\n"
            "1. **Derau acak LDR kecil, tetapi bias antar perangkat jauh lebih besar dan menguasai.** Tanpa kalibrasi, "
            "keunggulan LDR terhadap arus sebagian besar hilang. Ini membalik perkiraan awal saya bahwa LDR unggul "
            "satu sampai dua orde; yang unggul hanya derau acaknya.\n"
            "2. **Ketelitian azimuth memburuk saat matahari makin tinggi** (kemiringan mengecil). Itu wujud nyata "
            "Jacobian yang hampir singular di dekat zenith: de/dq1 sebanding sin(q2). Ketelitian kemiringan TIDAK ikut "
            "memburuk, karena sumbu itu tetap teramati di mana pun.\n"
            "3. **Langit tertutup memperburuk LDR tetapi hampir tidak menyentuh arus.** Derau acak LDR naik sekitar "
            "1.6 kali dari okta 0 ke okta 6, dan biasnya pun naik; angka arus hampir tetap 0.8 sampai 1.3 derajat. "
            "LDR tetap tahan terhadap perubahan irradiance menyeluruh karena memakai NISBAH antar sisi, tetapi nisbah "
            "itu tidak menolong terhadap ketidakcocokan antar sensornya sendiri.\n",
            "\n**Peringatan membandingkan kolom:** kolom arus adalah jari-jari 1 sigma untuk MENENTUKAN LETAK puncak "
            "arus, sedangkan puncak arus itu sendiri bergeser dari arah matahari sebesar -2.75 sampai +2.75 derajat "
            "menurut albedo tanah (lihat `panel_listrik_results.md`). Jadi arus juga punya bias sebesar itu, yang "
            "hanya hilang bila albedo tempat pemasangan diukur. Kedua sensor sama-sama perlu kalibrasi; bedanya "
            "kalibrasi LDR mengenai perangkatnya, kalibrasi arus mengenai lingkungannya.\n",
            "\n## 2. Bobot penggabungan astro dan LDR\n",
            "Bobot kebalikan ragam: w = 1/sigma^2, taksiran gabungan = (w_astro x astro + w_ldr x ldr) / (w_astro + w_ldr). "
            f"Dengan sigma astro {SIG_ASTRO} deg.\n",
            "| Langit | q2 (deg) | Bobot LDR untuk q2 | Bobot LDR untuk q1 | sigma gabungan q1 (deg) |\n|---|---|---|---|---|"]
    for okta, _el, q2, s, _sa in baris:
        w2 = (1 / s["sig_q2"] ** 2) / (1 / s["sig_q2"] ** 2 + 1 / SIG_ASTRO ** 2)
        w1 = (1 / s["sig_q1"] ** 2) / (1 / s["sig_q1"] ** 2 + 1 / SIG_ASTRO ** 2)
        sg = gabung(0.0, SIG_ASTRO, 0.0, s["sig_q1"])[1]
        out.append(f"| okta {okta:.0f} | {q2:.0f} | {w2 * 100:.1f} % | {w1 * 100:.1f} % | {sg:.3f} |")
    out += ["\nBobot ini keluar dari pengukuran, bukan disetel tangan. Perhatikan bobot LDR untuk azimuth TURUN "
            "sendiri saat matahari makin tinggi, tanpa aturan `if q2 < 5 derajat` seperti pada `hybrid_sim.py` "
            "(parameter `guard`). Itulah keuntungan memakai ketidakpastian sebagai bobot.\n",
            "\n## 3. Nilai kalibrasi LDR\n",
            "Bias antar LDR adalah satu satunya hal yang membuat LDR kalah dari model astro. Tabel ini menunjukkan "
            "apa yang didapat bila ketidakcocokan antar LDR dikalibrasi (langit cerah, elevasi 45 deg).\n",
            "| Ketidakcocokan antar LDR | sigma LDR q2 (deg) | sigma LDR q1 (deg) | Bobot LDR q2 | Bobot LDR q1 |\n|---|---|---|---|---|"]
    sky45 = langit(0.0, el_target=45.0)
    for mm, ket in ((0.03, "tanpa kalibrasi (ASUMSI sekarang)"), (0.01, "kalibrasi kasar"),
                    (0.003, "kalibrasi teliti"), (0.0, "sempurna (batas derau ADC)")):
        s = sigma_ldr(sky45, p=replace(H.P(), mism=mm), seed=5)
        w2 = (1 / s["sig_q2"] ** 2) / (1 / s["sig_q2"] ** 2 + 1 / SIG_ASTRO ** 2)
        w1 = (1 / s["sig_q1"] ** 2) / (1 / s["sig_q1"] ** 2 + 1 / SIG_ASTRO ** 2)
        out.append(f"| {mm * 100:.1f} % ({ket}) | {s['sig_q2']:.3f} | {s['sig_q1']:.3f} | {w2 * 100:.1f} % | {w1 * 100:.1f} % |")
    out += ["\n**Inilah hasil terpenting di langkah ini.** Tanpa kalibrasi, penggabungan yang jujur akan memberi LDR "
            "bobot kecil saja, artinya LDR nyaris tidak berguna dan tracker praktis berjalan secara astro saja. "
            "Dengan kalibrasi teliti, bobot LDR melonjak dan LDR menjadi sumber utama. Jadi **kalibrasi LDR bukan "
            "penyempurnaan, melainkan syarat agar LDR layak dipasang sama sekali.**\n",
            "\n## 4. Arus sebagai pemeriksa, bukan sebagai pengoreksi\n",
            "Arus terlalu kasar untuk mengoreksi arah (satu sampai dua orde lebih buruk daripada LDR), tetapi ia "
            "mengukur hasil yang SEBENARNYA dipanen, sehingga bisa menangkap kesalahan yang tidak terlihat LDR: "
            "LDR kotor atau tertutup, lepas langkah besar, bayangan sebagian, dan panel retak. Bila arus terukur "
            "turun lebih dari 5 persen di bawah ramalan dari sudut beban, ragam LDR dibesarkan sehingga penggabungan "
            "otomatis lebih percaya astro.\n",
            "| Arus terukur / ramalan | Faktor pembesar ragam LDR |\n|---|---|"]
    for f in (1.0, 0.98, 0.95, 0.90, 0.80, 0.50):
        out.append(f"| {f:.2f} | {periksa_arus(f, 1.0):.1f} |")
    out += ["\n## 4. Batas\n",
            "- Penggabungan ini statis per cuplikan, tanpa ingatan. Ia tidak memakai model dinamika untuk meramal "
            "antar pengukuran, dan tidak menaksir lepas langkah terkumpul. Itu tugas tapis Kalman di langkah berikut.\n"
            "- sigma astro 1.5 derajat masih ASUMSI dan sangat menentukan bobotnya. Nilai nyatanya ditentukan oleh "
            "ketelitian pemasangan dan seberapa sering lepas langkah terjadi.\n"
            "- Seluruh tetapan LDR (tinggi sekat r, ketidakcocokan 3 persen, kurva lux) masih ASUMSI.\n"
            "- Bias LDR dari ketidakcocokan antar sensor TIDAK hilang dengan merata-rata; itu memerlukan kalibrasi, "
            "dan hanya sebagian yang dapat dipisahkan (lihat catatan `hybrid_results.md`).\n"]
    (Path(__file__).parent / "fusi_sensor_results.md").write_text("\n".join(out), encoding="utf-8")
    print("tersimpan fusi_sensor_results.md")


if __name__ == "__main__":
    selftest()
    if "--selftest" not in sys.argv:
        tabel()
