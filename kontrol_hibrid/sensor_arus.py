"""Arus panel sebagai SENSOR: rantai pengukuran, derau, dan seberapa kecil galat arah yang masih terbaca.

Melanjutkan `panel_listrik.py` (yang memberi arus sebenarnya) dengan sisi pengukurannya: shunt, penguat, ADC,
derau, dan hanyutan. Lalu menjawab tiga pertanyaan yang menentukan apakah arus layak jadi umpan balik:
  1. Berapa galat arah terkecil yang masih terbaca di atas derau, dan berapa lama harus dirata-rata?
  2. Bagaimana dibandingkan perubahan arus karena AWAN, yang bukan karena salah arah?
  3. Apakah pendakian bukit (perturb and observe) memakai arus bisa menemukan optimum?

RANTAI PENGUKURAN (semua ASUMSI, ganti bila perangkat sudah dipilih):
    shunt + penguat + ADC 12 bit, jangkauan 0 sampai 4 A  -> 0.977 mA per LSB
    derau acak 2 mA rms (ADC + penguat + riak pensaklaran MPPT)
    hanyutan offset 5 mA (suhu dan penuaan, lambat; tidak hilang dengan merata-rata)
    toleransi pabrik panel +-3 % (tetap, tidak mengganggu PEMBANDINGAN dua posisi berurutan)
Catatan: hanyutan dan toleransi tidak mengganggu pendakian bukit, karena yang dipakai adalah SELISIH dua
pembacaan berurutan. Derau acak mengganggu, dan itu bisa ditekan dengan merata-rata.

Pakai: python sensor_arus.py [--selftest]
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "kontrol_hibrid"))
import panel_listrik as PL   # noqa: E402
import sun                   # noqa: E402
import tracker               # noqa: E402

ADC_BITS, ADC_FS = 12, 4.0        # bit dan jangkauan penuh (A)
NOISE_A = 0.002                   # A rms, derau acak
DRIFT_A = 0.005                   # A, hanyutan offset (lambat)
F_SAMPLE = 100.0                  # Hz, laju cuplik arus


def baca(i_true, rng, n_avg=1, drift=0.0):
    """Satu pembacaan arus terukur (A): hanyutan + derau acak + kuantisasi, dirata-rata n_avg cuplikan."""
    lsb = ADC_FS / (2 ** ADC_BITS - 1)
    v = i_true + drift + rng.normal(0.0, NOISE_A, n_avg)
    return float(np.mean(np.clip(np.round(v / lsb) * lsb, 0.0, ADC_FS)))


def sigma_baca(n_avg=1):
    """Simpangan baku pembacaan setelah rata-rata (A): derau acak ditekan akar n, kuantisasi tersisa."""
    lsb = ADC_FS / (2 ** ADC_BITS - 1)
    return math.sqrt((NOISE_A ** 2) / n_avg + (lsb ** 2) / 12 / n_avg)


def arus_pada(d_deg, geo):
    """Arus sebenarnya (A) pada simpangan d_deg dari arah matahari, untuk geometri uji `geo`."""
    q1, q2, az, el, dni, dhi, ghi = geo
    return PL.arus(q1, q2 + math.radians(d_deg), az, el, dni, dhi, ghi)


def geo_uji(az=90.0, el=45.0, okta=0.0):
    """Geometri acuan: panel tepat ke matahari, irradiance dari pvlib pada okta tertentu."""
    d = sun.sun_day("2026-10-07", "5min", okta)
    k = int(np.argmin(np.abs(d.el.values - el)))
    dni, dhi, ghi = float(d.dni.iloc[k]), float(d.dhi.iloc[k]), float(d.ghi.iloc[k])
    q1 = math.radians(az - math.degrees(tracker.HEAD))
    q2 = math.radians(90 - el)
    return (q1, q2, az, el, dni, dhi, ghi)


def galat_terkecil(geo, n_avg, k_sigma=3.0, dmax=30.0):
    """Simpangan arah terkecil (deg) yang perubahannya melewati k_sigma x derau pembacaan selisih.
    Selisih dua pembacaan punya derau sqrt(2) x sigma."""
    i0 = arus_pada(0.0, geo)
    amb = k_sigma * math.sqrt(2) * sigma_baca(n_avg)
    for d in np.arange(0.05, dmax, 0.05):
        if abs(arus_pada(-d, geo) - i0) > amb:    # sisi negatif: menjauh dari puncak albedo, lebih peka
            return float(d)
    return float("nan")


def sigma_sudut(geo, n_avg=100, k=1.0, rng_deg=25.0, n=1001):
    """Seberapa teliti arus dapat MENENTUKAN LETAK optimum (deg).

    Jarak dari puncak tempat arus turun sebesar k x derau pembacaan. Berbeda dari `galat_terkecil`, yang
    mengukur jarak gerak terkecil yang terbaca dari titik kerja sekarang; di titik kerja yang bukan puncak,
    kemiringan kurva tidak nol sehingga angkanya jauh lebih kecil dan tidak boleh dipakai sebagai sigma.
    Sisi yang turun lebih lambat yang dipakai (hasil lebih hati-hati)."""
    dd = np.linspace(-rng_deg, rng_deg, n)
    ii = np.array([arus_pada(d, geo) for d in dd])
    j = int(np.argmax(ii))
    amb = k * sigma_baca(n_avg)
    langkah = float(dd[1] - dd[0])
    for m in range(1, max(j, len(dd) - j)):
        lo, hi = j - m, j + m
        a = ii[lo] if lo >= 0 else -np.inf
        b = ii[hi] if hi < len(ii) else -np.inf
        if (ii[j] - max(a, b)) >= amb:
            return m * langkah
    return float("nan")


def awan_vs_arah(okta_list=(0.0, 2.0, 4.0, 6.0)):
    """Bandingkan perubahan arus karena AWAN dengan perubahan karena salah arah 5 derajat."""
    out = []
    g0 = geo_uji(okta=0.0)
    i_cerah = arus_pada(0.0, g0)
    for ok in okta_list:
        g = geo_uji(okta=ok)
        i_ok = arus_pada(0.0, g)
        i_5 = arus_pada(-5.0, g)
        out.append(dict(okta=ok, i=i_ok, turun_awan=(i_ok / i_cerah - 1) * 100,
                        turun_5deg=(i_5 / i_ok - 1) * 100))
    return out


def hill_climb(geo, langkah=2.0, n_avg=100, n_iter=60, d0=12.0, seed=0, drift_laju=0.0, awan_laju=0.0):
    """Pendakian bukit (perturb and observe) memakai arus saja.
    Tiap iterasi: geser satu `langkah`, bandingkan arus; kalau turun, balik arah. Satu iterasi = n_avg/F_SAMPLE detik.
    `drift_laju` A/s dan `awan_laju` (pecahan irradiance per detik) menguji ketahanan terhadap perubahan lambat."""
    rng = np.random.default_rng(seed)
    dt = n_avg / F_SAMPLE
    d, arah, drift, skala = d0, 1.0, 0.0, 1.0
    i_prev = baca(arus_pada(d, geo) * skala, rng, n_avg, drift)
    jejak = [d]
    for _ in range(n_iter):
        drift += drift_laju * dt
        skala *= (1.0 + awan_laju * dt)
        d_baru = d + arah * langkah
        i_baru = baca(arus_pada(d_baru, geo) * skala, rng, n_avg, drift)
        if i_baru >= i_prev:
            d, i_prev = d_baru, i_baru
        else:
            arah = -arah
            i_prev = i_baru
            d = d_baru
        jejak.append(d)
    return np.array(jejak), dt


def puncak(geo, rng_deg=15.0):
    """Simpangan (deg) tempat arus sebenarnya paling besar."""
    dd = np.linspace(-rng_deg, rng_deg, 601)
    return float(dd[int(np.argmax([arus_pada(d, geo) for d in dd]))])


def selftest():
    rng = np.random.default_rng(0)
    geo = geo_uji()
    # 1) rantai pengukuran: tak bias, dan derau turun seperti akar n
    i_true = arus_pada(0.0, geo)
    s1 = np.std([baca(i_true, rng) for _ in range(4000)])
    s100 = np.std([baca(i_true, rng, 100) for _ in range(400)])
    assert abs(s1 / sigma_baca(1) - 1) < 0.1, (s1, sigma_baca(1))
    assert abs(s100 / sigma_baca(100) - 1) < 0.2, (s100, sigma_baca(100))
    assert abs(np.mean([baca(i_true, rng, 100) for _ in range(200)]) - i_true) < 3 * sigma_baca(100)
    # 2) merata-rata memperbaiki galat terkecil yang terbaca, tapi tidak sebanding penuh (arus ~ kosinus)
    g1, g100 = galat_terkecil(geo, 1), galat_terkecil(geo, 100)
    assert g100 < g1, (g1, g100)
    # 3) awan mengubah arus jauh lebih besar daripada salah arah 5 derajat
    aw = awan_vs_arah()
    for r in aw[1:]:
        assert abs(r["turun_awan"]) > abs(r["turun_5deg"]), r
    # 4) pendakian bukit menemukan puncak saat cerah dan tenang
    pk = puncak(geo)
    jj, _ = hill_climb(geo, n_avg=100, n_iter=60, d0=12.0, seed=1)
    akhir = float(np.mean(jj[-10:]))
    assert abs(akhir - pk) < 3.0, f"pendakian berhenti di {akhir:.2f} deg, puncak di {pk:.2f}"
    # 5) awan yang berubah membuat pendakian tersesat (ini batas pokoknya)
    jj2, _ = hill_climb(geo, n_avg=100, n_iter=60, d0=12.0, seed=1, awan_laju=-0.02)
    assert abs(float(np.mean(jj2[-10:])) - pk) > abs(akhir - pk), "awan berubah harus memperburuk"
    print("selftest sensor_arus: lolos")
    print(f"  derau pembacaan: 1 cuplikan {sigma_baca(1) * 1e3:.2f} mA, 100 cuplikan {sigma_baca(100) * 1e3:.3f} mA "
          f"(terukur {s1 * 1e3:.2f} dan {s100 * 1e3:.3f} mA)")
    print(f"  galat arah terkecil yang terbaca (3 sigma): 1 cuplikan {g1:.2f} deg, 100 cuplikan {g100:.2f} deg")
    print(f"  puncak arus pada {pk:+.2f} deg; pendakian bukit berhenti di {akhir:+.2f} deg (cerah, tenang)")
    print(f"  dengan awan menipis 2 %/s, pendakian berhenti di {float(np.mean(jj2[-10:])):+.2f} deg")
    return dict(g1=g1, g100=g100, pk=pk, akhir=akhir, aw=aw, s1=s1, s100=s100)


def tabel(r):
    geo = geo_uji()
    out = ["# Arus panel sebagai sensor arah: derau dan batasnya\n",
           "Dibangkitkan `kontrol_hibrid/sensor_arus.py`. Arus sebenarnya dari `panel_listrik.py` (dioda tunggal, "
           "pelat nama MOS C-50-18-M), lalu dilewatkan rantai pengukuran.\n",
           "## 1. Rantai pengukuran (ASUMSI)\n",
           "| Besaran | Nilai |\n|---|---|",
           f"| ADC | {ADC_BITS} bit, jangkauan 0 sampai {ADC_FS} A, {ADC_FS / (2 ** ADC_BITS - 1) * 1e3:.3f} mA per LSB |",
           f"| Derau acak | {NOISE_A * 1e3:.0f} mA rms |",
           f"| Hanyutan offset | {DRIFT_A * 1e3:.0f} mA (lambat) |",
           f"| Laju cuplik | {F_SAMPLE:.0f} Hz |",
           "\n## 2. Galat arah terkecil yang masih terbaca\n",
           "Ambang 3 sigma pada SELISIH dua pembacaan (deraunya akar 2 kali derau satu pembacaan). "
           "Diukur ke sisi yang menjauh dari puncak albedo, yaitu sisi yang lebih peka.\n",
           "| Cuplikan dirata-rata | Waktu (s) | Derau pembacaan (mA) | Galat arah terkecil (deg) |\n|---|---|---|---|"]
    for n in (1, 10, 100, 1000, 10000):
        out.append(f"| {n} | {n / F_SAMPLE:.2f} | {sigma_baca(n) * 1e3:.3f} | {galat_terkecil(geo, n):.2f} |")
    out += ["\nMerata-rata menolong, tetapi hasilnya **melambat jauh lebih cepat daripada ketelitiannya membaik**: "
            "arus berubah sebagai kosinus, jadi menuruni galat dua kali lipat menuntut ketelitian empat kali lipat, "
            "yaitu 16 kali lebih banyak cuplikan. Merata-rata 100 detik pun tidak membawa arus ke ketelitian derajat.\n",
            "\n## 3. Pembanding: awan jauh lebih berpengaruh daripada salah arah\n",
            "| Tutupan awan (okta) | Imp (A) | Turun terhadap cerah | Turun bila salah arah 5 deg |\n|---|---|---|---|"]
    for a in r["aw"]:
        out.append(f"| {a['okta']:.0f} | {a['i']:.4f} | {a['turun_awan']:+.1f} % | {a['turun_5deg']:+.3f} % |")
    out += ["\nInilah hambatan pokok memakai arus sebagai umpan balik arah: **perubahan arus karena awan dua sampai tiga "
            "ORDE lebih besar daripada perubahan karena salah arah beberapa derajat.** Pengendali yang menyimpulkan arah "
            "dari naik turunnya arus akan tertipu awan, kecuali perbandingannya dilakukan jauh lebih cepat daripada "
            "perubahan awan, atau irradiance diukur terpisah sebagai acuan.\n",
            "\n## 4. Pendakian bukit (perturb and observe) memakai arus saja\n",
            "Mulai dari simpangan 12 deg, langkah 2 deg, 100 cuplikan (1 detik) tiap pembandingan, 60 iterasi.\n",
            "| Keadaan | Berhenti di (deg) | Puncak sebenarnya (deg) | Selisih |\n|---|---|---|---|"]
    pk = r["pk"]
    for nama, kw in (("cerah dan tenang", {}),
                     ("hanyutan offset 1 mA/s", dict(drift_laju=0.001)),
                     ("awan menipis 2 %/s", dict(awan_laju=0.02)),
                     ("awan menebal 2 %/s", dict(awan_laju=-0.02)),
                     ("awan menebal 5 %/s", dict(awan_laju=-0.05))):
        akhir = [float(np.mean(hill_climb(geo, n_avg=100, n_iter=60, d0=12.0, seed=s, **kw)[0][-10:])) for s in range(5)]
        m = float(np.mean(akhir))
        out.append(f"| {nama} | {m:+.2f} (sebaran {min(akhir):+.1f} sampai {max(akhir):+.1f}) | {pk:+.2f} | {m - pk:+.2f} |")
    out += ["\nSaat cerah dan tenang pendakian bukit berhasil, tetapi hanya sampai ketelitian seukuran langkahnya, "
            "dan butuh sekitar satu menit untuk menempuh 12 derajat. Begitu irradiance berubah lambat, pendakian "
            "**tertarik mengikuti perubahan irradiance, bukan arah matahari**: arus yang naik karena awan menipis "
            "dibaca sebagai bukti arah yang benar.\n",
            "\n## 5. Kesimpulan untuk rancangan kontrol\n",
            "- Arus **tidak layak** jadi umpan balik arah yang berdiri sendiri: buta di bawah beberapa derajat, "
            "tidak memberi arah, dan mudah tertipu awan.\n"
            "- Arus **layak** untuk: memeriksa bahwa penjejakan tidak melenceng jauh (di atas 10 derajat), mendeteksi "
            "lepas langkah besar, dan menjadi ukuran hasil yang sebenarnya untuk menilai algoritma.\n"
            "- Agar arus berguna untuk arah, perlu acuan irradiance terpisah (sel acuan, piranometer, atau LDR itu "
            "sendiri) supaya pengaruh awan dapat dibagi keluar.\n"
            "- Ini memperkuat rencana penggabungan: LDR memberi arah dan kepekaan, arus memberi kebenaran hasil. "
            "Bobot penggabungan harus mengikuti ketidakpastian masing-masing, bukan disetel tangan.\n",
            "\n## 6. Yang masih asumsi\n",
            "- Seluruh rantai pengukuran (bit ADC, derau, hanyutan); ganti setelah sensor arus dipilih.\n"
            "- Model awan hanya penskalaan irradiance seragam (Kasten-Czeplak lewat okta); awan nyata membuat bayangan "
            "sebagian pada panel, yang pengaruhnya lebih rumit dan lebih buruk.\n"
            "- Pendakian bukit di sini menggeser kemiringan saja, satu sumbu.\n"]
    (Path(__file__).parent / "sensor_arus_results.md").write_text("\n".join(out), encoding="utf-8")
    print("tersimpan sensor_arus_results.md")


if __name__ == "__main__":
    r = selftest()
    if "--selftest" not in sys.argv:
        tabel(r)
