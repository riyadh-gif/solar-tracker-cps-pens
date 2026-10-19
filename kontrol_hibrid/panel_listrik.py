"""Model listrik panel surya MOS C-50-18-M: dari pelat nama ke arus dan daya pada irradiance dan suhu apa pun.

Dipakai untuk menjadikan ARUS panel sebagai sensor penunjuk arah (objektif: arus tinggi = posisi optimal),
dan nanti sebagai salah satu masukan penggabungan dengan LDR.

Rantai hitungannya:
    sudut sendi -> normal panel (tracker.fk) -> sudut datang -> IAM (pantulan kaca) -> irradiance efektif
    -> model dioda tunggal De Soto (pvlib) -> Isc, Impp, Pmpp

Pelat nama (STC: AM 1.5, 1000 W/m2, 25 C):
    Pmax 50 W, Voc 22.5 V, Isc 3 A, Vmp 18.5 V, Imp 2.7 A, toleransi +-3 %, 670 x 445 x 25 mm, 2.55 kg
    Periksa: Vmp x Imp = 49.95 W (cocok dengan Pmax), faktor isi = 49.95 / (22.5 x 3) = 0.740 (wajar untuk silikon)

YANG DIASUMSIKAN (tidak ada di pelat nama, ganti bila dapat lembar data lengkap):
    cells_in_series 36   nama "18" menunjuk modul 18 V nominal (pengisi aki 12 V), itu susunan 36 sel baku;
                         Voc per sel = 22.5 / 36 = 0.625 V, wajar untuk mono-Si
    alpha_sc  +0.05 %/C  koefisien suhu Isc (khas mono-Si)
    beta_voc  -0.30 %/C  koefisien suhu Voc (khas mono-Si)
    NOCT      45 C       suhu sel pada 800 W/m2, udara 20 C, angin 1 m/s
    b0        0.05       koefisien IAM ASHRAE (kaca bersih)
Toleransi pabrik +-3 % berarti arus nyata bisa meleset 3 % dari model ini, SEBELUM memperhitungkan kotor dan penuaan.

Pakai: python panel_listrik.py --selftest
"""
import math
import sys
from pathlib import Path

import numpy as np
import pvlib

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import tracker   # noqa: E402

# --- pelat nama
PMAX, VOC, ISC, VMP, IMP = 50.0, 22.5, 3.0, 18.5, 2.7
LEBAR, PANJANG, LUAS = 0.445, 0.670, 0.670 * 0.445
TOLERANSI = 0.03

# --- asumsi
NS = 36                       # sel seri
ALPHA_SC = 0.0005 * ISC       # A/C  (+0.05 %/C)
BETA_VOC = -0.0030 * VOC      # V/C  (-0.30 %/C)
NOCT = 45.0                   # C
B0_IAM = 0.05                 # ASHRAE

N_IDEAL = 1.1                 # faktor ideal dioda; TIDAK dapat ditentukan dari pelat nama (4 persamaan, 5 anu).
                              # Diperiksa: n = 1.0 sampai 1.5 semuanya mereproduksi pelat nama persis dan
                              # arusnya hanya berbeda 0.3 persen, jadi pilihan ini tidak kritis (lihat selftest).
VT_STC = 1.380649e-23 * 298.15 / 1.602176634e-19   # tegangan termal pada 25 C (V)


def fit_stc(n=N_IDEAL):
    """Lima parameter dioda tunggal pada STC dari tiga titik pelat nama + syarat dP/dV = 0 di MPP.
    pvlib `fit_desoto` tidak konvergen untuk pelat nama ini (koefisien suhu hanya asumsi), jadi dicocokkan di sini
    lalu penerjemahan ke irradiance dan suhu lain diserahkan ke pvlib `calcparams_desoto`."""
    from scipy.optimize import fsolve
    a = n * NS * VT_STC

    def eqs(x):
        IL, I0, Rs, Rsh = x[0], np.exp(np.clip(x[1], -700.0, 50.0)), x[2], x[3]   # clip: fsolve sempat meraba nilai ekstrem
        sisa = lambda V, I: IL - I0 * np.expm1((V + I * Rs) / a) - (V + I * Rs) / Rsh - I
        E = I0 / a * np.exp((VMP + IMP * Rs) / a) + 1 / Rsh      # dI/dV = -E / (1 + Rs E)
        return [sisa(0.0, ISC), sisa(VOC, 0.0), sisa(VMP, IMP), IMP - VMP * E / (1 + Rs * E)]

    for rs0 in (0.1, 0.3, 0.5, 0.8):                             # beberapa tebakan awal: fsolve peka terhadap awalan
        for rsh0 in (80.0, 150.0, 400.0, 1500.0):
            for li0 in (-25, -21, -18):
                x, _, ok, _ = fsolve(eqs, [ISC, li0, rs0, rsh0], full_output=True)
                if ok == 1 and x[2] > 0 and x[3] > 0 and max(abs(np.array(eqs(x)))) < 1e-9:
                    return dict(I_L_ref=x[0], I_o_ref=float(np.exp(x[1])), R_s=x[2], R_sh_ref=x[3], a_ref=a)
    raise RuntimeError("pencocokan dioda tunggal gagal")


#: parameter acuan STC (dihitung sekali saat impor)
_P = fit_stc()


def params(poa, tcell):
    """Parameter dioda tunggal pada irradiance efektif (W/m2) dan suhu sel (C)."""
    return pvlib.pvsystem.calcparams_desoto(
        np.maximum(poa, 1e-6), tcell, alpha_sc=ALPHA_SC, a_ref=_P["a_ref"], I_L_ref=_P["I_L_ref"],
        I_o_ref=_P["I_o_ref"], R_sh_ref=_P["R_sh_ref"], R_s=_P["R_s"])


def listrik(poa, tcell=25.0):
    """Titik kerja panel: dict dengan i_sc, v_oc, i_mp, v_mp, p_mp (A, V, W)."""
    r = pvlib.pvsystem.singlediode(*params(poa, tcell))
    return {k: float(v) if np.isscalar(poa) or np.ndim(poa) == 0 else np.asarray(v) for k, v in r.items()}


def suhu_sel(poa, tamb=30.0, noct=NOCT):
    """Suhu sel (C) model NOCT. tamb bawaan 30 C (Surabaya siang, ASUMSI)."""
    return tamb + (noct - 20.0) / 800.0 * poa


def iam(aoi_deg):
    """Faktor sudut datang (pantulan kaca), ASHRAE. 0 di atas 90 derajat."""
    return np.nan_to_num(pvlib.iam.ashrae(np.asarray(aoi_deg, float), b=B0_IAM), nan=0.0)


def poa_efektif(q1, q2, az, el, dni, dhi, ghi, albedo=0.25):
    """Irradiance efektif di bidang panel (W/m2) dari sudut sendi, dengan IAM pada komponen langsung.
    Model isotropik, sama dengan sun.poa dan solar_tracker/control_web.py."""
    n = tracker.fk(q1, q2)
    s = tracker.sun_vec(az, el)
    cos_aoi = max(0.0, float(n @ s))
    ct = float(n[2])
    aoi = math.degrees(math.acos(min(1.0, cos_aoi)))
    langsung = dni * cos_aoi * float(iam(aoi))
    baur = dhi * (1 + ct) / 2 + ghi * albedo * (1 - ct) / 2    # IAM baur diabaikan (sudut datang efektifnya lebar)
    return langsung + baur


def arus(q1, q2, az, el, dni, dhi, ghi, tamb=30.0, mode="mpp"):
    """Arus panel (A) pada sudut sendi tertentu. mode 'mpp' (pengendali MPPT) atau 'sc' (hubung singkat, shunt)."""
    poa = poa_efektif(q1, q2, az, el, dni, dhi, ghi)
    r = listrik(poa, suhu_sel(poa, tamb))
    return r["i_mp"] if mode == "mpp" else r["i_sc"]


def skenario_uji(tanggal="2026-10-07", el_target=45.0, okta=0.0):
    """Satu titik langit NYATA dari pvlib (az, el, dni, dhi, ghi). Dipakai agar DNI, DHI, dan GHI saling konsisten:
    nilai pilihan tangan mudah melanggar GHI = DNI sin(el) + DHI dan membesar-besarkan suku albedo."""
    import sun
    d = sun.sun_day(tanggal, "5min", okta)
    d = d[d.el > 5]
    k = int(np.argmin(np.abs(d.el.values - el_target)))
    return (90.0, float(d.el.iloc[k]), float(d.dni.iloc[k]), float(d.dhi.iloc[k]), float(d.ghi.iloc[k]))


def selftest():
    # 1) pelat nama direproduksi di STC
    r = listrik(1000.0, 25.0)
    for nama, dapat, harap, tol in (("Isc", r["i_sc"], ISC, 0.01), ("Voc", r["v_oc"], VOC, 0.01),
                                    ("Imp", r["i_mp"], IMP, 0.01), ("Vmp", r["v_mp"], VMP, 0.01),
                                    ("Pmax", r["p_mp"], VMP * IMP, 0.01)):
        assert abs(dapat - harap) / harap < tol, f"{nama}: {dapat:.4f} vs {harap:.4f}"
    # 2) arus hampir sebanding dengan irradiance (dasar pemakaian arus sebagai sensor arah)
    for g in (200.0, 400.0, 600.0, 800.0):
        isc = listrik(g, 25.0)["i_sc"]
        assert abs(isc / (ISC * g / 1000.0) - 1.0) < 0.01, (g, isc)
    # 3) monoton naik terhadap irradiance
    gs = np.arange(50.0, 1100.0, 50.0)
    imp = np.array([listrik(g, 25.0)["i_mp"] for g in gs])
    assert np.all(np.diff(imp) > 0)
    # 4) suhu: Isc naik sedikit, Voc turun, daya turun (tanda koefisien benar)
    hangat = listrik(1000.0, 55.0)
    assert hangat["i_sc"] > r["i_sc"] and hangat["v_oc"] < r["v_oc"] and hangat["p_mp"] < r["p_mp"]
    kehilangan = (r["p_mp"] - hangat["p_mp"]) / r["p_mp"] / 30 * 100
    assert 0.3 < kehilangan < 0.6, f"koefisien daya {kehilangan:.3f} %/C di luar rentang silikon biasa"
    # 5) faktor ideal n tidak dapat ditentukan pelat nama: pastikan pilihannya tidak mengubah arus secara berarti
    sc6, rss = [], []
    for n in (1.0, 1.1, 1.2, 1.3, 1.4, 1.5):
        p = fit_stc(n)
        q = pvlib.pvsystem.singlediode(*pvlib.pvsystem.calcparams_desoto(
            600.0, 25.0, ALPHA_SC, p["a_ref"], p["I_L_ref"], p["I_o_ref"], p["R_sh_ref"], p["R_s"]))
        sc6.append(float(q["i_sc"])); rss.append(p["R_s"])
        r_n = pvlib.pvsystem.singlediode(p["I_L_ref"], p["I_o_ref"], p["R_s"], p["R_sh_ref"], p["a_ref"])
        assert abs(float(r_n["i_mp"]) - IMP) / IMP < 1e-4 and abs(float(r_n["v_mp"]) - VMP) / VMP < 1e-4, n
    spread = (max(sc6) - min(sc6)) / np.mean(sc6) * 100
    rs_min, rs_max = min(rss), max(rss)
    assert spread < 0.5, f"arus terlalu peka terhadap n: {spread:.2f} %"
    # 6) IAM: 1 saat tegak lurus, turun di sudut besar, 0 di 90 derajat
    assert abs(float(iam(0.0)) - 1.0) < 1e-12 and float(iam(60.0)) < 1.0 and float(iam(90.0)) == 0.0
    # 7) arus sebagai sensor arah: maksimum saat panel tepat ke matahari, turun mengikuti cos
    az, el, dni, dhi, ghi = skenario_uji()
    q1, q2 = math.radians(az - math.degrees(tracker.HEAD)), math.radians(90 - el)
    n, s = tracker.fk(q1, q2), tracker.sun_vec(az, el)
    assert abs(float(n @ s) - 1.0) < 1e-9, "pose acuan harus tepat ke matahari"
    i0 = arus(q1, q2, az, el, dni, dhi, ghi)
    for d in (10.0, 20.0, 30.0):
        assert arus(q1, q2 + math.radians(d), az, el, dni, dhi, ghi) < i0, d
        assert arus(q1, q2 - math.radians(d), az, el, dni, dhi, ghi) < i0, -d
    # Puncak arus TIDAK tepat di arah matahari, dan arah pergeserannya bergantung tempat pemasangan:
    # albedo menarik optimum ke arah lebih TEGAK (panel melihat lebih banyak tanah), baur langit menarik ke arah
    # lebih DATAR (panel melihat lebih banyak langit). Keduanya menarik menjauh dari arah matahari, berlawanan arah.
    dd = np.linspace(-15.0, 15.0, 601)
    ii = np.array([arus(q1, q2 + math.radians(d), az, el, dni, dhi, ghi) for d in dd])
    geser = float(dd[int(np.argmax(ii))])
    assert abs(geser) < 6.0, f"pergeseran puncak di luar dugaan: {geser}"
    assert (ii.max() - i0) / i0 < 0.001, "kenaikan di puncak harus sangat kecil"
    # Kendali 1: dengan komponen LANGSUNG saja puncaknya tepat di arah matahari.
    pp = np.array([poa_efektif(q1, q2 + math.radians(d), az, el, dni, 0.0, 0.0, albedo=0.0) for d in dd])
    assert abs(float(dd[int(np.argmax(pp))])) < 0.06, "dengan komponen langsung saja puncak harus di 0 deg"
    # Kendali 2: tanpa albedo, baur langit menarik optimum ke arah lebih datar (geser negatif)
    pb = np.array([poa_efektif(q1, q2 + math.radians(d), az, el, dni, dhi, ghi, albedo=0.0) for d in dd])
    assert float(dd[int(np.argmax(pb))]) < 0.0, "tanpa albedo, baur langit menarik optimum ke arah lebih datar"
    # Kendali 3: menaikkan albedo menggeser optimum ke arah lebih tegak, dan tandanya berbalik
    gser = [float(dd[int(np.argmax([poa_efektif(q1, q2 + math.radians(d), az, el, dni, dhi, ghi, albedo=a)
                                    for d in dd]))]) for a in (0.1, 0.2, 0.25, 0.4)]
    assert gser == sorted(gser) and gser[0] < 0 < gser[-1], gser
    print("selftest panel_listrik: lolos")
    print(f"  dioda tunggal: I_L {_P['I_L_ref']:.4f} A, I_o {_P['I_o_ref']:.3e} A, R_s {_P['R_s']:.4f} ohm, "
          f"R_sh {_P['R_sh_ref']:.1f} ohm, a_ref {_P['a_ref']:.4f} V (n = {N_IDEAL})")
    print(f"  kepekaan terhadap n (1.0 sampai 1.5): Isc pada 600 W/m2 berubah {spread:.2f} %, "
          f"Rs {rs_min:.3f} sampai {rs_max:.3f} ohm")
    print(f"  STC: Isc {r['i_sc']:.3f} A, Voc {r['v_oc']:.2f} V, Imp {r['i_mp']:.3f} A, Vmp {r['v_mp']:.2f} V, Pmax {r['p_mp']:.2f} W")
    print(f"  koefisien daya {-kehilangan:.3f} %/C; efisiensi modul STC {r['p_mp'] / (LUAS * 1000) * 100:.2f} %")
    print(f"  puncak arus bergeser {geser:+.2f} deg dari arah matahari (albedo 0.25, langit pvlib cerah); "
          f"albedo 0.1 sampai 0.4 memberi {gser[0]:+.2f} sampai {gser[-1]:+.2f} deg; "
          f"arus di puncak hanya {(ii.max() / i0 - 1) * 100:+.4f} % di atas arus saat tepat ke matahari")
    print(f"  kendali: hanya komponen langsung -> puncak di {float(dd[int(np.argmax(pp))]):+.2f} deg; tanpa albedo (baur saja) -> {float(dd[int(np.argmax(pb))]):+.2f} deg")


def tabel():
    """Tabel ringkas untuk dokumen."""
    out = ["# Model listrik panel MOS C-50-18-M\n",
           "Dibangkitkan `kontrol_hibrid/panel_listrik.py` (model dioda tunggal De Soto, pvlib "
           f"{pvlib.__version__}; parameter dicocokkan ke pelat nama).\n",
           "## 1. Parameter hasil pencocokan (acuan STC)\n",
           "| Besaran | Nilai |\n|---|---|",
           f"| Arus cahaya I_L | {_P['I_L_ref']:.4f} A |",
           f"| Arus jenuh I_o | {_P['I_o_ref']:.3e} A |",
           f"| Tahanan seri R_s | {_P['R_s']:.4f} ohm |",
           f"| Tahanan bocor R_sh | {_P['R_sh_ref']:.1f} ohm |",
           f"| a_ref (n Ns Vt) | {_P['a_ref']:.4f} V, faktor ideal n = {_P['a_ref'] / (NS * 0.025693):.3f} |",
           f"| Sel seri (ASUMSI) | {NS} |",
           "\n## 2. Reproduksi pelat nama di STC\n",
           "| Besaran | Pelat nama | Model | Selisih |\n|---|---|---|---|"]
    r = listrik(1000.0, 25.0)
    for nama, model, plat, sat in (("Isc", r["i_sc"], ISC, "A"), ("Voc", r["v_oc"], VOC, "V"),
                                   ("Imp", r["i_mp"], IMP, "A"), ("Vmp", r["v_mp"], VMP, "V"),
                                   ("Pmax", r["p_mp"], PMAX, "W")):
        out.append(f"| {nama} | {plat:.2f} {sat} | {model:.3f} {sat} | {(model - plat) / plat * 100:+.2f} % |")
    out += ["\n## 3. Arus terhadap irradiance dan suhu\n",
            "| POA (W/m2) | Suhu sel (C) | Isc (A) | Imp (A) | Vmp (V) | Pmp (W) |\n|---|---|---|---|---|---|"]
    for g in (100, 200, 400, 600, 800, 1000):
        tc = suhu_sel(g)
        q = listrik(float(g), tc)
        out.append(f"| {g} | {tc:.1f} | {q['i_sc']:.3f} | {q['i_mp']:.3f} | {q['v_mp']:.2f} | {q['p_mp']:.2f} |")
    out += ["\nArus hubung singkat sebanding dengan irradiance dalam 1 persen, dan hampir tidak terpengaruh suhu. "
            "Itulah dasar memakai arus sebagai sensor arah: arus mengukur irradiance bidang, dan irradiance bidang "
            "mengukur kosinus galat pointing.\n",
            "\n## 4. Arus sebagai sensor arah\n"]
    az, el, dni, dhi, ghi = skenario_uji()
    out += [f"Langit cerah NYATA dari pvlib (7 Okt 2026, matahari di Timur, elevasi {el:.1f} deg): "
            f"DNI {dni:.0f}, DHI {dhi:.0f}, GHI {ghi:.0f} W/m2, albedo {0.25}. Nilai ini saling konsisten "
            f"(GHI = DNI sin el + DHI); nilai pilihan tangan mudah melanggarnya dan membesar-besarkan suku albedo.\n",
            "Simpangan diukur dari ARAH MATAHARI. Perhatikan puncak arus TIDAK berada tepat di 0 derajat.\n",
            "| Simpangan dari matahari (deg) | POA (W/m2) | Imp (A) | Beda arus |\n|---|---|---|---|"]
    q1, q2 = math.radians(az - math.degrees(tracker.HEAD)), math.radians(90 - el)
    i0 = arus(q1, q2, az, el, dni, dhi, ghi)
    for d in (-20, -10, -5, -2, 0, 1, 2, 2.6, 5, 10, 20, 30):
        qq = q2 + math.radians(d)
        p = poa_efektif(q1, qq, az, el, dni, dhi, ghi)
        i = arus(q1, qq, az, el, dni, dhi, ghi)
        out.append(f"| {d} | {p:.1f} | {i:.4f} | {(i / i0 - 1) * 100:+.3f} % |")
    dd = np.linspace(-15.0, 15.0, 601)
    ii = np.array([arus(q1, q2 + math.radians(d), az, el, dni, dhi, ghi) for d in dd])
    geser = float(dd[int(np.argmax(ii))])
    out += [f"\n**Temuan 1: memaksimalkan arus TIDAK mengarahkan panel ke matahari, dan arah simpangannya bergantung "
            f"tempat pemasangan.** Pada albedo 0.25 puncak arus bergeser {geser:+.2f} derajat, dan arus di puncak hanya "
            f"{(ii.max() / i0 - 1) * 100:+.4f} persen lebih tinggi. Penyebabnya dua komponen sebaran yang menarik ke arah "
            "BERLAWANAN: albedo menarik optimum ke arah lebih tegak (panel melihat lebih banyak tanah), baur langit menarik "
            "ke arah lebih datar (panel melihat lebih banyak langit). Dengan komponen langsung saja puncaknya tepat di "
            "0 derajat (diperiksa di selftest).\n",
            "\n| Albedo tanah | Puncak arus (deg dari matahari) | Contoh permukaan |\n|---|---|---|"]
    for a, ket in ((0.10, "aspal, tanah basah"), (0.20, "rumput, tanah kering"), (0.25, "nilai bawaan model"),
                   (0.40, "beton terang, pasir")):
        g = float(dd[int(np.argmax([arus(q1, q2 + math.radians(d), az, el, dni, dhi, ghi) if a == 0.25 else
                                    poa_efektif(q1, q2 + math.radians(d), az, el, dni, dhi, ghi, albedo=a) for d in dd]))])
        out.append(f"| {a:.2f} | {g:+.2f} | {ket} |")
    out += ["\nTandanya berbalik di sekitar albedo 0.25. Jadi **albedo tempat pemasangan harus diukur** bila arus dipakai "
            "sebagai acuan posisi; tanpa itu, arah simpangannya pun tidak diketahui.\n",
            "\n**Temuan 2: arus buta terhadap galat kecil.** Di dekat optimum arus berubah sebagai kosinus, jadi turunannya "
            "NOL. Simpangan 2 derajat hanya mengubah arus 0.09 persen, jauh di bawah toleransi pabrik 3 persen dan di bawah "
            "derau pengukuran yang wajar. Arus juga tidak memberi ARAH: simpangan +5 dan -5 derajat memberi arus yang mirip. "
            "Arus baru berarti di atas sekitar 10 derajat (turun 0.6 persen) dan jelas di atas 20 derajat (turun 4 persen).\n",
            "\n**Akibatnya untuk rancangan kontrol:** arus dipakai sebagai (a) pemeriksa kasar bahwa penjejakan tidak melenceng "
            "jauh, (b) pembanding antara dua posisi pada pendakian bukit yang lambat, (c) pendeteksi lepas langkah besar. "
            "Koreksi halus dan informasi arah tetap dari LDR. Keduanya saling melengkapi: LDR peka dan berarah tetapi "
            "terpengaruh awan dan kotor, arus lamban dan buta arah tetapi langsung mengukur hasil yang dipanen.\n",
            "\n## 5. Asumsi yang dipakai\n",
            f"- Sel seri {NS} (dari nama modul 18 V nominal; Voc per sel {VOC / NS:.3f} V)\n"
            f"- Koefisien suhu Isc +0.05 %/C, Voc -0.30 %/C (khas mono-Si, TIDAK ada di pelat nama)\n"
            f"- NOCT {NOCT:.0f} C, suhu udara 30 C\n"
            f"- IAM ASHRAE b0 {B0_IAM}\n"
            f"- Toleransi pabrik +-{TOLERANSI * 100:.0f} %, belum termasuk kotor, penuaan, dan ketidakcocokan sel\n"]
    (Path(__file__).parent / "panel_listrik_results.md").write_text("\n".join(out), encoding="utf-8")
    print("tersimpan panel_listrik_results.md")


if __name__ == "__main__":
    selftest()
    if "--selftest" not in sys.argv:
        tabel()
