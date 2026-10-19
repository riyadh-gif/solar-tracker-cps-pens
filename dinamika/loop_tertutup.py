"""Dampak mengganti stepper LOOP TERBUKA dengan stepper LOOP TERTUTUP (enkoder di poros rotor).

Kunci fisika, dan ini yang menentukan seluruh hasil: enkoder ada di ROTOR, bukan di beban.

    perintah --(pegas magnet Kh = Th*Nr)-- rotor --(poros k)-- beban (panel)

Loop terbuka : beban merasakan Kh SERI dengan k  -> K_ef = 137.1 (az), 101.7 (el) N.m/rad
Loop tertutup: driver memaksa rotor ke perintah, pegas magnet diperkuat penguatan
               posisi driver Kp. Kp -> tak hingga berarti rotor terpaku dan beban
               hanya merasakan poros k -> K_ef = 1600 (az), 317 (el) N.m/rad

Akibatnya sistem jadi LEBIH KAKU tetapi KURANG TEREDAM, karena zeta = b / (2 sqrt(K J))
turun ketika K naik sedangkan b tetap. Lewatan justru memburuk.

CATATAN Th: pada limit rotor terpaku K_ser -> k, jadi Th HILANG sama sekali dari kekakuan.
Th (3.0 -> 1.5 N.m, 57HSE3.0N -> 57HSE1.5N) hanya menggeser kolom loop TERBUKA dan
langit-langit torsi; kolom loop tertutup tidak bergerak sedikit pun.

Jalankan: ~/.venvs/solar/bin/python dinamika/loop_tertutup.py
Keluaran: dinamika/loop_tertutup_results.md
"""
import math
from pathlib import Path

import numpy as np

TH, NR = 1.5, 50     # 57HSE1.5N (dulu 3.0 N.m; Th hanya memengaruhi kolom loop TERBUKA, lihat catatan)
KH = TH * NR          # 75 N.m/rad, pegas magnet stepper (Th * Nr)
JM = 1.1e-4           # kg.m2 inersia rotor (ASUMSI, NEMA23 3 N.m)
BL = 0.2              # N.m.s/rad redaman viskos beban (ASUMSI, paling menentukan)
STEP16 = 1.8 / 16     # deg per microstep 1/16

AX = {  # nama: (k poros N.m/rad, J beban kg.m2, torsi gravitasi maks N.m)
    "azimuth":    (1600.0, 0.1514, 0.0),
    "kemiringan": (317.0, 0.0543, 1.274),
}


def mode_rendah(k, JL, Kh):
    """Frekuensi mode terendah sistem dua massa (rotor + beban), rad/s."""
    K = np.array([[Kh + k, -k], [-k, k]])
    w2 = np.sort(np.linalg.eigvals(np.linalg.inv(np.diag([JM, JL])) @ K).real)
    return math.sqrt(w2[0])


def ringkas(k, JL, Kh, grav):
    wn = mode_rendah(k, JL, Kh)
    fn = wn / (2 * math.pi)
    K_ef = wn ** 2 * JL                 # kekakuan efektif yang benar-benar dirasakan beban
    zeta = BL / (2 * math.sqrt(K_ef * JL))
    return dict(fn=fn, K_ef=K_ef, zeta=zeta,
                Mp=100 * math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2)),
                tau=2 * JL / BL,                       # peluruhan: TIDAK bergantung kekakuan
                siklus=(2 * JL / BL) * fn,
                defleksi=math.degrees(grav / K_ef) if grav else 0.0)


def zvd(fn, zeta):
    """Tiga impuls ZVD: amplitudo dan waktu."""
    Ks = math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2))
    dt = 1 / (2 * fn * math.sqrt(1 - zeta ** 2))
    return np.array([1, 2 * Ks, Ks ** 2]) / (1 + Ks) ** 2, np.array([0.0, dt, 2 * dt])


def sisa(A, t, fn, zeta):
    """Persen getaran sisa bila shaper dipakai pada sistem berfrekuensi fn."""
    w = 2 * math.pi * fn
    wd = w * math.sqrt(1 - zeta ** 2)
    C = float(np.sum(A * np.exp(zeta * w * t) * np.cos(wd * t)))
    S = float(np.sum(A * np.exp(zeta * w * t) * np.sin(wd * t)))
    return 100 * math.exp(-zeta * w * t[-1]) * math.hypot(C, S)


def md(rows, cols):
    head = "| " + " | ".join(cols) + " |\n|" + "---|" * len(cols) + "\n"
    return head + "".join("| " + " | ".join(str(r[c]) for c in cols) + " |\n" for r in rows)


def main():
    out = ["# Dampak stepper loop tertutup (enkoder di rotor)\n",
           "Dihasilkan oleh `dinamika/loop_tertutup.py`.\n\n"
           "**PERHATIAN, tabel ini belum tervalidasi.** Angka Gazebo yang dulu tervalidasi (azimuth 4,79 Hz "
           "prediksi berbanding 5,19 Hz terukur, lewatan 93,3 % berbanding 90 %) berasal dari motor LAMA "
           "57HSE3.0N pada loop terbuka. Dengan 57HSE1.5N kolom loop terbuka di bawah menjadi 3,46 Hz, jadi "
           "tabel ini TIDAK lagi berjangkar pada pengukuran Gazebo mana pun. Validasi ulang lewat "
           "`rec_step.py` dan `rec_shaped.py` belum dijalankan.\n"]

    # A. perbandingan pokok
    rows, H = [], {}
    for nama, (k, JL, grav) in AX.items():
        H[nama] = (ringkas(k, JL, KH, grav), ringkas(k, JL, 1e9, grav))
        for label, r in zip(("loop terbuka", "loop tertutup ideal"), H[nama]):
            rows.append(dict(sumbu=nama, kasus=label, K_ef=f"{r['K_ef']:.1f}", f_n_Hz=f"{r['fn']:.2f}",
                             zeta=f"{r['zeta']:.4f}", lewatan_pct=f"{r['Mp']:.1f}", tau_s=f"{r['tau']:.2f}",
                             siklus_dering=f"{r['siklus']:.1f}", defleksi_grav_deg=f"{r['defleksi']:.3f}"))
    out += ["\n## A. Loop terbuka berbanding loop tertutup ideal (rotor terpaku)\n",
            md(rows, ["sumbu", "kasus", "K_ef", "f_n_Hz", "zeta", "lewatan_pct", "tau_s", "siklus_dering", "defleksi_grav_deg"]),
            "\nLebih kaku tetapi KURANG teredam. `tau` tidak berubah karena hanya bergantung J dan b, "
            "sehingga dering yang sama panjangnya berisi jauh lebih banyak siklus.\n"]

    # B. penguatan driver berhingga
    rows = []
    for nama, (k, JL, grav) in AX.items():
        for Kp in (0, 150, 500, 1500, 5000, 1e9):
            r = ringkas(k, JL, KH + Kp, grav)
            rows.append(dict(sumbu=nama, Kp=("tak hingga" if Kp > 1e8 else f"{Kp:.0f}"), f_n_Hz=f"{r['fn']:.2f}",
                             zeta=f"{r['zeta']:.4f}", lewatan_pct=f"{r['Mp']:.1f}", defleksi_deg=f"{r['defleksi']:.3f}"))
    out += ["\n## B. Driver nyata: penguatan posisi berhingga\n",
            "`Kp` = kekakuan tambahan yang diberikan loop posisi driver ke rotor, dalam N.m/rad. "
            "`Kp = 0` sama dengan loop terbuka. Nilai sebenarnya bergantung driver dan belum diketahui.\n\n",
            md(rows, ["sumbu", "Kp", "f_n_Hz", "zeta", "lewatan_pct", "defleksi_deg"])]

    # C. ZVD tidak diretune
    rows = []
    for nama in AX:
        buka, tutup = H[nama]
        A_l, t_l = zvd(buka["fn"], buka["zeta"])
        A_b, t_b = zvd(tutup["fn"], tutup["zeta"])
        rows.append(dict(sumbu=nama, f_rancang_Hz=f"{buka['fn']:.2f}", f_nyata_Hz=f"{tutup['fn']:.2f}",
                         tanpa_shaper_pct="100.0", zvd_lama_pct=f"{sisa(A_l, t_l, tutup['fn'], tutup['zeta']):.1f}",
                         zvd_diretune_pct=f"{sisa(A_b, t_b, tutup['fn'], tutup['zeta']):.1f}"))
    out += ["\n## C. ZVD yang TIDAK diretune\n",
            "Shaper masih dirancang untuk frekuensi loop terbuka tetapi dipakai pada sistem loop tertutup.\n\n",
            md(rows, ["sumbu", "f_rancang_Hz", "f_nyata_Hz", "tanpa_shaper_pct", "zvd_lama_pct", "zvd_diretune_pct"]),
            "\nIni temuan paling mendesak: ZVD lama kehilangan sebagian besar kemampuannya, "
            "dan pada sumbu kemiringan hampir tidak berguna lagi.\n"]

    # D. yang enkoder tidak bisa lihat
    rows = []
    w, rho, cn, area, arm = 6.22, 1.2, 1.2, 0.2982, 0.0668      # hembusan terburuk 11 tahun di panel
    tau_w = 0.5 * rho * w ** 2 * cn * area * arm
    for nama, (k, JL, grav) in AX.items():
        if not grav:
            continue
        buka, tutup = H[nama]
        for sumber, tq in (("gravitasi maks", grav), (f"angin terburuk 11 thn ({w} m/s)", tau_w)):
            rows.append(dict(sumbu=nama, sumber=sumber, torsi_Nm=f"{tq:.4f}",
                             puntir_terbuka_deg=f"{math.degrees(tq / buka['K_ef']):.4f}",
                             puntir_tertutup_deg=f"{math.degrees(tq / k):.4f}",
                             rugi_energi_pct=f"{100 * (1 - math.cos(tq / k)):.5f}"))
    out += ["\n## D. Puntiran poros: galat yang enkoder TIDAK bisa lihat\n",
            "Enkoder ada di rotor. Torsi beban memuntir poros antara rotor dan panel, dan puntiran itu "
            "tetap ada walau driver menolkan galat rotor.\n\n",
            md(rows, ["sumbu", "sumber", "torsi_Nm", "puntir_terbuka_deg", "puntir_tertutup_deg", "rugi_energi_pct"])]

    # E. resolusi enkoder berbanding microstep
    rows = []
    for ppr in (1000, 2500, 4000):
        res = 360.0 / (4 * ppr)
        rows.append(dict(enkoder_ppr=ppr, hitungan_per_putaran=4 * ppr, resolusi_deg=f"{res:.4f}",
                         berbanding_microstep_1_16=f"{res / STEP16:.2f} x",
                         lantai_kuantisasi=("enkoder" if res > STEP16 else "microstep")))
    out += ["\n## E. Resolusi enkoder berbanding microstep 1/16\n",
            f"Microstep 1/16 = {STEP16:.4f} deg. Kuantisasi inilah yang membangkitkan riak saat menjejak.\n\n",
            md(rows, ["enkoder_ppr", "hitungan_per_putaran", "resolusi_deg", "berbanding_microstep_1_16", "lantai_kuantisasi"]),
            "\nEnkoder 1000 ppr yang lazim pada driver stepper loop tertutup TIDAK memperhalus riak jejak, "
            "karena resolusinya sebanding dengan microstep. Loop tertutup memperbaiki lepas langkah, "
            "bukan kehalusan jejak.\n"]

    # F. rugi energi dari galat statis sisa
    rows = [dict(galat_deg=f"{d:.2f}", rugi_kosinus_pct=f"{100 * (1 - math.cos(math.radians(d))):.5f}")
            for d in (0.05, 0.10, 0.23, 0.50, 1.00, 2.00, 5.00, 7.20)]
    out += ["\n## F. Rugi energi dari galat arah statis\n",
            md(rows, ["galat_deg", "rugi_kosinus_pct"]),
            "\nPembanding: suhu sel 55 C = 14,9 %, toleransi pabrik 3 %, ayunan motor saat menjejak 0,007 %.\n"]

    p = Path(__file__).resolve().parent / "loop_tertutup_results.md"
    p.write_text("".join(out), encoding="utf-8")
    print(f"ditulis: {p}")
    print("".join(out))


if __name__ == "__main__":
    main()
