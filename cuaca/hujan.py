"""Hujan NYATA di titik PENS dari Open-Meteo, dan kebijakan PARKIR DATAR saat hujan.

Sumber sama dengan angin (angin/angin_openmeteo.py): Open-Meteo Archive API, reanalisis per jam,
2015 sampai 2026. WSL tidak punya internet, jadi JSON diunduh lewat PowerShell di Windows lalu
digabung di sini.

KENAPA PARKIR DATAR, dan ini terukur bukan diasumsikan:
Secara optik hujan itu mendung tebal. Pada okta 8 komponen langsung (DNI) runtuh ke sekitar 4 W/m2,
jadi hampir seluruh cahaya menjadi baur. Panel datar melihat SELURUH kubah langit, panel miring
melihat lebih sedikit. Energi sehari (7 Okt 2026, kWh/m2 bidang panel):

    okta      menjejak   datar    datar/menjejak
      0         7.70      7.25         94.1 %
      4         7.09      6.73         94.9 %
      6         5.29      5.20         98.3 %
      8         1.77      1.81        102.3 %   <- saat hujan

Jadi parkir datar saat hujan TIDAK merugikan panen, malah menang tipis. Alasan lain parkir datar
(tidak dihitung di sini, tetapi masuk akal): air mengalir keluar, beban angin pada bidang datar
lebih kecil, dan tetesan tidak menggenang di tepi bawah panel.

Jalankan sekali untuk membangun CSV:  python cuaca/hujan.py --impor
Ringkasan:                            python cuaca/hujan.py
"""
import csv
import datetime as dt
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CSV = HERE / "hujan_pens_2015-2026.csv"

# Ambang kebijakan. ASUMSI, menunggu sensor hujan sebenarnya di alat.
AMBANG_MM = 0.2        # mm/jam; di atas ini dianggap hujan dan tracker parkir
TAHAN_JAM = 1.0        # jam; tetap parkir selama ini setelah hujan berhenti (menghindari kedip)
Q2_PARKIR = 0.0        # deg, kemiringan saat parkir: DATAR


def impor(src=None):
    """Gabungkan part*.json hasil unduhan menjadi satu CSV."""
    src = Path(src) if src else HERE
    baris = {}
    for f in sorted(src.glob("part*.json")):
        d = json.loads(f.read_text(encoding="utf-8"))["hourly"]
        for t, p, r, rh in zip(d["time"], d["precipitation"], d["rain"], d["relative_humidity_2m"]):
            if p is None:
                continue
            baris[t] = (float(p), float(r if r is not None else p), float(rh if rh is not None else 0.0))
    with CSV.open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["waktu", "presipitasi_mm", "hujan_mm", "rh_pct"])
        for t in sorted(baris):
            p, r, rh = baris[t]
            w.writerow([t, f"{p:.2f}", f"{r:.2f}", f"{rh:.0f}"])
    print(f"ditulis {CSV}: {len(baris)} jam")
    return len(baris)


_CACHE = None


def muat():
    """[(datetime, presipitasi_mm, hujan_mm, rh_pct)] terurut waktu."""
    global _CACHE
    if _CACHE is None:
        if not CSV.exists():
            raise SystemExit(f"{CSV} belum ada. Jalankan: python cuaca/hujan.py --impor")
        out = []
        with CSV.open(encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                out.append((dt.datetime.fromisoformat(row["waktu"]), float(row["presipitasi_mm"]),
                            float(row["hujan_mm"]), float(row["rh_pct"])))
        _CACHE = out
    return _CACHE


def hari(tanggal):
    """Dua puluh empat baris untuk satu tanggal ISO."""
    d = dt.date.fromisoformat(tanggal)
    return [r for r in muat() if r[0].date() == d]


def mm_pada(tanggal, jam):
    """Presipitasi (mm/jam) pada jam desimal, interpolasi tangga (nilai jam berjalan)."""
    rows = hari(tanggal)
    if not rows:
        return 0.0
    i = max(0, min(len(rows) - 1, int(jam)))
    return rows[i][1]


def parkir(tanggal, jam, ambang=AMBANG_MM, tahan=TAHAN_JAM):
    """True bila tracker harus PARKIR DATAR: hujan sekarang, atau baru berhenti dalam `tahan` jam."""
    rows = hari(tanggal)
    if not rows:
        return False
    i = max(0, min(len(rows) - 1, int(jam)))
    for k in range(i, max(-1, i - int(math.ceil(tahan)) - 1), -1):
        if rows[k][1] >= ambang:
            return True
    return False


def ringkas():
    r = muat()
    mm = [x[1] for x in r]
    basah = [x for x in mm if x >= AMBANG_MM]
    siang = [x for x in r if 6 <= x[0].hour <= 18]
    basah_siang = [x for x in siang if x[1] >= AMBANG_MM]
    tahun = sorted({x[0].year for x in r})
    print(f"jam total        : {len(r)} ({r[0][0].date()} sampai {r[-1][0].date()})")
    print(f"jam hujan        : {len(basah)} ({100*len(basah)/len(r):.1f} %) pada ambang {AMBANG_MM} mm/jam")
    print(f"jam hujan SIANG  : {len(basah_siang)} dari {len(siang)} ({100*len(basah_siang)/len(siang):.1f} %)")
    print(f"presipitasi maks : {max(mm):.1f} mm/jam")
    print(f"hujan per tahun  : {sum(mm)/len(tahun):.0f} mm")
    print()
    print("jam hujan siang per bulan (rata-rata semua tahun):")
    per = {}
    for x in siang:
        per.setdefault(x[0].month, [0, 0])
        per[x[0].month][1] += 1
        if x[1] >= AMBANG_MM:
            per[x[0].month][0] += 1
    NAMA = "Jan Feb Mar Apr Mei Jun Jul Agu Sep Okt Nov Des".split()
    for m in range(1, 13):
        a, b = per.get(m, (0, 1))
        bar = "#" * int(40 * a / max(1, b))
        print(f"  {NAMA[m-1]}  {100*a/max(1,b):5.1f} %  {bar}")


if __name__ == "__main__":
    if "--impor" in sys.argv:
        impor()
    ringkas()
