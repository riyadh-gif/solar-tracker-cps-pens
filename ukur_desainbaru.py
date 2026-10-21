"""Ukur mesh desain baru (dibaca sebagai data, bukan dijalankan). Pakai: python ukur_desainbaru.py <dir_meshes>"""
import sys, math
from pathlib import Path
import numpy as np, trimesh

d = Path(sys.argv[1])
PENTING = ["Solar_Panel_1", "gear1_1", "gear2_1", "teropongLDR_1", "lensa_LDR_1",
           "casing_motor_elevasi_1", "57HSE_3.0N_1", "as_8mm_1", "base_link",
           "ALUMUNIUM_PROFILE_2020_1000_1", "ALUMUNIUM_PROFILE_2020443_1",
           "pangkon_limit_azimuth_1", "SWITCH_LIMIT_30x16x10_1", "stoppper_1", "alas_1"]

print(f"{'mesh':<32}{'dx':>9}{'dy':>9}{'dz':>9}{'volume cm3':>13}{'massa@alu kg':>14}")
tot = {}
for n in PENTING:
    f = d / f"{n}.stl"
    if not f.exists():
        print(f"{n:<32}  (tidak ada)"); continue
    m = trimesh.load(f, force="mesh")
    e = m.extents
    v = abs(m.volume) / 1000.0          # mm3 -> cm3
    tot[n] = (e, v)
    print(f"{n:<32}{e[0]:>9.1f}{e[1]:>9.1f}{e[2]:>9.1f}{v:>13.2f}{v*2.70/1000:>14.3f}")

print("\n--- GEAR: rasio reduksi ---")
g = {}
for n in ("gear1_1", "gear2_1"):
    if n in tot:
        e = tot[n][0]
        dia = sorted(e)[-1]            # diameter = dimensi terbesar pada piringan
        teb = sorted(e)[0]
        g[n] = dia
        print(f"  {n}: diameter {dia:.1f} mm, tebal {teb:.1f} mm")
if len(g) == 2:
    a, b = g["gear1_1"], g["gear2_1"]
    print(f"  RASIO (besar/kecil) = {max(a,b)/min(a,b):.3f} : 1")

print("\n--- TEROPONG LDR: sudut terima ---")
if "teropongLDR_1" in tot:
    e = tot["teropongLDR_1"][0]
    panjang = sorted(e)[-1]; bore = sorted(e)[0]
    print(f"  panjang {panjang:.1f} mm, lebar terkecil {bore:.1f} mm")
    print(f"  setengah sudut terima kasar = atan({bore:.1f}/2 / {panjang:.1f}) = {math.degrees(math.atan((bore/2)/panjang)):.1f} deg")

print("\n--- PANEL ---")
if "Solar_Panel_1" in tot:
    e = sorted(tot["Solar_Panel_1"][0], reverse=True)
    print(f"  {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm   (sekarang di model: 670.0 x 445.0 x 25.0)")

print("\n--- SELURUH RAKITAN (gabungan semua mesh) ---")
allm = []
for f in sorted(d.glob("*.stl")):
    try: allm.append(trimesh.load(f, force="mesh"))
    except Exception: pass
if allm:
    u = trimesh.util.concatenate(allm)
    e = u.extents
    print(f"  kotak batas {e[0]:.0f} x {e[1]:.0f} x {e[2]:.0f} mm")
    print(f"  jumlah mesh {len(allm)}, total segitiga {sum(len(x.faces) for x in allm)}")
