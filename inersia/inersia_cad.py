"""Massa, pusat massa, inersia tiap link dari mesh CAD (parts_v2). Jalankan dari folder inersia/ (atau mana saja).
Keluaran: inersia_link.json, inersia_results.md. Semua massa non-profil adalah ASUMSI (lihat SPEC)."""
import sys, json
from pathlib import Path
import numpy as np
import trimesh
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import build_model as bm  # load, link_mesh, GROUPS, P_AZ, P_EL, TILT, PANEL, EL_LIMIT

OUT = Path(__file__).resolve().parent
G, RHO_AL, RHO_ST = 10.0, 2700.0, 7850.0   # G = 10 m/s2 (KEPUTUSAN pengguna; sama dengan <gravity> di world.sdf)
MOTOR_KG, PANEL_KG = 1.2, 2.55   # motor ASUMSI; panel 2.55 kg dari spesifikasi MOS C-50-18-M
P_AZ, P_EL, PANEL = bm.P_AZ, bm.P_EL, bm.PANEL
LINK_OF = {i: n for n, g in bm.GROUPS.items() for i in g}
# bagian: (jenis, fill atau massa, kerapatan bahan, sumber, catatan asumsi satu baris)
# jenis: solid = volume mesh watertight x rho; hull = volume hull x fill x rho; motor = massa tetap, seragam di hull
SPEC = {i: ("solid", 1, RHO_AL, "mesh", "profil 2020 aluminium 2700, volume mesh watertight") for i in (0, 1, 2, 3, 4, 5, 6, 7, 20, 21)}
SPEC.update({
    8: ("hull", 0.15, RHO_AL, "asumsi", "mesh tidak watertight; tiang = profil 2020 miring, diagonal ~130 mm x 176 mm2 = ~15% volume hull, Al 2700"),
    9: ("hull", 0.15, RHO_AL, "asumsi", "sama dengan bagian 08"),
    10: ("motor", MOTOR_KG, 0, "asumsi", "motor 57HSE3.0N (NEMA23) 1.2 kg asumsi, seragam di volume hull"),
    11: ("solid", 1, RHO_ST, "asumsi", "komponen kecil (0.3 cm3), diasumsikan baja 7850 (pengencang), ~2.5 g"),
    12: ("solid", 1, RHO_AL, "asumsi", "pelat/braket 5.4 cm3 solid, diasumsikan Al 2700"),
    13: ("hull", 0.30, RHO_AL, "asumsi", "mesh tidak watertight; rumah pillow block, isi 30% hull, Al 2700 (jika besi cor massa ~2.6x lebih besar)"),
    14: ("solid", 1, RHO_ST, "asumsi", "komponen kecil (0.23 cm3), diasumsikan baja 7850 (kolar/baut), ~1.8 g"),
    15: ("solid", 1, RHO_ST, "asumsi", "sama dengan bagian 14"),
    16: ("solid", 1, RHO_AL, "asumsi", "dudukan poros SK8, volume mesh solid x Al 2700 (rongga baut tidak dimodelkan)"),
    17: ("solid", 1, RHO_AL, "asumsi", "sama dengan bagian 16"),
    18: ("motor", MOTOR_KG, 0, "asumsi", "motor 57HSE3.0N (NEMA23) 1.2 kg asumsi, seragam di volume hull"),
    19: ("solid", 1, RHO_ST, "mesh", "poros baja 8 mm, volume mesh x 7850"),
})
ORIGIN = {"base": np.zeros(3), "azimuth_link": P_AZ, "elevation_link": P_EL}
ZERO = np.zeros(3)


class Body:  # massa m, pusat massa c, tensor inersia I terhadap COM, semua dalam satu frame
    def __init__(s, m, c, I): s.m, s.c, s.I = float(m), np.asarray(c, float), np.asarray(I, float)

    def about(s, p):  # tensor inersia terhadap titik p (sumbu sejajar frame)
        r = s.c - p
        return s.I + s.m * (r @ r * np.eye(3) - np.outer(r, r))

    def axis(s, p, n):  # momen inersia terhadap sumbu lewat p arah n
        n = np.asarray(n, float) / np.linalg.norm(n)
        return float(n @ s.about(np.asarray(p, float)) @ n)

    def moved(s, R=np.eye(3), t=0.0):  # rotasi R (sekitar origin) lalu translasi t
        return Body(s.m, R @ s.c + t, R @ s.I @ R.T)


def combine(bs):
    m = sum(b.m for b in bs); c = sum(b.m * b.c for b in bs) / m
    return Body(m, c, sum(b.about(c) for b in bs))


def box(m, size, center):
    a, b, c = size
    return Body(m, center, m / 12 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b]))


def part_mesh(i):  # mesh bagian i di frame linknya (sama dengan build_model.link_mesh)
    m, link = bm.place(i, LINK_OF[i]), LINK_OF[i]
    m.apply_translation(-ORIGIN[link])
    return m


def part_body(i, motor_kg):
    kind, k, rho, *_ = SPEC[i]
    m = part_mesh(i)
    if kind == "solid":
        assert m.is_watertight, i
        vol, mm = abs(m.volume), m
    else:
        mm = m.convex_hull; vol = mm.volume
    mm.density = {"solid": rho, "hull": k * rho, "motor": motor_kg / vol}[kind]
    mp = mm.mass_properties
    return Body(mp.mass, mp.center_mass, mp.inertia), vol


def Rx(a):  # sumbu engsel elevasi = sumbu x alat (searah rel), sama dengan axis elevation_joint di URDF
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def panel_center():  # sama dengan build_model.main
    top = trimesh.util.concatenate([bm.load(20), bm.load(21)])
    top.apply_transform(trimesh.transformations.rotation_matrix(bm.TILT, [0, 1, 0], P_EL)); top.apply_translation(-P_EL)
    return np.array([top.bounds[:, 0].mean(), top.bounds[:, 1].mean(), top.bounds[1, 2] + PANEL[2] / 2])


def build(motor_kg=MOTOR_KG, panel_kg=PANEL_KG):
    parts = {i: part_body(i, motor_kg) for i in SPEC}
    links = {n: combine([parts[i][0] for i in bm.GROUPS[n]]) for n in ORIGIN}
    links["panel"] = box(panel_kg, PANEL, panel_center())  # frame panel = frame elevasi
    return parts, links


def j_az(links, q2):
    """Inersia terhadap sumbu z dunia (x=y=0) dari azimuth_link + elevation_link + panel; q2 = rotasi +y sekitar P_EL."""
    R = Rx(q2)
    el = combine([links["elevation_link"], links["panel"]])  # di frame elevasi
    rot = combine([links["azimuth_link"].moved(t=P_AZ), el.moved(R, P_EL)])
    return rot.axis(ZERO, [0, 0, 1]), rot.m


def run(motor_kg=MOTOR_KG, panel_kg=PANEL_KG):
    parts, links = build(motor_kg, panel_kg)
    (J0, M), (J45, _) = j_az(links, 0.0), j_az(links, np.radians(45))
    el = combine([links["elevation_link"], links["panel"]])
    r = el.c  # lengan terhadap sumbu elevasi (sumbu x lewat P_EL, frame elevasi berpusat di P_EL)
    th = np.linspace(-bm.EL_LIMIT, bm.EL_LIMIT, 2001)
    Mg = el.m * G * (r[1] * np.cos(th) - r[2] * np.sin(th))
    return dict(parts=parts, links=links, J_az0=J0, J_az45=J45, J_el=el.axis(ZERO, [1, 0, 0]), el=el, r=r,
                arm_xz=float(np.hypot(r[1], r[2])), Mg_max=float(np.abs(Mg).max()), Mg_q0=float(abs(el.m * G * r[1])),
                Mg_bound=float(el.m * G * np.hypot(r[1], r[2])), M_rot=M)


def selfcheck(res):
    # 1) konvensi trimesh: inersia kotak homogen terhadap COM
    b = trimesh.creation.box((0.1, 0.2, 0.3)); b.density = 1000
    m = 1000 * 0.006
    assert np.allclose(np.diag(b.mass_properties.inertia), m / 12 * np.array([0.13, 0.10, 0.05]), rtol=1e-6)
    # 2) mesh per bagian sama dengan build_model.link_mesh
    for n, o in ORIGIN.items():
        c = trimesh.util.concatenate([part_mesh(i) for i in bm.GROUPS[n]])
        assert np.allclose(c.bounds, bm.link_mesh(n, o).bounds, atol=1e-9), n
    # 3) massa dan inersia sumbu link = jumlah bagian (teorema sumbu sejajar)
    for n in ORIGIN:
        bs = [res["parts"][i][0] for i in bm.GROUPS[n]]
        assert abs(res["links"][n].m - sum(x.m for x in bs)) < 1e-9
        assert abs(res["links"][n].axis(ZERO, [0, 0, 1]) - sum(x.axis(ZERO, [0, 0, 1]) for x in bs)) < 1e-12
    # 4) J_az(45) dari jumlah per bagian sama dengan hasil gabungan
    R = Rx(np.radians(45)); tot = 0.0
    for n in ("azimuth_link", "elevation_link"):
        for i in bm.GROUPS[n]:
            bd = res["parts"][i][0]
            tot += (bd.moved(t=P_AZ) if n == "azimuth_link" else bd.moved(R, P_EL)).axis(ZERO, [0, 0, 1])
    tot += res["links"]["panel"].moved(R, P_EL).axis(ZERO, [0, 0, 1])
    assert abs(tot - res["J_az45"]) < 1e-10
    # 5) panel default dan massa putar masuk akal
    assert abs(res["links"]["panel"].m - PANEL_KG) < 1e-12 and 2.5 < res["M_rot"] < 5.0
    # 6) J_el invarian terhadap rotasi sekitar x
    el = res["el"]; q = np.radians(33)
    assert abs(el.moved(Rx(q)).axis(ZERO, [1, 0, 0]) - res["J_el"]) < 1e-12


def f(x, n=4): return float(f"{x:.{n}g}")


def v3(a): return [f(x, 5) for x in a]


def t6(I): return {k: f(I[a, b], 5) for k, (a, b) in dict(ixx=(0, 0), iyy=(1, 1), izz=(2, 2), ixy=(0, 1), ixz=(0, 2), iyz=(1, 2)).items()}


def main():
    res = run(); selfcheck(res)
    links, parts, el = res["links"], res["parts"], res["el"]
    sens = {}
    for mk, pk in [(MOTOR_KG, 1.0), (MOTOR_KG, 1.5), (MOTOR_KG, 3.0), (0.9, 1.5), (1.6, 1.5), (0.9, 1.0), (1.6, 3.0)]:
        r = run(mk, pk)
        sens[f"motor{mk}_panel{pk}"] = dict(motor_kg=mk, panel_kg=pk, J_az_q0=f(r["J_az0"]), J_az_q45=f(r["J_az45"]), J_el=f(r["J_el"]),
                                            lengan_xz_mm=f(r["arm_xz"] * 1e3), Mg_max_Nm=f(r["Mg_max"]), M_rot_kg=f(r["M_rot"]))
    d, Gs = 0.008, 79e9
    L = float(bm.load(19).extents[1])  # poros sejajar y, tak berubah oleh rotasi sekitar y
    kt = Gs * np.pi * d ** 4 / 32 / L
    jl = {}
    for n in ("base", "azimuth_link", "elevation_link", "panel"):
        b = links[n]; ax = {"base": "z", "azimuth_link": "z", "elevation_link": "x", "panel": "x"}[n]
        jl[n] = dict(massa_kg=f(b.m), com_m=v3(b.c), inersia_com_kgm2=t6(b.I), inersia_origin_link_kgm2=t6(b.about(ZERO)),
                     sumbu_sendi=ax, J_sumbu_sendi_kgm2=f(b.axis(ZERO, [0, 0, 1] if ax == "z" else [1, 0, 0])),
                     sumber="asumsi (panel 1.5 kg, kotak seragam)" if n == "panel" else "campuran: profil dan poros dari mesh, sisanya asumsi")
    jp = {f"{i:02d}": dict(link=LINK_OF[i], massa_kg=f(parts[i][0].m), volume_cm3=f(parts[i][1] * 1e6), com_m=v3(parts[i][0].c),
                           metode=SPEC[i][0], fill=SPEC[i][1] if SPEC[i][0] == "hull" else None, sumber=SPEC[i][3], catatan=SPEC[i][4]) for i in SPEC}
    dyn = dict(J_az_q0_kgm2=f(res["J_az0"]), J_az_q45_kgm2=f(res["J_az45"]), J_el_kgm2=f(res["J_el"]),
               com_elevasi_plus_panel_m=v3(el.c), lengan_y_mm=f(res["r"][1] * 1e3), lengan_z_mm=f(res["r"][2] * 1e3), lengan_xz_mm=f(res["arm_xz"] * 1e3),
               Mg_max_dalam_batas_45deg_Nm=f(res["Mg_max"]), Mg_pada_q0_Nm=f(res["Mg_q0"]), Mg_batas_atas_Nm=f(res["Mg_bound"]),
               massa_elevasi_plus_panel_kg=f(el.m), massa_total_berputar_kg=f(res["M_rot"]), sumber="asumsi dominan (motor 1.2 kg, panel 1.5 kg, fill hull)")
    tors = dict(G_Pa=Gs, d_m=d, L_m=f(L), J_polar_m4=f(np.pi * d ** 4 / 32), kt_Nm_per_rad=f(kt),
                catatan="angka referensi dari geometri poros, BUKAN kekakuan sendi sebenarnya")
    (OUT / "inersia_link.json").write_text(json.dumps(dict(link=jl, bagian=jp, dinamika=dyn, puntir_poros8=tors, sensitivitas=sens), indent=1, ensure_ascii=False), encoding="utf-8")

    o = ["# Massa dan inersia link dari CAD\n\nDibangkitkan oleh `inersia_cad.py`. Frame link = frame sendi (base di 0; azimuth_link di P_AZ = (0,0,0.165); elevation_link di P_EL = (0,0,0.2045), mesh sudah diputar balik 21.24 derajat). Sumbu azimuth = z (x=y=0), sumbu elevasi = y lewat P_EL. Satuan SI.\n",
         "## 1. Massa per bagian\n\n| Bagian | Link | Metode | Volume (cm3) | Massa (g) | Sumber | Dasar |\n|---|---|---|---|---|---|---|"]
    for i in SPEC:
        k, fl = SPEC[i][:2]
        o.append(f"| {i:02d} | {LINK_OF[i]} | {k}{f' fill {fl}' if k == 'hull' else ''} | {parts[i][1] * 1e6:.2f} | {parts[i][0].m * 1e3:.1f} | {SPEC[i][3]} | {SPEC[i][4]} |")
    o += [f"| panel | panel | kotak 0.44x0.30x0.02 | {np.prod(PANEL) * 1e6:.0f} | {PANEL_KG * 1e3:.0f} | asumsi | panel 1.5 kg seragam (sensitivitas 1 dan 3 kg) |\n",
          "Keterangan: \"mesh\" = massa dari volume mesh watertight x kerapatan bahan yang diketahui (profil Al 2700, poros baja 7850). \"asumsi\" = ada angka yang ditebak (massa motor, fill hull, bahan komponen kecil). Hasil total sangat bergantung pada massa motor dan panel.\n",
          "## 2. Per link (frame link)\n\n| Link | Massa (kg) | COM (mm) | Ixx, Iyy, Izz di COM (kg m2) | J sumbu sendi (kg m2) |\n|---|---|---|---|---|"]
    for n, v in jl.items():
        I = v["inersia_com_kgm2"]
        o.append(f"| {n} | {v['massa_kg']} | {np.round(np.array(v['com_m']) * 1e3, 1).tolist()} | {I['ixx']:.3e}, {I['iyy']:.3e}, {I['izz']:.3e} | {v['J_sumbu_sendi_kgm2']:.3e} (sumbu {v['sumbu_sendi']}) |")
    r = res["r"]
    o += ["\nTensor lengkap (termasuk produk inersia dan tensor terhadap origin link) ada di `inersia_link.json`. Panel berada di frame elevasi (link panel di URDF tanpa origin sendi), jadi COM panel relatif terhadap P_EL.\n",
          "## 3. Besaran dinamika (panel 1.5 kg, motor 1.2 kg)\n",
          f"- J_az (azimuth_link + elevation_link + panel, sumbu z), q2 = 0: **{res['J_az0']:.4e} kg m2**",
          f"- J_az pada q2 = 45 deg: **{res['J_az45']:.4e} kg m2** (selisih {100 * (res['J_az45'] / res['J_az0'] - 1):+.1f}%)",
          f"- J_el (elevation_link + panel, sumbu x lewat P_EL): **{res['J_el']:.4e} kg m2**",
          f"- COM elevation_link + panel di frame elevasi: {np.round(el.c * 1e3, 1).tolist()} mm; lengan ketidakseimbangan: y = {r[1] * 1e3:.1f} mm, z = {r[2] * 1e3:.1f} mm, radial yz = **{res['arm_xz'] * 1e3:.1f} mm**",
          f"- Momen gravitasi terhadap sumbu elevasi: M(q) = m g (y cos q - z sin q). Maksimum dalam batas +-45 deg = **{res['Mg_max']:.3f} N.m** (q = 0: {res['Mg_q0']:.3f} N.m; batas atas tanpa batas sudut {res['Mg_bound']:.3f} N.m). Massa elevasi + panel = {el.m:.3f} kg. Bandingkan dengan torsi motor 3 N.m.",
          f"- Massa total bagian yang berputar (azimuth + elevasi + panel): **{res['M_rot']:.3f} kg**\n",
          "## 4. Kekakuan puntir poros baja 8 mm (bagian 19)\n",
          f"k = G J / L, G = 79 GPa, d = 8 mm, J = pi d^4 / 32 = {np.pi * d ** 4 / 32:.3e} m4, L = {L * 1e3:.1f} mm (panjang bounding box mesh bagian 19 sepanjang y) menghasilkan **{kt:.0f} N.m/rad**. Ini hanya angka referensi geometri poros, bukan kekakuan sendi sebenarnya (sendi nyata dipengaruhi kopling, motor, kelonggaran).\n",
          "## 5. Sensitivitas\n\n| Motor (kg) | Panel (kg) | J_az q0 | J_az q45 | J_el | Lengan xz (mm) | M gravitasi maks (N.m) | Massa berputar (kg) |\n|---|---|---|---|---|---|---|---|"]
    for v in sens.values():
        o.append(f"| {v['motor_kg']} | {v['panel_kg']} | {v['J_az_q0']:.3e} | {v['J_az_q45']:.3e} | {v['J_el']:.3e} | {v['lengan_xz_mm']} | {v['Mg_max_Nm']} | {v['M_rot_kg']} |")
    o.append("\n## 6. Usulan blok URDF <inertial> (default; belum dipasang di URDF)\n\nOrigin = COM dalam frame link. Untuk `panel`, frame link = frame elevasi.\n\n```xml")
    for n, nm in (("base", "base_link"), ("azimuth_link", "azimuth_link"), ("elevation_link", "elevation_link"), ("panel", "panel")):
        b, I = links[n], links[n].I
        o.append(f"<!-- {nm} -->\n<inertial><origin xyz=\"{b.c[0]:.5f} {b.c[1]:.5f} {b.c[2]:.5f}\" rpy=\"0 0 0\"/><mass value=\"{b.m:.4f}\"/>\n  <inertia ixx=\"{I[0, 0]:.5e}\" iyy=\"{I[1, 1]:.5e}\" izz=\"{I[2, 2]:.5e}\" ixy=\"{I[0, 1]:.5e}\" ixz=\"{I[0, 2]:.5e}\" iyz=\"{I[1, 2]:.5e}\"/></inertial>")
    o += ["```\n", "## 7. Asumsi dan keterbatasan\n",
          "- Massa motor 1.2 kg (sensitivitas 0.9 sampai 1.6 kg) dan panel 1.5 kg (1 sampai 3 kg) adalah tebakan; motor adalah massa dominan.",
          "- Bagian tidak watertight (08, 09, 13) memakai volume convex hull x fill x kerapatan Al; fill 0.15 dan 0.30 adalah perkiraan teknik, bukan hasil ukur. Inersia dihitung seolah seragam di dalam hull.",
          "- Komponen kecil (11, 14, 15) diasumsikan baja; bagian 12, 16, 17 solid Al. Massanya kecil.",
          "- Tidak dimodelkan: kabel, baut, mur, sensor LDR, bearing sebenarnya, rongga baut SK8.",
          "- Profil dan poros memakai volume mesh x kerapatan nominal; mesh dianggap benar ukurannya.",
          "- Panel diperlakukan sebagai kotak seragam berpusat di frame elevasi sesuai build_model.",
          "- Base tidak berputar, inersianya tidak masuk J_az atau J_el.",
          "- Belum ada verifikasi dengan penimbangan atau pengukuran nyata.\n"]
    (OUT / "inersia_results.md").write_text("\n".join(o), encoding="utf-8")
    print(json.dumps(dyn, indent=1, ensure_ascii=False)); print(tors)
    for n in jl: print(n, jl[n]["massa_kg"], jl[n]["com_m"], jl[n]["J_sumbu_sendi_kgm2"])
    for k, v in sens.items(): print(k, v)


if __name__ == "__main__":
    main()
