"""Bangun URDF dari mesh DESAIN BARU (desainbaru/), menggantikan ekspor pabrikan yang rusak.

Ekspor asli (`desainbaru/extracted/(Unsaved)_description/`) tidak terpakai: hanya 1 link, 0 joint,
nama proyek "(Unsaved)". Skrip ini membaca 43 STL-nya dan menyusun ulang rantai kinematiknya.

KERANGKA CAD: sumbu Z menunjuk KE BAWAH (panel di cz = -325, lensa LDR di cz = -360, rangka dasar di
cz = 0). Jadi dipakai putaran 180 derajat terhadap X: (x, y, z) -> (x, -y, -z), sehingga +Z menjadi atas
seperti konvensi ROS dan seperti model lama.

SUMBU, diturunkan dari geometri, bukan ditebak:
  azimuth  : motor 57HSE_3.0N_1 tegak di (0, 0), panjangnya searah Z  -> sumbu Z
  kemiringan: poros as_8mm (194 mm) searah X di cy = 0, cz = -292.1   -> sumbu X, searah rel panjang
              (sama dengan konvensi model lama: panel berguling kanan-kiri, tidak jungkat-jungkit)

REDUKSI GIGI, temuan terbesar desain ini. Model lama memakai penggerak langsung tanpa gearbox.
  gear1 di poros motor elevasi (cz = -241.9), gear2 di poros engsel (cz = -292.1).
  RASIO = 30/6 = 5.00 : 1, DIUKUR PENGGUNA di alat (5 putaran pinion = 1 putaran gigi besar),
  dengan diameter luar 22 dan 88 mm. Modul 2.75: OD = m(z+2) memberi 2.75*8 = 22 dan 2.75*32 = 88;
  diameter jarak bagi 16.5 dan 82.5 (rasio tepat 5). Jarak sumbu 49.5 mm, terukur 50.2 di CAD.
  CATATAN: rasio 5.25 (modul 2, z 8/42) yang sempat diturunkan dari mesh SALAH, ia meramal pinion
  20 mm padahal yang terpasang 22 mm. Dugaan "gigi sektor" juga salah: gigi besar LINGKARAN PENUH,
  terbukti karena pinion bisa berputar 5 kali penuh terhadapnya.

Jalankan: ~/.venvs/solar/bin/python build_model_v2.py
Keluaran: solar_tracker_v2/{solar_tracker_v2.urdf, meshes/*.stl}
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "desainbaru/extracted/(Unsaved)_description/meshes"
OUT = ROOT / "solar_tracker_v2"
MESH_URI = "package://solar_tracker_v2/meshes"
sys.path.insert(0, str(ROOT / "dinamika"))
import dyn_model as D   # noqa: E402  (satu sumber konstanta stepper)

GEAR_N = D.AXES["elevasi"].N   # 5.25 : 1 pada KEMIRINGAN; satu sumber di dyn_model.AXES_V2
PANEL_KG = 2.55          # pelat nama MOS C-50-18-M
MOTOR_KG = 1.2           # ASUMSI, dibawa dari model lama. 57HSE1.5N kemungkinan lebih ringan, perlu ditimbang.
JOINT_DAMP, JOINT_FRIC = 0.2, 0.05   # ASUMSI, sama dengan model lama
EL_LIMIT = math.radians(45)          # ASUMSI, belum diukur
AZ_LIMIT = math.radians(185)         # BARU: azimuth tidak lagi kontinu (ada pangkon_limit + 4 saklar).
#                                      ASUMSI, menunggu angka asli dari CAD. 185 dipilih karena matahari PENS
#                                      sepanjang 2026 menuntut q1 dari -180.9 sampai +174.6 deg. Nilai +-170
#                                      yang dipakai semula membuat tracker MENTOK dan berhenti mengikuti
#                                      matahari (terbukti di Gazebo: sendi terkunci -170 saat perintah -178.9),
#                                      dan +-180 pun masih kurang tipis.

MAX_TRI = 40000        # batas segitiga mesh TAMPILAN; inersia tetap dari mesh penuh
RHO = {"baja": 7.85e-3, "alu": 2.70e-3, "plastik": 1.20e-3}   # kg/cm3

BAJA = ("as_8mm", "pillow_block", "SK8", "SHF8", "gear1", "gear2", "SWITCH_LIMIT")
PLASTIK = ("lensa_LDR", "teropongLDR", "pangkon_LDR")

LINK = {
    "base": ["ALUMUNIUM_PROFILE_2020_1000_1", "ALUMUNIUM_PROFILE_2020_1000_2", "base_link",
             "ALUMUNIUM_PROFILE_2020_70_1", "ALUMUNIUM_PROFILE_2020_70_2",
             "SWITCH_LIMIT_30x16x10_3", "SWITCH_LIMIT_30x16x10_4", "stoppper_1", "Part18_1",
             "casing_motor_elevasi_1", "57HSE_3.0N_1", "alas_motor_1", "SHF8_Shaft_support_1", "alas_1"],
    "azimuth_link": ["pangkon_limit_azimuth_1", "SWITCH_LIMIT_30x16x10_1", "SWITCH_LIMIT_30x16x10_2",
                     "57HSE_3.0N_2", "gear1_1", "pangkon_stepper3_1",
                     "pillow_block-KFL08_1", "pillow_block-KFL08_2"],
    "elevation_link": ["gear2_1", "as_8mm_1", "SK8_Linear_Shaft_Support_v1_1", "SK8_Linear_Shaft_Support_v1_2",
                       "ALUMUNIUM_PROFILE_2020443_1", "ALUMUNIUM_PROFILE_2020443_2",
                       "ALUMUNIUM_PROFILE_2020_160_1", "ALUMUNIUM_PROFILE_2020_160_2",
                       "Solar_Panel_1",
                       "pangkon_LDR_1", "pangkon_LDR_2", "pangkon_LDR_3", "pangkon_LDR_4",
                       "teropongLDR_1", "teropongLDR_2", "teropongLDR_3", "teropongLDR_4",
                       "lensa_LDR_1", "lensa_LDR_2", "lensa_LDR_3", "lensa_LDR_4"],
}


def ke_ros(m):
    """CAD (Z ke bawah) -> ROS (Z ke atas): putar 180 derajat terhadap X, lalu mm -> m."""
    T = np.diag([1.0, -1.0, -1.0, 1.0])
    T[:3, :3] *= 1e-3
    m.apply_transform(T)
    return m


def massa_part(nama, vol_cm3):
    if nama.startswith("Solar_Panel"):
        return PANEL_KG
    if nama.startswith("57HSE"):
        return MOTOR_KG
    if any(nama.startswith(p) for p in BAJA):
        return vol_cm3 * RHO["baja"]
    if any(nama.startswith(p) for p in PLASTIK):
        return vol_cm3 * RHO["plastik"]
    return vol_cm3 * RHO["alu"]


def muat_link(nama_link, asal=None):
    """asal = titik asal frame link (m, koordinat rakitan). Mesh dan COM digeser ke sana, karena di URDF
    geometri visual dan <inertial><origin> diukur dari FRAME LINK, bukan dari frame rakitan."""
    """Gabungkan mesh satu link, kembalikan (mesh gabungan, massa, COM, tensor inersia di COM)."""
    meshes, M, Mc, parts = [], 0.0, np.zeros(3), []
    for n in LINK[nama_link]:
        f = SRC / f"{n}.stl"
        if not f.exists():
            raise SystemExit(f"mesh hilang: {f}")
        m = ke_ros(trimesh.load(f, force="mesh"))
        if asal is not None:
            m.apply_translation(-np.asarray(asal, dtype=float))
        meshes.append(m)
        v = abs(m.volume) * 1e6          # m3 -> cm3
        kg = massa_part(n, v)
        c = m.bounds.mean(axis=0) if not m.is_watertight else m.center_mass
        parts.append((n, m, kg, c))
        M += kg
        Mc += kg * c
    com = Mc / M
    # tensor inersia: tiap part sebagai benda homogen pada kerapatan yang memberi massanya, lalu sumbu sejajar
    I = np.zeros((3, 3))
    for n, m, kg, c in parts:
        mm = m.copy()
        mm.density = kg / max(abs(mm.volume), 1e-12)   # kerapatan seragam yang menghasilkan massa kg
        d = c - com
        I += mm.moment_inertia + kg * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return trimesh.util.concatenate(meshes), M, com, I


def inertial_xml(M, com, I):
    return (f'<inertial><origin xyz="{com[0]:.5f} {com[1]:.5f} {com[2]:.5f}"/><mass value="{M:.4f}"/>'
            f'<inertia ixx="{I[0,0]:.6g}" iyy="{I[1,1]:.6g}" izz="{I[2,2]:.6g}" '
            f'ixy="{I[0,1]:.6g}" ixz="{I[0,2]:.6g}" iyz="{I[1,2]:.6g}"/></inertial>')


ZETA_DRIVER = 0.0      # redaman loop posisi driver. 0 = tidak dimodelkan; lihat catatan di gains().


def gains():
    """Gain Gazebo dari dyn_model.AXES. Reduksi gigi sudah masuk lewat Axis.K_ser() dan Axis.Th_L,
    jadi tidak ada rumus yang digandakan di sini.

    d_gain DIBIARKAN NOL. Driver loop tertutup yang sebenarnya hampir pasti punya umpan balik
    kecepatan, tetapi nilainya belum diketahui dan simulasi TIDAK membutuhkannya untuk stabil.
    Diuji langsung di Gazebo, satu peluncuran bersih, perintah az 0.5 el 0.6 rad:
        d_gain = 0.000  -> diam di 1.2e-17 rad, lalu menetap di 0.5000184 dan 0.6043358
        d_gain = 3.869  -> diam di 1.5e-17 rad, lalu menetap di 0.5000071 dan 0.6045875
    Keduanya stabil dan sama telitinya, jadi menambah d_gain hanya menambah satu asumsi tanpa
    manfaat yang terbukti. Naikkan ZETA_DRIVER hanya bila datasheet memberi angkanya.

    (Catatan kejujuran: sempat ada dugaan bahwa d_gain taknol WAJIB karena sendi terlihat
    mengembara sampai batas. Itu salah. Penyebab sebenarnya adalah empat wind_node dan dua
    sun_command_node sisa peluncuran sebelumnya yang menerbitkan perintah saling menimpa ke
    topik yang sama. Setelah proses sisa dibersihkan, d_gain = 0 berperilaku sempurna.)
    """
    g = {}
    for j, ax in (("azimuth_joint", D.AXES["azimuth"]), ("elevation_joint", D.AXES["elevasi"])):
        K = ax.K_ser()
        d = max(0.0, 2 * ZETA_DRIVER * math.sqrt(K * ax.JL) - ax.bl)   # dikurangi redaman sendi yang sudah ada
        g[j] = dict(p=K, d=d, cmd=ax.Th_L)
    return g


def main():
    (OUT / "meshes").mkdir(parents=True, exist_ok=True)
    # sumbu dari geometri, dihitung lebih dulu karena menjadi asal frame tiap link
    poros = ke_ros(trimesh.load(SRC / "as_8mm_1.stl", force="mesh"))
    z_el = float(poros.bounds.mean(axis=0)[2])          # tinggi poros engsel
    alas = ke_ros(trimesh.load(SRC / "alas_1.stl", force="mesh"))
    z_az = float(alas.bounds.mean(axis=0)[2])           # pelat dudukan putaran azimuth
    ASAL = {"base": (0.0, 0.0, 0.0), "azimuth_link": (0.0, 0.0, z_az), "elevation_link": (0.0, 0.0, z_el)}

    info, L = {}, {}
    for nm in LINK:
        mesh, M, com, I = muat_link(nm, ASAL[nm])
        # Mesh CAD sangat rapat (sampai 318k segitiga, 27 MB). Hanya dipakai untuk TAMPILAN, bukan tabrakan
        # maupun inersia (massa, COM dan tensor sudah dihitung dari mesh penuh di atas), jadi aman disederhanakan.
        n0 = len(mesh.faces)
        if n0 > MAX_TRI:
            try:
                mesh = mesh.simplify_quadric_decimation(face_count=MAX_TRI)
            except Exception as e:
                print(f"  ({nm}: penyederhanaan dilewati, {e})")
        if len(mesh.faces) != n0:
            print(f"  {nm}: segitiga {n0} -> {len(mesh.faces)}")
        mesh.export(OUT / "meshes" / f"{nm}.stl")
        L[nm] = (M, com, I)
        lo, hi = mesh.bounds
        print(f"{nm:<16} massa={M:7.3f} kg  com=({com[0]:+.4f},{com[1]:+.4f},{com[2]:+.4f})  "
              f"kotak=({hi[0]-lo[0]:.3f},{hi[1]-lo[1]:.3f},{hi[2]-lo[2]:.3f}) m  tri={len(mesh.faces)}")
        info[nm] = dict(massa_kg=M, com_m=com.tolist(),
                        inersia_com_kgm2=dict(ixx=I[0, 0], iyy=I[1, 1], izz=I[2, 2],
                                              ixy=I[0, 1], ixz=I[0, 2], iyz=I[1, 2]))

    print(f"\nsumbu azimuth   : Z di (x=0, y=0), origin z = {z_az:.4f} m")
    print(f"sumbu kemiringan: X di (y=0), origin z = {z_el:.4f} m  (selisih {z_el - z_az:+.4f} m)")

    g = gains()
    ctl = "".join(
        f'    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">\n'
        f'      <joint_name>{j}</joint_name><p_gain>{g[j]["p"]:.2f}</p_gain><i_gain>0</i_gain><d_gain>{g[j]["d"]:.3f}</d_gain>'
        f'<cmd_max>{g[j]["cmd"]:.3f}</cmd_max><cmd_min>-{g[j]["cmd"]:.3f}</cmd_min>\n    </plugin>\n'
        for j in ("azimuth_joint", "elevation_joint"))

    urdf = f"""<?xml version="1.0"?>
<robot name="solar_tracker_v2">
  <material name="alu"><color rgba="0.75 0.77 0.8 1"/></material>
  <material name="panel"><color rgba="0.1 0.2 0.5 0.8"/></material>
  <link name="world"/>
  <joint name="world_joint" type="fixed"><parent link="world"/><child link="base_link"/></joint>
  <link name="base_link"><visual><geometry><mesh filename="{MESH_URI}/base.stl"/></geometry><material name="alu"/></visual>
    {inertial_xml(*L["base"])}</link>
  <link name="azimuth_link"><visual><geometry><mesh filename="{MESH_URI}/azimuth_link.stl"/></geometry><material name="alu"/></visual>
    {inertial_xml(*L["azimuth_link"])}</link>
  <link name="elevation_link"><visual><geometry><mesh filename="{MESH_URI}/elevation_link.stl"/></geometry><material name="panel"/></visual>
    {inertial_xml(*L["elevation_link"])}</link>
  <joint name="azimuth_joint" type="revolute"><parent link="base_link"/><child link="azimuth_link"/>
    <origin xyz="0 0 {z_az:.4f}"/><axis xyz="0 0 -1"/><dynamics damping="{JOINT_DAMP}" friction="{JOINT_FRIC}"/>
    <limit lower="{-AZ_LIMIT:.4f}" upper="{AZ_LIMIT:.4f}" effort="{g['azimuth_joint']['cmd']:.3f}" velocity="1"/></joint>
  <joint name="elevation_joint" type="revolute"><parent link="azimuth_link"/><child link="elevation_link"/>
    <origin xyz="0 0 {z_el - z_az:.4f}"/><axis xyz="1 0 0"/><dynamics damping="{JOINT_DAMP}" friction="{JOINT_FRIC}"/>
    <limit lower="{-EL_LIMIT:.4f}" upper="{EL_LIMIT:.4f}" effort="{g['elevation_joint']['cmd']:.3f}" velocity="1"/></joint>
  <gazebo>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
{ctl}  </gazebo>
</robot>
"""
    (OUT / "solar_tracker_v2.urdf").write_text(urdf, encoding="utf-8")
    info["gear"] = dict(rasio=GEAR_N, z1=8, z2=42, modul_mm=2.0, pada="elevation_joint")
    info["joint"] = dict(azimuth=dict(sumbu=[0, 0, -1], z=z_az, batas_deg=math.degrees(AZ_LIMIT)),
                         elevasi=dict(sumbu=[1, 0, 0], z=z_el, batas_deg=math.degrees(EL_LIMIT)))
    (OUT / "inersia_v2.json").write_text(json.dumps(info, indent=1), encoding="utf-8")

    print(f"\ngain Gazebo (reduksi {GEAR_N:.2f}:1 di kemiringan):")
    for j in ("azimuth_joint", "elevation_joint"):
        print(f"  {j:<17} p_gain={g[j]['p']:8.2f}  d_gain={g[j]['d']:6.3f}  cmd_max={g[j]['cmd']:.3f} N.m")
    print(f"\nditulis: {OUT/'solar_tracker_v2.urdf'}")


if __name__ == "__main__":
    main()
