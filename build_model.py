"""parts_v2 (STL, meter) -> 3 mesh link + URDF (kinematik, massa/inersia dari CAD, siap Gazebo).
Massa dan inersia dari inersia/inersia_link.json (motor 1.2 kg dan panel 1.5 kg adalah ASUMSI); placeholder bila berkas tidak ada.
Pose nol: rel panel horizontal (CAD asli miring 21.24 derajat, diputar balik di sini).
Engsel kemiringan searah rel panjang (+x alat = Utara, KEPUTUSAN pengguna: panel hanya berguling kanan-kiri, tidak jungkat-jungkit sepanjang rel). Pada q1 = 0 panel miring ke Timur untuk q2 > 0.
Dunia = frame dasar: +x = Utara (ASUMSI), +y = Barat, +z = atas. Azimuth CW (kompas), arah miring kompas = q1 + HEAD (HEAD = 90).
Gazebo: link 'world' + sendi tetap mengikat dasar. Kontrol posisi (actuator pid) disetel ke kekakuan seri stepper dan poros dari dinamika/dyn_model.py:
linier, tanpa tundaan sirkuit, tanpa lepas langkah, torsi dibatasi torsi tahan Th (dyn_model.AXES).
"""
import json
import sys
import numpy as np
import trimesh
from pathlib import Path

ROOT = Path(__file__).parent
SRC = ROOT / "New folder (2)/solar_stl/parts_v2"
OUT = ROOT / "solar_tracker"
MESH_URI = "file:///home/riyadh/mk/solar_tracker/meshes"  # symlink WSL ~/mk -> folder proyek (hindari spasi di path)

GROUPS = {"base": range(0, 11), "azimuth_link": (11, 12, 13, 18),
          "elevation_link": (14, 15, 16, 17, 19, 20, 21)}
P_AZ = np.array([0, 0, 0.165])      # puncak poros motor azimuth
P_EL = np.array([0, 0, 0.2045])     # sumbu elevasi (sejajar y), dari bounds motor 18 dan poros 19
TILT = np.radians(21.24)            # kemiringan rel di pose CAD
EL_LIMIT = np.radians(45)           # asumsi kasar (rel menumbuk rangka dasar), ukur di alat nyata
PANEL = (0.670, 0.445, 0.025)       # MOS C-50-18-M (spesifikasi pabrik): sisi panjang 0.670 m SEARAH sumbu engsel (+x alat),
                                    # sisi 0.445 m melintang engsel. Orientasi ini KEPUTUSAN: dengan 0.670 m melintang, tepi bawah
                                    # menumbuk tanah sebelum batas kemiringan 45 derajat tercapai.
LDRS = {"ldr_x_pos": (1, 0), "ldr_x_neg": (-1, 0), "ldr_y_pos": (0, 1), "ldr_y_neg": (0, -1)}  # (sx, sy) sisi panel
INERSIA = ROOT / "inersia/inersia_link.json"
JOINT_DAMP, JOINT_FRIC = 0.2, 0.05  # N.m.s/rad dan N.m: redaman viskos dan gesekan Coulomb beban, ASUMSI dari dinamika/dyn_model.py
sys.path.insert(0, str(ROOT / "dinamika"))
import dyn_model as D   # noqa: E402  (satu sumber konstanta stepper: Th, Nr, k)



def stiffness_gains():
    """Gain kontrol posisi Gazebo per sendi: p = kekakuan seri stepper dan poros, 1/(1/(Th Nr) + 1/k), linearisasi hukum torsi-sudut di sekitar
    posisi perintah; i = d = 0 (redaman dan gesekan datang dari sendi); batas torsi = torsi tahan Th."""
    return {j: dict(p=ax.K_ser(), cmd=ax.Th)
            for j, ax in (("azimuth_joint", D.AXES["azimuth"]), ("elevation_joint", D.AXES["elevasi"]))}


def inertial_xml(name, key):
    """Blok <inertial> (COM dan tensor di COM) dari inersia_link.json; placeholder bila berkas tidak ada."""
    if not INERSIA.exists():
        return '<inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>'
    d = json.loads(INERSIA.read_text())["link"][key]
    i, c = d["inersia_com_kgm2"], d["com_m"]
    return (f'<inertial><origin xyz="{c[0]:.5f} {c[1]:.5f} {c[2]:.5f}"/><mass value="{d["massa_kg"]:.4f}"/>'
            f'<inertia ixx="{i["ixx"]:.6g}" iyy="{i["iyy"]:.6g}" izz="{i["izz"]:.6g}" '
            f'ixy="{i.get("ixy", 0):.6g}" ixz="{i.get("ixz", 0):.6g}" iyz="{i.get("iyz", 0):.6g}"/></inertial>')


def gazebo_xml(actuator="pid"):
    """actuator 'pid': kontrol posisi bergain kekakuan seri stepper (lihat stiffness_gains). 'stepper': ApplyJointForce, torsi dari stepper_node.py (eksperimental)."""
    if actuator == "stepper":
        ctl = "".join(f'    <plugin filename="gz-sim-apply-joint-force-system" name="gz::sim::systems::ApplyJointForce">\n'
                      f'      <joint_name>{j}</joint_name>\n    </plugin>\n' for j in ("azimuth_joint", "elevation_joint"))
        return ('  <gazebo>\n    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>\n'
                + ctl + '  </gazebo>\n')
    g = stiffness_gains()
    ctl = "".join(
        f'    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">\n'
        f'      <joint_name>{j}</joint_name><p_gain>{g[j]["p"]:.2f}</p_gain><i_gain>0</i_gain><d_gain>0</d_gain>'
        f'<cmd_max>{g[j]["cmd"]}</cmd_max><cmd_min>-{g[j]["cmd"]}</cmd_min>\n    </plugin>\n'
        for j in ("azimuth_joint", "elevation_joint"))
    return ('  <gazebo>\n    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>\n'
            + ctl + '  </gazebo>\n')


def load(i):
    return trimesh.load(next(SRC.glob(f"{i:02d}_part.stl")), force="mesh")


def rail_tilt(m):
    v = m.vertices[:, [0, 2]]
    d = np.linalg.svd(v - v.mean(0), full_matrices=False)[2][0]
    d *= np.sign(d[0])
    return np.degrees(np.arctan2(d[1], d[0]))


ZROT = (11, 12, 13, 18, 19)  # motor tilt, batang poros, dan dudukannya diputar 90 derajat di sumbu z: poros motor searah rel (sumbu engsel x)


BRACKET, BRACKET_DX = (14, 15, 16, 17), -0.10  # ASUMSI visual: posisi aslinya tidak diketahui setelah motor diputar


def place(i, link):
    """Mesh bagian i di frame dasar (pose nol: rel horizontal, poros motor tilt searah rel). Dipakai juga oleh inersia_cad.py."""
    m = load(i)
    if link == "elevation_link":  # putar balik ke horizontal di sekitar sumbu y lewat P_EL (pose CAD asli miring 21.24 derajat)
        m.apply_transform(trimesh.transformations.rotation_matrix(TILT, [0, 1, 0], P_EL))
    if i in ZROT:
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1], [0, 0, 0]))
    if i in BRACKET:  # braket rel ikut pindah ke palang penyambung di ujung batang (x = -0.1)
        m.apply_translation([BRACKET_DX, 0, 0])
    return m


def coupler(rod, rails):
    """Palang penyambung (VISUAL, ukuran tebakan): ujung batang poros ke kedua rel, supaya rel ikut tersambung ke poros motor setelah motor diputar.
    Dalam batas bounding box elevation_link, jadi massa dan inersia (inersia_cad.py) tidak berubah."""
    rb, ob = trimesh.util.concatenate(rails).bounds, rod.bounds
    x0, z0, z1 = ob[0][0], ob[0][2] - 0.004, rb[:, 2].mean()
    box = trimesh.creation.box((0.012, rb[1][1] - rb[0][1], z1 - z0))
    box.apply_translation([x0 + 0.004, (rb[0][1] + rb[1][1]) / 2, (z0 + z1) / 2])
    return box


def link_mesh(name, origin):
    parts = [place(i, name) for i in GROUPS[name]]
    if name == "elevation_link":
        assert abs(rail_tilt(parts[-1])) < 0.05, "rel belum horizontal, cek arah rotasi"
        parts.append(coupler(parts[GROUPS[name].index(19)], parts[-2:]))
    m = trimesh.util.concatenate(parts)
    m.apply_translation(-origin)  # frame link = frame sendi
    return m


def main():
    (OUT / "meshes").mkdir(parents=True, exist_ok=True)
    origins = {"base": np.zeros(3), "azimuth_link": P_AZ, "elevation_link": P_EL}
    meshes = {n: link_mesh(n, o) for n, o in origins.items()}
    for n, m in meshes.items():
        m.export(OUT / "meshes" / f"{n}.stl")
    top = trimesh.util.concatenate([load(20), load(21)])
    top.apply_transform(trimesh.transformations.rotation_matrix(TILT, [0, 1, 0], P_EL)); top.apply_translation(-P_EL)
    cx, cy = top.bounds[:, 0].mean(), top.bounds[:, 1].mean()
    pz = top.bounds[1, 2] + PANEL[2] / 2
    ldr_xml = "".join(  # LDR di tengah tiap sisi panel, di permukaan atas; sumbu z frame = normal panel
        f'  <link name="{n}"><visual><geometry><cylinder radius="0.006" length="0.004"/></geometry><material name="ldr"/></visual></link>\n'
        f'  <joint name="{n}_joint" type="fixed"><parent link="elevation_link"/><child link="{n}"/>'
        f'<origin xyz="{cx + sx * PANEL[0] / 2:.4f} {cy + sy * PANEL[1] / 2:.4f} {pz + PANEL[2] / 2:.4f}"/></joint>\n'
        for n, (sx, sy) in LDRS.items())
    urdf = f"""<?xml version="1.0"?>
<robot name="solar_tracker">
  <material name="alu"><color rgba="0.75 0.77 0.8 1"/></material>
  <material name="panel"><color rgba="0.1 0.2 0.5 0.8"/></material>
  <material name="ldr"><color rgba="0.9 0.1 0.1 1"/></material>
  <link name="world"/>
  <joint name="world_joint" type="fixed"><parent link="world"/><child link="base_link"/></joint>
  <link name="base_link"><visual><geometry><mesh filename="{MESH_URI}/base.stl"/></geometry><material name="alu"/></visual>
    {inertial_xml("base_link", "base")}</link>
  <link name="azimuth_link"><visual><geometry><mesh filename="{MESH_URI}/azimuth_link.stl"/></geometry><material name="alu"/></visual>
    {inertial_xml("azimuth_link", "azimuth_link")}</link>
  <link name="elevation_link"><visual><geometry><mesh filename="{MESH_URI}/elevation_link.stl"/></geometry><material name="alu"/></visual>
    {inertial_xml("elevation_link", "elevation_link")}</link>
  <link name="panel"><visual><origin xyz="{cx:.4f} {cy:.4f} {pz:.4f}"/><geometry><box size="{PANEL[0]} {PANEL[1]} {PANEL[2]}"/></geometry><material name="panel"/></visual>
    {inertial_xml("panel", "panel")}</link>
  <joint name="azimuth_joint" type="continuous"><parent link="base_link"/><child link="azimuth_link"/>
    <origin xyz="{P_AZ[0]} {P_AZ[1]} {P_AZ[2]}"/><axis xyz="0 0 -1"/><dynamics damping="{JOINT_DAMP}" friction="{JOINT_FRIC}"/></joint>
  <joint name="elevation_joint" type="revolute"><parent link="azimuth_link"/><child link="elevation_link"/>
    <origin xyz="{(P_EL - P_AZ)[0]} {(P_EL - P_AZ)[1]} {(P_EL - P_AZ)[2]:.4f}"/><axis xyz="1 0 0"/><dynamics damping="{JOINT_DAMP}" friction="{JOINT_FRIC}"/>
    <limit lower="{-EL_LIMIT:.4f}" upper="{EL_LIMIT:.4f}" effort="{D.AXES['elevasi'].Th}" velocity="1"/></joint>
  <joint name="panel_joint" type="fixed"><parent link="elevation_link"/><child link="panel"/></joint>
{ldr_xml}{gazebo_xml()}</robot>
"""
    (OUT / "solar_tracker.urdf").write_text(urdf)
    (OUT / "solar_tracker_stepper.urdf").write_text(urdf.replace(gazebo_xml(), gazebo_xml("stepper")))   # varian aktuator stepper (torsi dari node)
    for n, m in meshes.items():
        print(f"{n:15s} tri={len(m.faces):6d} bounds(m) min={m.bounds[0].round(3)} max={m.bounds[1].round(3)}")
    print("panel center di frame elevasi:", round(cx, 4), round(cy, 4), round(pz, 4))


if __name__ == "__main__":
    main()
