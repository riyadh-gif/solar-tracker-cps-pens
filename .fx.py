import io, sys
def sub(old, new):
    f = "build_model_v2.py"
    s = io.open(f, encoding="utf-8").read()
    if s.count(old) != 1:
        print(f"LEWAT ({s.count(old)}): {old[:60]!r}"); return False
    io.open(f, "w", encoding="utf-8").write(s.replace(old, new)); print("ok"); return True
ok = True
# geser mesh ke frame link sebelum digabung
ok &= sub("""        m = ke_ros(trimesh.load(f, force="mesh"))
        meshes.append(m)""",
"""        m = ke_ros(trimesh.load(f, force="mesh"))
        if asal is not None:
            m.apply_translation(-np.asarray(asal, dtype=float))
        meshes.append(m)""")
# sumbu dihitung SEBELUM memuat link, lalu dipakai sebagai asal frame
ok &= sub("""    (OUT / "meshes").mkdir(parents=True, exist_ok=True)
    info, L = {}, {}
    for nm in LINK:
        mesh, M, com, I = muat_link(nm)""",
"""    (OUT / "meshes").mkdir(parents=True, exist_ok=True)
    # sumbu dari geometri, dihitung lebih dulu karena menjadi asal frame tiap link
    poros = ke_ros(trimesh.load(SRC / "as_8mm_1.stl", force="mesh"))
    z_el = float(poros.bounds.mean(axis=0)[2])          # tinggi poros engsel
    alas = ke_ros(trimesh.load(SRC / "alas_1.stl", force="mesh"))
    z_az = float(alas.bounds.mean(axis=0)[2])           # pelat dudukan putaran azimuth
    ASAL = {"base": (0.0, 0.0, 0.0), "azimuth_link": (0.0, 0.0, z_az), "elevation_link": (0.0, 0.0, z_el)}

    info, L = {}, {}
    for nm in LINK:
        mesh, M, com, I = muat_link(nm, ASAL[nm])""")
# hapus perhitungan sumbu yang lama (sekarang duplikat)
ok &= sub("""    # sumbu dari geometri
    poros = ke_ros(trimesh.load(SRC / "as_8mm_1.stl", force="mesh"))
    z_el = float(poros.bounds.mean(axis=0)[2])          # tinggi poros engsel
    alas = ke_ros(trimesh.load(SRC / "alas_1.stl", force="mesh"))
    z_az = float(alas.bounds.mean(axis=0)[2])           # pelat dudukan putaran azimuth
    print(f"\nsumbu azimuth""",
"""    print(f"\nsumbu azimuth""")
sys.exit(0 if ok else 1)
