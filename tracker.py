"""Kinematik tracker 2 sumbu (azimuth z, elevasi y), sesuai solar_tracker.urdf.
q1 = sudut azimuth joint (CW dari atas), q2 = kemiringan panel dari horizontal (0 = horizontal).
Engsel kemiringan searah rel panjang (+x alat = Utara, ASUMSI): panel hanya berguling kanan-kiri, tidak jungkat-jungkit sepanjang rel.
Pada q1 = 0, q2 > 0 miring ke Timur, jadi arah miring (kompas) = q1 + HEAD dengan HEAD = 90 derajat.
Normal panel di frame dunia: n = (sin q2 cos(q1+HEAD), -sin q2 sin(q1+HEAD), cos q2)   [x=Utara, y=Barat, z=atas].
"""
import numpy as np

HEAD = np.radians(90)  # arah miring pada q1 = 0 (Timur) karena engsel searah +x alat (Utara); Rz(-q1) Rx(q2) z = fk
Q2_MAX = np.radians(45)  # sama dengan EL_LIMIT di build_model.py


def fk(q1, q2):
    """Vektor normal panel (frame base)."""
    a = q1 + HEAD
    return np.array([np.sin(q2) * np.cos(a), -np.sin(q2) * np.sin(a), np.cos(q2)])


def sun_vec(az_deg, el_deg):
    a, e = np.radians(az_deg), np.radians(el_deg)
    return np.array([np.cos(e) * np.cos(a), -np.cos(e) * np.sin(a), np.sin(e)])


def ik_track(az_deg, el_deg, q2_max=Q2_MAX):
    """Sudut sendi untuk mengikuti matahari (urut waktu). Dua solusi: (A, z) dan (A+180, -z).
    Pilih yang kontinu dari langkah sebelumnya, jadi lewat zenith tidak perlu putar azimuth 180 derajat.
    Matahari di luar jangkauan (zenith > q2_max) -> NaN."""
    A, z = np.radians(np.asarray(az_deg, float)) - HEAD, np.radians(90 - np.asarray(el_deg, float))
    q1 = np.full(A.shape, np.nan); q2 = np.full(A.shape, np.nan)
    prev = None
    for k in range(len(A)):
        if z[k] > q2_max:
            prev = None  # di luar jangkauan, mulai ulang
            continue
        cands = [(A[k], z[k]), (A[k] + np.pi, -z[k])]
        if prev is None:  # awal: ambil solusi q2 >= 0
            c = cands[0]
        else:
            def unwrap(a):  # azimuth terdekat ke prev (continuous joint, tanpa batas)
                return a + 2 * np.pi * np.round((prev[0] - a) / (2 * np.pi))
            cands = [(unwrap(a), b) for a, b in cands]
            c = min(cands, key=lambda s: abs(s[0] - prev[0]) + abs(s[1] - prev[1]))
        q1[k], q2[k] = c
        prev = c
    return q1, q2


if __name__ == "__main__":
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import sun

    d = sun.sun_day()
    day = d[d.el > 0]
    q1, q2 = ik_track(day.az.values, day.el.values)
    ok = ~np.isnan(q1)
    # cek FK(IK) = arah matahari
    for k in np.flatnonzero(ok)[::7]:
        assert np.allclose(fk(q1[k], q2[k]), sun_vec(day.az.values[k], day.el.values[k]), atol=1e-9)
    # tracker vs panel datar: energi (GHI*cos) di bidang normal terhadap matahari
    reach = ok.mean() * 100
    e_all = day.dni.sum() * 5 / 60 / 1000   # kWh/m2 (DNI tegak lurus, ideal)
    e_ok = day.dni[ok].sum() * 5 / 60 / 1000
    print(f"Matahari terbit-terbenam {len(day)*5/60:.1f} jam; dalam jangkauan (zenith <= 45 deg): {reach:.0f}% waktu, "
          f"{100*e_ok/e_all:.0f}% energi DNI")
    print(f"Azimuth joint total gerak: {np.degrees(np.nanmax(q1)-np.nanmin(q1)):.0f} deg, "
          f"laju maks {np.degrees(np.nanmax(np.abs(np.diff(q1)))/300):.3f} deg/s (rata 5 menit)")
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    t = day.index
    ax[0].plot(t, np.degrees(q1), label="q1 azimuth"); ax[0].plot(t, np.degrees(q2), label="q2 kemiringan")
    ax[0].axhline(45, ls=":", c="r"); ax[0].axhline(-45, ls=":", c="r"); ax[0].legend(); ax[0].set_ylabel("deg")
    ax[1].plot(t, day.el); ax[1].axhline(45, ls=":", c="r"); ax[1].set_ylabel("elevasi matahari (deg)")
    ax[0].set_title("Tracker di PENS, 2026-10-04: sudut sendi (garis merah = batas 45 deg)")
    plt.tight_layout(); plt.savefig("tracker_day.png", dpi=90)
    print("tersimpan tracker_day.png")
