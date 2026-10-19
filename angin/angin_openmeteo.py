"""Angin NYATA Surabaya dari Open-Meteo, menggantikan angin sintetis, lengkap dengan ARAH.

Mengapa diganti. Angin sintetis lama (`gabungan/coupled_sim.wind_series`) hanya memberi kecepatan dan tanda
acak plus minus, tanpa arah mata angin, dan torsinya tidak bergantung orientasi panel sama sekali. Akibatnya
dua kesalahan yang sama-sama membesar-besarkan risiko angin:
  1. Skenario "berangin" 7 m/s dan "badai" 11 m/s dimaksudkan sebagai angin DI PANEL, padahal maksimum
     setahun di PENS pada ketinggian panel hanya sekitar 5 m/s.
  2. Lengan momen azimuth 0.152 m adalah tebakan tanpa dasar fisis, dan model barunya menunjukkan torsi
     azimuth seharusnya NOL saat panel datar.

Sumber data: Open-Meteo Archive API (gratis, tanpa kunci), reanalisis per jam pada petak sekitar 9 km.
Berkas tersimpan: `angin_pens_2015-2026.csv` (102984 jam, 1 Jan 2015 sampai 30 Sep 2026, 11 tahun penuh).

MENGAPA 11 TAHUN, DAN MENGAPA PRESET SINTETIS DIBUANG. Maksimum hembusan tahunan sangat stabil: rata-rata
16.28 m/s pada 10 m, simpangan baku hanya 1.29, rentang 13.81 sampai 17.81. Surabaya terlalu dekat ekuator
untuk siklon tropis dan tidak kena badai lintang menengah, jadi ekstremnya datar. Maksimum 11 tahun hanya
1.06 kali maksimum 1 tahun. Karena itu data nyata sudah mencakup rentang penuh, dan preset karangan
(`tenang` 3, `berangin` 7, `badai` 11 m/s) tidak lagi diperlukan; `badai` bahkan dua kali di atas apa pun
yang pernah tercatat di tinggi panel selama 11 tahun. Untuk menguji di luar rekaman, pakai PENGALI terhadap
data nyata (`skala` di wind_node): "tiga kali hembusan terburuk 11 tahun" dapat dipertanggungjawabkan,
"badai 11 m/s" tidak.

PERBARUI DATA (WSL di mesin ini TIDAK punya akses internet, jalankan dari PowerShell Windows):
    $u = "https://archive-api.open-meteo.com/v1/archive?latitude=-7.276006&longitude=112.794327" +
         "&start_date=2015-01-01&end_date=2017-12-31" +   # unduh per potongan 3 tahun
         "&hourly=wind_speed_10m,wind_direction_10m,wind_gusts_10m&timezone=Asia%2FJakarta"
    Invoke-WebRequest $u -OutFile angin_mentah.json
lalu jalankan `python angin_openmeteo.py --impor angin_mentah.json`. Permintaan lebih dari sekitar 3 tahun
mudah terputus di tengah jalan; unduh per potongan 3 tahun lalu gabungkan.

DUA KOREKSI YANG WAJIB DIPAKAI
1. Tinggi. Reanalisis berlaku pada 10 m, panel Anda sekitar 0.3 m. Profil pangkat v(z) = v10 (z/10)^alpha.
   alpha 0.16 lapangan terbuka, 0.25 pinggiran, 0.35 kampus berbangunan (bawaan di sini 0.30, ASUMSI).
2. Arah. Angin DARI bearing phi berjalan MENUJU phi + 180.

MODEL TORSI BERARAH (menggantikan `dyn_model.wind_torque` yang buta arah)
Gaya pada pelat datar bekerja tegak lurus permukaan:
    F = q_dyn * A * Cn * (w_hat . n_hat),      q_dyn = 0.5 rho v^2
Pusat tekanan bergeser dari pusat panel: e_c searah chord (melintang engsel), e_s searah span (sejajar engsel).
Dengan r = h n + e_c xp + e_s yp dan F sejajar n, hasil perkalian silangnya:
    tau_elevasi = -e_c * F
    tau_azimuth = -e_s * F * sin(q2)
Perhatikan **torsi azimuth NOL saat panel datar** dan tumbuh dengan kemiringan. Itu benar secara fisika:
gaya tegak lurus panel datar tidak dapat memuntir sumbu tegak. Model lama tidak punya sifat ini.

Pakai: python angin_openmeteo.py [--selftest | --impor BERKAS.json]
"""
import csv
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "dinamika"))
import dyn_model as D   # noqa: E402  (luas panel, torsi tahan)
import tracker          # noqa: E402

BERKAS = HERE / "angin_pens_2015-2026.csv"
Z_PANEL = 0.30      # m, tinggi acuan panel di atas tanah (sumbu 0.2 m, tepi atas 0.4 m saat miring 45 deg)
ALPHA = 0.30        # eksponen profil angin, ASUMSI untuk kampus berbangunan sedang
RHO, CN = 1.2, 1.2  # kg/m3 dan koefisien gaya normal pelat datar (Cn ASUMSI)
E_CHORD = 0.15 * 0.445   # m, pergeseran pusat tekanan searah chord (ASUMSI 15 persen sisi melintang)
E_SPAN = 0.02            # m, pergeseran searah span: eksentrisitas geometris 9.2 mm + kelonggaran turbulensi (ASUMSI)
MATA_ANGIN = ("Utara", "Timur Laut", "Timur", "Tenggara", "Selatan", "Barat Daya", "Barat", "Barat Laut")


def nama_arah(deg):
    """Nama mata angin dari derajat kompas."""
    return MATA_ANGIN[int((deg % 360) / 45.0 + 0.5) % 8]


def muat(berkas=BERKAS):
    """Baca data angin tersimpan. Kembalikan dict array: jam lokal, v10, hembusan10, arah (derajat kompas, DARI).

    Pembersihan: pada 6 dari 8760 jam reanalisis memberi hembusan LEBIH KECIL daripada kecepatan rata-rata,
    semuanya saat angin lemah (di bawah 3 m/s). Itu artefak karena hembusan dan rata-rata berasal dari
    parameterisasi berbeda, bukan kesalahan pembacaan. Hembusan dinaikkan ke kecepatan rata-rata, dan
    jumlah baris yang dikoreksi dilaporkan lewat kunci `dikoreksi`."""
    jam, v, g, a, tgl = [], [], [], [], []
    with open(berkas, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            tgl.append(r["waktu"][:10])
            jam.append(int(r["waktu"][11:13]))
            v.append(float(r["v10_ms"])); g.append(float(r["hembusan10_ms"])); a.append(float(r["arah_deg"]))
    v, g = np.array(v), np.array(g)
    rusak = int((g < v).sum())
    g = np.maximum(g, v)
    return dict(tanggal=np.array(tgl), jam=np.array(jam), v10=v, hembusan10=g, arah=np.array(a), dikoreksi=rusak)


def hari(tanggal, berkas=BERKAS):
    """Satu hari (24 jam) dari data tersimpan. Bila tanggal tidak ada, pakai tanggal terdekat di bulan sama."""
    d = muat(berkas)
    m = d["tanggal"] == tanggal
    if not m.any():
        bulan = tanggal[5:7]
        kandidat = sorted({t for t in d["tanggal"] if t[5:7] == bulan})
        if not kandidat:
            raise ValueError(f"tanggal {tanggal} dan bulan {bulan} tidak ada di {berkas.name}")
        m = d["tanggal"] == kandidat[len(kandidat) // 2]
    return {k: (v[m] if isinstance(v, np.ndarray) else v) for k, v in d.items()}


def v_pada_panel(v10, alpha=ALPHA, z=Z_PANEL):
    """Kecepatan angin pada tinggi panel dari nilai 10 m (profil pangkat)."""
    return np.asarray(v10, float) * (z / 10.0) ** alpha


def arah_vektor(arah_deg):
    """Vektor satuan arah PERJALANAN angin di frame dunia (x=Utara, y=Barat, z=atas).
    arah_deg adalah bearing DARI MANA angin datang; angin berjalan menuju arah_deg + 180."""
    a = math.radians(float(arah_deg) + 180.0)
    return np.array([math.cos(a), -math.sin(a), 0.0])


def torsi(v_panel, arah_deg, q1_deg, q2_deg, cn=CN, rho=RHO, e_chord=E_CHORD, e_span=E_SPAN):
    """Torsi angin pada kedua sumbu (N.m): (tau_azimuth, tau_elevasi). Bertanda, bergantung orientasi panel."""
    n = tracker.fk(math.radians(q1_deg), math.radians(q2_deg))
    proyeksi = float(arah_vektor(arah_deg) @ n)          # negatif bila angin menerpa muka panel
    F = 0.5 * rho * float(v_panel) ** 2 * D.AXES["elevasi"].area * cn * proyeksi
    return -e_span * F * math.sin(math.radians(q2_deg)), -e_chord * F


def maks_tahunan(berkas=BERKAS):
    """Hembusan maksimum tiap tahun penuh (m/s pada 10 m). Dasar untuk menilai apakah rekaman cukup panjang."""
    d = muat(berkas)
    thn = np.array([t[:4] for t in d["tanggal"]])
    out = {}
    for y in sorted(set(thn)):
        m = thn == y
        if m.sum() >= 7000:          # hanya tahun penuh
            out[y] = float(d["hembusan10"][m].max())
    return out


def ringkas(berkas=BERKAS, siang=(7, 17)):
    """Statistik ringkas untuk dokumen dan pemeriksaan."""
    d = muat(berkas)
    s = (d["jam"] >= siang[0]) & (d["jam"] <= siang[1])
    out = dict(n=len(d["v10"]), n_siang=int(s.sum()))
    for nama, x in (("v10", d["v10"]), ("hembusan10", d["hembusan10"])):
        out[nama] = dict(rata=float(x[s].mean()), p95=float(np.percentile(x[s], 95)),
                         p99=float(np.percentile(x[s], 99)), maks=float(x[s].max()))
        xp = v_pada_panel(x[s])
        out[nama + "_panel"] = dict(rata=float(xp.mean()), p95=float(np.percentile(xp, 95)),
                                    p99=float(np.percentile(xp, 99)), maks=float(xp.max()))
    h, _ = np.histogram(np.mod(d["arah"][s] + 22.5, 360), bins=np.arange(0, 361, 45))
    out["arah_pct"] = {MATA_ANGIN[i]: round(float(h[i] / h.sum() * 100), 1) for i in range(8)}
    out["arah_dominan"] = MATA_ANGIN[int(np.argmax(h))]
    mt = maks_tahunan(berkas)
    out["maks_tahunan"] = mt
    out["hembusan_terburuk"] = max(mt.values()) if mt else float(d["hembusan10"].max())
    out["hembusan_terburuk_panel"] = float(v_pada_panel(out["hembusan_terburuk"]))
    return out


def impor(src):
    """Ubah JSON mentah Open-Meteo jadi CSV proyek."""
    import json
    d = json.load(open(src, encoding="utf-8"))
    h = d["hourly"]
    n = 0
    with BERKAS.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["waktu", "v10_ms", "hembusan10_ms", "arah_deg"])
        for t, v, g, a in zip(h["time"], h["wind_speed_10m"], h["wind_gusts_10m"], h["wind_direction_10m"]):
            if v is None:
                continue
            w.writerow([t, f"{v / 3.6:.3f}", f"{g / 3.6:.3f}", f"{a:.0f}"])
            n += 1
    print(f"tersimpan {n} jam ke {BERKAS.name} (lat {d['latitude']}, lon {d['longitude']}, elev {d.get('elevation')} m)")


def selftest():
    # 1) data tersimpan terbaca dan masuk akal
    d = muat()
    assert len(d["v10"]) > 8000, len(d["v10"])
    assert (d["v10"] >= 0).all() and d["v10"].max() < 40
    assert (d["hembusan10"] >= d["v10"] - 1e-9).all(), "setelah dibersihkan, hembusan tidak boleh di bawah rata-rata"
    assert d["dikoreksi"] < 0.01 * len(d["v10"]), f"terlalu banyak baris rusak: {d['dikoreksi']}"
    assert ((d["arah"] >= 0) & (d["arah"] <= 360)).all()
    # 2) koreksi tinggi menurunkan kecepatan, dan sesuai rumus
    assert abs(v_pada_panel(10.0) - 10.0 * (Z_PANEL / 10.0) ** ALPHA) < 1e-12
    assert v_pada_panel(10.0) < 10.0
    # 3) arah: angin DARI Timur (90) berjalan MENUJU Barat (+y pada frame x=Utara, y=Barat)
    w = arah_vektor(90.0)
    assert abs(w[0]) < 1e-9 and w[1] > 0.99, w
    w = arah_vektor(0.0)          # dari Utara, berjalan ke Selatan (-x)
    assert w[0] < -0.99 and abs(w[1]) < 1e-9, w
    assert nama_arah(0) == "Utara" and nama_arah(135) == "Tenggara" and nama_arah(350) == "Utara"
    # 4) panel DATAR: torsi azimuth harus NOL berapa pun arah angin (sifat yang tidak dimiliki model lama)
    for ar in (0, 45, 90, 180, 270):
        ta, te = torsi(10.0, ar, 0.0, 0.0)
        assert abs(ta) < 1e-12, (ar, ta)
    # 5) torsi azimuth tumbuh dengan kemiringan
    t10 = abs(torsi(10.0, 135.0, 0.0, 10.0)[0])
    t45 = abs(torsi(10.0, 135.0, 0.0, 45.0)[0])
    assert t45 > t10 > 0, (t10, t45)
    # 6) panel menghadap angin menerima torsi elevasi terbesar; menyamping hampir nol
    s = tracker.sun_vec(135.0, 45.0)      # arah "menghadap" tenggara elevasi 45
    q1_hadap = 135.0 - math.degrees(tracker.HEAD)
    hadap = abs(torsi(10.0, 135.0, q1_hadap, 45.0)[1])
    samping = abs(torsi(10.0, 135.0, q1_hadap + 90.0, 45.0)[1])
    assert hadap > samping * 3, (hadap, samping)
    # 7) torsi sebanding v kuadrat
    a1, b1 = torsi(5.0, 135.0, 0.0, 30.0)
    a2, b2 = torsi(10.0, 135.0, 0.0, 30.0)
    assert abs(b2 / b1 - 4.0) < 1e-9, (b1, b2)
    # 8) rekaman panjang: maksimum tahunan harus stabil, kalau tidak satu tahun tidak mewakili
    mt = maks_tahunan()
    nilai = np.array(list(mt.values()))
    assert len(mt) >= 10, f"butuh minimal 10 tahun penuh, ada {len(mt)}"
    assert nilai.std() / nilai.mean() < 0.15, f"maksimum tahunan terlalu berubah-ubah: sb/rata {nilai.std()/nilai.mean():.2f}"
    r = ringkas()
    print("selftest angin_openmeteo: lolos")
    print(f"  {r['n']} jam, {r['n_siang']} jam siang (07-17), {d['dikoreksi']} baris hembusan dikoreksi")
    print(f"  pada 10 m   : rata {r['v10']['rata']:.2f}  p99 {r['v10']['p99']:.2f}  maks {r['v10']['maks']:.2f} m/s; "
          f"hembusan maks {r['hembusan10']['maks']:.2f} m/s")
    print(f"  di panel {Z_PANEL} m (alpha {ALPHA}): rata {r['v10_panel']['rata']:.2f}  p99 {r['v10_panel']['p99']:.2f}  "
          f"maks {r['v10_panel']['maks']:.2f} m/s; hembusan maks {r['hembusan10_panel']['maks']:.2f} m/s")
    print(f"  arah dominan: {r['arah_dominan']} ({r['arah_pct'][r['arah_dominan']]} %)")
    print(f"  maksimum hembusan tahunan ({len(mt)} tahun): rata {nilai.mean():.2f}, sb {nilai.std():.2f}, "
          f"terendah {nilai.min():.2f}, tertinggi {nilai.max():.2f} m/s pada 10 m")
    print(f"  terburuk 11 tahun di panel: {r['hembusan_terburuk_panel']:.2f} m/s")
    print(f"  torsi pada kemiringan 45 deg terhadap torsi tahan {D.AXES['azimuth'].Th} N.m:")
    for nm, sk in (("terburuk 11 tahun", 1.0), ("x 1.5", 1.5), ("x 2", 2.0), ("x 3", 3.0)):
        ta, te = torsi(r["hembusan_terburuk_panel"] * sk, 135.0, 0.0, 45.0)
        print(f"    {nm:20s} v {r['hembusan_terburuk_panel']*sk:5.2f} m/s  az {ta:+.4f}  el {te:+.4f} N.m  "
              f"= {max(abs(ta), abs(te)) / D.AXES['azimuth'].Th * 100:6.2f} %")
    return r


if __name__ == "__main__":
    if "--impor" in sys.argv:
        impor(sys.argv[sys.argv.index("--impor") + 1])
    else:
        selftest()
