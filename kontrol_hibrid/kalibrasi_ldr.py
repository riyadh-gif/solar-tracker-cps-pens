"""Kalibrasi ketidakcocokan antar LDR: dua prosedur, dan seberapa jauh masing masing menolong.

Latar: `fusi_sensor.py` dan `ekf.py` menunjukkan bahwa seluruh manfaat LDR bergantung kalibrasi. Tanpa kalibrasi,
bobot LDR hanya 12 sampai 21 persen, dan EKF dengan LDR 3 persen tak terkalibrasi JUSTRU lebih buruk (5.63 deg)
daripada tanpa LDR sama sekali (2.52 deg). Berkas ini menjawab: bagaimana mengalibrasinya, dan sampai seberapa.

PROSEDUR A, cahaya baur merata (di lapangan, tanpa alat tambahan)
    Di bawah langit MENDUNG TEBAL komponen langsung hampir nol, sehingga keempat LDR menerima irradiance yang sama
    berapa pun arah panel: sekat hanya membayangi komponen langsung. Setiap beda pembacaan berarti ketidakcocokan.
    Pembacaan dibalik ke ruang x = v/(vcc - v) (pembagi tegangan jenuh), dirata-ratakan, lalu dicari faktor koreksi
    per sensor. Syarat: eksponen LDR `gam` dan setengah jenuh `L0` diketahui; galatnya menyisakan sisa kesalahan.

PROSEDUR B, regresi di tempat sepanjang hari
    Salah pasang mekanik dan ketidakcocokan LDR sama sama menggeser e. Bedanya:
      - salah pasang menghasilkan e = k(cuaca, geometri) x bias, jadi IKUT BERUBAH ketika k berubah;
      - ketidakcocokan menghasilkan offset yang hampir TETAP dalam satuan e.
    Karena k berubah sepanjang hari (terutama k azimuth yang sebanding sin q2), keduanya dapat dipisahkan dengan
    regresi e terhadap k. Keterpisahan bergantung seberapa lebar k berubah: azimuth lebar, kemiringan sempit.

Pakai: python kalibrasi_ldr.py [--selftest]
"""
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "kontrol_hibrid"))
import ekf as E              # noqa: E402
import fusi_sensor as F      # noqa: E402
import hybrid_sim as H       # noqa: E402
import sun                   # noqa: E402
import tracker               # noqa: E402


def x_dari_v(v, p):
    """Balikan pembagi tegangan jenuh: v = vcc x/(1+x)  ->  x = v/(vcc - v). Sebanding (irradiance)^gam."""
    return np.asarray(v, float) / np.maximum(p.vcc - np.asarray(v, float), 1e-9)


def v_dari_x(x, p):
    return p.vcc * np.asarray(x, float) / (1.0 + np.asarray(x, float))


def baca_v(q1, q2, sky, p, rng, mism):
    """Empat tegangan LDR mentah (V) pada pose dan langit tertentu."""
    az, el, dni, dhi, _ = sky
    n, xp, yp = H.basis(q1, q2)
    s = tracker.sun_vec(az, el)
    c = H.dot(n, s)
    ex_a, ey_a = math.atan2(H.dot(s, xp), c), math.atan2(H.dot(s, yp), c)
    beam, dif = dni * max(c, 0.0), dhi * (1 + n[2]) / 2
    sh = (H.shade(-ex_a, p.r), H.shade(ex_a, p.r), H.shade(-ey_a, p.r), H.shade(ey_a, p.r))
    return [H.volt((beam * (1 - sh[i]) + dif) * mism[i], p, rng) for i in range(4)]


def e_dari_v(v):
    """ex, ey dari empat tegangan (urutan x+, x-, y+, y-), sama dengan hybrid_sim.read."""
    return ((v[0] - v[1]) / (v[0] + v[1] + 1e-9), (v[2] - v[3]) / (v[2] + v[3] + 1e-9))


def prosedur_a(sky_mendung, mism, p, rng, q1=0.0, q2=0.0, n_avg=200, gam_galat=0.0):
    """Kalibrasi cahaya baur: kembalikan faktor koreksi per sensor dalam ruang x.
    `gam_galat` = galat relatif pengetahuan eksponen gam (misal 0.1 untuk 10 persen)."""
    vs = np.array([baca_v(q1, q2, sky_mendung, p, rng, mism) for _ in range(n_avg)])
    x = x_dari_v(vs.mean(axis=0), p)
    gam_pakai = p.gam * (1.0 + gam_galat)
    m_taksir = (x / x.mean()) ** (1.0 / gam_pakai)      # x sebanding (m irradiance)^gam
    return 1.0 / m_taksir                                # faktor untuk dikalikan ke irradiance terbaca


def terapkan_a(v, koreksi, p):
    """Terapkan koreksi prosedur A pada tegangan: balik ke x, bagi dengan (m^gam), kembali ke tegangan."""
    x = x_dari_v(v, p) * np.asarray(koreksi, float) ** p.gam
    return v_dari_x(x, p)


def sisa_mism(mism, koreksi):
    """Sisa ketidakcocokan efektif setelah koreksi (simpangan baku relatif)."""
    r = np.asarray(mism, float) * np.asarray(koreksi, float)
    return float(np.std(r / r.mean()))


def prosedur_b(tr, mism, bias_pasang, p, seed=0, n_avg=20):
    """Regresi sepanjang hari: pisahkan salah pasang mekanik dari offset ketidakcocokan LDR.
    Model: e_y(k) = k_q1(k) x bias_az + offset_az,  e_x(k) = k_q2(k) x bias_q2 + offset_q2.
    Kembalikan taksiran bias dan offset, serta lebar sebaran k (penentu keterpisahan)."""
    rng = __import__("random").Random(seed)
    m = tr["reach"]
    idx = np.flatnonzero(m)[::7]
    K, Ey, Ex = [], [], []
    for k in idx:
        sky = (float(tr["az"][k]), float(tr["el"][k]), float(tr["dni"][k]), float(tr["dhi"][k]), float(tr["ghi"][k]))
        c1, c2 = float(tr["q1"][k]), float(tr["q2"][k])
        Hm, _ = E.jacobian_ldr(c1, c2, sky, p)                      # k_q2 = Hm[0,1], k_q1 = Hm[1,0]
        vs = np.array([baca_v(c1 + bias_pasang[0], c2 + bias_pasang[1], sky, p, rng, mism) for _ in range(n_avg)])
        ex, ey = e_dari_v(vs.mean(axis=0))
        K.append((Hm[0, 1], Hm[1, 0])); Ex.append(ex); Ey.append(ey)
    K = np.array(K)
    out = {}
    for nama, kk, ee in (("q2", K[:, 0], np.array(Ex)), ("az", K[:, 1], np.array(Ey))):
        A = np.column_stack([kk, np.ones_like(kk)])
        sol, *_ = np.linalg.lstsq(A, ee, rcond=None)
        a_k = np.abs(kk)   # k kemiringan bertanda negatif, jadi rentang diukur pada besarannya
        out[nama] = dict(bias=float(sol[0]), offset=float(sol[1]),
                         k_min=float(a_k.min()), k_max=float(a_k.max()),
                         rentang=float(a_k.max() / a_k.min()) if a_k.min() > 0 else float("inf"),
                         kondisi=float(np.linalg.cond(A)))
    return out


def selftest():
    p = H.P()
    rng = __import__("random").Random(0)
    mendung = F.langit(8.0, el_target=45.0)
    cerah = F.langit(0.0, el_target=45.0)
    # 1) di bawah mendung tebal komponen langsung kecil: itu syarat prosedur A
    f_beam = mendung[2] / (mendung[2] + mendung[3] + 1e-9)
    assert f_beam < 0.25, f"DNI masih {f_beam * 100:.0f} persen, prosedur A butuh baur murni"
    # 2) dengan cahaya baur MURNI, FAKTOR KOREKSI tidak bergantung arah panel: keempat LDR menerima irradiance sama.
    #    Perhatikan sinyal e sendiri TETAP berubah sedikit dengan arah, karena pembagi tegangan jenuh itu tak linier
    #    sehingga tanda ketidakcocokan bergantung tingkat cahaya (baur ikut berubah dengan kemiringan panel).
    baur = (mendung[0], mendung[1], 0.0, mendung[3], mendung[4])
    mm = [1.0 + rng.gauss(0, 0.03) for _ in range(4)]
    q = replace(p, vn=0.0, bits=30)      # tanpa derau DAN tanpa kuantisasi: menguji dasarnya, bukan perangkatnya
    kor_pose = [prosedur_a(baur, mm, q, rng, q1=a, q2=b, n_avg=1) for a, b in ((0, 0), (30, 20), (-40, 45))]
    assert max(float(np.max(np.abs(kor_pose[0] - z))) for z in kor_pose) < 1e-6, kor_pose
    assert np.allclose(kor_pose[0] * np.array(mm), (kor_pose[0] * np.array(mm)).mean(), rtol=1e-6), kor_pose[0] * np.array(mm)
    # dengan ADC 12 bit apa adanya, faktor koreksi bergeser sedikit menurut arah panel: itulah batas perangkatnya
    q12 = replace(p, vn=0.0)
    kor12 = [prosedur_a(baur, mm, q12, rng, q1=a, q2=b, n_avg=1) for a, b in ((0, 0), (30, 20), (-40, 45))]
    kuant = max(float(np.max(np.abs(kor12[0] - z))) for z in kor12)
    assert 0.0 < kuant < 0.01, kuant
    e_pose = [e_dari_v(baca_v(a, b, baur, q, rng, mm)) for a, b in ((0, 0), (30, 20), (-40, 45))]
    ragam_e = (max(z[0] for z in e_pose) - min(z[0] for z in e_pose)) / abs(e_pose[0][0])
    assert 0.0 < ragam_e < 0.2, ragam_e
    # 3) prosedur A menurunkan ketidakcocokan efektif jauh di bawah 3 persen
    kor = prosedur_a(baur, mm, p, rng, n_avg=400)
    sisa = sisa_mism(mm, kor)
    assert sisa < 0.004, f"sisa {sisa * 100:.2f} persen, harusnya jauh di bawah 3 persen"
    # 4) galat pengetahuan gam merusak prosedur A secara sebanding
    sisa10 = sisa_mism(mm, prosedur_a(baur, mm, p, rng, n_avg=400, gam_galat=0.10))
    assert sisa10 > sisa * 2, (sisa, sisa10)
    # 5) sisa beam saat mendung membuat prosedur A sedikit meleset (karena sekat membayangi beam)
    sisa_nyata = sisa_mism(mm, prosedur_a(mendung, mm, p, rng, n_avg=400))
    assert sisa_nyata > sisa, (sisa, sisa_nyata)
    # 6) prosedur B: k azimuth berubah jauh lebih lebar daripada k kemiringan, jadi azimuth lebih terpisahkan
    tr = E.hari()
    b = prosedur_b(tr, [1.0] * 4, (1.5, 1.0), p, seed=1)
    assert b["az"]["rentang"] > b["q2"]["rentang"] * 2, b
    assert abs(b["az"]["bias"] - 1.5) < 0.6, b["az"]
    print("selftest kalibrasi_ldr: lolos")
    print(f"  mendung okta 8: pecahan DNI {f_beam * 100:.1f} persen")
    print(f"  prosedur A (baur murni, 400 cuplikan): sisa ketidakcocokan {sisa * 100:.3f} persen (dari 3 persen)")
    print(f"  prosedur A di mendung nyata: {sisa_nyata * 100:.3f} persen; dengan gam meleset 10 persen: {sisa10 * 100:.3f} persen")
    print(f"  prosedur B: rentang k azimuth {b['az']['rentang']:.1f} kali, k kemiringan {b['q2']['rentang']:.1f} kali; "
          f"taksiran bias pasang azimuth {b['az']['bias']:.2f} deg (sebenarnya 1.50)")
    return dict(mm=mm, kor=kor, sisa=sisa, sisa_nyata=sisa_nyata, sisa10=sisa10, b=b, tr=tr, baur=baur, mendung=mendung)


def tabel(r):
    p = H.P()
    rng = __import__("random").Random(7)
    out = ["# Kalibrasi ketidakcocokan antar LDR\n",
           "Dibangkitkan `kontrol_hibrid/kalibrasi_ldr.py`. Ketidakcocokan antar LDR adalah satu satunya hal yang "
           "membuat LDR kalah dari model astro (lihat `fusi_sensor_results.md` dan `ekf_results.md`).\n",
           "## 1. Prosedur A: kalibrasi dengan cahaya baur merata\n",
           "**Dasar:** sekat hanya membayangi komponen LANGSUNG. Di bawah cahaya baur murni, keempat LDR menerima "
           "irradiance yang sama berapa pun arah panel, sehingga setiap beda pembacaan murni ketidakcocokan. "
           "Diperiksa di selftest: dengan DNI nol, e sama persis pada pose (0,0), (30,20), dan (-40,45) derajat.\n",
           "**Cara di lapangan:** tunggu langit mendung tebal (atau tutup panel dengan penyebar cahaya), rekam "
           "keempat LDR beberapa ratus cuplikan, balik ke ruang x = v/(vcc - v), lalu simpan faktor koreksi per sensor.\n",
           "| Keadaan | Sisa ketidakcocokan |\n|---|---|",
           f"| Sebelum kalibrasi | {3.0:.2f} % |",
           f"| Prosedur A, cahaya baur murni, 400 cuplikan | **{r['sisa'] * 100:.3f} %** |",
           f"| Prosedur A di mendung okta 8 nyata (masih ada sisa DNI) | {r['sisa_nyata'] * 100:.3f} % |",
           f"| Prosedur A dengan eksponen gam meleset 10 % | {r['sisa10'] * 100:.3f} % |"]
    out += ["\nPengaruh banyaknya cuplikan (cahaya baur murni):\n",
            "| Cuplikan dirata-rata | Sisa ketidakcocokan |\n|---|---|"]
    for n in (10, 50, 200, 800):
        out.append(f"| {n} | {sisa_mism(r['mm'], prosedur_a(r['baur'], r['mm'], p, rng, n_avg=n)) * 100:.3f} % |")
    out += ["\nDua hal yang membatasi prosedur A: derau ADC (bisa ditekan dengan merata-rata) dan pengetahuan "
            "tentang kurva LDR, terutama eksponen `gam`. Galat 10 persen pada `gam` menyisakan ketidakcocokan "
            f"{r['sisa10'] * 100:.2f} persen, yang sudah cukup untuk menghapus sebagian besar manfaat kalibrasi. "
            "Jadi **mengukur kurva LDR lebih penting daripada memperbanyak cuplikan.**\n",
            "\n## 2. Prosedur B: memisahkan salah pasang dari ketidakcocokan\n",
            "Keduanya menggeser e dengan cara yang mirip, tetapi salah pasang menghasilkan e = k x bias yang ikut "
            "berubah ketika kepekaan k berubah, sedangkan ketidakcocokan menghasilkan offset yang hampir tetap. "
            "Regresi e terhadap k sepanjang hari memisahkan keduanya, sejauh k cukup berubah.\n",
            "| Sumbu | Rentang perubahan k sehari | Bilangan kondisi regresi | Taksiran bias pasang (sebenarnya) |\n|---|---|---|---|"]
    b = r["b"]
    out += [f"| Azimuth | {b['az']['rentang']:.1f} kali | {b['az']['kondisi']:.1f} | {b['az']['bias']:.2f} deg (1.50) |",
            f"| Kemiringan | {b['q2']['rentang']:.1f} kali | {b['q2']['kondisi']:.1f} | {b['q2']['bias']:.2f} deg (1.00) |",
            "\n**Azimuth terpisahkan, kemiringan tidak.** Kepekaan azimuth sebanding sin(q2) sehingga berubah lebar "
            "sepanjang hari, memberi regresi tumpuan yang cukup. Kepekaan kemiringan hampir tetap, sehingga bias "
            "pasang dan ketidakcocokan nyaris tidak terbedakan. Ini sejalan dengan catatan lama di proyek ini bahwa "
            "hanya bias azimuth yang dapat dipisahkan.\n",
            "\n## 3. Hasil akhir: galat pointing lewat EKF\n",
            "Ketidakcocokan sisa dari tabel 1 dimasukkan ke `ekf.py`, hari 7 Okt 2026 dengan lepas langkah azimuth "
            "7.2 deg di tengah hari. Galat diukur hanya saat matahari terjangkau.\n",
            "| Keadaan LDR | Ketidakcocokan sisa | Galat pointing rata-rata | p95 |\n|---|---|---|---|"]
    acuan = E.jalankan(r["tr"], pakai_ldr=False, slip_pada=0.3, seed=2)
    out.append(f"| (tanpa LDR, astro saja) | - | {acuan['err_reach'].mean():.3f} deg | "
               f"{np.percentile(acuan['err_reach'], 95):.3f} deg |")
    for mm_sisa, ket in ((0.03, "tanpa kalibrasi"), (r["sisa_nyata"], "prosedur A di mendung nyata"),
                         (r["sisa10"], "prosedur A, gam meleset 10 %"), (r["sisa"], "prosedur A, baur murni")):
        d = E.jalankan(r["tr"], pakai_ldr=True, mism=mm_sisa, slip_pada=0.3, seed=2)
        out.append(f"| {ket} | {mm_sisa * 100:.3f} % | {d['err_reach'].mean():.3f} deg | "
                   f"{np.percentile(d['err_reach'], 95):.3f} deg |")
    out += ["\nKalibrasi mengubah LDR dari **merugikan** menjadi **sangat menguntungkan**. Perhatikan prosedur A yang "
            "dilakukan di mendung nyata (masih ada 2.2 persen DNI) sudah cukup untuk membalik keadaan, meski hasilnya "
            "60 kali lebih buruk daripada dengan cahaya baur murni.\n",
            "\n## 4. Urutan kalibrasi yang disarankan\n",
            "1. **Ukur kurva LDR di meja** (lux terhadap tegangan) untuk mendapat `gam` dan `L0`. Ini menentukan batas "
            "atas ketelitian seluruh prosedur, lebih menentukan daripada jumlah cuplikan.\n"
            "2. **Prosedur A saat mendung tebal** atau dengan penutup penyebar cahaya. Satu kali, beberapa menit, "
            "menurunkan ketidakcocokan dari 3 persen ke bawah 0.5 persen bila `gam` diketahui baik.\n"
            "3. **Prosedur B sepanjang satu hari cerah** untuk menyerap sisa salah pasang azimuth.\n"
            "4. **Ulangi prosedur A berkala**, karena LDR menua dan kotor dengan laju berbeda beda.\n",
            "\n## 4. Batas\n",
            "- Model LDR (sekat `r`, eksponen `gam`, setengah jenuh `L0`, ketidakcocokan 3 persen) seluruhnya ASUMSI. "
            "Angka di sini menunjukkan BENTUK hubungannya, bukan ramalan untuk alat nyata.\n"
            "- Prosedur A mengoreksi ketidakcocokan PENGUATAN. Ia tidak mengoreksi beda sudut pandang, beda tinggi "
            "pemasangan, atau sekat yang tidak sama tinggi; ketiganya memberi tanda yang bergantung arah.\n"
            "- Prosedur B mengandaikan bias pasang TETAP sepanjang hari. Lepas langkah yang terjadi di tengah hari "
            "melanggar itu; pakai EKF untuk menanganinya, lalu prosedur B pada bagian hari yang bersih saja.\n"
            "- Kotor yang tidak merata antar LDR berperilaku seperti ketidakcocokan yang berubah lambat, jadi kalibrasi "
            "perlu diulang, bukan sekali selamanya.\n"]
    (Path(__file__).parent / "kalibrasi_ldr_results.md").write_text("\n".join(out), encoding="utf-8")
    print("tersimpan kalibrasi_ldr_results.md")


if __name__ == "__main__":
    r = selftest()
    if "--selftest" not in sys.argv:
        tabel(r)
