"""Dinamika satu sumbu: stepper hibrid (hukum torsi-sudut) + poros fleksibel + beban + angin dan gravitasi.
Model: rotor Jm digerakkan Tm = -Th sin(Nr (thm - cmd)) - bm wm, poros kaku-puntir k dan redaman c, beban JL.
Driver diasumsikan sumber arus ideal (tanpa penurunan torsi terhadap kecepatan, tanpa dinamika arus), langkah buka-loop.
Integrasi RK4 vektor atas banyak kasus. SEMUA parameter bertanda (asumsi) menunggu pengukuran atau CAD.
"""
import math
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
G_STEEL = 79e9


@dataclass
class Axis:
    name: str = "elevasi"
    Th: float = 1.5        # N.m torsi tahan 57HSE1.5N (nominal, dari nama motor; datasheet belum ada)
    Nr: int = 50           # gigi rotor (stepper 1.8 deg)
    Jm: float = 1.1e-4     # kg.m2 inersia rotor (asumsi, NEMA23; datasheet 57HSE1.5N belum ada)
    Kp: float = 0.0        # N.m/rad kekakuan tambahan loop posisi driver pada ROTOR. 0 = loop TERBUKA (stepper polos).
    #                        Kp besar = loop TERTUTUP: driver memaku rotor ke perintah, beban hanya merasakan poros k.
    k: float = 321.0       # N.m/rad kekakuan puntir poros (poros baja 8 mm, 100 mm; hitung G J / L) (asumsi)
    zeta: float = 0.02     # rasio redaman mode poros (asumsi)
    bm: float = 5e-3       # N.m.s/rad redaman rotor dan driver (asumsi)
    bl: float = 0.2        # N.m.s/rad redaman viskos beban, bantalan dan pillow block (asumsi, sangat menentukan dering)
    fc: float = 0.05       # N.m gesekan Coulomb beban (asumsi)
    JL: float = 0.05       # kg.m2 inersia beban (nilai AXES di bawah dari inersia/inersia_link.json)
    grav: float = 0.0      # N.m beban gravitasi tetap pada sumbu (elevasi: maks 1.274 dalam +-45 deg, panel 2.55 kg, g = 10)
    arm: float = 0.0668    # m lengan pusat tekanan angin terhadap sumbu engsel (ASUMSI: 0.15 x 0.445 m, sisi melintang engsel)
    area: float = 0.2982   # m2 luas panel 0.670 x 0.445 (MOS C-50-18-M)
    step: float = 1.8 / 16  # deg per microstep (DI MOTOR; di beban dibagi N, lihat step_beban)
    N: float = 1.0         # reduksi gigi motor:beban. 1.0 = penggerak langsung (desain lama).
    #                        Desain baru (build_model_v2.py): kemiringan 42/8 = 5.25, azimuth tetap 1.0.

    # --- besaran yang DIRUJUK KE SISI BEBAN. Dengan N = 1 semuanya sama dengan nilai motor,
    #     jadi seluruh hasil penggerak langsung tidak bergeser satu digit pun. ---
    @property
    def Th_L(self):
        """Langit-langit torsi di beban (N.m): reduksi melipatkan torsi N kali."""
        return self.Th * self.N

    @property
    def Nr_L(self):
        """Gigi rotor efektif di beban: satu siklus listrik di motor menjadi 1/N putaran di beban."""
        return self.Nr * self.N

    @property
    def Kp_L(self):
        """Penguatan posisi driver dirujuk ke beban: naik N kuadrat."""
        return self.Kp * self.N ** 2

    @property
    def Jm_L(self):
        """Inersia rotor dirujuk ke beban: naik N kuadrat."""
        return self.Jm * self.N ** 2

    @property
    def step_beban(self):
        """Microstep yang terlihat DI BEBAN (deg). Reduksi memperhalus resolusi N kali."""
        return self.step / self.N

    @property
    def c(self):
        return 2 * self.zeta * math.sqrt(self.k * self.Jm * self.JL / (self.Jm + self.JL))

    def K_ser(self):
        """Kekakuan kecil-sinyal yang DIRASAKAN BEBAN: pegas magnet (Th Nr) diperkuat loop posisi driver (Kp),
        seri dengan poros k. Satu-satunya sumber kebenaran; build_model, sun_command_node, wind_node dan
        osilasi_energi semuanya memanggil ini. Kp = 0 memberi nilai loop terbuka lama persis.
        Reduksi gigi N menaikkan pegas magnet yang dirujuk ke beban sebesar N kuadrat; poros k sudah di
        sisi beban jadi tidak diskalakan. N = 1 memberi hasil penggerak langsung lama persis."""
        return 1 / (1 / (self.Th_L * self.Nr_L + self.Kp_L) + 1 / self.k)

    def freqs_hz(self):
        """Frekuensi alami (Hz) dari rotor yang tertahan kekakuan stepper, poros k, dan beban JL.
        Semuanya dirujuk ke sisi beban supaya reduksi gigi ikut terhitung."""
        Kh = self.Th_L * self.Nr_L + self.Kp_L
        K = np.array([[Kh + self.k, -self.k], [-self.k, self.k]])
        M = np.diag([self.Jm_L, self.JL])
        w2 = np.linalg.eigvals(np.linalg.inv(M) @ K).real
        return np.sqrt(np.sort(w2)) / (2 * math.pi)


def wind_torque(ax, v, cn=1.2, rho=1.2):
    """Momen angin (N.m) dari kecepatan v (m/s): 0.5 rho v2 Cn A arm. Cn dan arm asumsi."""
    return 0.5 * rho * v ** 2 * cn * ax.area * ax.arm


def rotor_offset(ax, tau, iters=60):
    """Simpangan rotor e (rad) yang menahan torsi tau: selesaikan sat(Th sin(Nr e) + Kp e, +-Th) = tau.

    Kp = 0 memakai bentuk tertutup asin(tau/Th)/Nr yang lama, jadi hasil loop terbuka tidak bergeser
    satu digit pun. Kp > Th Nr membuat g(e) monoton sehingga bisection sah."""
    if ax.Kp == 0.0:
        return np.arcsin(np.clip(tau / ax.Th_L, -1, 1)) / ax.Nr_L
    t = np.clip(np.asarray(tau, dtype=float), -ax.Th_L, ax.Th_L)
    hi = np.abs(t) / ax.Kp_L + math.pi / (2 * ax.Nr_L)
    lo = -hi
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        turun = ax.Th_L * np.sin(ax.Nr_L * mid) + ax.Kp_L * mid < t
        lo = np.where(turun, mid, lo)
        hi = np.where(turun, hi, mid)
    return 0.5 * (lo + hi)


def integrate(ax, cmd, tau, T, tau0=0.0, M=1, dt=1e-4, rec=10):
    """cmd(t) dan tau(t) mengembalikan larik (M,) (rad, N.m). Awal: kesetimbangan statis dengan beban tau0.
    Kembalikan t, thl, thm_dev (Nr (thm - cmd) pada rotor, untuk deteksi lepas sinkron)."""
    th0 = cmd(0.0) - rotor_offset(ax, tau0)
    y = np.array([th0, np.zeros(M), th0 - tau0 / ax.k, np.zeros(M)], dtype=float)
    c, n = ax.c, int(round(T / dt))
    ts, thl, dev = [], [], []

    def f(t, y, cm, tw):
        thm, wm, tl, wl = y
        e = thm - cm
        # Kejenuhan di Th berlaku untuk pegas magnet DAN loop driver: driver tidak bisa melebihi torsi tahan.
        # Kp = 0 membuat clip tidak aktif (|Th sin| <= Th), jadi identik dengan model loop terbuka lama.
        tm = -np.clip(ax.Th_L * np.sin(ax.Nr_L * e) + ax.Kp_L * e, -ax.Th_L, ax.Th_L) - ax.bm * wm
        tsh = ax.k * (thm - tl) + c * (wm - wl)
        return np.array([wm, (tm - tsh) / ax.Jm_L, wl, (tsh - ax.bl * wl - ax.fc * np.tanh(wl / 2e-3) - tw) / ax.JL])

    for i in range(n):
        t = i * dt
        c0, c1, c2 = cmd(t), cmd(t + dt / 2), cmd(t + dt)
        w0, w1, w2 = tau(t), tau(t + dt / 2), tau(t + dt)
        k1 = f(t, y, c0, w0)
        k2 = f(t + dt / 2, y + dt / 2 * k1, c1, w1)
        k3 = f(t + dt / 2, y + dt / 2 * k2, c1, w1)
        k4 = f(t + dt, y + dt * k3, c2, w2)
        y = y + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        if i % rec == 0:
            ts.append(t)
            thl.append(y[2].copy())
            dev.append(ax.Nr * (y[0] - c0))
    return np.array(ts), np.array(thl), np.array(dev)


def step_response(ax, T=3.0, t0=0.1):
    """Satu microstep pada t0: overshoot, waktu menetap 2%, frekuensi dering (FFT) dan amplitudo sisa."""
    s = math.radians(ax.step)
    t, th, _ = integrate(ax, lambda t: np.array([s if t >= t0 else 0.0]), lambda t: np.zeros(1), T)
    th = th[:, 0] / s
    i0 = int(t0 / (t[1] - t[0]))
    after = th[i0:]
    over = max(0.0, after.max() - 1.0) * 100
    out = np.flatnonzero(np.abs(after - 1.0) > 0.02)
    ts_settle = (out[-1] + 1) * (t[1] - t[0]) if len(out) else 0.0
    # frekuensi dering dari persilangan nilai akhir (FFT didominasi transien naik saat teredam)
    cr = np.flatnonzero(np.diff(np.sign(after - after[-1])) != 0)
    fdom = 1.0 / (2 * np.mean(np.diff(cr[:6])) * (t[1] - t[0])) if len(cr) >= 3 else float("nan")
    return dict(overshoot_pct=over, settle_s=ts_settle, f_ring_hz=float(fdom), final_norm=float(th[-1]), t=t, th_norm=th)


def rate_sweep(ax, freqs, T=4.0):
    """Gerak kontinu dengan frekuensi langkah f (langkah/detik): riak sudut beban (puncak ke puncak, deg) terhadap tren linear."""
    s = math.radians(ax.step)
    fr = np.asarray(freqs, float)
    t, th, _ = integrate(ax, lambda t: s * np.floor(fr * t + 1e-9), lambda t: np.zeros(len(fr)), T, M=len(fr))
    m = t > 0.4 * T
    out = []
    for j in range(len(fr)):
        p = np.polyfit(t[m], th[m, j], 1)
        r = th[m, j] - np.polyval(p, t[m])
        out.append((np.degrees(r.max() - r.min()), np.degrees(p[0]) / (fr[j] * ax.step)))
    return np.array(out)   # (riak pp deg, rasio kecepatan terukur / kecepatan perintah)


def wind_gust(ax, speeds, tau0=0.0, T=4.0, ramp=0.3, t0=0.5):
    """Hembusan naik linear dalam 'ramp' detik dari 0 ke momen angin v: defleksi puncak beban (deg), defleksi akhir, dan lepas sinkron."""
    sp = np.asarray(speeds, float)
    tw = wind_torque(ax, sp)
    cmd = lambda t: np.zeros(len(sp))
    tau = lambda t: tau0 + tw * min(1.0, max(0.0, (t - t0) / ramp))   # tau0 (gravitasi) tetap bekerja sepanjang waktu
    t, th, dev = integrate(ax, cmd, tau, T, tau0=tau0, M=len(sp))
    base = th[0]
    peak = np.degrees(np.abs(th - base).max(axis=0))
    final = np.degrees(np.abs(th[-1] - base))
    lost = np.abs(dev).max(axis=0) > math.pi / 2
    return dict(tau=tw + tau0, peak_deg=peak, final_deg=final, lost=lost)


def self_check():
    ax = Axis()
    # (a) kesetimbangan statis dengan beban tetap: tidak bergerak
    t, th, _ = integrate(ax, lambda t: np.zeros(1), lambda t: np.full(1, 0.5), 0.5, tau0=0.5)
    assert np.abs(th - th[0]).max() < 1e-6, "kesetimbangan awal tidak statis"
    # (b) frekuensi dering FFT cocok dengan nilai eigen (mode rendah, beban berat dibanding rotor)
    f_lo = ax.freqs_hz()[0]
    r0 = step_response(replace(ax, fc=0.0), T=3.0)   # tanpa Coulomb agar osilasi tidak mati dini
    assert abs(r0["f_ring_hz"] - f_lo) / f_lo < 0.05, (r0["f_ring_hz"], f_lo)
    # (c) satu microstep akhirnya tercapai (dengan gesekan)
    r = step_response(ax, T=3.0)
    assert abs(r["th_norm"][-1] - 1.0) < 0.05, r["th_norm"][-1]
    # (d) torsi angin di bawah torsi tahan tidak membuat lepas sinkron, di atasnya ya
    v_lo, v_hi = math.sqrt(0.5 * ax.Th / (0.5 * 1.2 * 1.2 * ax.area * ax.arm)), math.sqrt(1.6 * ax.Th / (0.5 * 1.2 * 1.2 * ax.area * ax.arm))
    w = wind_gust(ax, [v_lo, v_hi])
    assert (not w["lost"][0]) and w["lost"][1], w["lost"]
    # (e) beban gravitasi tetap tidak boleh menimbulkan defleksi sendiri (bug lama: tau0 hanya dipakai di kondisi awal)
    g = wind_gust(replace(ax, grav=0.64), [2.0], tau0=0.64)
    assert g["peak_deg"][0] < 0.3, g["peak_deg"]
    print("self-check dinamika: lolos")


# JL, k elevasi, grav dari inersia/ (motor 1.2 kg dan panel 1.5 kg adalah ASUMSI; J_el 0.0543 (sumbu x searah rel), J_az(q2=0) 0.1514; sensitivitas massa motor 0.9 sampai 1.6 kg mengubah J_az kurang dari 1.5%)
# k azimuth 1600: poros motor 8 mm sepanjang 20 mm (asumsi); arm angin azimuth 0.152 m (ASUMSI, eksentrisitas 0.23 x sisi panjang 0.670 m)
# Penguatan posisi driver loop tertutup. ASUMSI: datasheet 57HSE1.5N belum ada.
#
# BATAS FISIS YANG MENENTUKAN PILIHAN INI. Torsi driver tetap dibatasi torsi tahan Th, jadi kekakuan
# kecil-sinyal K_ser hanya terwujud selama simpangan rotor di bawah e_jenuh = Th / (Th Nr + Kp).
# Di luar itu torsi jenuh dan sistem jadi terbatas-torsi, bukan terbatas-kekakuan.
#
#     Kp      K_ser   f_n az   e_jenuh      terhadap microstep 0.1125 deg
#       0      71.6   3.46 Hz  1.1459 deg   10.2 x      loop terbuka
#     150     197.3   5.74 Hz  0.3820 deg    3.4 x
#     300     303.8   7.13 Hz  0.2292 deg    2.0 x      <- KP_NOM
#     500     423.0   8.41 Hz  0.1495 deg    1.3 x
#    1500     793.7  11.52 Hz  0.0546 deg    0.5 x      jenuh dalam satu microstep
#     1e9    1600.0  16.36 Hz  0.0000 deg    0.0 x      TIDAK FISIS
#
# "Rotor terpaku" (Kp -> tak hingga) TIDAK dapat dipakai sebagai batas konservatif: pada nilai itu
# simpangan linier runtuh ke nol, torsi selalu jenuh, dan f_n 16,36 Hz tidak pernah terwujud pada
# amplitudo mana pun. ODE di osilasi_energi.py membuktikannya: ia mengukur dering 6,20 Hz, bukan 16,36.
#
# KP_NOM = 300 dipilih sebagai nilai terkaku yang masih menyisakan jangkauan linier sekitar dua
# microstep, yaitu syarat minimum agar loop posisi punya arti di antara langkah. Ganti begitu
# datasheet tiba. Kp = 0 mengembalikan seluruh perilaku loop terbuka.
# DIPAKAI. Stepper memang LOOP TERTUTUP (dikonfirmasi pengguna), jadi AXES di bawah memakai Kp=KP_NOM.
#
# Pemilihan nilai dibatasi KEJENUHAN, bukan oleh keinginan kaku. Torsi driver tetap dibatasi Th, jadi
# kekakuan kecil-sinyal hanya terwujud selama simpangan rotor di bawah e_jenuh = Th / (Th Nr + Kp):
#     Kp     e_jenuh di motor    terhadap microstep 0.1125 deg
#       0        1.1459 deg           10.2 x
#     300        0.2292 deg            2.0 x   <- KP_NOM
#    1000        0.0799 deg            0.7 x   jenuh dalam satu microstep
#     1e9        0.0000 deg            0.0 x   TIDAK FISIS
# KP_NOM = 300 adalah nilai terkaku yang masih menyisakan sekitar dua microstep jangkauan linier.
#
# SEBERAPA PENTING NILAI INI, dan jawabannya tidak simetris:
#   kemiringan (bergigi 5:1): f_n hanya 8.06 .. 8.41 Hz untuk Kp = 0 sampai tak hingga, sebaran 4.3 %.
#       Reduksi menaikkan pegas magnet yang dirujuk ke beban ke 1875 N.m/rad ke atas, jauh melewati
#       poros 163.8, sehingga POROS yang menentukan dan penguatan driver hampir tidak berpengaruh.
#   azimuth (penggerak langsung): f_n 3.46 .. 16.36 Hz, sebaran 373 %. Di sini nilai Kp menentukan
#       segalanya dan datasheet driver masih DIBUTUHKAN.
KP_NOM = 300.0

# AXES adalah DESAIN BARU (desainbaru/, dibangun build_model_v2.py). Desain lama disimpan sebagai
# AXES_V1 hanya untuk membandingkan hasil historis; tidak ada kode yang memakainya.
#
# Yang berubah terhadap desain lama:
#   reduksi gigi 30/6 = 5.00 pada KEMIRINGAN (azimuth tetap penggerak langsung).
#     DIUKUR PENGGUNA di alat: 5 putaran pinion = 1 putaran gigi besar; diameter luar 22 dan 88 mm.
#     Itu konsisten dengan modul 2.75, z1 = 6, z2 = 30 (OD = m(z+2): 2.75*8 = 22, 2.75*32 = 88;
#     diameter jarak bagi 16.5 dan 82.5, rasionya tepat 5). Jarak sumbu 49.5 mm, terukur 50.2 di CAD.
#     Nilai 42/8 = 5.25 yang sempat dipakai berasal dari mesh CAD dan SALAH: ia meramal pinion 20 mm.
#   panel hampir seimbang: gravitasi maks 0.884 N.m, pada q2 = 0 hanya 0.0088 N.m (dulu 0.276)
#   J_el 0.0543 -> 0.0587 kg.m2 (inersia dari mesh CAD baru)
#   k kemiringan 317 -> 163.8 N.m/rad: poros as_8mm TERUKUR 194 mm di CAD, bukan 100 mm yang
#     diasumsikan dulu. G J / L dengan baja 8 mm memberi 163.8.
AXES = {"elevasi": Axis(JL=0.0587, k=163.8, grav=0.884, N=30 / 6, Kp=KP_NOM),
        "azimuth": Axis(name="azimuth", k=1600.0, JL=0.1514, arm=0.152, grav=0.0, N=1.0, Kp=KP_NOM)}

# Desain LAMA, penggerak langsung, poros 100 mm. Arsip untuk perbandingan.
AXES_V1 = {"elevasi": Axis(JL=0.0543, k=317.0, grav=1.274, N=1.0),
           "azimuth": Axis(name="azimuth", k=1600.0, JL=0.1514, arm=0.152, grav=0.0, N=1.0)}


def md(rows, cols):
    s = "| " + " | ".join(cols) + " |\n|" + "---|" * len(cols) + "\n"
    for r in rows:
        s += "| " + " | ".join(f"{r[c]:.3g}" if isinstance(r[c], float) else str(r[c]) for c in cols) + " |\n"
    return s


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    self_check()
    out = ["# Tabel hasil dinamika (dibuat otomatis oleh dyn_model.py)\n",
           "SEMUA parameter adalah asumsi atau placeholder: inersia beban (menunggu CAD), kekakuan poros, redaman dan gesekan beban, lengan angin. Lihat dyn_results.md.\n"]
    # E-A: respons satu microstep
    rows, fig, ax_ = [], *plt.subplots(1, 2, figsize=(11, 3.8))
    for j, (nm, a0) in enumerate(AXES.items()):
        for den in (8, 16, 32):
            a = replace(a0, step=1.8 / den)
            r = step_response(a)
            rows.append(dict(sumbu=nm, microstep=f"1/{den}", f_eigen_hz=float(a0.freqs_hz()[0]), overshoot_pct=r["overshoot_pct"], settle_s=r["settle_s"], final_norm=r["final_norm"]))
            if den == 16:
                ax_[j].plot(r["t"] - 0.1, r["th_norm"], label=f"{nm}, 1/16")
        ax_[j].set_xlim(-0.05, 2.0); ax_[j].axhline(1, c="k", lw=0.5); ax_[j].set_title(f"Respons satu microstep, {nm}"); ax_[j].set_xlabel("detik"); ax_[j].set_ylabel("sudut beban / langkah")
    plt.tight_layout(); plt.savefig(HERE / "dyn_step.png", dpi=90); plt.close()
    out += ["\n## A. Respons satu microstep (default). f_eigen = mode terendah analitik; final_norm = posisi akhir beban / langkah (1 = tercapai penuh)\n", md(rows, ["sumbu", "microstep", "f_eigen_hz", "overshoot_pct", "settle_s", "final_norm"])]
    # A2: sensitivitas redaman beban (kasus 1/16)
    rows = []
    for nm, a0 in AXES.items():
        for bl in (0.05, 0.2, 0.5, 1.0):
            r = step_response(replace(a0, bl=bl))
            rows.append(dict(sumbu=nm, bl=bl, overshoot_pct=r["overshoot_pct"], settle_s=r["settle_s"]))
    out += ["\n## A2. Sensitivitas redaman viskos beban (microstep 1/16)\n", md(rows, ["sumbu", "bl", "overshoot_pct", "settle_s"])]
    # E-B: sapuan frekuensi langkah
    f = np.logspace(0, math.log10(400), 36)
    fig, ax_ = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    rows = []
    for j, (nm, a0) in enumerate(AXES.items()):
        for den in (8, 16, 32):
            a = replace(a0, step=1.8 / den)
            res = rate_sweep(a, f)
            ax_[j].loglog(f, res[:, 0], label=f"1/{den}")
            k = int(np.argmax(np.where(f >= 2.0, res[:, 0], -1.0)))   # hanya laju valid (>= 2 langkah/detik)
            rows.append(dict(sumbu=nm, microstep=f"1/{den}", riak_maks_deg=float(res[k, 0]), di_langkah_per_s=float(f[k]), di_deg_per_s=float(f[k] * a.step),
                             riak_langkah_deg=a.step, f_eigen_langkah_per_s=float(a0.freqs_hz()[0])))
        ax_[j].axvline(a0.freqs_hz()[0], c="k", ls=":", lw=0.8)
        ax_[j].set_title(f"Riak sudut beban vs frekuensi langkah, {nm} (garis putus = f eigen)"); ax_[j].set_xlabel("langkah per detik"); ax_[j].legend(); ax_[j].grid(alpha=0.3, which="both")
    ax_[0].set_ylabel("riak puncak-ke-puncak (deg)")
    plt.tight_layout(); plt.savefig(HERE / "dyn_rate.png", dpi=90); plt.close()
    out += ["\n## B. Sapuan frekuensi langkah: riak terbesar dan lokasinya\n", md(rows, ["sumbu", "microstep", "riak_langkah_deg", "riak_maks_deg", "di_langkah_per_s", "di_deg_per_s", "f_eigen_langkah_per_s"])]
    # laju keyhole 0.5 dan 1 deg/s pada tiap microstep (acuan ke analisis_zenith)
    rows = []
    for nm, a0 in AXES.items():
        for den in (8, 16, 32):
            a = replace(a0, step=1.8 / den)
            fk = np.array([0.1, 0.5, 1.0, 2.0]) / a.step
            res = rate_sweep(a, fk)
            for fv, (rp, vr) in zip(fk, res):
                rows.append(dict(sumbu=nm, microstep=f"1/{den}", laju_deg_per_s=float(fv * a.step), langkah_per_s=float(fv), riak_pp_deg=float(rp), rasio_kecepatan=float(vr), valid="ya" if fv >= 2.0 else "tidak (kurang dari 2 langkah/detik)"))
    out += ["\n## B2. Riak pada laju keyhole (acuan analisis_zenith: 0.5 sampai 2 deg/s). Baris valid = tidak: jendela 4 s terlalu sedikit langkah untuk regresi tren, abaikan\n", md(rows, ["sumbu", "microstep", "laju_deg_per_s", "langkah_per_s", "riak_pp_deg", "rasio_kecepatan", "valid"])]
    # E-C: angin
    v = np.array([5, 10, 15, 20, 25, 30, 35.0])
    fig, ax_ = plt.subplots(figsize=(6, 4))
    rows = []
    for nm, a0 in AXES.items():
        w = wind_gust(a0, v, tau0=a0.grav)
        ax_.plot(v, w["peak_deg"], marker="o", label=f"{nm} (puncak)")
        for i in range(len(v)):
            rows.append(dict(sumbu=nm, angin_m_per_s=float(v[i]), torsi_Nm=float(w["tau"][i]), defleksi_puncak_deg=float(w["peak_deg"][i]), defleksi_akhir_deg=float(w["final_deg"][i]), lepas_sinkron=bool(w["lost"][i])))
    ax_.set_xlabel("kecepatan angin (m/s)"); ax_.set_ylabel("defleksi sudut beban (deg)"); ax_.legend(); ax_.grid(alpha=0.3)
    plt.tight_layout(); plt.savefig(HERE / "dyn_wind.png", dpi=90); plt.close()
    out += ["\n## C. Hembusan angin (naik 0.3 s, beban gravitasi awal = grav sumbu, elevasi 0.64 N.m kasus terburuk)\n", md(rows, ["sumbu", "angin_m_per_s", "torsi_Nm", "defleksi_puncak_deg", "defleksi_akhir_deg", "lepas_sinkron"])]
    (HERE / "dyn_results_tables.md").write_text("\n".join(out) + "\n", encoding="utf-8")
    print("selesai: dyn_results_tables.md")


if __name__ == "__main__":
    main() if "--full" in __import__("sys").argv else self_check()
