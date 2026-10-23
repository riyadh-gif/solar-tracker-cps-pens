"""Node ROS 2 PENGGERAK MOTOR di Raspberry Pi: dua stepper loop tertutup, azimuth dan kemiringan.

FUNGSI 1 dari tiga fungsi Raspi. Node ini satu-satunya yang menyentuh GPIO.

Masukan : /solar_tracker/azimuth_cmd, /solar_tracker/elevation_cmd (Float64, radian, sudut BEBAN)
Keluaran: /motor/state (JSON) dan pulsa STEP/DIR ke driver

RANTAI SUDUT, dan ini sumber kekeliruan yang mudah terjadi:
    sudut BEBAN (yang diperintahkan)  --(reduksi gigi N)--  sudut MOTOR  --(microstep)--  jumlah langkah
Sumbu kemiringan punya N = 5, azimuth N = 1. Jadi satu derajat di panel kemiringan butuh lima kali
lebih banyak langkah daripada satu derajat di azimuth. Faktornya diambil dari dyn_model.AXES,
bukan ditulis ulang di sini.

LOOP TERTUTUP ADA DI DRIVER, BUKAN DI SINI. Raspi hanya mengirim pulsa; enkoder ada di dalam driver
dan tidak terbaca host. Yang bisa dibaca host hanyalah pin ALARM driver, yang menyala saat driver
menyerah mengejar galat. Karena itu node ini TIDAK menutup loop posisi; ia melaporkan alarm dan
menyerahkan taksiran simpangan sisi beban ke ekf_node.

MODE TIRUAN (bawaan): tanpa pustaka lgpio atau tanpa perangkat keras, node tetap jalan dan
mengintegrasikan posisi secara internal, supaya seluruh rantai ROS bisa diuji tanpa motor.
Pi 5 memakai lgpio; RPi.GPIO TIDAK bekerja di Pi 5.
"""
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
for sub in (".", "dinamika"):
    sys.path.insert(0, str(ROOT / sub))

import dyn_model as D          # noqa: E402  (N, microstep: satu sumber kebenaran)

import rclpy                   # noqa: E402
from rclpy.node import Node    # noqa: E402
from rcl_interfaces.msg import ParameterDescriptor   # noqa: E402
from std_msgs.msg import Float64, String             # noqa: E402

# Peta pin BCM. ASUMSI, menunggu peta pin sebenarnya dari pengguna.
DEFAULTS = dict(
    enabled=True, tiruan=True, hz=50.0,
    az_pul=17, az_dir=27, az_ena=22, az_alm=23,
    el_pul=5, el_dir=6, el_ena=13, el_alm=19,
    ena_aktif_rendah=True,          # kebanyakan driver: ENA LOW = motor aktif
    # Batas laju PULSA driver, bukan pembentuk gerak. Pembentukan sudah dilakukan
    # sun_command_node (profil trapesium + ZVD), sama seperti di Gazebo.
    az_langkah_maks=20000.0, el_langkah_maks=20000.0,   # langkah/detik
    balik_az=False, balik_el=False,  # balik arah bila kabel terbalik
    pul_lebar_us=5.0,
    # Batas sudut BEBAN, deg, harus sama dengan build_model_v2.AZ_LIMIT dan EL_LIMIT.
    # Dijepit DI SINI, bukan hanya di pembangkit perintah: node ini satu satunya yang menyentuh
    # motor, jadi ia yang harus menolak melewati stopper mekanik apa pun yang dikirim kepadanya.
    # Terbukti perlu: profil perintah pernah mengeluarkan 45.072 deg pada batas 45 deg, yaitu
    # lewatan integrasi satu tick (tilt_rate 90 deg/s pada dt 10 ms memberi sampai 0.9 deg).
    az_batas=185.0, el_batas=45.0,
)


class Sumbu:
    """Satu sumbu: ubah sudut beban yang DIPERINTAHKAN menjadi langkah motor.

    SENGAJA TIDAK ADA PROFIL TRAPESIUM DI SINI, dan ini penting. Di Gazebo, pembentukan perintah
    (profil trapesium lalu input shaper ZVD) dikerjakan sun_command_node SEBELUM diterbitkan ke
    /solar_tracker/*_cmd; aktuatornya hanya mengikuti perintah itu dengan kejenuhan torsi.
    Menambahkan profil kedua di sini akan menapis ulang perintah yang sudah dibentuk dan merusak
    peredaman ZVD yang sudah divalidasi (lewatan azimuth 85,61 % turun ke 2,16 %).

    Satu-satunya pembatas di sini adalah LAJU LANGKAH MAKSIMUM, dan itu batas perangkat keras
    (driver tidak menerima pulsa lebih cepat dari itu), bukan pembentuk gerak. Padanannya di
    Gazebo adalah kejenuhan torsi, bukan profil.
    """

    def __init__(self, nama, ax, pin, balik, batas):
        self.nama, self.pin, self.balik = nama, pin, balik
        self.N = ax.N
        self.step_motor_deg = ax.step            # deg per microstep DI MOTOR
        # langkah per radian BEBAN = (N * deg_beban) / step_motor
        self.langkah_per_rad = self.N * math.degrees(1.0) / self.step_motor_deg
        self.batas = math.radians(batas)   # rad beban, simetris
        self.tgt = 0.0        # rad beban, perintah yang SUDAH dibentuk oleh sun_command_node
        self.pos = 0.0        # rad beban, posisi yang sudah benar-benar dikeluarkan sebagai pulsa
        self.langkah = 0      # akumulasi langkah bertanda
        self.alarm = False
        self.terjepit = False   # true bila perintah terakhir melewati batas dan dipotong

    def minta(self, nilai):
        """Terima perintah sudut beban (rad), jepit ke batas sumbu."""
        self.terjepit = abs(nilai) > self.batas
        self.tgt = max(-self.batas, min(self.batas, nilai))

    def maju(self, dt, langkah_per_detik_maks):
        """Ikuti target apa adanya, hanya dibatasi laju pulsa maksimum driver."""
        n_ingin = int(round((self.tgt - self.pos) * self.langkah_per_rad))
        batas = max(1, int(langkah_per_detik_maks * dt))
        n = max(-batas, min(batas, n_ingin))
        if n:
            self.pos += n / self.langkah_per_rad
            self.langkah += n
        return n


class Gpio:
    """Pembungkus lgpio. Pi 5 WAJIB lgpio; RPi.GPIO tidak bekerja di Pi 5."""

    def __init__(self, pins_keluar, pins_masuk):
        import lgpio
        self.l = lgpio
        self.h = lgpio.gpiochip_open(4)   # Pi 5: gpiochip4 adalah header 40 pin
        for p in pins_keluar:
            lgpio.gpio_claim_output(self.h, p, 0)
        for p in pins_masuk:
            lgpio.gpio_claim_input(self.h, p)

    def tulis(self, pin, nilai):
        self.l.gpio_write(self.h, pin, 1 if nilai else 0)

    def baca(self, pin):
        return self.l.gpio_read(self.h, pin)

    def tutup(self):
        try:
            self.l.gpiochip_close(self.h)
        except Exception:
            pass


def main():
    rclpy.init()
    n = Node("motor_node")
    for k, v in DEFAULTS.items():
        n.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
    p = lambda k: n.get_parameter(k).value

    pin = {
        "azimuth": dict(pul=int(p("az_pul")), dir=int(p("az_dir")), ena=int(p("az_ena")), alm=int(p("az_alm"))),
        "elevasi": dict(pul=int(p("el_pul")), dir=int(p("el_dir")), ena=int(p("el_ena")), alm=int(p("el_alm"))),
    }
    sb = {
        "azimuth": Sumbu("azimuth", D.AXES["azimuth"], pin["azimuth"], bool(p("balik_az")),
                         float(p("az_batas"))),
        "elevasi": Sumbu("elevasi", D.AXES["elevasi"], pin["elevasi"], bool(p("balik_el")),
                         float(p("el_batas"))),
    }

    gpio = None
    if not bool(p("tiruan")):
        try:
            keluar = [pin[s][k] for s in pin for k in ("pul", "dir", "ena")]
            masuk = [pin[s]["alm"] for s in pin]
            gpio = Gpio(keluar, masuk)
            for s in pin:   # aktifkan driver
                gpio.tulis(pin[s]["ena"], 0 if bool(p("ena_aktif_rendah")) else 1)
            n.get_logger().info(f"GPIO siap (lgpio, gpiochip4): {pin}")
        except Exception as e:
            n.get_logger().error(f"GPIO tidak tersedia, kembali ke mode TIRUAN: {e}")
            gpio = None

    pub = n.create_publisher(String, "/motor/state", 10)
    for nm, topik in (("azimuth", "/solar_tracker/azimuth_cmd"),
                      ("elevasi", "/solar_tracker/elevation_cmd")):
        n.create_subscription(Float64, topik,
                              (lambda s: lambda m: sb[s].minta(float(m.data)))(nm), 10)

    lebar = float(p("pul_lebar_us")) * 1e-6
    st = dict(t=time.time(), n=0)

    def kirim_pulsa(s, jml):
        """Keluarkan |jml| pulsa STEP dengan DIR sesuai tanda."""
        if gpio is None or jml == 0:
            return
        q = pin[s]
        maju = (jml > 0) != sb[s].balik
        gpio.tulis(q["dir"], 1 if maju else 0)
        for _ in range(abs(jml)):
            gpio.tulis(q["pul"], 1)
            time.sleep(lebar)
            gpio.tulis(q["pul"], 0)
            time.sleep(lebar)

    def tick():
        if not bool(p("enabled")):
            return
        sekarang = time.time()
        dt = min(0.5, max(1e-3, sekarang - st["t"]))
        st["t"] = sekarang
        lap = {}
        for nm, s in sb.items():
            lmaks = float(p("az_langkah_maks" if nm == "azimuth" else "el_langkah_maks"))
            jml = s.maju(dt, lmaks)
            kirim_pulsa(nm, jml)
            if gpio is not None:
                s.alarm = bool(gpio.baca(pin[nm]["alm"]))
            lap[nm] = dict(
                tgt_deg=round(math.degrees(s.tgt), 4),
                pos_deg=round(math.degrees(s.pos), 4),
                sisa_deg=round(math.degrees(s.tgt - s.pos), 4),
                langkah=s.langkah, langkah_tick=jml,
                N=s.N, langkah_per_deg_beban=round(s.langkah_per_rad * math.pi / 180.0, 3),
                batas_deg=round(math.degrees(s.batas), 2), terjepit=s.terjepit,
                alarm=s.alarm)
        st["n"] += 1
        pub.publish(String(data=json.dumps(dict(
            sumber="gpio" if gpio is not None else "tiruan", n=st["n"], **lap))))

    n.create_timer(1.0 / max(1.0, float(p("hz"))), tick)
    n.get_logger().info("motor_node siap (" + ("GPIO" if gpio else "TIRUAN") + "); "
                        + ", ".join(f"{k}: {v.langkah_per_rad * math.pi / 180.0:.2f} langkah/deg beban"
                                    for k, v in sb.items()))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if gpio is not None:
            for s in pin:   # nonaktifkan driver saat berhenti
                gpio.tulis(pin[s]["ena"], 1 if bool(p("ena_aktif_rendah")) else 0)
            gpio.tutup()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
