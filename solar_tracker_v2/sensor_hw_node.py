"""Node ROS 2 PEMBACA PERANGKAT KERAS di Raspberry Pi: pengganti sensor_node.py pada alat NYATA.

Ini jembatan Digital Model -> Digital Shadow. Formatnya SENGAJA sama persis dengan sensor_node.py,
jadi ekf_node.py, control_web.py dan seluruh rantai di belakangnya jalan TANPA diubah sama sekali.

Perangkat keras (I2C, keduanya 3.3 V, alamat berbeda jadi bisa satu bus):
  ADS1115 di 0x48   4 LDR pada kanal A0..A3, 16 bit, PGA 4.096 V
  INA226  di 0x40   arus dan tegangan panel, shunt 0.01 ohm
                    (dipilih begitu karena 3 A hanya memberi 30 mV dan membuang 0.09 W;
                     INA219 dengan shunt 0.1 ohm akan membuang 0.9 W dan melewati rentangnya)

YANG TIDAK BISA DIBACA DI ALAT NYATA, dan ini pokok perkaranya:
Sudut BEBAN tidak terukur. Enkoder ada di dalam driver loop tertutup dan tidak terbaca host, lagipula
ia melihat ROTOR, bukan panel, jadi berada di seberang gigi dan poros. Karena itu node ini menerbitkan
sudut PERINTAH sebagai `sendi.cmd_*`, dan `sendi.beban_*` diisi perintah juga sebagai tebakan awal.
Selisih sebenarnya ditaksir ekf_node dari LDR dan arus. Lihat bagian 11 sistem_cps_tracker_pens.pdf.

Mode tiruan: tanpa perangkat keras (atau tanpa smbus2) node tetap jalan dan menerbitkan nilai tiruan,
supaya rantai ROS bisa diuji di meja. Lihat parameter `tiruan`.

Pakai:
  ros2 run ... sensor_hw_node.py --ros-args -p tiruan:=false -p i2c_bus:=1
"""
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
for sub in (".", "dinamika", "kontrol_hibrid", "cuaca"):
    sys.path.insert(0, str(ROOT / sub))

import sun                      # noqa: E402
import tracker                  # noqa: E402
import panel_listrik as PL      # noqa: E402

import rclpy                    # noqa: E402
from rclpy.node import Node     # noqa: E402
from rcl_interfaces.msg import ParameterDescriptor   # noqa: E402
from std_msgs.msg import Float64, Float64MultiArray, String   # noqa: E402

DEFAULTS = dict(enabled=True, tiruan=True, i2c_bus=1, hz=5.0,
                ads_addr=0x48, ina_addr=0x40, shunt_ohm=0.01, arus_maks=4.0,
                tamb=30.0, albedo=0.25, vcc=3.3)

# --- ADS1115 ---
ADS_KONV, ADS_KONF = 0x00, 0x01
ADS_MUX = {0: 0x4000, 1: 0x5000, 2: 0x6000, 3: 0x7000}   # single-ended A0..A3
ADS_PGA_4V096 = 0x0200
ADS_LSB_V = 4.096 / 32768.0

# --- INA226 ---
INA_KONF, INA_SHUNT, INA_BUS, INA_KAL = 0x00, 0x01, 0x02, 0x05
INA_SHUNT_LSB_V = 2.5e-6      # 2.5 uV per bit
INA_BUS_LSB_V = 1.25e-3       # 1.25 mV per bit


class I2C:
    """Pembungkus tipis smbus2. Semua register 16 bit big-endian."""

    def __init__(self, bus):
        from smbus2 import SMBus
        self.b = SMBus(bus)

    def tulis16(self, addr, reg, nilai):
        self.b.write_i2c_block_data(addr, reg, [(nilai >> 8) & 0xFF, nilai & 0xFF])

    def baca16(self, addr, reg, bertanda=False):
        d = self.b.read_i2c_block_data(addr, reg, 2)
        v = (d[0] << 8) | d[1]
        return v - 65536 if bertanda and v > 32767 else v


def baca_ldr(i2c, addr, kanal):
    """Satu pembacaan single-shot ADS1115 pada kanal, kembalikan volt."""
    konf = 0x8000 | ADS_MUX[kanal] | ADS_PGA_4V096 | 0x0100 | 0x0080 | 0x0003   # single-shot, 128 SPS
    i2c.tulis16(addr, ADS_KONF, konf)
    t0 = time.time()
    while time.time() - t0 < 0.05:
        if i2c.baca16(addr, ADS_KONF) & 0x8000:
            break
        time.sleep(0.001)
    mentah = i2c.baca16(addr, ADS_KONV, bertanda=True)
    return max(0.0, mentah * ADS_LSB_V)


def baca_panel(i2c, addr, shunt_ohm):
    """(arus A, tegangan V) dari INA226."""
    v_shunt = i2c.baca16(addr, INA_SHUNT, bertanda=True) * INA_SHUNT_LSB_V
    v_bus = i2c.baca16(addr, INA_BUS) * INA_BUS_LSB_V
    return v_shunt / shunt_ohm, v_bus


def ex_ey(v, vcc):
    """Sinyal selisih dari empat tegangan LDR [x+, x-, y+, y-]. Sama dengan hybrid_sim."""
    sx, sy = v[0] + v[1], v[2] + v[3]
    return ((v[0] - v[1]) / sx if sx > 1e-9 else 0.0,
            (v[2] - v[3]) / sy if sy > 1e-9 else 0.0)


def main():
    rclpy.init()
    n = Node("sensor_hw_node")
    for k, val in DEFAULTS.items():
        n.declare_parameter(k, val, ParameterDescriptor(dynamic_typing=True))
    p = lambda k: n.get_parameter(k).value

    pub_diag = n.create_publisher(String, "/tracker/diag", 10)
    pub_ldr = n.create_publisher(Float64MultiArray, "/ldr/state", 10)

    st = dict(sun=None, cmd=[0.0, 0.0], wh=0.0, jam_lalu=None)
    n.create_subscription(Float64MultiArray, "/sun/state",
                          lambda m: st.__setitem__("sun", list(m.data)), 10)
    for i, t in enumerate(("/solar_tracker/azimuth_cmd", "/solar_tracker/elevation_cmd")):
        n.create_subscription(Float64, t,
                              (lambda idx: lambda m: st["cmd"].__setitem__(idx, math.degrees(m.data)))(i), 10)

    i2c = None
    if not bool(p("tiruan")):
        try:
            i2c = I2C(int(p("i2c_bus")))
            i2c.tulis16(int(p("ina_addr")), INA_KONF, 0x4527)     # rata-rata 16, 1.1 ms, kontinu
            n.get_logger().info(f"I2C bus {p('i2c_bus')} siap: ADS1115 0x{int(p('ads_addr')):02x}, "
                                f"INA226 0x{int(p('ina_addr')):02x}")
        except Exception as e:
            n.get_logger().error(f"perangkat keras tidak terbaca, kembali ke mode TIRUAN: {e}")
            i2c = None

    def tick():
        if not bool(p("enabled")) or st["sun"] is None:
            return
        s = st["sun"]
        jam, az, el, ghi, dni, dhi, c1, c2 = s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]
        vcc, tamb, alb = float(p("vcc")), float(p("tamb")), float(p("albedo"))

        if i2c is not None:
            v = [baca_ldr(i2c, int(p("ads_addr")), k) for k in range(4)]
            arus, tegangan = baca_panel(i2c, int(p("ina_addr")), float(p("shunt_ohm")))
        else:
            # TIRUAN: pakai model yang sama dengan simulasi, supaya rantai ROS bisa diuji di meja
            import random
            import hybrid_sim as H
            P = H.P()
            nrm, xp, yp = H.basis(c1, c2)
            sv = tracker.sun_vec(az, el)
            # H.read hanya memberi (ex, ey, V rata-rata), sedangkan di sini perlu KEEMPAT tegangan,
            # jadi perhitungannya disusun ulang dari H.shade dan H.volt yang sama persis.
            cth = H.dot(nrm, sv)
            ex_a = math.atan2(H.dot(sv, xp), cth)
            ey_a = math.atan2(H.dot(sv, yp), cth)
            beam, dif = dni * max(cth, 0.0), dhi * (1 + nrm[2]) / 2
            sh = (H.shade(-ex_a, P.r), H.shade(ex_a, P.r), H.shade(-ey_a, P.r), H.shade(ey_a, P.r))
            rng = random.Random(0)   # H.volt memakai rng.gauss, jadi random.Random bukan numpy
            v = [H.volt(beam * (1 - sh[i]) + dif, P, rng) for i in range(4)]
            poa_t = PL.poa_efektif(math.radians(c1), math.radians(c2), az, el, dni, dhi, ghi, alb)
            r_t = PL.listrik(poa_t, PL.suhu_sel(poa_t, tamb))
            arus, tegangan = float(r_t["i_mp"]), float(r_t["v_mp"])

        ex, ey = ex_ey(v, vcc)

        # Sudut BEBAN tidak terukur di alat nyata: diisi perintah, selisihnya urusan ekf_node.
        b1, b2 = c1, c2
        poa = PL.poa_efektif(math.radians(b1), math.radians(b2), az, el, dni, dhi, ghi, alb)
        tc = PL.suhu_sel(poa, tamb)
        r = PL.listrik(poa, tc)
        daya = arus * tegangan

        galat = lambda q1, q2: math.degrees(math.acos(max(-1.0, min(1.0, float(
            tracker.fk(math.radians(q1), math.radians(q2)) @ tracker.sun_vec(az, el))))))
        g_cmd = galat(c1, c2)

        if st["jam_lalu"] is not None and 0 < jam - st["jam_lalu"] < 0.25:
            st["wh"] += daya * (jam - st["jam_lalu"])
        elif st["jam_lalu"] is not None and jam < st["jam_lalu"] - 0.5:
            st["wh"] = 0.0
        st["jam_lalu"] = jam

        d = dict(
            jam=round(jam, 4),
            sumber="perangkat keras" if i2c is not None else "tiruan",
            matahari=dict(az=round(az, 3), el=round(el, 3), zen=round(90 - el, 3),
                          dni=round(dni, 1), dhi=round(dhi, 1), ghi=round(ghi, 1)),
            sendi=dict(cmd_q1=round(c1, 4), cmd_q2=round(c2, 4), beban_q1=round(b1, 4), beban_q2=round(b2, 4),
                       d_q1=0.0, d_q2=0.0),
            galat=dict(perintah=round(g_cmd, 4), beban=round(g_cmd, 4), dinamik=0.0,
                       terjangkau=bool(el > 0 and abs(c2) < 44.9)),
            ldr=dict(v=[round(x, 4) for x in v], ex=round(ex, 6), ey=round(ey, 6), mism=[1.0] * 4),
            listrik=dict(poa=round(poa, 1), tcell=round(tc, 2),
                         i_ukur=round(arus, 4), v_ukur=round(tegangan, 3), p_ukur=round(daya, 3),
                         imp_model=round(float(r["i_mp"]), 4), vmp_model=round(float(r["v_mp"]), 3),
                         pmp_model=round(float(r["p_mp"]), 3),
                         selisih_arus_pct=round(100 * (arus / float(r["i_mp"]) - 1.0), 2)
                         if float(r["i_mp"]) > 1e-6 else 0.0),
            energi=dict(wh=round(st["wh"], 3)),
        )
        pub_diag.publish(String(data=json.dumps(d)))
        m = Float64MultiArray(); m.data = [float(x) for x in v] + [ex, ey]
        pub_ldr.publish(m)

    n.create_timer(1.0 / max(0.5, float(p("hz"))), tick)
    n.get_logger().info("sensor_hw_node siap (" + ("perangkat keras" if i2c else "TIRUAN") + ")")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
