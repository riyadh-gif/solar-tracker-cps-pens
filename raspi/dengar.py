"""Ambil satu pesan dari tiap topik JSON dan tampilkan terbaca.

Memakai rclpy, BUKAN `ros2 topic echo --field data`: echo membungkus string JSON sebagai YAML,
jadi tanda kutip dan baris lipatan merusak json.load. Itu yang membuat uji pertama di Raspi
seolah gagal padahal node sehat.
"""
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Hanya topik bertipe std_msgs/String. /sun/state dan /hujan/state bertipe Float64MultiArray,
# jadi TIDAK boleh ada di sini: langganan dengan tipe salah tidak pernah menerima apa pun dan
# tampak seperti node yang mati. Konteks matahari tetap terbaca lewat /tracker/diag.
TOPIK = ["/motor/state", "/tracker/diag", "/ekf/state"]
TUNGGU_S = 15.0


def main():
    rclpy.init()
    n = Node("pendengar")
    kotak = {}
    for t in TOPIK:
        n.create_subscription(String, t, (lambda k: lambda m: kotak.setdefault(k, m.data))(t), 10)
    batas = n.get_clock().now().nanoseconds + TUNGGU_S * 1e9
    while n.get_clock().now().nanoseconds < batas and len(kotak) < len(TOPIK):
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node()
    rclpy.try_shutdown()

    def ambil(t):
        if t not in kotak:
            print(f"   TIDAK ADA pesan di {t}")
            return None
        try:
            return json.loads(kotak[t])
        except Exception as e:
            print(f"   {t} bukan JSON: {e}; mentah={kotak[t][:120]}")
            return None

    print("== 1. PENGGERAK 2 MOTOR ==")
    d = ambil("/motor/state")
    if d:
        print(f"   sumber={d['sumber']}  tick={d['n']}")
        for s in ("azimuth", "elevasi"):
            a = d[s]
            print(f"   {s:<8} N={a['N']:.2f}  {a['langkah_per_deg_beban']:8.2f} langkah/deg beban"
                  f"   tgt={a['tgt_deg']:9.3f}  pos={a['pos_deg']:9.3f}"
                  f"   sisa={a['sisa_deg']:+.4f}  langkah={a['langkah']:>7}"
                  f"   batas=+-{a.get('batas_deg')}  terjepit={a.get('terjepit')}"
                  f"  alarm={a['alarm']}")

    print()
    print("== 2. PEMERATAAN INTENSITAS CAHAYA (LDR) ==")
    d = ambil("/tracker/diag")
    e = None
    if d:
        L, e = d["ldr"], d["listrik"]
        print(f"   sumber={d['sumber']}  jam={d['jam']:.2f}")
        print(f"   v 4 kuadran = {[round(x, 4) for x in L['v']]}")
        print(f"   galat sumbu ex={L['ex']:+.5f}  ey={L['ey']:+.5f}  (0 = cahaya rata)")

    print()
    print("== 3. TEGANGAN DAN ARUS PANEL ==")
    if e:
        print(f"   arus    = {e['i_ukur']:.3f} A   (model imp {e['imp_model']:.3f} A,"
              f" selisih {e['selisih_arus_pct']:+.2f} %)")
        print(f"   tegangan= {e['v_ukur']:.2f} V")
        print(f"   daya    = {e['p_ukur']:.2f} W")

    print()
    print("== EKF: taksiran simpangan sisi beban dari 2 dan 3 ==")
    d = ambil("/ekf/state")
    if d:
        print("   " + str({a: round(b, 4) for a, b in list(d.items())[:8]}))

    print()
    print("== Konteks matahari (dari /tracker/diag) ==")
    d = ambil("/tracker/diag")
    if d:
        m, g = d["matahari"], d["galat"]
        print(f"   az={m['az']} el={m['el']} zen={m['zen']} deg;"
              f" dni={m['dni']} dhi={m['dhi']} ghi={m['ghi']} W/m2")
        print(f"   galat pointing: perintah={g['perintah']} deg,"
              f" sebenarnya={g.get('tiruan_sebenarnya')} deg, terjangkau={g['terjangkau']}")

    return 0 if "/motor/state" in kotak and "/tracker/diag" in kotak else 1


if __name__ == "__main__":
    sys.exit(main())
