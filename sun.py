"""Posisi matahari + irradiance di PENS (Surabaya), dengan efek awan."""
import numpy as np
import pandas as pd
import pvlib

LAT, LON, TZ = -7.276006, 112.794327, "Asia/Jakarta"  # titik peletakan panel di PENS
loc = pvlib.location.Location(LAT, LON, TZ, altitude=5, name="PENS")


def sun_day(date="2026-10-04", freq="5min", okta=0):
    """Az/el matahari + GHI/DNI/DHI (W/m2). okta = tutupan awan 0..8."""
    t = pd.date_range(date, f"{date} 23:55", freq=freq, tz=TZ)
    sp = loc.get_solarposition(t)
    ghi = loc.get_clearsky(t).ghi * (1 - 0.75 * (okta / 8) ** 3.4)  # Kasten-Czeplak
    # ponytail: Erbs decomposition, kasar; ganti dengan data awan nyata (Open-Meteo/ERA5) jika perlu
    er = pvlib.irradiance.erbs(ghi, sp.zenith, t)
    return pd.DataFrame({"az": sp.azimuth, "el": sp.apparent_elevation, "zen": sp.zenith,
                         "ghi": ghi, "dni": er.dni, "dhi": er.dhi})


def poa(d, tilt, saz):
    """Irradiance di bidang panel (W/m2). tilt dari horizontal, saz = azimuth normal panel (CW dari utara)."""
    return pvlib.irradiance.get_total_irradiance(
        tilt, saz, d.zen, d.az, d.dni, d.ghi, d.dhi).poa_global.fillna(0)


def lux(w_m2):
    return w_m2 * 110  # ~110 lm/W efikasi cahaya matahari


if __name__ == "__main__":
    d = sun_day()
    noon = d.el.idxmax()
    assert d.el.max() > 60 and d.ghi.max() > 700, "matahari siang hari PENS harus tinggi"
    assert sun_day(okta=8).ghi.max() < 0.4 * d.ghi.max(), "awan penuh harus memotong GHI"
    print(f"Puncak {noon:%H:%M}  el={d.el.max():.1f}  GHI={d.ghi.max():.0f} W/m2 (~{lux(d.ghi.max()):.0f} lux)")
