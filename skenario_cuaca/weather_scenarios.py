"""Skenario cuaca (okta 0/4/8), rasio sinyal langsung F, energi tetap vs tracker ideal, sensitivitas dekomposisi.
Jalankan: python weather_scenarios.py  -> PNG + weather_scenarios_results.md di folder ini."""
import os, sys
import numpy as np, pandas as pd, pvlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from sun import sun_day  # noqa: E402

DAYS = {"21 Mar": "2026-03-21", "21 Jun": "2026-06-21", "04 Okt": "2026-10-04", "21 Des": "2026-12-21"}
SCEN = {"cerah (0)": 0, "berawan sebagian (4)": 4, "mendung tebal (8)": 8}
MODELS = ["erbs", "boland", "orgill_hollands", "disc"]
SAPUTRA = {"cerah (0)": 28.4, "berawan sebagian (4)": 19.7, "mendung tebal (8)": 10.0}
STEP_H = 5 / 60
I = pvlib.irradiance


def decomp(d, model):
    """Pecah GHI (dari sun.py) menjadi DNI/DHI dengan model pvlib pilihan."""
    t = d.index
    if model == "disc":
        dni = I.disc(d.ghi, d.zen, t).dni
    else:
        dni = getattr(I, model)(d.ghi, d.zen, t).dni
    cz = np.cos(np.radians(d.zen)).clip(lower=0)
    dni = dni.fillna(0).clip(lower=0).where(d.zen < 90, 0)
    dhi = (d.ghi - dni * cz).clip(lower=0)  # tutup persamaan GHI = DNI cosz + DHI
    return d.assign(dni=dni, dhi=dhi)


def run(day, okta, model="erbs"):
    d = decomp(sun_day(day, "5min", okta), model)
    up = d.zen < 90
    cz = np.cos(np.radians(d.zen)).clip(lower=0)
    d["f"] = (d.dni * cz / d.ghi).where(d.ghi > 1)
    sky = lambda tilt, saz: I.get_total_irradiance(tilt, saz, d.zen, d.az, d.dni, d.ghi, d.dhi).poa_global.fillna(0).where(up, 0)
    d["fix"] = sky(7, 0)          # tetap miring 7 deg, normal menghadap utara
    d["trk"] = sky(d.zen, d.az)   # tracker ideal: normal panel = arah matahari
    return d


def stats(d):
    kwh = lambda s: s.sum() * STEP_H / 1000
    day = d.el > 10
    ef, et = kwh(d.fix), kwh(d.trk)
    return dict(ghi=kwh(d.ghi), dni=kwh(d.dni), dhi=kwh(d.dhi),
                f_mean=d.f[day].mean(), f_w=(d.dni * np.cos(np.radians(d.zen)).clip(lower=0)).sum() / d.ghi.sum(),
                e_fix=ef, e_trk=et, gain=100 * (et / ef - 1))


def md(df, fmt="{:.2f}"):
    h = "| " + " | ".join(map(str, [df.index.name or ""] + list(df.columns))) + " |\n|" + "---|" * (len(df.columns) + 1) + "\n"
    return h + "".join("| " + " | ".join([str(i)] + [fmt.format(v) if isinstance(v, float) else str(v) for v in r]) + " |\n"
                       for i, r in df.iterrows())


# ---- komputasi ----
data = {(dn, sn): run(day, ok) for dn, day in DAYS.items() for sn, ok in SCEN.items()}
main = pd.DataFrame({k: stats(v) for k, v in data.items()}).T
main.index.names = ["hari", "skenario"]

sens_rows = []
for ok in (4, 6, 8):
    for m in MODELS:
        s = {dn: stats(run(day, ok, m)) for dn, day in DAYS.items()}
        sens_rows.append(dict(okta=ok, model=m, F_1004=s["04 Okt"]["f_w"], gain_1004=s["04 Okt"]["gain"],
                              F_rata4=np.mean([x["f_w"] for x in s.values()]),
                              gain_rata4=np.mean([x["gain"] for x in s.values()]),
                              ghi_1004=s["04 Okt"]["ghi"]))
sens = pd.DataFrame(sens_rows)
for c in ("F_rata4", "gain_rata4"):
    base = sens.groupby("okta")[c].transform("first")  # erbs = baris pertama tiap okta
    sens[c + "_vs_erbs"] = sens[c] - base

# ---- self-check ----
d0, d4, d8 = (data[("04 Okt", s)] for s in SCEN)
assert np.allclose(d8.ghi.max() / d0.ghi.max(), 0.25, atol=0.01), "Kasten-Czeplak okta 8 = 0.25 x cerah"
assert np.allclose((d0.dni * np.cos(np.radians(d0.zen)).clip(lower=0) + d0.dhi)[d0.zen < 90], d0.ghi[d0.zen < 90], atol=1e-6)
assert 0 <= main.f_w.min() and main.f_w.max() <= 1.0001
for dn in DAYS:
    assert main.loc[(dn, "cerah (0)"), "f_w"] > main.loc[(dn, "mendung tebal (8)"), "f_w"], "F harus turun saat mendung"
    assert main.loc[(dn, "cerah (0)"), "ghi"] > main.loc[(dn, "mendung tebal (8)"), "ghi"]
assert (main.xs("cerah (0)", level="skenario").e_trk > main.xs("cerah (0)", level="skenario").e_fix).all(), "tracker ideal harus menang saat cerah"
print("self-check OK")

# ---- plot ----
cols = dict(zip(SCEN, ["#d95f02", "#1b9e77", "#4a4a8a"]))
fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharey=True)
for a, dn in zip(ax.flat, DAYS):
    for sn in SCEN:
        d = data[(dn, sn)]; d = d[d.el > 5]
        a.plot(d.index.hour + d.index.minute / 60, d.f, color=cols[sn], label=sn)
    a.set_title(dn); a.set_ylim(0, 1.05); a.grid(alpha=.3)
for a in ax[1]: a.set_xlabel("jam lokal (WIB)")
for a in ax[:, 0]: a.set_ylabel("F = DNI cos(zen) / GHI")
ax[0, 0].legend(fontsize=8); fig.suptitle("Fraksi langsung horizontal F (Erbs, el > 5 deg)"); fig.tight_layout()
fig.savefig(os.path.join(HERE, "F_vs_waktu.png"), dpi=130); plt.close(fig)

fig, ax = plt.subplots(1, 3, figsize=(12, 3.6), sharey=True)
for a, sn in zip(ax, SCEN):
    d = data[("04 Okt", sn)]; h = d.index.hour + d.index.minute / 60
    for c, lb in (("ghi", "GHI"), ("dni", "DNI"), ("dhi", "DHI")): a.plot(h, d[c], label=lb)
    a.set_title(f"04 Okt, {sn}"); a.set_xlabel("jam lokal (WIB)"); a.grid(alpha=.3)
ax[0].set_ylabel("W/m2"); ax[0].legend(); fig.tight_layout()
fig.savefig(os.path.join(HERE, "irradiance_per_skenario.png"), dpi=130); plt.close(fig)

fig, a = plt.subplots(figsize=(8, 4)); w = 0.2
for i, dn in enumerate(DAYS):
    a.bar(np.arange(3) + (i - 1.5) * w, [main.loc[(dn, sn), "gain"] for sn in SCEN], w, label=dn)
a.plot(range(3), list(SAPUTRA.values()), "k_", ms=40, mew=2, label="Saputra 2026 (Palembang)")
a.set_xticks(range(3)); a.set_xticklabels(SCEN); a.set_ylabel("gain tracker ideal vs tetap (%)")
a.legend(fontsize=8, loc="lower left"); a.grid(axis="y", alpha=.3); fig.tight_layout()
fig.savefig(os.path.join(HERE, "gain_tracker.png"), dpi=130); plt.close(fig)

# ---- laporan md ----
m = main.copy()
m.columns = ["GHI kWh/m2", "DNI kWh/m2", "DHI kWh/m2", "F rata-rata (el>10)", "F berbobot energi", "E tetap kWh/m2", "E tracker kWh/m2", "gain %"]
gm = main.groupby(level="skenario").gain.mean()
fm = main.groupby(level="skenario").f_w.mean()
mean_tab = pd.DataFrame({"F berbobot energi (rata 4 hari)": fm, "gain % (rata 4 hari)": gm,
                         "gain % Saputra": pd.Series(SAPUTRA)}).loc[list(SCEN)]
mean_tab.index.name = "skenario"
st = sens.rename(columns={"F_1004": "F 04 Okt", "gain_1004": "gain % 04 Okt", "F_rata4": "F rata 4 hari",
                          "gain_rata4": "gain % rata 4 hari", "ghi_1004": "GHI 04 Okt kWh/m2",
                          "F_rata4_vs_erbs": "dF vs Erbs", "gain_rata4_vs_erbs": "dgain vs Erbs (poin %)"}).set_index(["okta", "model"])
tpl = open(os.path.join(HERE, "results_template.md"), encoding="utf-8").read()
out = tpl.format(
                 tab_main=md(main.reset_index().assign(idx=lambda x: x.hari + " | " + x.skenario).set_index("idx").rename_axis("hari | skenario").drop(columns=["hari", "skenario"]).rename(columns=dict(zip(main.columns, m.columns))), "{:.3f}"),
                 tab_mean=md(mean_tab, "{:.3f}"),
                 tab_sens=md(st.reset_index().assign(idx=lambda x: x.okta.astype(str) + " | " + x.model).set_index("idx").rename_axis("okta | model").drop(columns=["okta", "model"]), "{:.3f}"))
open(os.path.join(HERE, "weather_scenarios_results.md"), "w", encoding="utf-8").write(out)
assert "2014" not in out
print(mean_tab.round(3)); print(main.round(3).to_string()); print(sens.round(3).to_string())
