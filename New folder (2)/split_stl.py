#!/usr/bin/env python3
"""Pecah solar.stl (mm, satu mesh) jadi STL per komponen (meter), diputar 180 derajat di sumbu X supaya tegak
(rangka di bawah, mekanisme di atas). Keluaran: solar_stl/parts_v2/NN_<kelas>.stl, kelas dibaca sim lewat awalan indeks."""
import os, struct, sys
HERE = os.path.dirname(os.path.abspath(__file__))
SRC, OUT = os.path.join(HERE, "solar.stl"), os.path.join(HERE, "solar_stl", "parts_v2")
S, Z0 = 0.001, 0.010                      # mm -> m; z' = -z + Z0 (dasar rangka tepat di z=0)

d = open(SRC, "rb").read()
n = struct.unpack("<I", d[80:84])[0]
tris, par = [], {}
def find(a):
    while par.setdefault(a, a) != a:
        par[a] = par[par[a]]
        a = par[a]
    return a
for i in range(n):
    v = struct.unpack("<12f", d[84 + i * 50:84 + i * 50 + 48])
    ks = [tuple(round(c, 3) for c in v[3 + j * 3:6 + j * 3]) for j in range(3)]
    tris.append(ks)
    for k in ks[1:]:
        a, b = find(ks[0]), find(k)
        if a != b:
            par[a] = b
comps = {}
for ks in tris:
    comps.setdefault(find(ks[0]), []).append(ks)
def key(ts):                                # urut stabil: z tengah (atas dulu), lalu y, lalu x
    vs = [k for t in ts for k in t]
    return tuple(-round(sum(v[a] for v in vs) / len(vs), 1) for a in (2, 1, 0))
os.makedirs(OUT, exist_ok=True)
for f in os.listdir(OUT):
    os.remove(os.path.join(OUT, f))
for idx, ts in enumerate(sorted(comps.values(), key=key)):
    out = bytearray(b"solar part".ljust(80, b"\0") + struct.pack("<I", len(ts)))
    for a, b, c in ts:
        p = [(x * S, -y * S, -z * S + Z0) for x, y, z in (a, b, c)]      # rotasi 180 derajat sumbu X (urutan sudut tetap)
        u = [p[1][k] - p[0][k] for k in range(3)]; w = [p[2][k] - p[0][k] for k in range(3)]
        nx = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
        m = sum(c * c for c in nx) ** 0.5 or 1.0
        out += struct.pack("<12fH", *(c / m for c in nx), *p[0], *p[1], *p[2], 0)
    open(os.path.join(OUT, f"{idx:02d}_part.stl"), "wb").write(out)
print(len(comps), "komponen ->", OUT)
