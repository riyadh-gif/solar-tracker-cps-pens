# Massa dan inersia link dari CAD

Dibangkitkan oleh `inersia_cad.py`. Frame link = frame sendi (base di 0; azimuth_link di P_AZ = (0,0,0.165); elevation_link di P_EL = (0,0,0.2045), mesh sudah diputar balik 21.24 derajat). Sumbu azimuth = z (x=y=0), sumbu elevasi = y lewat P_EL. Satuan SI.

## 1. Massa per bagian

| Bagian | Link | Metode | Volume (cm3) | Massa (g) | Sumber | Dasar |
|---|---|---|---|---|---|---|
| 00 | base | solid | 26.40 | 71.3 | mesh | profil 2020 aluminium 2700, volume mesh watertight |
| 01 | base | solid | 26.40 | 71.3 | mesh | profil 2020 aluminium 2700, volume mesh watertight |
| 02 | base | solid | 70.73 | 191.0 | mesh | profil 2020 aluminium 2700, volume mesh watertight |
| 03 | base | solid | 26.40 | 71.3 | mesh | profil 2020 aluminium 2700, volume mesh watertight |
| 04 | base | solid | 26.40 | 71.3 | mesh | profil 2020 aluminium 2700, volume mesh watertight |
| 05 | base | solid | 70.73 | 191.0 | mesh | profil 2020 aluminium 2700, volume mesh watertight |
| 06 | base | solid | 26.40 | 71.3 | mesh | profil 2020 aluminium 2700, volume mesh watertight |
| 07 | base | solid | 26.40 | 71.3 | mesh | profil 2020 aluminium 2700, volume mesh watertight |
| 20 | elevation_link | solid | 76.77 | 207.3 | mesh | profil 2020 aluminium 2700, volume mesh watertight |
| 21 | elevation_link | solid | 76.76 | 207.3 | mesh | profil 2020 aluminium 2700, volume mesh watertight |
| 08 | base | hull fill 0.15 | 153.85 | 62.3 | asumsi | mesh tidak watertight; tiang = profil 2020 miring, diagonal ~130 mm x 176 mm2 = ~15% volume hull, Al 2700 |
| 09 | base | hull fill 0.15 | 153.85 | 62.3 | asumsi | sama dengan bagian 08 |
| 10 | base | motor | 245.18 | 1200.0 | asumsi | motor 57HSE3.0N (NEMA23) 1.2 kg asumsi, seragam di volume hull |
| 11 | azimuth_link | solid | 0.32 | 2.5 | asumsi | komponen kecil (0.3 cm3), diasumsikan baja 7850 (pengencang), ~2.5 g |
| 12 | azimuth_link | solid | 5.35 | 14.5 | asumsi | pelat/braket 5.4 cm3 solid, diasumsikan Al 2700 |
| 13 | azimuth_link | hull fill 0.3 | 274.02 | 222.0 | asumsi | mesh tidak watertight; rumah pillow block, isi 30% hull, Al 2700 (jika besi cor massa ~2.6x lebih besar) |
| 14 | elevation_link | solid | 0.23 | 1.8 | asumsi | komponen kecil (0.23 cm3), diasumsikan baja 7850 (kolar/baut), ~1.8 g |
| 15 | elevation_link | solid | 0.23 | 1.8 | asumsi | sama dengan bagian 14 |
| 16 | elevation_link | solid | 8.88 | 24.0 | asumsi | dudukan poros SK8, volume mesh solid x Al 2700 (rongga baut tidak dimodelkan) |
| 17 | elevation_link | solid | 8.88 | 24.0 | asumsi | sama dengan bagian 16 |
| 18 | azimuth_link | motor | 245.18 | 1200.0 | asumsi | motor 57HSE3.0N (NEMA23) 1.2 kg asumsi, seragam di volume hull |
| 19 | elevation_link | solid | 4.94 | 38.8 | mesh | poros baja 8 mm, volume mesh x 7850 |
| panel | panel | kotak 0.44x0.30x0.02 | 7454 | 2550 | asumsi | panel 1.5 kg seragam (sensitivitas 1 dan 3 kg) |

Keterangan: "mesh" = massa dari volume mesh watertight x kerapatan bahan yang diketahui (profil Al 2700, poros baja 7850). "asumsi" = ada angka yang ditebak (massa motor, fill hull, bahan komponen kecil). Hasil total sangat bergantung pada massa motor dan panel.

## 2. Per link (frame link)

| Link | Massa (kg) | COM (mm) | Ixx, Iyy, Izz di COM (kg m2) | J sumbu sendi (kg m2) |
|---|---|---|---|---|
| base | 2.134 | [-0.7, 0.0, 68.7] | 1.812e-02, 1.325e-02, 1.960e-02 | 1.960e-02 (sumbu z) |
| azimuth_link | 1.439 | [29.5, 1.6, 36.4] | 7.869e-04, 1.506e-03, 1.439e-03 | 2.696e-03 (sumbu z) |
| elevation_link | 0.505 | [-15.0, 8.3, 25.6] | 3.882e-03, 7.164e-03, 1.089e-02 | 4.248e-03 (sumbu x) |
| panel | 2.55 | [-0.6, 9.2, 54.7] | 4.221e-02, 9.552e-02, 1.375e-01 | 5.007e-02 (sumbu x) |

Tensor lengkap (termasuk produk inersia dan tensor terhadap origin link) ada di `inersia_link.json`. Panel berada di frame elevasi (link panel di URDF tanpa origin sendi), jadi COM panel relatif terhadap P_EL.

## 3. Besaran dinamika (panel 1.5 kg, motor 1.2 kg)

- J_az (azimuth_link + elevation_link + panel, sumbu z), q2 = 0: **1.5142e-01 kg m2**
- J_az pada q2 = 45 deg: **1.3097e-01 kg m2** (selisih -13.5%)
- J_el (elevation_link + panel, sumbu x lewat P_EL): **5.4320e-02 kg m2**
- COM elevation_link + panel di frame elevasi: [-2.9, 9.0, 49.9] mm; lengan ketidakseimbangan: y = 9.0 mm, z = 49.9 mm, radial yz = **50.7 mm**
- Momen gravitasi terhadap sumbu elevasi: M(q) = m g (y cos q - z sin q). Maksimum dalam batas +-45 deg = **1.274 N.m** (q = 0: 0.276 N.m; batas atas tanpa batas sudut 1.550 N.m). Massa elevasi + panel = 3.055 kg. Bandingkan dengan torsi motor 3 N.m.
- Massa total bagian yang berputar (azimuth + elevasi + panel): **4.494 kg**

## 4. Kekakuan puntir poros baja 8 mm (bagian 19)

k = G J / L, G = 79 GPa, d = 8 mm, J = pi d^4 / 32 = 4.021e-10 m4, L = 100.3 mm (panjang bounding box mesh bagian 19 sepanjang y) menghasilkan **317 N.m/rad**. Ini hanya angka referensi geometri poros, bukan kekakuan sendi sebenarnya (sendi nyata dipengaruhi kopling, motor, kelonggaran).

## 5. Sensitivitas

| Motor (kg) | Panel (kg) | J_az q0 | J_az q45 | J_el | Lengan xz (mm) | M gravitasi maks (N.m) | Massa berputar (kg) |
|---|---|---|---|---|---|---|---|
| 1.2 | 1.0 | 6.773e-02 | 5.855e-02 | 2.388e-02 | 45.85 | 0.5733 | 2.944 |
| 1.2 | 1.5 | 9.473e-02 | 8.191e-02 | 3.370e-02 | 48.25 | 0.7993 | 3.444 |
| 1.2 | 3.0 | 1.757e-01 | 1.520e-01 | 6.316e-02 | 51.36 | 1.477 | 4.944 |
| 0.9 | 1.5 | 9.411e-02 | 8.129e-02 | 3.370e-02 | 48.25 | 0.7993 | 3.144 |
| 1.6 | 1.5 | 9.555e-02 | 8.273e-02 | 3.370e-02 | 48.25 | 0.7993 | 3.844 |
| 0.9 | 1.0 | 6.712e-02 | 5.793e-02 | 2.388e-02 | 45.85 | 0.5733 | 2.644 |
| 1.6 | 3.0 | 1.765e-01 | 1.528e-01 | 6.316e-02 | 51.36 | 1.477 | 5.344 |

## 6. Usulan blok URDF <inertial> (default; belum dipasang di URDF)

Origin = COM dalam frame link. Untuk `panel`, frame link = frame elevasi.

```xml
<!-- base_link -->
<inertial><origin xyz="-0.00067 0.00000 0.06873" rpy="0 0 0"/><mass value="2.1343"/>
  <inertia ixx="1.81188e-02" iyy="1.32473e-02" izz="1.95966e-02" ixy="6.45959e-19" ixz="-8.36622e-05" iyz="3.85382e-19"/></inertial>
<!-- azimuth_link -->
<inertial><origin xyz="0.02951 0.00163 0.03640" rpy="0 0 0"/><mass value="1.4390"/>
  <inertia ixx="7.86901e-04" iyy="1.50584e-03" izz="1.43879e-03" ixy="-4.26555e-05" ixz="-1.25487e-04" iyz="-9.06774e-06"/></inertial>
<!-- elevation_link -->
<inertial><origin xyz="-0.01495 0.00832 0.02563" rpy="0 0 0"/><mass value="0.5050"/>
  <inertia ixx="3.88183e-03" iyy="7.16355e-03" izz="1.08929e-02" ixy="-1.66351e-04" ixz="-1.47150e-04" iyz="-9.00166e-05"/></inertial>
<!-- panel -->
<inertial><origin xyz="-0.00057 0.00918 0.05475" rpy="0 0 0"/><mass value="2.5500"/>
  <inertia ixx="4.22131e-02" iyy="9.55241e-02" izz="1.37472e-01" ixy="0.00000e+00" ixz="0.00000e+00" iyz="0.00000e+00"/></inertial>
```

## 7. Asumsi dan keterbatasan

- Massa motor 1.2 kg (sensitivitas 0.9 sampai 1.6 kg) dan panel 1.5 kg (1 sampai 3 kg) adalah tebakan; motor adalah massa dominan.
- Bagian tidak watertight (08, 09, 13) memakai volume convex hull x fill x kerapatan Al; fill 0.15 dan 0.30 adalah perkiraan teknik, bukan hasil ukur. Inersia dihitung seolah seragam di dalam hull.
- Komponen kecil (11, 14, 15) diasumsikan baja; bagian 12, 16, 17 solid Al. Massanya kecil.
- Tidak dimodelkan: kabel, baut, mur, sensor LDR, bearing sebenarnya, rongga baut SK8.
- Profil dan poros memakai volume mesh x kerapatan nominal; mesh dianggap benar ukurannya.
- Panel diperlakukan sebagai kotak seragam berpusat di frame elevasi sesuai build_model.
- Base tidak berputar, inersianya tidak masuk J_az atau J_el.
- Belum ada verifikasi dengan penimbangan atau pengukuran nyata.
