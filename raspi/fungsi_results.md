# Verifikasi tiga fungsi utama Raspberry Pi 5

Mesin: `proyekssf@192.168.100.68`, Raspberry Pi 5, Debian 13 trixie, Docker, citra `ssf-ros:jazzy`.
Tanggal uji: 2026-10-23. Semua angka di bawah ini dihasilkan **di Pi**, bukan di PC.

Semua uji memakai **masukan tiruan**, karena panel, motor dan sensor belum tersambung.
`i2cdetect -y 1` masih kosong di 0x48 dan 0x40, jadi `sensor_hw_node` otomatis jatuh ke mode tiruan.

Cara menjalankan ulang:

```bash
# ketiga fungsi sekaligus, jam dibekukan di tengah hari
sudo docker run --rm --network host --ipc host -e JAM=12.0 \
  -v /home/proyekssf/projekssf:/proyek ssf-ros:jazzy bash /proyek/raspi/uji3.sh

# rantai LDR -> EKF -> perintah -> motor, dengan salah pasang 2 deg disuntikkan
sudo docker run --rm --network host --ipc host \
  -e SP1=2.0 -e SP2=2.0 -e EKF=true -e JAM=12.0 -e DURASI=60 \
  -v /home/proyekssf/projekssf:/proyek ssf-ros:jazzy bash /proyek/raspi/uji_loop.sh
```

---

## Hasil ringkas

| Fungsi | Node | Status | Bukti |
|---|---|---|---|
| 1. Penggerak 2 motor (tilt dan azimuth) | `motor_node.py` | jalan, mode tiruan | 1522 langkah azimuth, 484 langkah kemiringan, sisa di bawah satu langkah |
| 2. Pemerataan intensitas cahaya oleh LDR | `sensor_hw_node.py` + `ekf_node.py` | jalan, dan terbukti **menentukan sudut motor** | galat pointing sebenarnya 2,0417 -> 0,0258 deg |
| 3. Tegangan dan arus panel | `sensor_hw_node.py` | jalan, mode tiruan | 2,735 A, 15,03 V, 41,11 W pada DNI 812 W/m2 |

Nol baris `Traceback` atau `ERROR` di keempat log node.

---

## Fungsi 1: penggerak dua motor

`motor_node.py` adalah satu-satunya node yang menyentuh GPIO. Pada Pi 5 ia memakai `lgpio`
dengan `gpiochip4`; `RPi.GPIO` tidak bekerja di Pi 5.

Hasil pada jam 12.00 (tanggal 2026-10-07):

```
azimuth  N=1.00    8.89 langkah/deg beban  tgt=-171.268  pos=-171.225  sisa=-0.0427  langkah=-1522
elevasi  N=5.00   44.44 langkah/deg beban  tgt=  10.892  pos=  10.890  sisa=+0.0024  langkah=  484
```

Rantai sudutnya:

```
sudut BEBAN  --(reduksi gigi N)-->  sudut MOTOR  --(microstep)-->  jumlah langkah
```

Faktornya diambil dari `dinamika/dyn_model.AXES`, tidak ditulis ulang. Kemiringan punya N = 5,
jadi satu derajat di panel kemiringan butuh **lima kali** lebih banyak langkah daripada azimuth
(44,44 lawan 8,89 langkah/deg). Satu langkah berarti 0,0225 deg di kemiringan dan 0,1125 deg di
azimuth, dan sisa perintah di kedua sumbu memang lebih kecil daripada satu langkah, yaitu
kuantisasi, bukan galat pelacakan.

Periksa silang: 45,000 deg x 44,444 langkah/deg = **2000 langkah bulat**, dan itu persis yang
dilaporkan node pada jam 06.00 saat kemiringan menyentuh batas.

### Mengapa TIDAK ada profil trapesium di node ini

Ini sengaja, dan penting. Di Gazebo, pembentukan perintah (profil trapesium lalu input shaper
ZVD) dikerjakan `sun_command_node` **sebelum** diterbitkan ke `/solar_tracker/*_cmd`. Aktuator
Gazebo hanya mengikuti perintah itu dengan kejenuhan torsi, tidak membentuk ulang.

Menambahkan profil kedua di `motor_node` akan menapis ulang perintah yang sudah dibentuk dan
merusak peredaman ZVD yang sudah divalidasi, yaitu yang menurunkan lewatan azimuth dari 85,61 %
ke 2,16 %. Satu-satunya pembatas di `motor_node` adalah **laju pulsa maksimum driver**, dan itu
batas perangkat keras, padanan kejenuhan torsi di Gazebo.

### Loop tertutupnya ada di driver, bukan di Raspi

Enkoder ada di dalam driver dan **tidak terbaca host**. Yang bisa dibaca host hanyalah pin ALARM,
yang menyala saat driver menyerah mengejar galat. Karena itu `motor_node` tidak menutup loop
posisi; ia melaporkan alarm dan menyerahkan taksiran simpangan sisi beban ke `ekf_node`. Itulah
alasan EKF tetap ada di tesis meski motor sudah loop tertutup.

### Penjepit batas sumbu

`motor_node` menjepit perintah ke +-185 deg (azimuth) dan +-45 deg (kemiringan), nilai yang sama
dengan `build_model_v2.AZ_LIMIT` dan `EL_LIMIT`. Penjepitan dilakukan **di node penggerak**, bukan
hanya di pembangkit perintah, karena node inilah satu-satunya yang menyentuh motor, jadi ia yang
harus menolak masuk ke stopper mekanik apa pun yang dikirim kepadanya.

Terbukti perlu: pada satu uji, profil perintah mengeluarkan **45,072 deg** pada batas 45 deg,
yaitu lewatan integrasi satu tick (`tilt_rate` 90 deg/s pada dt 10 ms memberi sampai 0,9 deg).
Di simulasi itu tak berakibat, di alat nyata itu berarti menabrak stopper. Pada operasi normal
penjepit tidak pernah aktif (`terjepit=False`), yang memang perilaku yang diinginkan untuk sebuah
penjaga.

### Masih asumsi

Peta pin PUL/DIR/ENA/ALM (azimuth BCM 17/27/22/23, kemiringan 5/6/13/19) **belum diketahui** dan
nilai itu hanya bawaan. Semuanya parameter ROS, jadi tinggal diganti saat peta pin sebenarnya ada,
tanpa menyunting kode.

---

## Fungsi 2: pemerataan intensitas cahaya oleh LDR

Pertanyaan sebenarnya bukan "apakah LDR terbaca", tetapi **apakah ketidakrataan cahaya benar-benar
menjadi keputusan sudut motor**. Rantai lengkapnya:

```
LDR 4 kuadran -> ex, ey -> ekf_node -> /ekf/koreksi -> sun_command_node (pakai_ekf) -> motor_node
```

### Mode tiruan semula tidak bisa menguji ini

Mode tiruan menghitung LDR dari sudut yang **diperintahkan**, sehingga cahaya selalu rata dan
fungsi pemerataan tidak pernah punya apa pun untuk dikoreksi. Karena itu ditambahkan parameter
`salah_pasang_q1` dan `salah_pasang_q2` (deg, hanya mode tiruan): optik LDR melihat orientasi
**sebenarnya**, yaitu perintah ditambah salah pasang.

Salah pasang mekanik dipilih dengan sengaja, bukan sembarang gangguan: inilah galat yang memang
menjadi tugas LDR dan EKF, karena ia terjadi **setelah** poros sehingga enkoder di rotor tidak
dapat melihatnya.

### Hasil: salah pasang 2,0 deg di kedua sumbu, jam 12.00

| `pakai_ekf` | taksiran EKF b_az | taksiran EKF b_q2 | galat pointing SEBENARNYA |
|---|---|---|---|
| `false` | (dihitung, tidak dipakai) | | **2,0417 deg**, tetap |
| `true` | +2,0817 deg | +2,0046 deg | **0,0258 deg** (purata 5 cuplikan terakhir) |

Kebenaran dasar 2,0 deg di kedua sumbu. Taksiran EKF melencengnya 4,1 % (azimuth) dan 0,2 %
(kemiringan). Galat pointing turun **79 kali**.

Tanpa koreksi, galat 2,0417 deg sama sekali tidak berubah selama 60 detik, karena tidak ada
yang menutup loopnya. Itu uji pembanding yang membuktikan penurunan di baris kedua benar-benar
berasal dari LDR, bukan dari hal lain.

### Dua jebakan metrik yang harus dicatat

**Pertama**, `galat.perintah` **tidak** boleh dipakai menilai mutu penjejakan saat koreksi aktif.
Angka itu mengukur selisih sudut *perintah* terhadap matahari, dan koreksi EKF memang menggeser
perintah dengan sengaja, jadi `galat.perintah` justru **membesar** sebesar koreksinya (dari 0,0 ke
sekitar 2,03 deg). Yang mengukur mutu penjejakan adalah `galat.tiruan_sebenarnya`, yaitu orientasi
perintah ditambah salah pasang. Keduanya sekarang diterbitkan terpisah di `/tracker/diag`.

**Kedua**, `selisih_arus_pct` selalu 0,00 % di mode tiruan, dan itu **tautologi**: arus "terukur"
diturunkan dari model yang sama dengan pembandingnya. Angka itu baru bermakna setelah INA226
nyata terpasang.

### Cacat yang ditemukan dan diperbaiki: derau ADC yang membeku

`sensor_hw_node` membuat `random.Random(0)` **di dalam tick**, jadi seed yang sama berulang setiap
cuplikan dan derau ADC membeku menjadi **offset tetap**:

| | nilai | setara LSB ADC 12 bit |
|---|---|---|
| `ex` palsu | +0,000926 | 5,8 |
| `ey` palsu | −0,000416 | 2,6 |

Offset sebesar itu **sama besar atau lebih besar daripada sinyal salah pasang yang hendak diukur**
dan melanggar asumsi EKF bahwa derau berpurata nol dan tidak berkorelasi. Akibatnya EKF mengejar
salah pasang hantu: dengan kebenaran 2,0 deg ia berhenti di 0,37 dan 1,23 deg, dan dengan
kebenaran nol ia melaporkan −0,65 dan −0,74 deg.

Perbaikan: satu `random.Random(0)` untuk seluruh masa hidup node, dibuat sekali di luar tick.
Setelah itu taksiran menjadi 2,0817 dan 2,0046 deg seperti tabel di atas.

### Gerbang NIS bekerja

Pada jam 06.00, kemiringan terpotong di 45 deg sementara matahari menuntut lebih, sehingga galat
pointing besar dan optik LDR jenuh (`ex = 0,0766`, jauh di luar jangkauan sekatnya). EKF
melaporkan `dipakai: 0`, yaitu menolak pembacaan itu. Perilaku ini benar: di luar jangkauan
penerimaan, LDR tidak memberi gradien yang sah dan tidak boleh dipercaya.

---

## Fungsi 3: tegangan dan arus panel

Dibaca `sensor_hw_node` lewat INA226 di 0x40, shunt 0,01 ohm. Pada jam 12.00, DNI 812,1 W/m2,
DHI 193,0, GHI 990,5:

| Besaran | Nilai |
|---|---|
| Arus | 2,735 A |
| Tegangan | 15,03 V |
| Daya | 41,11 W |

Pada jam 06.00 (DNI rendah, kemiringan terpotong): 0,411 A, 16,46 V, 6,77 W. Arus turun hampir
7 kali sementara tegangan naik, yaitu tanda khas kurva I-V panel pada iradiasi rendah.

Keempat tegangan LDR juga dibaca lewat ADS1115 di 0x48, PGA +-4,096 V, single-shot 128 SPS.

`sensor_hw_node` menerbitkan `/tracker/diag` dalam **format yang sama persis** dengan
`sensor_node.py` simulasi, sehingga `ekf_node` berjalan tanpa perubahan apa pun antara simulasi
dan Pi. Itu yang membuat angka di atas langsung sebanding dengan hasil Gazebo.

---

## Catatan jujur tentang keterbatasan uji ini

1. **Belum ada perangkat keras.** Bus I2C sudah aktif (`/dev/i2c-1` ada, `dtparam=i2c_arm=on`,
   sudah reboot) tetapi `i2cdetect -y 1` masih kosong. Ketiga fungsi terbukti **rantai
   perangkat lunaknya**, bukan sensornya.
2. **Sudut beban tidak terukur.** `sendi.beban_q1` dan `beban_q2` diisi nilai perintah, karena
   enkoder ada di rotor di dalam driver. Jadi `galat.beban` sama dengan `galat.perintah` dan
   `d_q1`, `d_q2` nol. Itu bukan hasil pengukuran.
3. **Backlash gigi belum dimodelkan**, dan ia tak terlihat enkoder rotor karena ada setelah poros.
4. **Model optik LDR masih sekat datar**, belum teropong dan lensa dengan penerimaan +-17,5 deg.
5. **Kalibrasi LDR prosedur A belum dikerjakan.** Tanpa itu, pada data terdahulu LDR justru
   memperburuk EKF (5,63 lawan 2,52 deg).
6. **`velocity="1"` rad/s di URDF** masih asumsi lama dan mendominasi uji langkah mentah.
7. **Pemicu hujan masih data historis Open-Meteo**, belum sensor hujan nyata.

## Berkas

| Berkas | Isi |
|---|---|
| `raspi/Dockerfile` | citra `ssf-ros:jazzy` dari `ros:jazzy-ros-base` arm64 |
| `raspi/jalan.sh` | pembungkus `docker run` dengan `--network host --ipc host` dan lewatkan perangkat bila ada |
| `raspi/uji3.sh` | jalankan dan periksa ketiga fungsi sekaligus |
| `raspi/uji_loop.sh` | uji rantai LDR sampai motor dengan salah pasang disuntikkan |
| `raspi/dengar.py` | pembaca topik berbasis rclpy |
| `raspi/dengar_loop.py` | pemantau konvergensi EKF terhadap waktu |
