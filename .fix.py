import io
f="dinamika/loop_tertutup.py"
s=io.open(f,encoding="utf-8").read()
old='''           "Dihasilkan oleh `dinamika/loop_tertutup.py`. Kolom loop terbuka mereproduksi angka yang sudah "
           "tervalidasi terhadap Gazebo (4,79 Hz, 93,3 %, 1,51 s), jadi modelnya berjangkar.\n"]'''
new='''           "Dihasilkan oleh `dinamika/loop_tertutup.py`.\n\n"
           "PERHATIAN: angka Gazebo yang tervalidasi (azimuth 4,79 Hz prediksi / 5,19 Hz terukur, lewatan 93,3 % / 90 %) "
           "berasal dari motor LAMA 57HSE3.0N pada loop terbuka. Dengan 57HSE1.5N kolom loop terbuka di bawah "
           "menjadi 3,46 Hz, jadi tabel ini TIDAK lagi berjangkar pada pengukuran Gazebo mana pun. "
           "Validasi ulang (`rec_step.py`, `rec_shaped.py`) belum dijalankan.\n"]'''
assert s.count(old)==1, s.count(old)
io.open(f,"w",encoding="utf-8").write(s.replace(old,new))
print("ok header diperbaiki")
