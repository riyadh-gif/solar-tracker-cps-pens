import io, sys
f = "build_model_v2.py"
s = io.open(f, encoding="utf-8").read()
old = "AZ_LIMIT = math.radians(170)         # BARU: azimuth tidak lagi kontinu (ada pangkon limit + 4 saklar)"
new = ("AZ_LIMIT = math.radians(185)         # BARU: azimuth tidak lagi kontinu (pangkon limit + 4 saklar).\n"
       "#                                      ASUMSI, menunggu angka asli dari CAD. 185 dipilih karena matahari\n"
       "#                                      PENS 2026 menuntut q1 dari -180.9 sampai +174.6 deg: +-170 membuat\n"
       "#                                      tracker MENTOK dan tidak bisa mengikuti matahari (terbukti di Gazebo),\n"
       "#                                      dan +-180 pun masih kurang tipis.")
assert s.count(old) == 1, s.count(old)
io.open(f, "w", encoding="utf-8").write(s.replace(old, new))
print("ok AZ_LIMIT 170 -> 185")
