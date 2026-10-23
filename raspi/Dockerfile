# Citra ROS 2 untuk pengendali solar tracker di Raspberry Pi 5.
#
# Kenapa Docker: Raspberry Pi OS adalah Debian 13 trixie, dan repo apt ROS untuk trixie HANYA
# berisi perkakas bangun (bloom, colcon, catkin), nol paket distro ROS. Sudah diperiksa langsung:
# 78 paket, tidak satu pun ros-jazzy-*. Jadi apt tidak bisa, dan menulis ulang kartu ke Ubuntu
# akan menghapus sistem yang sudah terpasang. Citra resmi ros:jazzy-ros-base punya arm64.
#
# ros-base dipilih, bukan desktop: Pi tidak menjalankan RViz maupun Gazebo.
#
# Python di dalam citra ini 3.12, sedangkan host Raspberry Pi OS 3.13. Keduanya TIDAK berbagi
# venv, jadi pustaka Python dipasang di sini, bukan dipinjam dari host.
FROM ros:jazzy-ros-base

# smbus2 untuk ADS1115 dan INA226; sisanya inti model
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-numpy python3-scipy python3-pandas i2c-tools \
    && rm -rf /var/lib/apt/lists/*

# pvlib tidak ada di apt Ubuntu; --break-system-packages aman di dalam kontainer
RUN pip3 install --no-cache-dir --break-system-packages pvlib smbus2

WORKDIR /proyek
ENV PYTHONPATH=/proyek:/proyek/dinamika:/proyek/kontrol_hibrid:/proyek/cuaca:/proyek/angin

# Jalankan selalu dengan lingkungan ROS aktif
SHELL ["/bin/bash", "-lc"]
ENTRYPOINT ["/bin/bash", "-lc", "source /opt/ros/jazzy/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
