#!/bin/bash

echo "==========================================="
echo " MEMULAI SISTEM GCS ROV MAIVS EVO...       "
echo "==========================================="

# 1. Buka Terminal Baru untuk Video Receiver
echo "-> Menyalakan Video Receiver Node..."
gnome-terminal --title="Video Receiver" -- bash -c "source /opt/ros/jazzy/setup.bash; cd ~/rov_kki26_ws; source install/setup.bash; ros2 run rov_kki26 video_receiver; exec bash"

# 1.5 Buka Terminal untuk Telemetry Receiver
echo "-> Menyalakan Telemetry Receiver Node..."
gnome-terminal --title="Telemetry Receiver" -- bash -c "source /opt/ros/jazzy/setup.bash; cd ~/rov_kki26_ws; source install/setup.bash; ros2 run rov_kki26 telemetry_receiver; exec bash"

# 1.6 Buka Terminal untuk Data Logger (mulai mencatat sedini mungkin)
echo "-> Menyalakan Data Logger Node..."
gnome-terminal --title="Data Logger" -- bash -c "source /opt/ros/jazzy/setup.bash; cd ~/rov_kki26_ws; source install/setup.bash; ros2 run rov_kki26 data_logger; exec bash"

# Beri jeda 2 detik agar Video Receiver siap dulu
sleep 2

# 2. Buka Terminal Baru untuk GUI Dashboard
echo "-> Menyalakan GUI Dashboard..."
gnome-terminal --title="ROV Dashboard" -- bash -c "source /opt/ros/jazzy/setup.bash; cd ~/rov_kki26_ws; source install/setup.bash; ros2 run rov_kki26 rov_dashboard; exec bash"

# 3. Buka Terminal Baru untuk QR Scanner
echo "-> Menyalakan QR..."
gnome-terminal --title="QR Scanner" -- bash -c "source /opt/ros/jazzy/setup.bash; cd ~/rov_kki26_ws; source install/setup.bash; ros2 run rov_kki26 qr_scanner; exec bash"

# Beri jeda 1 detik
sleep 1

# 4. Buka Terminal Baru untuk Gamepad Controller
echo "-> Menyalakan Gamepad Controller..."
gnome-terminal --title="Gamepad Controller" -- bash -c "source /opt/ros/jazzy/setup.bash; cd ~/rov_kki26_ws; source install/setup.bash; ros2 run rov_kki26 mavis_gamepad; exec bash"



echo "==========================================="
echo " SEMUA SUBSISTEM GCS TELAH DILUNCURKAN!    "
echo " Tutup terminal ini jika tidak diperlukan. "
echo "==========================================="