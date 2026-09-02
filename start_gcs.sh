#!/bin/bash
echo "==========================================="
echo " MEMULAI SISTEM GCS ROV MAIVS EVO...       "
echo "==========================================="

WS=~/rov_kki26_ws
LOG_DIR="$WS/logs"
REC_DIR="$WS/recordings"

# ==========================================
# 0. PILIHAN PEREKAMAN
# ==========================================
# Bisa dilewati lewat argumen supaya enak dipanggil dari skrip lain:
#   ./start_gcs.sh --record      -> langsung rekam, tanpa tanya
#   ./start_gcs.sh --no-record   -> langsung tanpa rekam
RECORD=""
case "$1" in
    --record|-r)     RECORD="yes" ;;
    --no-record|-n)  RECORD="no"  ;;
esac

if [ -z "$RECORD" ]; then
    echo ""
    read -p " Aktifkan perekaman (video + log data)? (Y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Nn]$ ]]; then
        RECORD="no"
    else
        RECORD="yes"
    fi
fi

if [ "$RECORD" = "yes" ]; then
    echo "-> Mode: REKAM AKTIF (video + log)"
else
    echo "-> Mode: TANPA REKAM (live only)"
fi
echo ""

# ==========================================
# 0.5 PERSIAPAN FOLDER & CEK RUANG DISK
# ==========================================
# Hanya relevan kalau memang merekam. Kalau tidak merekam, tidak ada gunanya
# menghalangi start sistem cuma gara-gara disk tipis.
if [ "$RECORD" = "yes" ]; then
    mkdir -p "$LOG_DIR" "$REC_DIR"

    # Rekaman MJPG memakan ~2-4 GB per kamera per jam, dikali 2 kamera.
    # Lebih baik tahu sekarang daripada disk penuh di tengah misi.
    FREE_GB=$(df -BG --output=avail "$REC_DIR" | tail -1 | tr -dc '0-9')
    echo "-> Sisa ruang disk: ${FREE_GB} GB"
    if [ "$FREE_GB" -lt 10 ]; then
        echo ""
        echo "   !!! PERINGATAN: sisa disk di bawah 10 GB."
        echo "   !!! Rekaman 2 kamera butuh ~4-8 GB per jam."
        echo "   !!! Node akan berhenti merekam otomatis di sisa 1 GB,"
        echo "   !!! tapi sebaiknya kosongkan $REC_DIR dulu."
        echo ""
        read -p "   Tetap lanjut? (y/N) " -n 1 -r
        echo ""
        [[ ! $REPLY =~ ^[Yy]$ ]] && echo "Dibatalkan." && exit 1
    fi
fi

# Argumen yang dioper ke node
if [ "$RECORD" = "yes" ]; then
    VIDEO_ARGS="--ros-args -p record:=true"
    VIDEO_TITLE="Video Receiver + Rekaman"
else
    VIDEO_ARGS="--ros-args -p record:=false"
    VIDEO_TITLE="Video Receiver (tanpa rekam)"
fi

# ==========================================
# 1. VIDEO RECEIVER (sekaligus PEREKAM VIDEO)
# ==========================================
# CATATAN: perekaman video ada DI DALAM node ini, bukan node terpisah.
# Frame hasil decode sudah ada di memori proses ini, jadi menulisnya ke file
# tidak menambah lalu lintas DDS sama sekali. Node perekam terpisah akan
# menambah proses subscriber ketiga ke topic image_raw (~75 MB/detik).
echo "-> Menyalakan Video Receiver Node..."
gnome-terminal --title="$VIDEO_TITLE" -- bash -c "source /opt/ros/jazzy/setup.bash; cd $WS; source install/setup.bash; ros2 run rov_kki26 video_receiver $VIDEO_ARGS; exec bash"

# 1.5 Telemetry Receiver (selalu nyala, ini sumber data dashboard)
echo "-> Menyalakan Telemetry Receiver Node..."
gnome-terminal --title="Telemetry Receiver" -- bash -c "source /opt/ros/jazzy/setup.bash; cd $WS; source install/setup.bash; ros2 run rov_kki26 telemetry_receiver; exec bash"

# 1.6 Data Logger (mulai mencatat sedini mungkin) - hanya kalau merekam
if [ "$RECORD" = "yes" ]; then
    echo "-> Menyalakan Data Logger Node..."
    gnome-terminal --title="Data Logger" -- bash -c "source /opt/ros/jazzy/setup.bash; cd $WS; source install/setup.bash; ros2 run rov_kki26 data_logger; exec bash"
else
    echo "-> Data Logger DILEWATI (mode tanpa rekam)."
fi

# Beri jeda 2 detik agar Video Receiver siap dulu
sleep 2

# 2. GUI Dashboard
echo "-> Menyalakan GUI Dashboard..."
gnome-terminal --title="ROV Dashboard" -- bash -c "source /opt/ros/jazzy/setup.bash; cd $WS; source install/setup.bash; ros2 run rov_kki26 rov_dashboard; exec bash"

# 3. QR Scanner
echo "-> Menyalakan QR..."
gnome-terminal --title="QR Scanner" -- bash -c "source /opt/ros/jazzy/setup.bash; cd $WS; source install/setup.bash; ros2 run rov_kki26 qr_scanner; exec bash"

# Beri jeda 1 detik
sleep 1

# 4. Gamepad Controller
echo "-> Menyalakan Gamepad Controller..."
gnome-terminal --title="Gamepad Controller" -- bash -c "source /opt/ros/jazzy/setup.bash; cd $WS; source install/setup.bash; ros2 run rov_kki26 mavis_gamepad; exec bash"

echo "==========================================="
echo " SEMUA SUBSISTEM GCS TELAH DILUNCURKAN!    "
echo ""
if [ "$RECORD" = "yes" ]; then
    echo " Rekaman video : $REC_DIR"
    echo " Log data      : $LOG_DIR"
else
    echo " Perekaman NONAKTIF - tidak ada file yang ditulis."
fi
echo ""
echo " Tutup terminal ini jika tidak diperlukan. "
echo "==========================================="
