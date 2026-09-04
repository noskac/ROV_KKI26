#!/bin/bash

# ==========================================
# 1. BERSIHKAN PROSES MENGGANTUNG (AUTO-RESET)
# ==========================================
echo "[INFO] Membersihkan memori kamera dari proses sebelumnya..."
killall -9 gst-launch-1.0 2>/dev/null
# Beri jeda 1 detik agar sistem operasi Jetson melepas kunci /dev/video
sleep 1 

# ==========================================
# 2. FUNGSI UNTUK KELUAR DENGAN BERSIH (CTRL+C)
# ==========================================
cleanup() {
    echo ""
    echo "[INFO] Ctrl+C terdeteksi! Menutup streaming kamera dengan bersih..."
    killall -9 gst-launch-1.0 2>/dev/null
    echo "[INFO] Kamera berhasil dilepas."
    exit 0
}

# Tangkap sinyal SIGINT (Ctrl+C) dan jalankan fungsi cleanup di atas
trap cleanup SIGINT SIGTERM

# ==========================================
# 3. PIPELINE KAMERA
# ==========================================
IP_GCS="192.168.1.5"

echo "[INFO] Memulai Kamera 1 (Depan) di port 5000 dengan resolusi 640x640..."
gst-launch-1.0 -v v4l2src device=/dev/video0 ! \
  image/jpeg,width=1280,height=720,framerate=30/1 ! \
  jpegdec ! video/x-raw ! \
  nvvidconv ! video/x-raw\(memory:NVMM\),width=864,height=486,format=NV12 ! \
  nvv4l2h264enc bitrate=4000000 insert-sps-pps=true iframeinterval=15 control-rate=1 preset-level=1 maxperf-enable=1 ! \
  h264parse ! rtph264pay config-interval=1 pt=96 ! \
  udpsink host=$IP_GCS port=5000 sync=false async=false &

echo "[INFO] Memulai Kamera 2 (Bawah/Samping) di port 5001 dengan resolusi 640x640..."
gst-launch-1.0 -v v4l2src device=/dev/video1 ! \
  image/jpeg,width=1280,height=720,framerate=30/1 ! \
  jpegdec ! video/x-raw ! \
  nvvidconv flip-method=2 ! video/x-raw\(memory:NVMM\),width=864,height=486,format=NV12 ! \
  nvv4l2h264enc bitrate=4000000 insert-sps-pps=true iframeinterval=15 control-rate=1 preset-level=1 maxperf-enable=1 ! \
  h264parse ! rtph264pay config-interval=1 pt=96 ! \
  udpsink host=$IP_GCS port=5001 sync=false async=false &

# Menunggu proses di latar belakang selesai (atau sampai di-interupsi)
wait
