#!/usr/bin/env python3
"""
video_receiver_node.py  —  GCS side
======================================
FIX dari versi lama:

  BUG LAMA #1 (thread safety):
      receive_stream() berjalan di worker thread lalu langsung memanggil
      publisher.publish() dari sana — race condition DDS, sama seperti telemetry.

  BUG LAMA #2 (CPU hog / CPU spike):
      Ketika cap.read() mengembalikan ret=False (stream belum siap atau
      GStreamer pipeline error), loop while berputar tanpa sleep() apapun.
      Akibatnya 1 thread mengkonsumsi 100% CPU core, menghambat ROS 2 executor
      dan semua node lain di proses yang sama — termasuk QR scanner & telemetry.

  FIX:
      Worker thread HANYA menyimpan frame terbaru ke dalam buffer (dilindungi
      threading.Lock). Pemanggilan publish() dipindahkan ke ROS 2 Timer 30 Hz
      yang berjalan di executor thread.
      Ketika cap.read() gagal, ditambahkan time.sleep(0.01) agar tidak spin.

  FIX TAMBAHAN:
      QoS Image diubah ke BEST_EFFORT depth=1 agar hanya frame terbaru yang
      dikirim. Ini juga mencegah backlog 10 frame (333ms lag) di koneksi
      yang sempat hiccup.

  CATATAN GStreamer & OpenCV:
      cv2.VideoCapture dengan backend GStreamer HANYA jalan kalau OpenCV
      dikompilasi dengan flag -DWITH_GSTREAMER=ON.
      pip install opencv-python TIDAK menyertakan GStreamer.
      Gunakan: sudo apt install python3-opencv (versi apt Ubuntu 22/24).
      Cek dengan: python3 -c "import cv2; print(cv2.getBuildInformation())"
      cari baris "GStreamer: YES".
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import threading
import time

# ─── QoS: Video real-time, selalu frame terbaru ──────────────────────────────
VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class FrameBuffer:
    """Thread-safe buffer yang hanya menyimpan frame terbaru."""
    def __init__(self):
        self._frame = None
        self._lock  = threading.Lock()
        self._new   = False

    def put(self, frame):
        with self._lock:
            self._frame = frame
            self._new   = True

    def get_and_clear(self):
        """Ambil frame terbaru dan tandai sudah diambil. Return None jika tidak ada."""
        with self._lock:
            if not self._new:
                return None
            frame = self._frame
            self._new = False
            return frame


class VideoReceiverNode(Node):
    def __init__(self):
        super().__init__('video_receiver_node')

        self.pub_cam1 = self.create_publisher(Image, '/rov/cam1/image_raw', VIDEO_QOS)
        self.pub_cam2 = self.create_publisher(Image, '/rov/cam2/image_raw', VIDEO_QOS)
        self.bridge   = CvBridge()

        self.buf_cam1 = FrameBuffer()
        self.buf_cam2 = FrameBuffer()

        self.running = True

        # Worker thread hanya receive & simpan ke buffer
        threading.Thread(
            target=self._receive_stream,
            args=(5000, self.buf_cam1, 'CAM1'),
            daemon=True
        ).start()
        threading.Thread(
            target=self._receive_stream,
            args=(5001, self.buf_cam2, 'CAM2'),
            daemon=True
        ).start()

        # Timer 30 Hz di executor thread → aman memanggil publish()
        self.create_timer(1.0 / 30.0, self._publish_frames)

        self.get_logger().info('[VIDEO] Receiver aktif. Menunggu GStreamer di port 5000 & 5001...')

    # ──────────────────────────────────────────────────────────────────────────
    # WORKER THREAD: hanya baca frame, JANGAN publish di sini
    # ──────────────────────────────────────────────────────────────────────────
    def _receive_stream(self, port: int, buf: FrameBuffer, label: str):
        gst_pipeline = (
            f'udpsrc port={port} '
            '! application/x-rtp,media=(string)video,clock-rate=(int)90000,'
            'encoding-name=(string)H264,payload=(int)96 '
            '! rtpjitterbuffer latency=80 drop-on-latency=true '
            '! rtph264depay ! h264parse ! avdec_h264 '
            '! videoconvert ! video/x-raw,format=(string)BGR '
            '! appsink drop=true max-buffers=1 sync=false'
        )

        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            self.get_logger().error(
                f'[{label}] GStreamer pipeline GAGAL dibuka!\n'
                '  → Pastikan OpenCV dikompilasi dengan GStreamer:\n'
                '    sudo apt install python3-opencv  (BUKAN pip install opencv-python)\n'
                '  → Cek: python3 -c "import cv2; bi=cv2.getBuildInformation(); '
                'print([l for l in bi.split(chr(10)) if \"GStreamer\" in l])"'
            )

        consecutive_failures = 0

        while self.running and rclpy.ok():
            ret, frame = cap.read()

            if ret:
                buf.put(frame)
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                # ← FIX KRITIS: sleep agar tidak spin 100% CPU
                # Backoff bertahap: makin lama gagal, makin jarang retry
                sleep_ms = min(10 + consecutive_failures * 2, 100)
                time.sleep(sleep_ms / 1000.0)

                # Log jika gagal terlalu lama (tapi tidak spam setiap frame)
                if consecutive_failures == 50:
                    self.get_logger().warn(f'[{label}] Stream belum masuk atau terputus. Menunggu...')

        if cap.isOpened():
            cap.release()

    # ──────────────────────────────────────────────────────────────────────────
    # EXECUTOR THREAD (via timer): ambil frame dari buffer lalu publish
    # ──────────────────────────────────────────────────────────────────────────
    def _publish_frames(self):
        for buf, pub in ((self.buf_cam1, self.pub_cam1), (self.buf_cam2, self.pub_cam2)):
            frame = buf.get_and_clear()
            if frame is None:
                continue
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f'[VIDEO] Gagal publish frame: {e}', throttle_duration_sec=5.0)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()