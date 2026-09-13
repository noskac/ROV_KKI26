#!/usr/bin/env python3
"""
video_recorder_node.py  —  GCS side
=====================================
Node ROS 2 TERPISAH untuk merekam video dari 2 kamera ROV ke file di laptop.

KENAPA SUBSCRIBE KE TOPIC, BUKAN gst-launch LANGSUNG KE FILE:
    Stream dikirim unicast dari Jetson ke UDP port 5000 & 5001, dan port itu
    sudah di-bind oleh video_receiver_node.py. Kalau proses perekam ikut
    membuka udpsrc di port yang sama, di Linux hanya SATU proses yang menerima
    paket — video live di dashboard bisa mati/patah saat rekaman jalan.
    Dengan subscribe ke /rov/cam1/image_raw & /rov/cam2/image_raw, perekam
    menumpang frame yang sudah di-decode. Tidak ada rebutan port.

KENAPA NODE TERPISAH (bukan nempel di GUI dashboard):
    Sama alasannya dengan data_logger_node.py — rekaman tetap jalan walau GUI
    crash, dan siklus hidupnya ikut start_gcs.sh.

DESAIN PENTING — WRITER THREAD, BUKAN TIMER ROS:
    Encoding MJPG ~2-4 ms per frame per kamera. Kalau ditulis di dalam timer
    callback ROS, executor ikut tertahan setiap frame. Jadi di sini:
      - Callback subscription HANYA menyimpan frame terbaru ke slot (dengan Lock).
      - Thread writer terpisah per kamera yang menulis ke file pada laju tetap.
    Thread writer tidak pernah memanggil publish() — hanya tulis file — jadi
    aman dari race condition DDS (lihat catatan di telemetry_receiver_node.py).

LAJU FRAME TETAP (PENTING):
    Writer menulis pada laju tetap (default 20 FPS) memakai jam dinding, dan
    mengulang frame terakhir kalau belum ada frame baru. Hasilnya durasi video
    = durasi asli misi, sehingga bisa disandingkan dengan timestamp CSV dari
    data_logger_node.py. Kalau frame ditulis apa adanya saat callback datang,
    video akan "ngebut" atau "slow-mo" begitu jaringan hiccup.

FORMAT:
    MJPG di dalam .avi. Dipilih karena tiap frame berdiri sendiri — kalau proses
    mati mendadak (baterai habis, Ctrl+C keras, laptop hang), file yang sudah
    ditulis TETAP bisa diputar. MP4/H.264 butuh finalisasi di akhir; kalau
    proses mati, file bisa rusak total. Trade-off-nya ukuran lebih besar
    (~2-4 GB per kamera per jam di 864x486).
    Bisa diganti lewat parameter: --ros-args -p codec:=XVID -p ext:=.avi

OUTPUT:
    ~/rov_kki26_ws/recordings/rov_cam1_<YYYYMMDD_HHMMSS>.avi
    ~/rov_kki26_ws/recordings/rov_cam2_<YYYYMMDD_HHMMSS>.avi
    ~/rov_kki26_ws/recordings/rov_rec_<YYYYMMDD_HHMMSS>.txt   (metadata sesi)

CARA PAKAI:
    python3 video_recorder_node.py
    python3 video_recorder_node.py --ros-args -p fps:=30.0 -p overlay:=false

    Tambahkan ke start_gcs.sh setelah video_receiver_node.py dijalankan.
"""

import os
import time
import signal
import shutil
import datetime
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image

import cv2
import numpy as np

# ─── QoS: HARUS identik dengan publisher di video_receiver_node.py ───────────
# Publisher BEST_EFFORT + subscriber RELIABLE = DDS diam-diam tidak mengirim
# apa pun. Tidak ada error, frame cuma tidak pernah datang.
VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

REC_DIR = os.path.expanduser('~/rov_kki26_ws/recordings')

# Berhenti merekam kalau sisa disk di bawah ini, supaya laptop tidak penuh
# di tengah misi (OS bisa ikut tidak stabil kalau disk 100%).
MIN_FREE_BYTES = 1024 * 1024 * 1024   # 1 GB

CAMERAS = [
    ('CAM1', '/rov/cam1/image_raw', 'cam1'),
    ('CAM2', '/rov/cam2/image_raw', 'cam2'),
]


def ros_image_to_bgr(msg: Image):
    """
    Konversi sensor_msgs/Image ke numpy BGR tanpa cv_bridge.

    Sengaja tidak pakai cv_bridge supaya node ini tetap jalan walau instalasi
    cv_bridge bermasalah (cukup sering terjadi di Ubuntu 24 + ROS 2 dari apt).
    Pendekatannya sama dengan yang dipakai qr_scanner_node.py.
    """
    enc = (msg.encoding or 'bgr8').lower()

    if enc in ('bgr8', 'rgb8'):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return arr if enc == 'bgr8' else cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

    if enc == 'mono8':
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

    if enc in ('bgra8', 'rgba8'):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
        code = cv2.COLOR_BGRA2BGR if enc == 'bgra8' else cv2.COLOR_RGBA2BGR
        return cv2.cvtColor(arr, code)

    raise ValueError(f'Encoding tidak didukung: {msg.encoding}')


class CameraRecorder:
    """Satu perekam untuk satu kamera: subscription + slot frame + writer thread."""

    def __init__(self, node: Node, label: str, topic: str, out_path: str,
                 fps: float, codec: str, overlay: bool):
        self.node = node
        self.label = label
        self.out_path = out_path
        self.fps = fps
        self.codec = codec
        self.overlay = overlay

        self._lock = threading.Lock()
        self._latest = None          # frame BGR terakhir yang diterima
        self._got_new = False        # apakah ada frame baru sejak tulis terakhir

        self.writer = None
        self.size = None             # (w, h) dikunci dari frame pertama
        self.frames_written = 0
        self.frames_received = 0
        self.frames_duplicated = 0   # frame diulang karena stream sepi
        self.started_at = None
        self.stopped_reason = None

        self._running = True
        self._warned_stall = False

        node.create_subscription(Image, topic, self._on_image, VIDEO_QOS)

        self._thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._thread.start()

    # ── Callback ROS: HANYA simpan frame, jangan tulis file di sini ──────────
    def _on_image(self, msg: Image):
        try:
            frame = ros_image_to_bgr(msg)
        except Exception as e:
            self.node.get_logger().warn(
                f'[{self.label}] Frame tidak bisa dikonversi: {e}',
                throttle_duration_sec=5.0,
            )
            return

        with self._lock:
            # .copy() wajib: np.frombuffer menunjuk ke buffer msg yang akan
            # dilepas begitu callback selesai. Tanpa copy, frame bisa berubah
            # atau corrupt di tengah proses tulis.
            self._latest = frame.copy()
            self._got_new = True
            self.frames_received += 1

    # ── Thread writer: tulis ke file pada laju tetap ─────────────────────────
    def _writer_loop(self):
        period = 1.0 / self.fps
        next_tick = None

        while self._running:
            with self._lock:
                frame = self._latest
                was_new = self._got_new
                self._got_new = False

            if frame is None:
                # Belum ada frame sama sekali → jangan buat file dulu, supaya
                # tidak ada .avi kosong 0 byte kalau kamera tidak pernah nyala.
                time.sleep(0.05)
                continue

            if self.writer is None and not self._open_writer(frame):
                return
            if next_tick is None:
                next_tick = time.monotonic()

            if not was_new:
                self.frames_duplicated += 1
                if self.frames_duplicated == int(self.fps * 3) and not self._warned_stall:
                    self._warned_stall = True
                    self.node.get_logger().warn(
                        f'[{self.label}] Tidak ada frame baru >3 detik. '
                        'Frame terakhir diulang agar durasi video tetap akurat.'
                    )
            else:
                self._warned_stall = False

            try:
                self.writer.write(self._decorate(frame) if self.overlay else frame)
                self.frames_written += 1
            except Exception as e:
                self.node.get_logger().error(f'[{self.label}] Gagal menulis frame: {e}')
                self.stopped_reason = f'error tulis: {e}'
                return

            # Jadwal berbasis jam dinding. Kalau writer sempat tertinggal
            # (disk lambat), tick yang lewat dilompati — bukan dikejar — supaya
            # video tidak jadi "ngebut" setelahnya.
            next_tick += period
            now = time.monotonic()
            if next_tick < now:
                next_tick = now
            else:
                time.sleep(next_tick - now)

    def _open_writer(self, frame) -> bool:
        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        writer = cv2.VideoWriter(self.out_path, fourcc, self.fps, (w, h), True)

        if not writer.isOpened():
            self.node.get_logger().error(
                f'[{self.label}] VideoWriter GAGAL dibuka untuk {self.out_path}\n'
                f'  → Codec "{self.codec}" mungkin tidak tersedia di OpenCV Anda.\n'
                '  → Coba: --ros-args -p codec:=XVID  (atau MJPG)'
            )
            self.stopped_reason = 'VideoWriter gagal dibuka'
            return False

        self.writer = writer
        self.size = (w, h)
        self.started_at = datetime.datetime.now()
        self.node.get_logger().info(
            f'[{self.label}] Mulai merekam {w}x{h} @ {self.fps:g} FPS → {self.out_path}'
        )
        return True

    def _decorate(self, frame):
        """Overlay timestamp + label kamera, untuk sinkronisasi dengan CSV log."""
        out = frame.copy()
        text = f"{self.label}  {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}"
        # Digambar dua kali: hitam tebal sebagai outline, putih tipis di atasnya,
        # supaya tetap terbaca di latar terang maupun gelap (air keruh/pantulan).
        cv2.putText(out, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        return out

    def file_size(self) -> int:
        try:
            return os.path.getsize(self.out_path)
        except OSError:
            return 0

    def stop(self, reason: str = 'shutdown'):
        if not self._running:
            return
        self._running = False
        if self.stopped_reason is None:
            self.stopped_reason = reason
        self._thread.join(timeout=3.0)
        if self.writer is not None:
            try:
                self.writer.release()
            except Exception:
                pass
            self.writer = None


class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__('video_recorder_node')

        self.declare_parameter('fps', 20.0)
        self.declare_parameter('codec', 'MJPG')
        self.declare_parameter('ext', '.avi')
        self.declare_parameter('overlay', True)
        self.declare_parameter('output_dir', REC_DIR)

        fps = float(self.get_parameter('fps').value)
        codec = str(self.get_parameter('codec').value).upper()
        ext = str(self.get_parameter('ext').value)
        overlay = bool(self.get_parameter('overlay').value)
        out_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))

        os.makedirs(out_dir, exist_ok=True)
        self.out_dir = out_dir
        self.session_start = datetime.datetime.now()
        stamp = self.session_start.strftime('%Y%m%d_%H%M%S')
        self.meta_path = os.path.join(out_dir, f'rov_rec_{stamp}.txt')

        self.recorders = []
        for label, topic, slug in CAMERAS:
            path = os.path.join(out_dir, f'rov_{slug}_{stamp}{ext}')
            self.recorders.append(
                CameraRecorder(self, label, topic, path, fps, codec, overlay)
            )

        self.status_timer = self.create_timer(10.0, self._status_tick)

        free_gb = shutil.disk_usage(out_dir).free / (1024 ** 3)
        self.get_logger().info(
            f'[RECORDER] Siap. Folder: {out_dir} | {codec}{ext} @ {fps:g} FPS | '
            f'sisa disk {free_gb:.1f} GB\n'
            '           Menunggu frame pertama dari CAM1 & CAM2...'
        )

    def _status_tick(self):
        # Rem darurat: kalau disk hampir penuh, tutup file baik-baik selagi
        # masih bisa, daripada file rusak karena kehabisan ruang di tengah tulis.
        free = shutil.disk_usage(self.out_dir).free
        if free < MIN_FREE_BYTES:
            self.get_logger().error(
                f'[RECORDER] Sisa disk tinggal {free / (1024**2):.0f} MB. '
                'Rekaman DIHENTIKAN untuk menjaga file tetap utuh.'
            )
            for r in self.recorders:
                r.stop('disk hampir penuh')
            self.status_timer.cancel()
            return

        parts = []
        for r in self.recorders:
            if r.writer is None:
                parts.append(f'{r.label}: menunggu stream')
            else:
                dur = (datetime.datetime.now() - r.started_at).total_seconds()
                parts.append(
                    f'{r.label}: {dur:.0f}s, {r.frames_written} frame, '
                    f'{r.file_size() / (1024**2):.0f} MB'
                )
        self.get_logger().info('[RECORDER] ' + ' | '.join(parts))

    def _write_metadata(self):
        """Catatan sesi supaya rekaman gampang disandingkan dengan CSV log."""
        end = datetime.datetime.now()
        lines = [
            'ROV KKI-26 — Sesi Rekaman Video',
            f'Mulai   : {self.session_start.isoformat(timespec="seconds")}',
            f'Selesai : {end.isoformat(timespec="seconds")}',
            f'Durasi  : {(end - self.session_start).total_seconds():.1f} detik',
            '',
        ]
        for r in self.recorders:
            lines += [
                f'[{r.label}]',
                f'  File            : {r.out_path}',
                f'  Resolusi        : {r.size[0]}x{r.size[1]}' if r.size else '  Resolusi        : -',
                f'  FPS target      : {r.fps:g}',
                f'  Frame diterima  : {r.frames_received}',
                f'  Frame ditulis   : {r.frames_written}',
                f'  Frame diulang   : {r.frames_duplicated} (stream sepi)',
                f'  Ukuran          : {r.file_size() / (1024**2):.1f} MB',
                f'  Berhenti karena : {r.stopped_reason or "-"}',
                '',
            ]
        try:
            with open(self.meta_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines))
        except Exception as e:
            self.get_logger().warn(f'[RECORDER] Gagal menulis metadata: {e}')

    def shutdown(self):
        self.get_logger().info('[RECORDER] Menutup file video...')
        for r in self.recorders:
            r.stop()
        self._write_metadata()

        summary = []
        for r in self.recorders:
            if r.frames_written:
                summary.append(
                    f'  {r.out_path}  ({r.frames_written} frame, '
                    f'{r.file_size() / (1024**2):.1f} MB)'
                )
            else:
                summary.append(f'  {r.label}: tidak ada frame yang terekam')
        self.get_logger().info('[RECORDER] Selesai.\n' + '\n'.join(summary))


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()

    # SIGTERM (dari pkill saat GUI ditutup) diubah jadi KeyboardInterrupt supaya
    # blok finally tetap jalan dan writer.release() sempat dipanggil.
    # Tanpa ini, file .avi berhenti mendadak tanpa index.
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()