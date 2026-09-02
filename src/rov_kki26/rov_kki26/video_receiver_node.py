#!/usr/bin/env python3
"""
video_receiver_node.py  —  GCS side  (+ PEREKAMAN TERINTEGRASI)
================================================================
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

═══════════════════════════════════════════════════════════════════════════════
PEREKAMAN DIBUAT DI DALAM NODE INI — BUKAN NODE TERPISAH. INI DISENGAJA.
═══════════════════════════════════════════════════════════════════════════════
    Percobaan sebelumnya memakai node perekam terpisah yang subscribe ke
    /rov/cam1/image_raw & /rov/cam2/image_raw. Hasilnya video di GUI HILANG
    TOTAL. Penyebabnya:

      - Satu frame BGR 864x486 = 1,26 MB. Dikali 30 FPS dikali 2 kamera
        = ~75 MB/detik per proses subscriber.
      - DDS mengirim salinan terpisah ke SETIAP proses subscriber lewat UDP
        loopback. Dashboard + QR scanner sudah = ~150 MB/detik. Menambah
        perekam sebagai proses ketiga = ~225 MB/detik.
      - Pesan 1,26 MB dipecah jadi ribuan datagram UDP. Buffer socket kernel
        default hanya ~208 KB. Satu pecahan hilang → SELURUH frame dibuang,
        dan karena BEST_EFFORT tidak ada pengiriman ulang.
      - Gejalanya bukan video patah-patah, tapi gambar hilang sama sekali,
        sementara topic kecil (depth, IMU, PWM) tetap normal karena muat
        dalam satu datagram.

    Di sini frame hasil decode SUDAH ADA di memori proses ini. Menulisnya ke
    file tidak menambah satu byte pun lalu lintas DDS. Jumlah subscriber topic
    gambar tetap dua seperti semula.

    ATURAN UNTUK KE DEPAN: jangan tambah proses baru yang subscribe ke topic
    image_raw. Kalau butuh konsumen gambar baru, tumpangkan ke node ini atau
    pindahkan ke topic terkompresi.

FORMAT REKAMAN:
    MJPG di dalam .avi. Tiap frame berdiri sendiri, jadi kalau proses mati
    mendadak (baterai habis, laptop hang) file yang sudah tertulis TETAP bisa
    diputar. MP4/H.264 butuh finalisasi di akhir — mati di tengah bisa
    merusak file total. Konsekuensinya ukuran besar: ~2-4 GB per kamera
    per jam. Pakai codec:=XVID kalau ruang disk jadi kendala.

OUTPUT:
    ~/rov_kki26_ws/recordings/rov_cam1_<YYYYMMDD_HHMMSS>.avi
    ~/rov_kki26_ws/recordings/rov_cam2_<YYYYMMDD_HHMMSS>.avi

CARA PAKAI:
    python3 video_receiver_node.py                          # merekam (default)
    python3 video_receiver_node.py --ros-args -p record:=false
    python3 video_receiver_node.py --ros-args -p rec_fps:=30.0 -p codec:=XVID
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import os
import time
import signal
import shutil
import datetime
import threading

# ─── QoS: Video real-time, selalu frame terbaru ──────────────────────────────
VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

REC_DIR = os.path.expanduser('~/rov_kki26_ws/recordings')

# Rem darurat: hentikan rekaman kalau sisa disk di bawah ini, supaya file
# tertutup baik-baik selagi masih bisa. Disk 100% penuh bisa bikin OS ikut
# tidak stabil di tengah misi.
MIN_FREE_BYTES = 1024 * 1024 * 1024   # 1 GB


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


class Recorder:
    """
    Perekam satu kamera. Menerima frame lewat submit() dari worker GStreamer,
    lalu menulisnya ke file dari thread sendiri pada laju TETAP.

    KENAPA LAJU TETAP, BUKAN TULIS SAAT FRAME DATANG:
        Kalau frame ditulis apa adanya begitu tiba, durasi video tidak akan
        sama dengan durasi misi — video jadi slow-motion setiap jaringan
        hiccup. Dengan laju tetap berbasis jam dinding dan frame terakhir
        diulang saat stream sepi, durasi video = durasi asli, sehingga bisa
        disandingkan dengan timestamp CSV dari data_logger_node.py.

    KENAPA THREAD SENDIRI, BUKAN NUMPANG WORKER GSTREAMER:
        Encoding MJPG makan ~2-4 ms per frame. Kalau ditulis di dalam loop
        cap.read(), pembacaan stream ikut tertahan dan frame RTP menumpuk
        di jitterbuffer → video live ikut delay.
    """

    def __init__(self, node: Node, label: str, out_path: str,
                 fps: float, codec: str, overlay: bool):
        self.node = node
        self.label = label
        self.out_path = out_path
        self.fps = fps
        self.codec = codec
        self.overlay = overlay

        self._lock = threading.Lock()
        self._latest = None
        self._got_new = False

        self.writer = None
        self.size = None
        self.frames_written = 0
        self.frames_received = 0
        self.started_at = None
        self.stopped_reason = None

        self._running = True
        self._stall_ticks = 0
        self._thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._thread.start()

    def submit(self, frame):
        """Dipanggil dari worker GStreamer. Murah — hanya tukar pointer."""
        with self._lock:
            self._latest = frame
            self._got_new = True
            self.frames_received += 1

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
                # tidak ada .avi kosong kalau kamera tidak pernah menyala.
                time.sleep(0.05)
                continue

            if self.writer is None and not self._open_writer(frame):
                return
            if next_tick is None:
                next_tick = time.monotonic()

            if not was_new:
                self._stall_ticks += 1
                if self._stall_ticks == int(self.fps * 3):
                    self.node.get_logger().warn(
                        f'[{self.label}] Tidak ada frame baru >3 detik. Frame terakhir '
                        'diulang agar durasi video tetap akurat.'
                    )
            else:
                self._stall_ticks = 0

            try:
                self.writer.write(self._decorate(frame) if self.overlay else frame)
                self.frames_written += 1
            except Exception as e:
                self.node.get_logger().error(f'[{self.label}] Gagal menulis frame: {e}')
                self.stopped_reason = f'error tulis: {e}'
                return

            # Jadwal berbasis jam dinding. Kalau writer sempat tertinggal
            # (disk lambat), tick yang lewat DILOMPATI, bukan dikejar — supaya
            # video tidak jadi "ngebut" setelahnya.
            next_tick += period
            now = time.monotonic()
            if next_tick < now:
                next_tick = now
            else:
                time.sleep(next_tick - now)

    def _open_writer(self, frame) -> bool:
        h, w = frame.shape[:2]
        writer = cv2.VideoWriter(
            self.out_path, cv2.VideoWriter_fourcc(*self.codec), self.fps, (w, h), True
        )
        if not writer.isOpened():
            self.node.get_logger().error(
                f'[{self.label}] VideoWriter GAGAL dibuka untuk {self.out_path}\n'
                f'  → Codec "{self.codec}" mungkin tidak tersedia di OpenCV Anda.\n'
                '  → Coba: --ros-args -p codec:=XVID'
            )
            self.stopped_reason = 'VideoWriter gagal dibuka'
            return False

        self.writer = writer
        self.size = (w, h)
        self.started_at = datetime.datetime.now()
        self.node.get_logger().info(
            f'[REC {self.label}] Merekam {w}x{h} @ {self.fps:g} FPS → {self.out_path}'
        )
        return True

    def _decorate(self, frame):
        """Overlay timestamp, untuk menyandingkan video dengan CSV log."""
        out = frame.copy()
        text = f"{self.label}  {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}"
        # Digambar dua kali — outline hitam lalu teks putih — supaya tetap
        # terbaca di latar terang maupun gelap (air keruh, pantulan lampu).
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


class VideoReceiverNode(Node):
    def __init__(self):
        super().__init__('video_receiver_node')

        self.declare_parameter('record', True)
        self.declare_parameter('rec_fps', 20.0)
        self.declare_parameter('codec', 'MJPG')
        self.declare_parameter('ext', '.avi')
        self.declare_parameter('overlay', True)
        self.declare_parameter('output_dir', REC_DIR)

        self.pub_cam1 = self.create_publisher(Image, '/rov/cam1/image_raw', VIDEO_QOS)
        self.pub_cam2 = self.create_publisher(Image, '/rov/cam2/image_raw', VIDEO_QOS)
        self.bridge   = CvBridge()

        self.buf_cam1 = FrameBuffer()
        self.buf_cam2 = FrameBuffer()

        self.running = True

        # ── Perekam (opsional) ───────────────────────────────────────────────
        self.rec_cam1 = None
        self.rec_cam2 = None
        self.rec_dir = None
        self.status_timer = None

        if bool(self.get_parameter('record').value):
            self._setup_recorders()

        # Worker thread hanya receive & simpan ke buffer
        threading.Thread(
            target=self._receive_stream,
            args=(5000, self.buf_cam1, 'CAM1', lambda: self.rec_cam1),
            daemon=True
        ).start()
        threading.Thread(
            target=self._receive_stream,
            args=(5001, self.buf_cam2, 'CAM2', lambda: self.rec_cam2),
            daemon=True
        ).start()

        # Timer 30 Hz di executor thread → aman memanggil publish()
        self.create_timer(1.0 / 30.0, self._publish_frames)

        self.get_logger().info('[VIDEO] Receiver aktif. Menunggu GStreamer di port 5000 & 5001...')

    def _setup_recorders(self):
        out_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        os.makedirs(out_dir, exist_ok=True)
        self.rec_dir = out_dir

        fps     = float(self.get_parameter('rec_fps').value)
        codec   = str(self.get_parameter('codec').value).upper()
        ext     = str(self.get_parameter('ext').value)
        overlay = bool(self.get_parameter('overlay').value)

        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.rec_cam1 = Recorder(self, 'CAM1', os.path.join(out_dir, f'rov_cam1_{stamp}{ext}'),
                                 fps, codec, overlay)
        self.rec_cam2 = Recorder(self, 'CAM2', os.path.join(out_dir, f'rov_cam2_{stamp}{ext}'),
                                 fps, codec, overlay)

        self.status_timer = self.create_timer(15.0, self._rec_status_tick)

        free_gb = shutil.disk_usage(out_dir).free / (1024 ** 3)
        self.get_logger().info(
            f'[REC] Perekaman AKTIF → {out_dir} | {codec}{ext} @ {fps:g} FPS | '
            f'sisa disk {free_gb:.1f} GB'
        )

    def _rec_status_tick(self):
        free = shutil.disk_usage(self.rec_dir).free
        if free < MIN_FREE_BYTES:
            self.get_logger().error(
                f'[REC] Sisa disk tinggal {free / (1024**2):.0f} MB. '
                'Rekaman DIHENTIKAN agar file tetap utuh. Video live tetap jalan.'
            )
            for r in (self.rec_cam1, self.rec_cam2):
                if r:
                    r.stop('disk hampir penuh')
            self.status_timer.cancel()
            return

        parts = []
        for r in (self.rec_cam1, self.rec_cam2):
            if r is None or r.writer is None:
                parts.append(f'{r.label if r else "?"}: menunggu stream')
            else:
                dur = (datetime.datetime.now() - r.started_at).total_seconds()
                parts.append(f'{r.label}: {dur:.0f}s, {r.file_size() / (1024**2):.0f} MB')
        self.get_logger().info('[REC] ' + ' | '.join(parts))

    # ──────────────────────────────────────────────────────────────────────────
    # WORKER THREAD: baca frame → buffer (untuk publish) + recorder (untuk file)
    # JANGAN publish di sini.
    # ──────────────────────────────────────────────────────────────────────────
    def _receive_stream(self, port: int, buf: FrameBuffer, label: str, get_rec):
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
                # cap.read() mengalokasikan array baru tiap panggilan, jadi
                # buffer & recorder boleh berbagi objek yang sama — keduanya
                # hanya membaca, tidak ada yang menimpa isinya.
                rec = get_rec()
                if rec is not None:
                    rec.submit(frame)
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

    def close_recordings(self):
        for r in (self.rec_cam1, self.rec_cam2):
            if r is None:
                continue
            r.stop()
            if r.frames_written:
                self.get_logger().info(
                    f'[REC {r.label}] Tersimpan: {r.out_path} '
                    f'({r.frames_written} frame, {r.file_size() / (1024**2):.1f} MB)'
                )
            else:
                self.get_logger().warn(f'[REC {r.label}] Tidak ada frame yang terekam.')

    def destroy_node(self):
        self.running = False
        self.close_recordings()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoReceiverNode()

    # SIGTERM (dari pkill saat GUI ditutup) diubah jadi KeyboardInterrupt supaya
    # blok finally tetap jalan dan writer.release() sempat dipanggil.
    # Tanpa ini, file .avi berhenti mendadak tanpa index.
    #
    # SIGHUP juga ditangani. Alasannya: closeEvent dashboard memanggil
    # `pkill -f video_receiver`, dan pola -f itu ikut cocok dengan proses
    # `bash -c "... video_receiver ..."` milik gnome-terminal. Terminalnya ikut
    # mati, lalu mengirim SIGHUP ke node ini. Tanpa handler, SIGHUP membunuh
    # proses seketika dan file .avi tertinggal tanpa index.
    def _on_term(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, _on_term)
    signal.signal(signal.SIGHUP, _on_term)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()