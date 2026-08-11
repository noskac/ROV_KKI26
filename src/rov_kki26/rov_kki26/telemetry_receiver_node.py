#!/usr/bin/env python3
"""
telemetry_receiver_node.py  —  GCS side
=========================================
FIX dari versi lama:

  BUG LAMA (penyebab freeze "terbaca lalu berhenti"):
      udp_loop() berjalan di threading.Thread biasa, lalu langsung memanggil
      self.imu_pub.publish() dari thread tersebut.
      Dalam ROS 2, memanggil publish() dari thread yang BUKAN executor thread
      menyebabkan race condition pada mutex internal DDS (FastDDS / CycloneDDS).
      Hasilnya: data mengalir lalu tiba-tiba macet (deadlock sementara),
      kemudian lanjut lagi — persis gejala yang Anda alami.

  FIX:
      Thread UDP hanya bertugas menerima raw bytes dan menaruhnya ke queue.Queue
      (thread-safe, tidak menyentuh DDS sama sekali).
      Semua pemanggilan publish() dipindahkan ke dalam ROS 2 Timer callback
      (drain_timer, 100 Hz) yang dijamin berjalan di executor thread.

  FIX TAMBAHAN:
      QoS diubah dari default (RELIABLE, depth=10) menjadi BEST_EFFORT, depth=1
      untuk semua data sensor real-time (IMU, Depth, PWM, Servo).
      Ini mencegah DDS menumpuk 10 pesan lama yang kemudian "meledak" sekaligus
      saat koneksi pulih, yang menimbulkan burst jitter.
      Topic /rov/system_mode tetap RELIABLE karena dia jarang dan penting.

  PENTING — KONSISTENSI QoS:
      Karena publisher sekarang pakai BEST_EFFORT, SEMUA subscriber juga HARUS
      pakai BEST_EFFORT (rov_dashboard_node.py & qr_scanner_node.py).
      DDS akan DIAM-DIAM menolak delivery jika publisher=BEST_EFFORT
      tapi subscriber=RELIABLE. Tidak ada error, data hanya tidak sampai.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import socket
import threading
import queue

from std_msgs.msg import String, Int32MultiArray, Float32
from geometry_msgs.msg import Vector3

# ─── QoS Profiles ─────────────────────────────────────────────────────────────
# Untuk data real-time sensor: "selalu yang terbaru, buang yang lama"
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Untuk mode/status: "wajib sampai, boleh sedikit delay"
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class TelemetryReceiver(Node):
    def __init__(self):
        super().__init__('telemetry_receiver_node')

        # ─── Publishers (semua sensor pakai SENSOR_QOS) ───────────────────────
        self.imu_pub   = self.create_publisher(Vector3,         '/rov/imu_euler',   SENSOR_QOS)
        self.depth_pub = self.create_publisher(Float32,         '/rov/depth',       SENSOR_QOS)
        self.pwm_pub   = self.create_publisher(Int32MultiArray, '/rov/thruster_pwm',SENSOR_QOS)
        self.servo_pub = self.create_publisher(Int32MultiArray, '/rov/servo_data',  SENSOR_QOS)
        self.mode_pub  = self.create_publisher(String,          '/rov/system_mode', RELIABLE_QOS)

        # ─── Thread-safe queue ────────────────────────────────────────────────
        # Thread UDP hanya put() ke sini. Timer drain() yang publish().
        # maxsize=100: kalau sistem terlambat >1 detik, buang yang paling lama.
        self.data_queue = queue.Queue(maxsize=100)

        # ─── UDP Socket ───────────────────────────────────────────────────────
        self.udp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.udp_port))
        self.sock.settimeout(1.0)  # Penting: agar thread bisa shutdown bersih

        self.running = True
        self.udp_thread = threading.Thread(target=self._udp_receive_loop, daemon=True)
        self.udp_thread.start()

        # ─── ROS 2 Timer (berjalan di executor thread — thread-safe!) ─────────
        # 10ms = 100 Hz, sesuai dengan output rate Teensy
        self.drain_timer = self.create_timer(0.01, self._drain_and_publish)

        self.get_logger().info(
            f'[TELEMETRY] Mendengarkan port {self.udp_port} | QoS: BEST_EFFORT depth=1'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # UDP THREAD: hanya terima dan taruh ke queue, JANGAN publish di sini
    # ──────────────────────────────────────────────────────────────────────────
    def _udp_receive_loop(self):
        """Jalan di background thread. TIDAK boleh memanggil publish()."""
        while self.running and rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(4096)
                try:
                    self.data_queue.put_nowait(data)
                except queue.Full:
                    # Queue penuh: buang 1 paket lama, masukkan yang baru
                    try:
                        self.data_queue.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        self.data_queue.put_nowait(data)
                    except queue.Full:
                        pass
            except socket.timeout:
                continue  # Normal, lanjut
            except OSError:
                break     # Socket ditutup saat shutdown

    # ──────────────────────────────────────────────────────────────────────────
    # EXECUTOR THREAD (via timer): drain queue lalu publish — thread-safe
    # ──────────────────────────────────────────────────────────────────────────
    def _drain_and_publish(self):
        """Dipanggil 100x/detik oleh ROS 2 executor. Aman memanggil publish()."""
        # Proses semua paket yang sudah terkumpul sejak timer terakhir
        while not self.data_queue.empty():
            try:
                raw = self.data_queue.get_nowait()
                self._parse_and_publish(raw)
            except queue.Empty:
                break

    def _parse_and_publish(self, raw_bytes: bytes):
        try:
            msg_str = raw_bytes.decode('utf-8').strip()
        except Exception:
            return

        # ── 1. Data Sensor: "P:12.5 R:-3.2 Y:45.0 D:1.23" ──────────────────
        if msg_str.startswith('P:'):
            imu = Vector3()
            depth_val = 0.0
            for token in msg_str.split():
                try:
                    if token.startswith('P:'): imu.x   = float(token[2:])
                    elif token.startswith('R:'): imu.y  = float(token[2:])
                    elif token.startswith('Y:'): imu.z  = float(token[2:])
                    elif token.startswith('D:'): depth_val = float(token[2:])
                except ValueError:
                    pass
            self.imu_pub.publish(imu)
            self.depth_pub.publish(Float32(data=depth_val))

        # ── 2. Data PWM: "PWM DKIRI:1500 DKANAN:1500 ..." ───────────────────
        elif msg_str.startswith('PWM'):
            pwm_data = [1500] * 6
            label_map = {
                'DKIRI': 0, 'TKIRI': 1, 'BKIRI': 2,
                'DKANAN': 3, 'TKANAN': 4, 'BKANAN': 5,
            }
            for token in msg_str[4:].split():
                parts = token.split(':', 1)
                if len(parts) == 2 and parts[0] in label_map:
                    try:
                        pwm_data[label_map[parts[0]]] = int(parts[1])
                    except ValueError:
                        pass
            self.pwm_pub.publish(Int32MultiArray(data=pwm_data))

        # ── 3. Mode System: "MODE:MANUAL (Remote)" ───────────────────────────
        elif msg_str.startswith('MODE:'):
            self.mode_pub.publish(String(data=msg_str[5:]))

        # ── 4. Data Servo dari echo CMD Teensy: "CMD ... TiltArm:90 Gripper:90"
        elif msg_str.startswith('CMD'):
            tilt, grip = 90, 90
            for token in msg_str.split():
                kv = token.split(':', 1)
                if len(kv) == 2:
                    try:
                        if kv[0] == 'TiltArm': tilt = int(kv[1])
                        elif kv[0] == 'Gripper': grip = int(kv[1])
                    except ValueError:
                        pass
            self.servo_pub.publish(Int32MultiArray(data=[tilt, grip]))

    # ──────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.running = False
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()