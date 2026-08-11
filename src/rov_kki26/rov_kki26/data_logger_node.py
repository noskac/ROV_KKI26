#!/usr/bin/env python3
"""
data_logger_node.py  —  GCS side
=====================================
Node ROS 2 TERPISAH untuk mencatat semua data telemetry ROV ke file.

KENAPA NODE TERPISAH (bukan nempel di GUI dashboard):
    - Tetap jalan & mencatat data walau GUI dashboard crash / belum dibuka.
    - Siklus hidup node ini konsisten dengan start_gcs.sh: mulai dicatat
      begitu sistem GCS dinyalakan, berhenti & di-export begitu sistem
      dimatikan (SIGINT dari Ctrl+C, atau SIGTERM dari pkill saat GUI ditutup).
    - Independen dari QThread/Qt event loop, jadi tidak mungkin ikut ke-freeze
      kalau ada masalah rendering di GUI.

DATA YANG DICATAT (semua topic yang sama dipakai rov_dashboard_node.py):
    /rov/depth          → kedalaman (m)
    /rov/imu_euler       → pitch, roll, yaw (derajat, dari Teensy/Mahony AHRS)
    /rov/thruster_pwm    → PWM 6 thruster (DKIRI, TKIRI, BKIRI, DKANAN, TKANAN, BKANAN)
    /rov/servo_data      → posisi servo (tilt arm, gripper)
    /rov/system_mode     → mode sistem (MANUAL/AUTO/EMERGENCY dll)
    /rov/qr_data         → data QR terakhir yang terbaca

    surge_est dihitung lokal dari PWM TKIRI/TKANAN (rumus sama persis dengan
    yang dipakai rov_dashboard_node.py untuk trajectory map), supaya konsisten.

QoS:
    Sama seperti node lain di sistem ini — HARUS BEST_EFFORT depth=1 untuk
    data sensor, RELIABLE depth=10 untuk /rov/system_mode. Kalau tidak sama,
    DDS akan diam-diam menolak delivery (lihat catatan di rov_dashboard_node.py).

OUTPUT:
    ~/rov_kki26_ws/logs/rov_log_<YYYYMMDD_HHMMSS>.csv
    ~/rov_kki26_ws/logs/rov_log_<YYYYMMDD_HHMMSS>.json

    CSV ditulis & di-flush baris per baris (real-time, tahan crash).
    JSON disimpan penuh (snapshot) setiap 20 baris (~10 detik) DAN saat
    node berhenti, supaya tetap ada file JSON walau proses mati mendadak.
"""

import os
import csv
import json
import signal
import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Int32MultiArray, Float32
from geometry_msgs.msg import Vector3

# ─── QoS Profiles (harus identik dengan publisher) ────────────────────────────
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

LOG_DIR = os.path.expanduser('~/rov_kki26_ws/logs')
SAMPLE_INTERVAL_SEC = 0.5   # 2 Hz, sesuai keputusan: cukup detail, ringan
JSON_SNAPSHOT_EVERY = 20    # tulis ulang file JSON tiap 20 baris (~10 detik)

CSV_FIELDS = [
    'timestamp_iso', 'timestamp_unix',
    'depth_m', 'pitch_deg', 'roll_deg', 'yaw_deg',
    'pwm_dkiri', 'pwm_tkiri', 'pwm_bkiri',
    'pwm_dkanan', 'pwm_tkanan', 'pwm_bkanan',
    'servo_tilt_arm', 'servo_gripper',
    'mode', 'qr_data', 'surge_est',
]


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        os.makedirs(LOG_DIR, exist_ok=True)
        start_time = datetime.datetime.now()
        stamp = start_time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(LOG_DIR, f'rov_log_{stamp}.csv')
        self.json_path = os.path.join(LOG_DIR, f'rov_log_{stamp}.json')

        # ── State: nilai terakhir dari tiap topic ──────────────────────────
        self.latest = {
            'depth_m': 0.0,
            'pitch_deg': 0.0, 'roll_deg': 0.0, 'yaw_deg': 0.0,
            'pwm_dkiri': 1500, 'pwm_tkiri': 1500, 'pwm_bkiri': 1500,
            'pwm_dkanan': 1500, 'pwm_tkanan': 1500, 'pwm_bkanan': 1500,
            'servo_tilt_arm': 180, 'servo_gripper': 180,
            'mode': 'UNKNOWN', 'qr_data': '',
        }
        self.json_rows = []
        self.row_count = 0

        # ── Buka file CSV, tulis header sekali ──────────────────────────────
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv_writer.writeheader()
        self.csv_file.flush()

        # ── Subscriptions (topic & QoS sama persis dengan node lain) ───────
        self.create_subscription(Float32,         '/rov/depth',        self.depth_cb, SENSOR_QOS)
        self.create_subscription(Vector3,         '/rov/imu_euler',    self.imu_cb,   SENSOR_QOS)
        self.create_subscription(Int32MultiArray, '/rov/thruster_pwm', self.pwm_cb,   SENSOR_QOS)
        self.create_subscription(Int32MultiArray, '/rov/servo_data',   self.servo_cb, SENSOR_QOS)
        self.create_subscription(String,          '/rov/qr_data',      self.qr_cb,    SENSOR_QOS)
        self.create_subscription(String,          '/rov/system_mode',  self.mode_cb,  RELIABLE_QOS)

        # ── Timer sampling (2 Hz) ───────────────────────────────────────────
        self.sample_timer = self.create_timer(SAMPLE_INTERVAL_SEC, self.sample_and_write)

        self.get_logger().info(f'[LOGGER] Mulai mencatat ke:\n  {self.csv_path}\n  {self.json_path}')

    # ── Callbacks: cuma update state, TIDAK menulis file di sini ───────────
    def depth_cb(self, msg):
        self.latest['depth_m'] = msg.data

    def imu_cb(self, msg):
        self.latest['pitch_deg'] = msg.x
        self.latest['roll_deg']  = msg.y
        self.latest['yaw_deg']   = msg.z

    def pwm_cb(self, msg):
        if len(msg.data) >= 6:
            tags = ['pwm_dkiri', 'pwm_tkiri', 'pwm_bkiri', 'pwm_dkanan', 'pwm_tkanan', 'pwm_bkanan']
            for tag, val in zip(tags, msg.data):
                self.latest[tag] = val

    def servo_cb(self, msg):
        if len(msg.data) >= 2:
            self.latest['servo_tilt_arm'] = msg.data[0]
            self.latest['servo_gripper']  = msg.data[1]

    def qr_cb(self, msg):
        self.latest['qr_data'] = msg.data

    def mode_cb(self, msg):
        self.latest['mode'] = msg.data

    # ── Estimasi surge dari PWM (rumus sama dengan trajectory map GUI) ─────
    def estimate_surge(self):
        tkiri  = self.latest['pwm_tkiri']
        tkanan = self.latest['pwm_tkanan']
        dorongan_kiri  = (tkiri  - 1500) if abs(tkiri  - 1500) > 20 else 0
        dorongan_kanan = (tkanan - 1500) if abs(tkanan - 1500) > 20 else 0
        return (dorongan_kiri + dorongan_kanan) / 2.0

    # ── Dipanggil tiap 500ms: catat satu baris ──────────────────────────────
    def sample_and_write(self):
        now = datetime.datetime.now()
        row = {
            'timestamp_iso': now.isoformat(timespec='milliseconds'),
            'timestamp_unix': now.timestamp(),
            'surge_est': self.estimate_surge(),
        }
        row.update(self.latest)

        # Tulis CSV langsung + flush → aman kalau proses mati mendadak
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        # Simpan ke memori untuk snapshot JSON berkala
        self.json_rows.append(row)
        self.row_count += 1
        if self.row_count % JSON_SNAPSHOT_EVERY == 0:
            self._dump_json()

    def _dump_json(self):
        try:
            with open(self.json_path, 'w', encoding='utf-8') as f:
                json.dump(self.json_rows, f, indent=2, ensure_ascii=False)
        except Exception as e:
            self.get_logger().warn(f'[LOGGER] Gagal menulis JSON: {e}')

    # ── Shutdown bersih: flush & tutup file ─────────────────────────────────
    def shutdown(self):
        self.get_logger().info('[LOGGER] Menutup file log & menyimpan JSON final...')
        try:
            self._dump_json()
        finally:
            try:
                self.csv_file.close()
            except Exception:
                pass
        self.get_logger().info(f'[LOGGER] Selesai. Total {self.row_count} baris tersimpan di:\n  {self.csv_path}\n  {self.json_path}')


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()

    # Tangani SIGTERM (dikirim oleh pkill -f data_logger saat GUI ditutup)
    # supaya file tetap ter-flush & JSON final tetap tersimpan, bukan mati mendadak.
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