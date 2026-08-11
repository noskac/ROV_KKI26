#!/usr/bin/env python3
"""
qr_scanner_node.py  —  GCS side (Dual Camera Scan)
==================================================
Modifikasi:
 - Subscribe ke /rov/cam1/image_raw DAN /rov/cam2/image_raw
 - Memisahkan frame_count untuk masing-masing kamera agar frame skip 
   berjalan independen.
 - Menggunakan fungsi helper `process_image` agar kode tidak duplikat.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from pyzbar.pyzbar import decode

from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String

# Harus cocok dengan QoS publisher di video_receiver_node.py
VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class QRScannerNode(Node):
    def __init__(self):
        super().__init__('qr_scanner_node')

        # ─── Subscribe ke Kamera 1 ───
        self.create_subscription(
            RosImage,
            '/rov/cam1/image_raw',
            self.cam1_callback,
            VIDEO_QOS,
        )

        # ─── Subscribe ke Kamera 2 ───
        self.create_subscription(
            RosImage,
            '/rov/cam2/image_raw',
            self.cam2_callback,
            VIDEO_QOS,
        )

        self.qr_pub = self.create_publisher(String, '/rov/qr_data', 10)

        # Scan QR hanya setiap N frame agar tidak memberatkan CPU
        self.frame_skip  = 5
        self.frame_count_cam1 = 0
        self.frame_count_cam2 = 0
        self.last_qr     = ''

        self.get_logger().info('QR Scanner aktif. QoS: BEST_EFFORT — membaca dari CAM 1 dan CAM 2...')

    # Callback untuk Kamera 1
    def cam1_callback(self, msg: RosImage):
        self.frame_count_cam1 += 1
        if self.frame_count_cam1 % self.frame_skip == 0:
            self.process_image(msg, "CAM 1")

    # Callback untuk Kamera 2
    def cam2_callback(self, msg: RosImage):
        self.frame_count_cam2 += 1
        if self.frame_count_cam2 % self.frame_skip == 0:
            self.process_image(msg, "CAM 2")

    # Fungsi utama untuk decode gambar (Dipakai bersama)
    def process_image(self, msg: RosImage, cam_label: str):
        try:
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            gray     = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            decoded  = decode(gray)

            if decoded:
                for obj in decoded:
                    text = obj.data.decode('utf-8')
                    # Hanya publish jika teks baru atau berbeda dari sebelumnya
                    if text != self.last_qr:
                        self.last_qr = text
                        
                        # Kirim raw text ke GUI
                        self.qr_pub.publish(String(data=text))
                        
                        # Log di terminal untuk tahu kamera mana yang membaca
                        self.get_logger().info(f'[{cam_label}] QR Terdeteksi: {text}')
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = QRScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()