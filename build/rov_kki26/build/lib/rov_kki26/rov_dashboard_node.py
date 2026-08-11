#!/usr/bin/env python3
"""
rov_dashboard_node.py  —  GCS side
=====================================
FIX dari versi lama:

  BUG #1 — QoS INCOMPATIBLE (penyebab data tidak masuk setelah fix node lain):
      Semua subscription di Ros2Worker menggunakan queue_size=10 (shorthand
      untuk QoS default: RELIABLE, depth=10).
      Setelah publisher di telemetry/video_receiver diubah ke BEST_EFFORT,
      DDS menolak delivery secara DIAM-DIAM — tidak ada error, data tidak masuk.
      Fix: semua subscription sensor/video diubah ke BEST_EFFORT depth=1.
           mode_cb tetap RELIABLE karena jarang tapi penting.

  BUG #2 — closeEvent didefinisikan DUA KALI:
      Python silently menimpa definisi pertama dengan yang kedua.
      Yang kedua sudah benar (kill subprocess + os._exit), tapi yang pertama
      menjadi dead code. Fix: hapus yang pertama, sisakan yang kedua.

  BUG #3 — self.ros_worker.terminate() terlalu kasar:
      QThread.terminate() membunuh thread tanpa memberi kesempatan cleanup.
      rclpy.shutdown() dan node.destroy_node() tidak pernah dipanggil,
      mengakibatkan port DDS tidak dilepas → startup berikutnya bisa gagal.
      Fix: gunakan requestInterruption() + spin loop yang cek flag tersebut,
           sehingga shutdown bersih dan DDS teardown berjalan normal.
"""

import sys
import os
import cv2
import numpy as np
import datetime
import socket
import subprocess

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel,
                             QVBoxLayout, QHBoxLayout, QGridLayout, QFrame, QPushButton)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap, QFont

import vtk
from pyvistaqt import QtInteractor

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String, Int32MultiArray, Float32
from geometry_msgs.msg import Vector3

# ─── QoS Profiles ─────────────────────────────────────────────────────────────
# Harus IDENTIK dengan yang dipakai publisher di telemetry/video_receiver
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


# ==========================================
# 1. KELAS WORKER ROS 2 (Thread Terpisah)
# ==========================================
class Ros2Worker(QThread):
    sig_cam1  = pyqtSignal(np.ndarray)
    sig_cam2  = pyqtSignal(np.ndarray)
    sig_qr    = pyqtSignal(str)
    sig_imu   = pyqtSignal(float, float, float)
    sig_depth = pyqtSignal(float)
    sig_pwm   = pyqtSignal(list)
    sig_mode  = pyqtSignal(str)
    sig_servo = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.node   = None
        self.bridge = CvBridge()

    def run(self):
        rclpy.init()
        self.node = Node('pyqt_dashboard_node')

        # ── Subscriptions sensor real-time → SENSOR_QOS (BEST_EFFORT depth=1)
        self.node.create_subscription(RosImage,         '/rov/cam1/image_raw',  self.cam1_cb,  SENSOR_QOS)
        self.node.create_subscription(RosImage,         '/rov/cam2/image_raw',  self.cam2_cb,  SENSOR_QOS)
        self.node.create_subscription(Vector3,          '/rov/imu_euler',       self.imu_cb,   SENSOR_QOS)
        self.node.create_subscription(Float32,          '/rov/depth',           self.depth_cb, SENSOR_QOS)
        self.node.create_subscription(Int32MultiArray,  '/rov/thruster_pwm',    self.pwm_cb,   SENSOR_QOS)
        self.node.create_subscription(Int32MultiArray,  '/rov/servo_data',      self.servo_cb, SENSOR_QOS)
        # ── QR data dikategorikan sensor (ringan, tidak butuh RELIABLE)
        self.node.create_subscription(String,           '/rov/qr_data',         self.qr_cb,    SENSOR_QOS)
        # ── Mode system: RELIABLE karena jarang tapi penting
        self.node.create_subscription(String,           '/rov/system_mode',     self.mode_cb,  RELIABLE_QOS)

        # ── FIX: Pakai spin loop dengan cek isInterruptionRequested()
        #    agar bisa shutdown bersih lewat requestInterruption()
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)

        while rclpy.ok() and not self.isInterruptionRequested():
            executor.spin_once(timeout_sec=0.05)  # 50ms timeout → 20Hz event check

        # Cleanup bersih (tidak pernah terjadi di versi lama karena terminate())
        self.node.destroy_node()
        rclpy.shutdown()

    # ── Callbacks — emit sinyal ke thread GUI (Qt signals thread-safe) ────────
    def cam1_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.sig_cam1.emit(cv2.resize(img, (864, 486)))
        except Exception:
            pass

    def cam2_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.sig_cam2.emit(cv2.resize(img, (864, 486)))
        except Exception:
            pass

    def qr_cb(self, msg):      self.sig_qr.emit(msg.data)
    def imu_cb(self, msg):     self.sig_imu.emit(msg.x, msg.y, msg.z)
    def depth_cb(self, msg):   self.sig_depth.emit(msg.data)
    def mode_cb(self, msg):    self.sig_mode.emit(msg.data)

    def pwm_cb(self, msg):
        if len(msg.data) >= 6:
            self.sig_pwm.emit(list(msg.data))

    def servo_cb(self, msg):
        if len(msg.data) >= 2:
            self.sig_servo.emit(list(msg.data))


# ==========================================
# 2. KELAS JENDELA UTAMA GUI (PyQt5)
# ==========================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROV GCS - Dashboard (RTU-MAVIS EVO)')
        self.setStyleSheet('background-color: #1e1e1e; color: white;')
        self.showMaximized()

        self.gcs_cmd_sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.jetson_address  = ('192.168.1.10', 5006)
        self.emergency_active = False

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        self.setup_top_bar()
        self.setup_camera_row()
        self.setup_telemetry_row()
        self.setup_footer()

        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self.update_clock)
        self.clock_timer.start(1000)

        self.latest_pitch = 0.0
        self.latest_roll  = 0.0
        self.latest_yaw   = 0.0

        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self.update_3d_render)
        self.render_timer.start(33)

        self.ros_worker = Ros2Worker()
        self.connect_signals()
        self.ros_worker.start()

    # ── Setup layout (tidak berubah dari versi sebelumnya) ───────────────────
    def setup_top_bar(self):
        top_frame = QFrame()
        top_frame.setStyleSheet('background-color: #2d2d2d; border: 1px solid #555555;')
        top_layout = QHBoxLayout(top_frame)

        team_lbl = QLabel('Team: RTU-MAVIS EVO | Univ: Universitas Negeri Yogyakarta')
        team_lbl.setFont(QFont('Courier', 12, QFont.Bold))
        team_lbl.setStyleSheet('border: none;')

        self.btn_emergency = QPushButton('⚠ EMERGENCY STOP ⚠')
        self.btn_emergency.setFont(QFont('Courier', 12, QFont.Bold))
        self.btn_emergency.setStyleSheet(
            'background-color: red; color: white; padding: 5px; border-radius: 5px;'
        )
        self.btn_emergency.clicked.connect(self.toggle_emergency)

        self.lbl_time = QLabel('Day, Date Time')
        self.lbl_time.setFont(QFont('Courier', 12, QFont.Bold))
        self.lbl_time.setStyleSheet('color: #00ff00; border: none;')

        top_layout.addWidget(team_lbl)
        top_layout.addStretch()
        top_layout.addWidget(self.btn_emergency)
        top_layout.addStretch()
        top_layout.addWidget(self.lbl_time)
        self.main_layout.addWidget(top_frame)

    def setup_camera_row(self):
        cam_layout = QHBoxLayout()
        cam_layout.setContentsMargins(0, 0, 0, 0)
        cam_layout.setSpacing(5)

        for label_text, attr_name in [
            ('CAMERA 1 (Front Cam)', 'lbl_cam1'),
            ('CAMERA 2 (Bottom / Side Cam)', 'lbl_cam2'),
        ]:
            frame = QFrame()
            frame.setStyleSheet('background-color: #2d2d2d; border: 1px solid #555;')
            lay = QVBoxLayout(frame)
            lay.setContentsMargins(5, 5, 5, 5)

            title = QLabel(label_text)
            title.setFont(QFont('Courier', 12, QFont.Bold))
            title.setAlignment(Qt.AlignCenter)
            title.setStyleSheet('border: none;')

            lbl = QLabel()
            lbl.setFixedSize(864, 486)
            lbl.setStyleSheet('background-color: black; border: none;')

            lay.addWidget(title)
            lay.addWidget(lbl, alignment=Qt.AlignCenter)
            setattr(self, attr_name, lbl)
            cam_layout.addWidget(frame)

        self.main_layout.addLayout(cam_layout)

    def setup_telemetry_row(self):
        telemetry_layout = QHBoxLayout()

        # ── Kiri: HUD + Thruster ──────────────────────────────────────────────
        left_frame = QFrame()
        left_layout = QHBoxLayout(left_frame)
        left_layout.setContentsMargins(0, 0, 0, 0)

        hud_frame = QFrame()
        hud_frame.setStyleSheet('background-color: #222222; border: 1px solid #555;')
        hud_lay = QVBoxLayout(hud_frame)

        self.lbl_alt_title = QLabel('ALTITUDE')
        self.lbl_alt_title.setFont(QFont('Courier', 10, QFont.Bold))
        self.lbl_alt_title.setStyleSheet('color: #888888; border: none;')
        self.lbl_alt_title.setAlignment(Qt.AlignCenter)

        self.lbl_depth = QLabel('0.00 m')
        self.lbl_depth.setFont(QFont('Courier', 35, QFont.Bold))
        self.lbl_depth.setStyleSheet('color: cyan; border: none;')
        self.lbl_depth.setAlignment(Qt.AlignCenter)

        imu_lay = QHBoxLayout()
        self.lbl_pitch = QLabel('P: 0.0°')
        self.lbl_roll  = QLabel('R: 0.0°')
        self.lbl_yaw   = QLabel('Y: 0.0°')
        for lbl, color in [(self.lbl_pitch, '#00ff00'), (self.lbl_roll, '#00ffff'), (self.lbl_yaw, '#ffff00')]:
            lbl.setStyleSheet(f'color: {color}; font-weight: bold; font-family: Courier; font-size: 14pt; border:none;')
            imu_lay.addWidget(lbl, alignment=Qt.AlignCenter)

        self.lbl_qr_data = QLabel('Data : Wait')
        self.lbl_qr_stat = QLabel('Stat : Invalid')
        self.lbl_qr_data.setStyleSheet('color: yellow; font-family: Courier; font-weight: bold; font-size: 11pt; border:none;')
        self.lbl_qr_stat.setStyleSheet('color: red; font-family: Courier; font-weight: bold; font-size: 11pt; border:none;')

        hud_lay.addWidget(self.lbl_alt_title)
        hud_lay.addWidget(self.lbl_depth)
        hud_lay.addLayout(imu_lay)
        hud_lay.addWidget(self.lbl_qr_data, alignment=Qt.AlignCenter)
        hud_lay.addWidget(self.lbl_qr_stat, alignment=Qt.AlignCenter)

        thruster_frame = QFrame()
        thruster_frame.setStyleSheet('background-color: #2d2d2d; border: 1px solid #555;')
        thr_lay = QVBoxLayout(thruster_frame)
        lbl_thr = QLabel('ROV THRUSTER (PWM)')
        lbl_thr.setFont(QFont('Courier', 12, QFont.Bold))
        lbl_thr.setStyleSheet('border:none;')
        lbl_thr.setAlignment(Qt.AlignCenter)
        thr_lay.addWidget(lbl_thr)

        self.pwm_labels = {}
        grid_lay = QGridLayout()
        names = [('DKIRI', 0, 0), ('TKIRI', 1, 0), ('BKIRI', 2, 0),
                 ('DKANAN', 0, 2), ('TKANAN', 1, 2), ('BKANAN', 2, 2)]
        for name, r, c in names:
            lbl = QLabel(f'{name}\n1500')
            lbl.setFont(QFont('Courier', 9, QFont.Bold))
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(
                'background-color: #222; border: 1px solid #888; border-radius: 20px; padding: 10px;'
            )
            self.pwm_labels[name] = lbl
            grid_lay.addWidget(lbl, r, c)

        self.lbl_tilt_arm = QLabel('ARM TILT\n180°')
        self.lbl_tilt_arm.setFont(QFont('Courier', 9, QFont.Bold))
        self.lbl_tilt_arm.setAlignment(Qt.AlignCenter)
        self.lbl_tilt_arm.setStyleSheet(
            'background-color: #222; border: 1px solid #888; border-radius: 20px; padding: 10px;'
        )
        self.lbl_gripper = QLabel('GRIPPER\n180°')
        self.lbl_gripper.setFont(QFont('Courier', 9, QFont.Bold))
        self.lbl_gripper.setAlignment(Qt.AlignCenter)
        self.lbl_gripper.setStyleSheet(
            'background-color: #222; border: 1px solid #888; border-radius: 20px; padding: 10px;'
        )
        grid_lay.addWidget(self.lbl_tilt_arm, 3, 0)
        grid_lay.addWidget(self.lbl_gripper,  3, 2)

        rov_body = QLabel()
        rov_body.setFixedSize(60, 140)
        rov_body.setStyleSheet('background-color: #1a2b3c; border: 2px solid #00aaff;')
        grid_lay.addWidget(rov_body, 0, 1, 4, 1, Qt.AlignCenter)
        thr_lay.addLayout(grid_lay)

        left_layout.addWidget(hud_frame, stretch=1)
        left_layout.addWidget(thruster_frame, stretch=1)

        # ── Kanan: Map + 3D ──────────────────────────────────────────────────
        right_frame = QFrame()
        right_layout = QHBoxLayout(right_frame)
        right_layout.setContentsMargins(0, 0, 0, 0)

        map_frame = QFrame()
        map_frame.setStyleSheet('background-color: #2d2d2d; border: 1px solid #555;')
        map_lay = QVBoxLayout(map_frame) 
        def update_3d_render(self):
            if hasattr(self, 'importer'):
                actors = self.plotter.renderer.GetActors()
                actors.InitTraversal()
                for _ in range(actors.GetNumberOfItems()):
                    actor = actors.GetNextActor()
                    if actor:
                        # --- KALIBRASI SUMBU (YAW & ROLL DITUKAR) ---
                        # Karena roll dan yaw tertukar, kita silangkan:
                        # rot_y diisi yaw, dan rot_z diisi roll.
                        
                        rot_x = self.latest_pitch 
                        
                        # --- OFFSET MENDATARKAN MODEL ---
                        # Pindahkan offset +90 atau -90 ke rot_y atau rot_z.
                        # Mulai dengan memberikan offset pada rot_y terlebih dahulu.
                        rot_y = self.latest_yaw -90 
                        
                        rot_z = self.latest_roll 
                        
                        actor.SetOrientation(rot_x, rot_y, rot_z)
                self.plotter.render()
        lbl_map = QLabel('TRAJECTORY MAP')
        lbl_map.setFont(QFont('Courier', 12, QFont.Bold))
        lbl_map.setStyleSheet('border:none;')
        map_lay.addWidget(lbl_map, alignment=Qt.AlignCenter)
        lbl_map_canvas = QLabel('S')
        lbl_map_canvas.setStyleSheet(
            'background-color: black; color: green; font-weight: bold; padding: 10px;'
        )
        lbl_map_canvas.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        map_lay.addWidget(lbl_map_canvas, stretch=1)

        model_frame = QFrame()
        model_frame.setStyleSheet('background-color: #2d2d2d; border: 1px solid #555;')
        model_lay = QVBoxLayout(model_frame)
        lbl_model = QLabel('3D ROV MODEL')
        lbl_model.setFont(QFont('Courier', 12, QFont.Bold))
        lbl_model.setStyleSheet('border:none;')
        model_lay.addWidget(lbl_model, alignment=Qt.AlignCenter)

        self.plotter = QtInteractor(model_frame)
        self.plotter.set_background('#111111')
        self.plotter.add_axes()
        model_lay.addWidget(self.plotter.interactor)

        obj_path = r'/home/cakson/rov_kki26_ws/src/rov_kki26/rov_kki26/Assembly1.obj'
        self.load_obj_with_color(obj_path)

        right_layout.addWidget(map_frame, stretch=1)
        right_layout.addWidget(model_frame, stretch=1)

        telemetry_layout.addWidget(left_frame, stretch=1)
        telemetry_layout.addWidget(right_frame, stretch=1)
        self.main_layout.addLayout(telemetry_layout)

    def setup_footer(self):
        footer_frame = QFrame()
        footer_frame.setStyleSheet('background-color: #1e1e1e; border: 1px solid #555;')
        f_lay = QHBoxLayout(footer_frame)
        self.lbl_status = QLabel(
            ' Mode: MENUNGGU BRIDGE...   |   Connection: ONLINE   |   Sensor Status: OK '
        )
        self.lbl_status.setFont(QFont('Courier', 10, QFont.Bold))
        self.lbl_status.setStyleSheet('color: orange; border: none;')
        f_lay.addWidget(self.lbl_status, alignment=Qt.AlignCenter)
        self.main_layout.addWidget(footer_frame)

    # ── Signal connections ────────────────────────────────────────────────────
    def connect_signals(self):
        self.ros_worker.sig_cam1.connect(self.update_cam1)
        self.ros_worker.sig_cam2.connect(self.update_cam2)
        self.ros_worker.sig_qr.connect(self.update_qr)
        self.ros_worker.sig_imu.connect(self.update_imu)
        self.ros_worker.sig_depth.connect(self.update_depth)
        self.ros_worker.sig_pwm.connect(self.update_pwm)
        self.ros_worker.sig_mode.connect(self.update_mode)
        self.ros_worker.sig_servo.connect(self.update_servo)

    # ── Update callbacks (dipanggil di Qt main thread via signals) ────────────
    def update_clock(self):
        self.lbl_time.setText(datetime.datetime.now().strftime('%A, %d-%m-%Y %H:%M:%S'))

    def cv2_to_qpixmap(self, cv_img):
        rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        return QPixmap.fromImage(QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888))

    def update_cam1(self, cv_img): self.lbl_cam1.setPixmap(self.cv2_to_qpixmap(cv_img))
    def update_cam2(self, cv_img): self.lbl_cam2.setPixmap(self.cv2_to_qpixmap(cv_img))

    def update_qr(self, text):
        if text not in ('Menunggu...', 'Belum terdeteksi'):
            self.lbl_qr_data.setText(f'Data : {text}')
            self.lbl_qr_data.setStyleSheet('color: white; border: none;')
            self.lbl_qr_stat.setText('Stat : Valid')
            self.lbl_qr_stat.setStyleSheet('color: #00ff00; border: none;')

    def update_depth(self, val): self.lbl_depth.setText(f'{val:.2f} m')

    def update_imu(self, pitch, roll, yaw):
        self.lbl_pitch.setText(f'P: {pitch:>4.1f}°')
        self.lbl_roll.setText(f'R: {roll:>4.1f}°')
        self.lbl_yaw.setText(f'Y: {yaw:>4.1f}°')
        self.latest_pitch, self.latest_roll, self.latest_yaw = pitch, roll, yaw

    def update_pwm(self, pwm_list):
        tags = ['DKIRI', 'TKIRI', 'BKIRI', 'DKANAN', 'TKANAN', 'BKANAN']
        for i, tag in enumerate(tags):
            val  = pwm_list[i]
            text = f'{tag}\n{val}'
            if self.pwm_labels[tag].text() != text:
                color = '#00ffff' if val > 1510 else ('#ff4444' if val < 1490 else 'white')
                self.pwm_labels[tag].setText(text)
                self.pwm_labels[tag].setStyleSheet(
                    f'background-color: #222; border: 1px solid {color}; '
                    f'color: {color}; border-radius: 20px; padding: 10px;'
                )

    def update_servo(self, servo_list):
        tilt, grip = servo_list[0], servo_list[1]
        t_text = f'ARM TILT\n{tilt}°'
        if self.lbl_tilt_arm.text() != t_text:
            self.lbl_tilt_arm.setText(t_text)
            c = '#00ff00' if tilt != 180 else 'white'
            self.lbl_tilt_arm.setStyleSheet(
                f'background-color: #222; border: 1px solid {c}; color: {c}; border-radius: 20px; padding: 10px;'
            )
        g_text = f'GRIPPER\n{grip}°'
        if self.lbl_gripper.text() != g_text:
            self.lbl_gripper.setText(g_text)
            c = '#00ff00' if grip != 180 else 'white'
            self.lbl_gripper.setStyleSheet(
                f'background-color: #222; border: 1px solid {c}; color: {c}; border-radius: 20px; padding: 10px;'
            )

    def update_mode(self, mode):
        self.lbl_status.setText(f' Mode: {mode.upper()}   |   Connection: ONLINE   |   Sensor Status: OK ')
        if 'EMERGENCY' in mode.upper() or 'FAILSAFE' in mode.upper():
            self.emergency_active = True
            self.lbl_status.setStyleSheet('color: red; background-color: #220000; border: none;')
            self.btn_emergency.setStyleSheet(
                'background-color: darkred; color: yellow; padding: 5px; border-radius: 5px;'
            )
            self.btn_emergency.setText('[ RELEASE EMERGENCY ]')
        else:
            self.emergency_active = False
            self.lbl_status.setStyleSheet('color: orange; background-color: #1e1e1e; border: none;')
            self.btn_emergency.setStyleSheet(
                'background-color: red; color: white; padding: 5px; border-radius: 5px;'
            )
            self.btn_emergency.setText('⚠ EMERGENCY STOP ⚠')

        if 'LOCK' in mode.upper():
            self.lbl_alt_title.setText('ALTITUDE (LOCKED)')
            self.lbl_alt_title.setStyleSheet('color: #00ff00; font-weight: bold; border: none;')
            self.lbl_depth.setStyleSheet('color: #00ff00; border: none;')
        else:
            self.lbl_alt_title.setText('ALTITUDE')
            self.lbl_alt_title.setStyleSheet('color: #888888; font-weight: bold; border: none;')
            self.lbl_depth.setStyleSheet('color: cyan; border: none;')

    def toggle_emergency(self):
        cmd = b'GUI_EMERGENCY_ON' if not self.emergency_active else b'GUI_EMERGENCY_OFF'
        self.gcs_cmd_sock.sendto(cmd, self.jetson_address)

    # ── 3D Viewer ────────────────────────────────────────────────────────────
    def load_obj_with_color(self, obj_path):
        if not os.path.exists(obj_path):
            print(f"[WARNING] File '{obj_path}' tidak ditemukan.")
            return
        try:
            self.importer = vtk.vtkOBJImporter()
            self.importer.SetFileName(obj_path)
            mtl = obj_path.replace('.obj', '.mtl')
            if os.path.exists(mtl):
                self.importer.SetFileNameMTL(mtl)
            self.importer.SetTexturePath(os.path.dirname(obj_path))
            self.importer.SetRenderWindow(self.plotter.render_window)
            self.importer.Update()
            
            # --- MENCARI TITIK TENGAH (CENTER PIVOT) ---
            actors = self.plotter.renderer.GetActors()
            
            min_x, max_x = float('inf'), float('-inf')
            min_y, max_y = float('inf'), float('-inf')
            min_z, max_z = float('inf'), float('-inf')
            
            actors.InitTraversal()
            for _ in range(actors.GetNumberOfItems()):
                actor = actors.GetNextActor()
                if actor:
                    b = actor.GetBounds()
                    # Filter bounds yang valid saja (menghindari part kosong)
                    if b[0] <= b[1]: 
                        min_x, max_x = min(min_x, b[0]), max(max_x, b[1])
                        min_y, max_y = min(min_y, b[2]), max(max_y, b[3])
                        min_z, max_z = min(min_z, b[4]), max(max_z, b[5])
                        
            # Dapatkan titik koordinat tepat di tengah robot
            center_x = (min_x + max_x) / 2.0
            center_y = (min_y + max_y) / 2.0
            center_z = (min_z + max_z) / 2.0
            
            # --- SET POROS ROTASI TANPA MENGGESER POSISI ---
            actors.InitTraversal()
            for _ in range(actors.GetNumberOfItems()):
                actor = actors.GetNextActor()
                if actor:
                    # Jadikan koordinat tengah tadi sebagai poros rotasi utama
                    actor.SetOrigin(center_x, center_y, center_z)
            
            # --- PENYESUAIAN KAMERA (3RD PERSON VIEW) ---
            def setup_camera():
                actors.InitTraversal()
                for _ in range(actors.GetNumberOfItems()):
                    actor = actors.GetNextActor()
                    if actor:
                        actor.SetOrientation(0, 90, 0) # Ganti sesuai offset yang pas sebelumnya
                
                self.plotter.renderer.ResetCamera()
                cam = self.plotter.renderer.GetActiveCamera()
                
                cam.Elevation(15)  
                cam.Azimuth(20)    
                cam.Zoom(0.8) # Zoom out
                
                self.plotter.render()
                
            QTimer.singleShot(200, setup_camera)
        except Exception as e:
            print(f'Gagal memuat model: {e}')

    def update_3d_render(self):
        if hasattr(self, 'importer'):
            actors = self.plotter.renderer.GetActors()
            actors.InitTraversal()
            for _ in range(actors.GetNumberOfItems()):
                actor = actors.GetNextActor()
                if actor:
                    # --- VARIABEL KALIBRASI ---
                    # Ubah angka ini (misal: 0, 90, 180, -90) sampai posisi awal 
                    # robot persis seperti Gambar 1 saat robot fisik diletakkan datar.
                    offset_x = 0
                    offset_y = 0    
                    offset_z = 0
                    
                    # --- MAPPING SUMBU IMU ---
                    # Jika saat fisik robot menunduk (pitch) tapi di layar malah miring (roll), 
                    # tukar self.latest_pitch dengan self.latest_roll di sini.
                    # Tambahkan tanda minus (-) jika arah putarannya terbalik.
                    rot_x = -self.latest_pitch + offset_x
                    rot_y = -self.latest_yaw + offset_y
                    rot_z = self.latest_roll + offset_z
                    
                    actor.SetOrientation(rot_x, rot_y, rot_z)
            self.plotter.render()

    # ── Shutdown bersih (FIX: closeEvent tidak duplikat) ─────────────────────
    def closeEvent(self, event):
        print('[INFO] Menutup GUI — menghentikan ROS 2 worker...')

        # FIX: requestInterruption() memberi sinyal agar spin loop berhenti
        # secara bersih, memungkinkan destroy_node() dan rclpy.shutdown()
        # terpanggil dengan benar.
        self.ros_worker.requestInterruption()
        self.ros_worker.wait(3000)  # tunggu max 3 detik

        # Matikan node-node lain
        for proc_name in ('video_receiver', 'telemetry_receiver', 'qr_scanner', 'mavis_gamepad'):
            try:
                subprocess.Popen(['pkill', '-f', proc_name])
            except Exception:
                pass

        event.accept()
        os._exit(0)


# ==========================================
# 3. ENTRY POINT
# ==========================================
def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()