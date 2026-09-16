#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import pygame
import socket
import time
import statistics
from pynput import keyboard
from std_msgs.msg import String, Float32, Empty
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import SetParametersResult

from rov_kki26.auto_mission import (
    AUTO_ABORT_AXIS_THRESHOLD,
    AUTO_ARM_HOLD_TIME,
    AUTO_HEAVE_AUTHORITY,
    AUTO_HEAVE_AUTHORITY_MAX,
    AUTO_HEAVE_AUTHORITY_MIN,
    AUTO_HEAVE_SIGN,
    AUTO_STALE_TIMEOUT,
    AUTO_ZERO_DURATION,
    AutoMission,
)

# Status depth-hold: RELIABLE seperti /rov/system_mode, karena ini status
# kritis yang jarang berubah tapi wajib sampai ke dashboard/logger — beda
# dengan data sensor kontinu yang boleh BEST_EFFORT.
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Data sensor kontinu (depth/IMU) dari telemetry_receiver_node: sama persis
# dengan SENSOR_QOS di telemetry_receiver_node.py/rov_dashboard_node.py —
# QoS publisher/subscriber HARUS identik, lihat catatan QoS di CLAUDE.md.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

class GamepadNode(Node):
    def __init__(self):
        super().__init__('mavis_gamepad_node')

        # ================= KONFIGURASI JARINGAN =================
        self.JETSON_IP = "192.168.1.10"
        self.UDP_PORT = 5006
        self.get_logger().info(f"Menyiapkan transmisi UDP ke {self.JETSON_IP}:{self.UDP_PORT}...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ================= INISIALISASI GAMEPAD =================
        pygame.init()
        pygame.joystick.init()
        self.has_gamepad = pygame.joystick.get_count() > 0
        self.joy = None

        if self.has_gamepad:
            self.joy = pygame.joystick.Joystick(0)
            self.joy.init()
            self.get_logger().info(f"Gamepad Terdeteksi: {self.joy.get_name()}")
        else:
            self.get_logger().warn("Gamepad TIDAK terdeteksi! -> Beralih ke kontrol KEYBOARD.")

        # ================= INISIALISASI KEYBOARD =================
        self.pressed_keys = set()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # ================= STATE VARIABEL =================
        self.tilt_arm_val = 180
        self.gripper_val = 180
        self.current_mode = 1

        # ============================================================
        # TUNING SENSITIVITAS / SMOOTHING  (axis 1, 2, 5)
        # ============================================================
        # Loop berjalan 20 Hz (timer 0.05 s). Semua konstanta di bawah
        # dihitung relatif terhadap frekuensi itu.

        # --- 1. DEADZONE ---
        # Buang pembacaan kecil di sekitar netral (jitter stick / drift
        # potensiometer) supaya PWM benar-benar diam di 1500 saat lepas.
        # Nilai di-rescale, jadi mentok stick tetap menghasilkan 1.0.
        self.DEADZONE = 0.08          # stick analog (axis 1)
        self.DEADZONE_TRIG = 0.05     # trigger (axis 2 & 5)

        # --- 2. EXPO ---
        # 0.0 = linear (perilaku lama), 1.0 = cubic penuh.
        # Makin besar -> gerakan di sekitar tengah makin halus/pelan,
        # tapi ujung stick tetap bisa 1000/2000. Ini yang paling terasa
        # untuk menghilangkan kesan "jumping".
        self.EXPO_ROLL = 0.60         # axis 1
        self.EXPO_HEAVE = 0.60        # axis 2 & 5

        # --- 3. LOW-PASS FILTER (EMA) ---
        # Menghaluskan perubahan mendadak. Makin kecil = makin halus tapi
        # makin terasa delay. 0.25 @20Hz ~ time constant 0.15 detik.
        self.LPF_ALPHA_ROLL = 0.25
        self.LPF_ALPHA_HEAVE = 0.20

        # --- 4. SLEW-RATE LIMIT ---
        # Batas perubahan PWM maksimum per frame. Ini jaring pengaman
        # terakhir: berapa pun cepatnya stick digerakkan, PWM tidak akan
        # melompat lebih dari nilai ini.
        # Waktu netral -> full = (500 / MAX_STEP) / 20 detik.
        self.ROLL_MAX_STEP = 20       # 500/20/20  = 1.25 s
        self.HEAVE_MAX_STEP = 15      # 500/15/20  = 1.67 s

        # --- 5. DEPTH-HOLD RE-LOCK DELAY ---
        # Jeda antara h_val kembali netral (1500) dan pengiriman depth_hold=1
        # lagi ke Teensy. Memberi waktu momentum ROV berhenti dulu supaya
        # setpoint (depthSetpoint = depth saat rising edge, di main.cpp)
        # tidak meleset karena ROV masih bergerak naik/turun.
        self.DEPTH_HOLD_RELOCK_DELAY = 0.5   # detik

        # State internal filter (jangan diubah manual)
        self.f_r_val = 1500.0
        self.f_h_val = 1500.0
        self.last_r_val = 1500
        self.last_heave_val = 1500

        # Depth-hold (toggle B0 / spacebar).
        # Pada konfigurasi gamepad saat ini, B0 menggunakan button index 0.
        # Setpoint & PID sepenuhnya ditangani di Teensy (main.cpp) lewat
        # rising-edge detection (depthSetpoint = depth saat depth_hold 0->1).
        #
        # depth_hold_wanted = status yang DIINGINKAN operator (dari toggle
        # B0/spasi). depth_hold_val (dihitung tiap frame, tidak disimpan
        # sebagai atribut) = status yang BENAR-BENAR dikirim ke Teensy.
        # Saat heave (h_val, nilai setelah smoothing) tidak netral, kirim
        # depth_hold_val=0 supaya operator dapat kontrol heave penuh, tanpa
        # mematikan depth_hold_wanted. Begitu h_val netral lagi selama
        # DEPTH_HOLD_RELOCK_DELAY detik berturut-turut, kirim depth_hold=1
        # lagi -> Teensy melihat rising edge baru -> re-lock otomatis.
        self.depth_hold_wanted = False
        self.prev_depthhold_btn_state = False
        # None = heave sedang tidak netral / baru saja jadi netral belum
        # ditandai; selain itu berisi timestamp (time.monotonic()) saat
        # h_val pertama kali terbaca netral.
        self.heave_neutral_since = None

        # Relevel (re-zero setpoint IMU): tombol B3 / tombol 'r'.
        self.relevel_flag = 0

        # ============================================================
        # MODE AUTO (mode 2): dive vertikal otomatis via AutoMission
        # ============================================================
        self.auto = AutoMission()
        self.pending_target = None                 # diisi topic/param, WAJIB
        self.heave_sign_pending = AUTO_HEAVE_SIGN   # ganti via ros2 param set
        self.heave_sign_locked = AUTO_HEAVE_SIGN    # dikunci saat arm() sukses
        # Pola identik heave_sign_pending/locked di atas: ganti via ros2
        # param set, dikunci ke AutoMission.arm() hanya saat arm() sukses,
        # tidak mengubah misi yang sedang berjalan.
        self.heave_authority_pending = AUTO_HEAVE_AUTHORITY
        self.heave_authority_locked = AUTO_HEAVE_AUTHORITY
        self.arm_hold_since = None                  # mulai tahan tombol 7/'2'

        # Telemetri depth/IMU dari GCS (dipublish telemetry_receiver_node).
        # last_depth_stamp sengaja sangat lampau di awal supaya AutoMission
        # menganggap telemetri basi sampai sample pertama tiba — default
        # aman: tidak bisa arm/bergerak tanpa depth asli.
        self.last_depth = 0.0
        self.last_depth_stamp = -1e9
        self.last_pitch = 0.0
        self.last_roll = 0.0

        # Zero manual (/rov/auto_zero_cmd): kumpulkan sampel depth mentah
        # selama AUTO_ZERO_DURATION detik, lalu median jadi titik nol.
        # Dipakai juga untuk /rov/depth_rel supaya operator tetap lihat
        # kedalaman relatif walau belum pernah arm().
        self.manual_depth_zero = None
        self._zero_collecting = False
        self._zero_collect_samples = []
        self._zero_collect_until = 0.0

        # Abort jarak jauh dari dashboard (/rov/auto_abort_cmd): one-shot,
        # dikonsumsi lalu direset di control_loop -- lihat _auto_abort_cb.
        self._remote_abort_requested = False

        self.declare_parameter('auto_target_depth', 0.0)
        self.declare_parameter('auto_heave_sign', int(AUTO_HEAVE_SIGN))
        self.declare_parameter('auto_heave_authority', float(AUTO_HEAVE_AUTHORITY))
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.auto_status_pub = self.create_publisher(
            String, '/rov/auto_status', RELIABLE_QOS)
        self.auto_setpoint_pub = self.create_publisher(
            Float32, '/rov/auto_setpoint', SENSOR_QOS)
        self.depth_rel_pub = self.create_publisher(
            Float32, '/rov/depth_rel', SENSOR_QOS)
        self.auto_zero_value_pub = self.create_publisher(
            Float32, '/rov/auto_zero_value', RELIABLE_QOS)

        self.create_subscription(
            Float32, '/rov/depth', self._depth_cb, SENSOR_QOS)
        self.create_subscription(
            Vector3, '/rov/imu_euler', self._imu_cb, SENSOR_QOS)
        self.create_subscription(
            Float32, '/rov/auto_target_cmd', self._auto_target_cb,
            RELIABLE_QOS)
        self.create_subscription(
            Empty, '/rov/auto_zero_cmd', self._auto_zero_cb, RELIABLE_QOS)
        self.create_subscription(
            Empty, '/rov/auto_abort_cmd', self._auto_abort_cb, RELIABLE_QOS)

        # Publisher status depth-hold untuk dashboard/logger. Dihitung dari
        # depth_hold_wanted + depth_hold_val yang SUDAH ada di node ini,
        # jadi tidak perlu menunggu echo balik dari Teensy (lebih cepat &
        # tidak ambigu — lihat catatan di CLAUDE.md).
        self.status_pub = self.create_publisher(String, '/rov/depth_hold_status', RELIABLE_QOS)

        # ROS 2 Timer -> 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("--- Sistem Siap Mengirim Data ---")

    # --- HELPER FUNCTIONS ---
    def on_press(self, key):
        try: self.pressed_keys.add(key.char.lower())
        except AttributeError: self.pressed_keys.add(key)

    def on_release(self, key):
        try:
            if key.char.lower() in self.pressed_keys: self.pressed_keys.remove(key.char.lower())
        except AttributeError:
            if key in self.pressed_keys: self.pressed_keys.remove(key)

    def safe_button(self, btn_idx):
        if self.joy and btn_idx < self.joy.get_numbuttons():
            return self.joy.get_button(btn_idx)
        return False

    def safe_axis(self, axis_idx):
        if self.joy and axis_idx < self.joy.get_numaxes():
            return self.joy.get_axis(axis_idx)
        return 0.0

    # ---------- BLOK SMOOTHING ----------
    def apply_deadzone(self, val, dz):
        """Buang nilai kecil di sekitar 0, sisanya di-rescale ke 0..1 penuh."""
        if abs(val) < dz:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return sign * (abs(val) - dz) / (1.0 - dz)

    def apply_expo(self, val, expo):
        """expo=0 -> linear, expo=1 -> cubic. Campuran keduanya di antaranya."""
        return (1.0 - expo) * val + expo * (val ** 3)

    def lpf(self, target, state, alpha):
        """Exponential moving average."""
        return state + alpha * (target - state)

    def slew(self, target, last, max_step):
        """Batasi perubahan PWM per frame."""
        delta = target - last
        if delta > max_step:
            return int(last + max_step)
        if delta < -max_step:
            return int(last - max_step)
        return int(target)

    def map_pwm(self, axis_val, invert=False, expo=0.0, deadzone=0.0):
        v = axis_val
        if invert:
            v = -v
        if deadzone > 0.0:
            v = self.apply_deadzone(v, deadzone)
        if expo > 0.0:
            v = self.apply_expo(v, expo)
        return max(1000, min(2000, int(1500 + (v * 500))))

    # ---------- MODE AUTO: helper & callback ----------
    def _auto_is_running(self):
        """Kembalikan True bila AutoMission aktif (bukan IDLE/terminal)."""
        return self.auto.state not in ('IDLE', 'DONE', 'ABORTED', 'FAILED')

    def _on_set_parameters(self, params):
        """
        Jalur cadangan: ros2 param set untuk target, tanda, & otoritas heave.

        auto_heave_sign dan auto_heave_authority HANYA diterapkan ke misi
        BERIKUTNYA (heave_sign_locked/heave_authority_locked baru dikunci
        ulang saat arm() sukses), supaya perubahan parameter di tengah
        misi tidak mengubah misi yang sedang berjalan.
        """
        for param in params:
            if param.name == 'auto_target_depth':
                self.pending_target = float(param.value)
            elif param.name == 'auto_heave_sign':
                if int(param.value) not in (1, -1):
                    return SetParametersResult(
                        successful=False,
                        reason='auto_heave_sign harus 1 atau -1')
                self.heave_sign_pending = int(param.value)
            elif param.name == 'auto_heave_authority':
                value = float(param.value)
                if not (AUTO_HEAVE_AUTHORITY_MIN <= value
                        <= AUTO_HEAVE_AUTHORITY_MAX):
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            'auto_heave_authority harus di rentang '
                            '{:.1f}-{:.1f}'
                        ).format(
                            AUTO_HEAVE_AUTHORITY_MIN,
                            AUTO_HEAVE_AUTHORITY_MAX))
                self.heave_authority_pending = value
        return SetParametersResult(successful=True)

    def _depth_cb(self, msg):
        self.last_depth = msg.data
        self.last_depth_stamp = time.monotonic()

    def _imu_cb(self, msg):
        self.last_pitch = msg.x
        self.last_roll = msg.y

    def _auto_target_cb(self, msg):
        self.pending_target = msg.data

    def _auto_zero_cb(self, msg):
        self._zero_collecting = True
        self._zero_collect_samples = []
        self._zero_collect_until = time.monotonic() + AUTO_ZERO_DURATION

    def _auto_abort_cb(self, msg):
        self._remote_abort_requested = True

    # --- LOOP KONTROL UTAMA ---
    def control_loop(self):
        now = time.monotonic()

        # Koleksi zero manual (/rov/auto_zero_cmd), independen dari state
        # arm/mode -- lihat _auto_zero_cb.
        if self._zero_collecting:
            self._zero_collect_samples.append(self.last_depth)
            if now >= self._zero_collect_until:
                self._zero_collecting = False
                if self._zero_collect_samples:
                    zero_val = statistics.median(self._zero_collect_samples)
                    self.manual_depth_zero = zero_val
                    self.auto.set_surface_zero(self._zero_collect_samples)
                    self.auto_zero_value_pub.publish(Float32(data=zero_val))

        # Dipakai HANYA untuk memutuskan apakah tombol 0/3 boleh dibaca di
        # bawah -- lihat catatan "ABAIKAN SEPENUHNYA" di blok override.
        pre_auto_active = self._auto_is_running()

        s_val = y_val = t_val = 1500
        r_target = h_target = 1500
        depthhold_btn_pressed = False
        heave_axis_raw = 0.0
        mode1_btn = False
        mode3_btn = False
        arm_btn_pressed = False

        if self.has_gamepad:
            pygame.event.pump()
            mode1_btn = self.safe_button(6)
            mode3_btn = self.safe_button(8)
            # Tombol 7 TIDAK LAGI menyetel mode secara instan (dulu:
            # current_mode = 2 sekali tap) -- sekarang jadi arm press-and-
            # hold, lihat blok "MODE AUTO: ARM / ABORT" di bawah.
            arm_btn_pressed = self.safe_button(7)
            if mode1_btn:
                self.current_mode = 1
            elif mode3_btn:
                self.current_mode = 3

            # --- AXIS 1 (roll/surge) : deadzone + expo, lalu dihaluskan di bawah
            r_target = self.map_pwm(
                self.safe_axis(1),
                expo=self.EXPO_ROLL,
                deadzone=self.DEADZONE,
            )

            t_val = self.map_pwm(self.safe_axis(0), invert=True, deadzone=self.DEADZONE)
            y_val = self.map_pwm(self.safe_axis(3), invert=True, deadzone=self.DEADZONE)
            s_val = self.map_pwm(self.safe_axis(4), invert=True, deadzone=self.DEADZONE)

            # --- AXIS 2 & 5 (trigger heave) ---
            # Normalisasi tiap trigger ke 0..1, beri deadzone masing-masing
            # supaya trigger yang tidak ditekan penuh-penuh tidak bocor,
            # lalu gabung jadi satu sumbu -1..1.
            trig_down = self.apply_deadzone((self.safe_axis(5) + 1.0) / 2.0, self.DEADZONE_TRIG)
            trig_up = self.apply_deadzone((self.safe_axis(2) + 1.0) / 2.0, self.DEADZONE_TRIG)
            heave_axis = max(-1.0, min(1.0, trig_down - trig_up))
            h_target = self.map_pwm(heave_axis, expo=self.EXPO_HEAVE)
            # Sumbu MENTAH sebelum LPF/slew -- dipakai AutoMission untuk
            # deteksi sentuhan operator tanpa tertunda smoothing.
            heave_axis_raw = heave_axis

            if not pre_auto_active:
                # Mode auto aktif: tombol 0/3 diabaikan SEPENUHNYA, tidak
                # dibaca sama sekali -- lihat blok override di bawah.
                depthhold_btn_pressed = self.safe_button(0)
                self.relevel_flag = 1 if self.safe_button(3) else 0

            if self.joy.get_numhats() > 0:
                hat_x, hat_y = self.joy.get_hat(0)
                if hat_y == -1: self.tilt_arm_val = min(360, self.tilt_arm_val + 5)
                elif hat_y == 1: self.tilt_arm_val = max(0, self.tilt_arm_val - 5)
                if hat_x == -1: self.gripper_val = max(0, self.gripper_val - 5)
                elif hat_x == 1: self.gripper_val = min(360, self.gripper_val + 5)

        else:
            # Kontrol Keyboard
            is_on_trigger = ('=' in self.pressed_keys or '+' in self.pressed_keys) and '[' in self.pressed_keys
            is_off_trigger = ('-' in self.pressed_keys or '_' in self.pressed_keys) and ']' in self.pressed_keys

            mode3_btn = is_on_trigger
            mode1_btn = is_off_trigger or ('1' in self.pressed_keys)
            # '2' TIDAK LAGI menyetel mode secara instan -- sekarang jadi
            # arm press-and-hold (padanan keyboard utk tombol 7 gamepad).
            arm_btn_pressed = '2' in self.pressed_keys

            if mode3_btn:
                self.current_mode = 3
            elif mode1_btn:
                self.current_mode = 1

            if 'w' in self.pressed_keys: s_val = 2000
            elif 's' in self.pressed_keys: s_val = 1000
            if 'd' in self.pressed_keys: y_val = 2000
            elif 'a' in self.pressed_keys: y_val = 1000
            if 'e' in self.pressed_keys: r_target = 2000
            elif 'q' in self.pressed_keys: r_target = 1000
            if keyboard.Key.up in self.pressed_keys: t_val = 2000
            elif keyboard.Key.down in self.pressed_keys: t_val = 1000

            if keyboard.Key.shift in self.pressed_keys: h_target = 2000
            elif keyboard.Key.ctrl in self.pressed_keys: h_target = 1000
            # Padanan sumbu mentah keyboard: shift=+1 (h_target=2000, tanpa
            # invert, sama seperti konvensi gamepad), ctrl=-1, lepas=0.0.
            if keyboard.Key.shift in self.pressed_keys: heave_axis_raw = 1.0
            elif keyboard.Key.ctrl in self.pressed_keys: heave_axis_raw = -1.0

            if not pre_auto_active:
                depthhold_btn_pressed = keyboard.Key.space in self.pressed_keys
                self.relevel_flag = 1 if ('r' in self.pressed_keys) else 0

            if 'i' in self.pressed_keys: self.tilt_arm_val = min(360, self.tilt_arm_val + 1)
            elif 'k' in self.pressed_keys: self.tilt_arm_val = max(0, self.tilt_arm_val - 1)
            if 'o' in self.pressed_keys: self.gripper_val = min(360, self.gripper_val + 1)
            elif 'l' in self.pressed_keys: self.gripper_val = max(0, self.gripper_val - 1)

        # ================= MODE AUTO: ARM / ABORT (mode 2) =================
        # Tombol 6/8 (atau padanan keyboard): minta abort. AutoMission aman
        # dipanggil dengan request_abort=True kapan pun (no-op saat IDLE).
        request_abort = mode1_btn or mode3_btn or self._remote_abort_requested
        self._remote_abort_requested = False  # one-shot, sudah dikonsumsi

        # Tombol 6 juga jadi "retry": kalau misi sudah di status terminal,
        # bersihkan supaya operator tinggal tahan tombol 7/'2' lagi.
        if mode1_btn and self.auto.state in ('DONE', 'ABORTED', 'FAILED'):
            self.auto.reset()

        auto_status_override = None
        if arm_btn_pressed:
            if self.arm_hold_since is None:
                self.arm_hold_since = now
            held_for = now - self.arm_hold_since
            if self.current_mode == 1 and self.auto.state == 'IDLE':
                if held_for >= AUTO_ARM_HOLD_TIME:
                    reject_reason = None
                    if self.pending_target is None:
                        reject_reason = 'target belum diterapkan'
                    elif abs(heave_axis_raw) > AUTO_ABORT_AXIS_THRESHOLD:
                        reject_reason = 'trigger tidak netral'
                    elif (now - self.last_depth_stamp) > AUTO_STALE_TIMEOUT:
                        reject_reason = 'telemetri depth tidak tersedia'

                    if reject_reason is not None:
                        auto_status_override = 'ARM_REJECTED: ' + reject_reason
                    elif self.auto.arm(
                            now, self.pending_target,
                            heave_authority=self.heave_authority_pending):
                        # heave_sign_locked/heave_authority_locked baru
                        # dikunci di sini (bukan sebelum arm() dipanggil),
                        # persis saat misi benar-benar mulai -- lihat
                        # catatan di _on_set_parameters.
                        self.heave_sign_locked = self.heave_sign_pending
                        self.heave_authority_locked = (
                            self.heave_authority_pending)
                        self.current_mode = 2
                        self.depth_hold_wanted = False
                        self.heave_neutral_since = None
                    else:
                        auto_status_override = (
                            'ARM_REJECTED: ' + self.auto.reason)
                else:
                    auto_status_override = 'ARMING {:.1f}s'.format(held_for)
        else:
            self.arm_hold_since = None

        # Snapshot SETELAH keputusan arm/reset di atas, supaya misi yang
        # baru saja di-arm tick ini langsung dianggap aktif (bukan menunggu
        # satu tick lagi).
        auto_active = self._auto_is_running()

        # ================= PENGHALUSAN AKHIR =================
        # Berlaku untuk gamepad maupun keyboard (keyboard justru paling
        # butuh, karena tombol itu on/off 1000<->2000 tanpa gradasi).
        self.f_r_val = self.lpf(r_target, self.f_r_val, self.LPF_ALPHA_ROLL)
        r_val = self.slew(self.f_r_val, self.last_r_val, self.ROLL_MAX_STEP)
        self.last_r_val = r_val

        self.f_h_val = self.lpf(h_target, self.f_h_val, self.LPF_ALPHA_HEAVE)
        h_val = self.slew(self.f_h_val, self.last_heave_val, self.HEAVE_MAX_STEP)
        self.last_heave_val = h_val

        # Snap ke netral kalau sudah sangat dekat, supaya tidak ada sisa
        # PWM 1499/1501 yang bikin thruster mendengung terus.
        if abs(r_val - 1500) <= 2:
            r_val = 1500
            self.f_r_val = 1500.0
            self.last_r_val = 1500
        if abs(h_val - 1500) <= 2:
            h_val = 1500
            self.f_h_val = 1500.0
            self.last_heave_val = 1500

        # ================= MODE AUTO: OVERRIDE HEAVE & SUMBU LAIN =========
        # tick() selalu dipanggil (aman: no-op/heave=1500 saat IDLE atau
        # status terminal) supaya filter & timer internal AutoMission tetap
        # konsisten walau belum/tidak sedang aktif.
        auto_output = self.auto.tick(
            now, self.last_depth, self.last_pitch, self.last_roll,
            self.last_depth_stamp, heave_axis_raw, request_abort)

        if auto_active:
            # Dive harus murni vertikal: sumbu horizontal tidak boleh bocor
            # dari gamepad/keyboard selagi misi berjalan.
            s_val = y_val = r_val = t_val = 1500
            h_val = auto_output.heave_pwm
            if self.heave_sign_locked != AUTO_HEAVE_SIGN:
                # Refleksi linear terhadap 1500: setara memakai tanda
                # kebalikannya di dalam AutoMission, tapi tanpa menyentuh
                # auto_mission.py. Slew tetap terjaga karena refleksi
                # linear mempertahankan besar selisih antar-tick.
                h_val = 3000 - h_val

        # Logika Transmisi
        # Emergency: matikan keinginan depth-hold & reset timer relock,
        # supaya keluar dari mode 3 tidak langsung re-lock diam-diam.
        if self.current_mode == 3:
            self.depth_hold_wanted = False
            self.heave_neutral_since = None

        if auto_active:
            # Mode auto aktif: tombol 0 (depth-hold) & tombol 3 (relevel)
            # diabaikan SEPENUHNYA -- tidak dibaca (lihat guard di atas)
            # DAN blok toggle/auto-release/relock di bawah dilewati total,
            # supaya prev_depthhold_btn_state & depth_hold_wanted tidak
            # diam-diam berubah selagi misi berjalan (PID Teensy tidak
            # boleh berebut sumbu heave dengan AutoMission).
            depth_hold_val = 0
        else:
            # Toggle B0/spasi (rising edge, bukan status mentah).
            if depthhold_btn_pressed and not self.prev_depthhold_btn_state:
                self.depth_hold_wanted = not self.depth_hold_wanted
            self.prev_depthhold_btn_state = depthhold_btn_pressed

            # Auto-release saat heave dipakai, auto re-lock begitu h_val netral
            # selama DEPTH_HOLD_RELOCK_DELAY detik (lihat komentar di init).
            if h_val != 1500:
                self.heave_neutral_since = None
                depth_hold_val = 0
            else:
                if self.heave_neutral_since is None:
                    self.heave_neutral_since = time.monotonic()
                settled = (time.monotonic() - self.heave_neutral_since) >= self.DEPTH_HOLD_RELOCK_DELAY
                depth_hold_val = 1 if (self.depth_hold_wanted and settled) else 0

        # Status untuk dashboard/logger: 3 kondisi dibedakan tegas supaya
        # tidak "berkedip" mengikuti depth_hold_val mentah yang naik-turun
        # tiap kali operator menyentuh heave.
        if not self.depth_hold_wanted:
            status_str = 'OFF'
        elif depth_hold_val == 1:
            status_str = 'HOLDING'
        else:
            status_str = 'MANUAL HEAVE'
        self.status_pub.publish(String(data=status_str))

        data_string = (
            f"{s_val},{y_val},{r_val},{t_val},{h_val},"
            f"{self.tilt_arm_val},{self.gripper_val},{self.current_mode},"
            f"{depth_hold_val},{self.relevel_flag}"
        )
        self.sock.sendto(data_string.encode('utf-8'), (self.JETSON_IP, self.UDP_PORT))

        # ================= MODE AUTO: TELEMETRI (20 Hz) =================
        if auto_status_override is not None:
            auto_status_str = auto_status_override
        elif auto_output.state in ('ABORTED', 'FAILED') and auto_output.reason:
            # Sertakan alasan supaya dashboard bisa menampilkannya --
            # auto_output.reason tidak pernah dipublish terpisah.
            auto_status_str = '{}: {}'.format(
                auto_output.state, auto_output.reason)
        else:
            auto_status_str = auto_output.state
        self.auto_status_pub.publish(String(data=auto_status_str))
        self.auto_setpoint_pub.publish(
            Float32(data=auto_output.setpoint_rel))
        if self.manual_depth_zero is not None:
            depth_rel_val = self.last_depth - self.manual_depth_zero
        else:
            depth_rel_val = float('nan')
        self.depth_rel_pub.publish(Float32(data=depth_rel_val))

        mode_str = "EMERGENCY" if self.current_mode == 3 else ("AUTO" if self.current_mode == 2 else "MANUAL")
        dh_str = status_str

        # self.get_logger().info(f"[{'GAMEPAD' if self.has_gamepad else 'KEYBOARD'}] Mode:{mode_str} | DepthHold:{dh_str} | {data_string}")

def main(args=None):
    rclpy.init(args=args)
    node = GamepadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.has_gamepad: pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()