#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import socket
from pynput import keyboard

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

        # State internal filter (jangan diubah manual)
        self.f_r_val = 1500.0
        self.f_h_val = 1500.0
        self.last_r_val = 1500
        self.last_heave_val = 1500

        # Depth-hold (toggle B0 / spacebar).
        # Pada konfigurasi gamepad saat ini, B0 menggunakan button index 5.
        # Setpoint & PID sepenuhnya ditangani di Teensy (main.cpp).
        # Node ini hanya mengirim status ON/OFF + heave PWM mentah.
        self.depth_hold_active = False
        self.prev_depthhold_btn_state = False

        # Relevel (re-zero setpoint IMU): tombol B3 / tombol 'r'.
        self.relevel_flag = 0

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

    # --- LOOP KONTROL UTAMA ---
    def control_loop(self):
        s_val = y_val = t_val = 1500
        r_target = h_target = 1500
        depthhold_btn_pressed = False

        if self.has_gamepad:
            pygame.event.pump()
            if self.safe_button(8): self.current_mode = 1
            elif self.safe_button(9): self.current_mode = 2
            elif self.safe_button(10): self.current_mode = 3

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

            depthhold_btn_pressed = self.safe_button(5)
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

            if is_on_trigger: self.current_mode = 3
            elif is_off_trigger: self.current_mode = 1
            else:
                if '1' in self.pressed_keys: self.current_mode = 1
                if '2' in self.pressed_keys: self.current_mode = 2

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

            depthhold_btn_pressed = keyboard.Key.space in self.pressed_keys
            self.relevel_flag = 1 if ('r' in self.pressed_keys) else 0

            if 'i' in self.pressed_keys: self.tilt_arm_val = min(360, self.tilt_arm_val + 1)
            elif 'k' in self.pressed_keys: self.tilt_arm_val = max(0, self.tilt_arm_val - 1)
            if 'o' in self.pressed_keys: self.gripper_val = min(360, self.gripper_val + 1)
            elif 'l' in self.pressed_keys: self.gripper_val = max(0, self.gripper_val - 1)

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

        # Logika Transmisi
        if depthhold_btn_pressed and not self.prev_depthhold_btn_state:
            self.depth_hold_active = not self.depth_hold_active
        self.prev_depthhold_btn_state = depthhold_btn_pressed

        depth_hold_val = 1 if self.depth_hold_active else 0

        data_string = (
            f"{s_val},{y_val},{r_val},{t_val},{h_val},"
            f"{self.tilt_arm_val},{self.gripper_val},{self.current_mode},"
            f"{depth_hold_val},{self.relevel_flag}"
        )
        self.sock.sendto(data_string.encode('utf-8'), (self.JETSON_IP, self.UDP_PORT))

        mode_str = "EMERGENCY" if self.current_mode == 3 else ("AUTO" if self.current_mode == 2 else "MANUAL")
        dh_str = "ON" if self.depth_hold_active else "OFF"

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