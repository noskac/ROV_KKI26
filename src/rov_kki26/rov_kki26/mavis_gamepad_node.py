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
        # ================= STATE VARIABEL =================
        self.tilt_arm_val = 180  # Ubah dari 90 ke 180
        self.gripper_val = 180   # Ubah dari 90 ke 180
        self.current_mode = 1  
        self.is_h_locked = False
        self.prev_lock_btn_state = False
        self.calculated_h = 1500 
        self.h_locked_val = 1500

        # ROS 2 Timer (Menggantikan while True dan time.sleep)
        # Akan mengeksekusi self.control_loop setiap 0.05 detik (20 Hz)
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

    def map_pwm(self, axis_val, invert=False):
        # Terapkan efek Ramping / Expo menggunakan kurva pangkat 3 (Cubic)
        # Dengan pangkat ganjil (3), nilai minus akan tetap minus.
        # Contoh efek:
        # Jika joystick digeser 50% (0.5), output PWM hanya bergeser 12.5% (0.125) -> Sangat halus!
        # Jika joystick digeser 100% (1.0), output PWM tetap 100% (1.0) -> Tetap bisa full speed!
        ramped_val = axis_val ** 3
        
        if invert: ramped_val = -ramped_val
        return max(1000, min(2000, int(1500 + (ramped_val * 500))))

    # --- LOOP KONTROL UTAMA ---
    def control_loop(self):
        s_val = y_val = r_val = t_val = 1500
        lock_btn_pressed = False

        if self.has_gamepad:
            pygame.event.pump()
            if self.safe_button(8): self.current_mode = 1
            elif self.safe_button(9): self.current_mode = 2
            elif self.safe_button(10): self.current_mode = 3

            # r_val (Roll) menggunakan Sumbu 0 (Kanan/Kiri). 
            # invert=True ditambahkan untuk membalik arah PWM yang terbalik.
            r_val = self.map_pwm(self.safe_axis(1))
            
            t_val = self.map_pwm(self.safe_axis(0), invert=True)
            
            # Tambahkan invert=True untuk membalik arah PWM Yaw
            y_val = self.map_pwm(self.safe_axis(3), invert=True) 
            s_val = self.map_pwm(self.safe_axis(4), invert=True)
            
            trig_down = (self.safe_axis(5) + 1.0) / 2.0
            trig_up = (self.safe_axis(2) + 1.0) / 2.0
            
            if trig_down > 0.05: self.calculated_h += 3 * trig_down
            if trig_up > 0.05: self.calculated_h -= 3 * trig_up

            self.calculated_h = max(1000, min(2000, self.calculated_h))
            lock_btn_pressed = self.safe_button(0)

            if self.joy.get_numhats() > 0:
                hat_x, hat_y = self.joy.get_hat(0)
                if hat_y == -1: self.tilt_arm_val = min(360, self.tilt_arm_val + 5) # Ubah 180 jadi 360
                elif hat_y == 1: self.tilt_arm_val = max(0, self.tilt_arm_val - 5)
                if hat_x == -1: self.gripper_val = max(0, self.gripper_val - 5)
                elif hat_x == 1: self.gripper_val = min(360, self.gripper_val + 5)  # Ubah 180 jadi 360

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
            if 'e' in self.pressed_keys: r_val = 2000
            elif 'q' in self.pressed_keys: r_val = 1000
            if keyboard.Key.up in self.pressed_keys: t_val = 2000
            elif keyboard.Key.down in self.pressed_keys: t_val = 1000

            if keyboard.Key.shift in self.pressed_keys: self.calculated_h = min(2000, self.calculated_h + 3)
            elif keyboard.Key.ctrl in self.pressed_keys: self.calculated_h = max(1000, self.calculated_h - 3)

            lock_btn_pressed = keyboard.Key.space in self.pressed_keys

            if 'i' in self.pressed_keys: self.tilt_arm_val = min(360, self.tilt_arm_val + 1) # Ubah 180 jadi 360
            elif 'k' in self.pressed_keys: self.tilt_arm_val = max(0, self.tilt_arm_val - 1)
            if 'o' in self.pressed_keys: self.gripper_val = min(360, self.gripper_val + 1)   # Ubah 180 jadi 360
            elif 'l' in self.pressed_keys: self.gripper_val = max(0, self.gripper_val - 1)

        # Logika Transmisi
        if lock_btn_pressed and not self.prev_lock_btn_state:
            self.is_h_locked = not self.is_h_locked
        self.prev_lock_btn_state = lock_btn_pressed
 
        if not self.is_h_locked:
            self.h_locked_val = int(self.calculated_h)
        
        data_string = f"{s_val},{y_val},{r_val},{t_val},{self.h_locked_val},{self.tilt_arm_val},{self.gripper_val},{self.current_mode}"
        self.sock.sendto(data_string.encode('utf-8'), (self.JETSON_IP, self.UDP_PORT))
        
        mode_str = "EMERGENCY" if self.current_mode == 3 else ("AUTO" if self.current_mode == 2 else "MANUAL")
        lock_str = "LOCKED" if self.is_h_locked else "FREE"
        
        # Gunakan sistem logging ROS 2 alih-alih print()
        # self.get_logger().info(f"[{'GAMEPAD' if self.has_gamepad else 'KEYBOARD'}] Mode:{mode_str} | H:{self.h_locked_val} ({lock_str}) | {data_string}")

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