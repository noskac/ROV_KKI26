import pygame
import socket
import time
from pynput import keyboard

# ================= KONFIGURASI JARINGAN =================
JETSON_IP = "192.168.1.10"  
UDP_PORT = 5006

print(f"[INFO] Menyiapkan transmisi UDP ke {JETSON_IP}:{UDP_PORT}...")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ================= INISIALISASI GAMEPAD =================
pygame.init()
pygame.joystick.init()

has_gamepad = pygame.joystick.get_count() > 0
joy = None

if has_gamepad:
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"[INFO] Gamepad Terdeteksi: {joy.get_name()}")
else:
    print("[WARNING] Gamepad TIDAK terdeteksi! -> Beralih ke kontrol KEYBOARD.")

def safe_button(joy_obj, btn_idx):
    if joy_obj and btn_idx < joy_obj.get_numbuttons():
        return joy_obj.get_button(btn_idx)
    return False

def safe_axis(joy_obj, axis_idx):
    if joy_obj and axis_idx < joy_obj.get_numaxes():
        return joy_obj.get_axis(axis_idx)
    return 0.0

# ================= INISIALISASI KEYBOARD =================
pressed_keys = set()

def on_press(key):
    try: pressed_keys.add(key.char.lower())
    except AttributeError: pressed_keys.add(key)

def on_release(key):
    try:
        if key.char.lower() in pressed_keys: pressed_keys.remove(key.char.lower())
    except AttributeError:
        if key in pressed_keys: pressed_keys.remove(key)

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# ================= FUNGSI & STATE =================
def map_pwm(axis_val, invert=False):
    if invert: axis_val = -axis_val
    return max(1000, min(2000, int(1500 + (axis_val * 500))))

tilt_arm_val = 90
gripper_val = 90
current_mode = 1  

is_h_locked = False
prev_lock_btn_state = False

# Dipindah ke luar loop agar nilai Heave bertahan saat tombol dilepas
calculated_h = 1500 
h_locked_val = 1500

print("\n--- Sistem Siap Mengirim Data ---")

try:
    while True:
        s_val = y_val = r_val = t_val = 1500
        lock_btn_pressed = False

        # ================= KONTROL GAMEPAD =================
        if has_gamepad:
            pygame.event.pump()
            
            # --- LOGIKA MODE SYSTEM ---
            # Mode Manual = B8, Mode Auto = B9, Mode Emergency = B10
            if safe_button(joy, 8):
                current_mode = 1
            elif safe_button(joy, 9):
                current_mode = 2
            elif safe_button(joy, 10):
                current_mode = 3

            # --- MAPPING AXIS KONTROL UTAMA ---
            r_val = map_pwm(safe_axis(joy, 0)) # Roll (Kiri/Kanan)
            t_val = map_pwm(safe_axis(joy, 1)) # Tilt (Depan/Belakang)
            y_val = map_pwm(safe_axis(joy, 3)) # Yaw (Kiri/Kanan)
            s_val = map_pwm(safe_axis(joy, 4), invert=True) # Surge (Maju/Mundur)
            
            # --- LOGIKA HEAVE (Tertahan / State Integrator) ---
            # Axis 5 untuk Heave Down (PWM +), Axis 2 untuk Heave Up (PWM -)
            trig_down = (safe_axis(joy, 5) + 1.0) / 2.0
            trig_up = (safe_axis(joy, 2) + 1.0) / 2.0
            
            # Jika trigger ditekan melebihi deadzone, update posisi Heave
            if trig_down > 0.05:
                calculated_h += 3 * trig_down # Kecepatan penambahan (sesuaikan jika kurang responsif)
            if trig_up > 0.05:
                calculated_h -= 3 * trig_up

            # Batasi nilai Heave agar tidak melebihi limit ESC
            calculated_h = max(1000, min(2000, calculated_h))

            # Tombol B0 untuk Lock Kedalaman
            lock_btn_pressed = safe_button(joy, 0)

            # --- LOGIKA ARM & GRIPPER (D-PAD / HAT 0) ---
            if joy.get_numhats() > 0:
                hat_x, hat_y = joy.get_hat(0)
                
                # Arm 0 Tilt: Keatas (0, -1), Kebawah (0, 1)
                if hat_y == -1:   
                    tilt_arm_val = min(180, tilt_arm_val + 1)
                elif hat_y == 1: 
                    tilt_arm_val = max(0, tilt_arm_val - 1)
                
                # Arm 1 Open/Close: Open (-1, 0), Close (1, 0)
                if hat_x == -1:   
                    gripper_val = max(0, gripper_val - 1)
                elif hat_x == 1: 
                    gripper_val = min(180, gripper_val + 1)

        # ================= KONTROL KEYBOARD =================
        else:
            # --- LOGIKA MODE EMERGENCY BARU (=+[ untuk ON, -+] untuk OFF) ---
            is_on_trigger = ('=' in pressed_keys or '+' in pressed_keys) and '[' in pressed_keys
            is_off_trigger = ('-' in pressed_keys or '_' in pressed_keys) and ']' in pressed_keys
            
            if is_on_trigger:
                current_mode = 3
            elif is_off_trigger:
                current_mode = 1
            else:
                if '1' in pressed_keys: current_mode = 1
                if '2' in pressed_keys: current_mode = 2

            if 'w' in pressed_keys: s_val = 2000
            elif 's' in pressed_keys: s_val = 1000

            if 'd' in pressed_keys: y_val = 2000
            elif 'a' in pressed_keys: y_val = 1000

            if 'e' in pressed_keys: r_val = 2000
            elif 'q' in pressed_keys: r_val = 1000

            if keyboard.Key.up in pressed_keys: t_val = 2000
            elif keyboard.Key.down in pressed_keys: t_val = 1000

            if keyboard.Key.shift in pressed_keys: calculated_h = min(2000, calculated_h + 3)
            elif keyboard.Key.ctrl in pressed_keys: calculated_h = max(1000, calculated_h - 3)

            lock_btn_pressed = keyboard.Key.space in pressed_keys

            if 'i' in pressed_keys: tilt_arm_val = min(180, tilt_arm_val + 1)
            elif 'k' in pressed_keys: tilt_arm_val = max(0, tilt_arm_val - 1)

            if 'o' in pressed_keys: gripper_val = min(180, gripper_val + 1)
            elif 'l' in pressed_keys: gripper_val = max(0, gripper_val - 1)

        # ================= LOGIKA TRANSMISI & LOCK =================
        # Toggle lock flag saat B0 ditekan
        if lock_btn_pressed and not prev_lock_btn_state:
            is_h_locked = not is_h_locked
        prev_lock_btn_state = lock_btn_pressed
 
        # Jika tidak dilock, update nilai transmisi dengan nilai kalkulasi terbaru
        if not is_h_locked:
            h_locked_val = int(calculated_h)
        
        data_string = f"{s_val},{y_val},{r_val},{t_val},{h_locked_val},{tilt_arm_val},{gripper_val},{current_mode}"
        sock.sendto(data_string.encode('utf-8'), (JETSON_IP, UDP_PORT))
        
        mode_str = "EMERGENCY" if current_mode == 3 else ("AUTO" if current_mode == 2 else "MANUAL")
        lock_str = "LOCKED" if is_h_locked else "FREE"
        
        print(f"[{'GAMEPAD' if has_gamepad else 'KEYBOARD'}] Mode:{mode_str} | H:{h_locked_val} ({lock_str}) | {data_string}      ", end="\r")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n[INFO] Program dihentikan.")
finally:
    if has_gamepad: pygame.quit()