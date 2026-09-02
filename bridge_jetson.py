import socket
import serial
import time
import sys
import os

# ================= KONFIGURASI =================
if os.path.exists('/dev/ttyACM1'):
    SERIAL_PORT = '/dev/ttyACM1'
# Jika tidak ada, cek apakah ttyACM0 ada
elif os.path.exists('/dev/ttyACM0'):
    SERIAL_PORT = '/dev/ttyACM0'
# Jika keduanya tidak ada
else:
    SERIAL_PORT = None
    print("Peringatan: Port /dev/ttyACM1 dan /dev/ttyACM0 tidak ditemukan!")

if SERIAL_PORT:
    print(f"Menggunakan port: {SERIAL_PORT}")
BAUD_RATE = 115200

GCS_IP = '192.168.1.5'    
TELEMETRY_PORT = 5005     
COMMAND_PORT = 5006       
WATCHDOG_TIMEOUT = 0.5    

last_teensy_write_time = 0
TEENSY_WRITE_INTERVAL = 0.05 
last_mode_send_time = 0

# ================= INISIALISASI =================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.02)
    print(f"[INFO] Terhubung ke Teensy di {SERIAL_PORT}")
except Exception as e:
    print(f"[ERROR] Gagal membuka serial: {e}")
    sys.exit(1)

sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv.bind(('0.0.0.0', COMMAND_PORT))
sock_recv.setblocking(False)

print(f"[INFO] Jetson Bridge Aktif. Mendengarkan perintah di port {COMMAND_PORT}")

last_command_time = time.time()
watchdog_triggered = False

last_command_time = time.time()
watchdog_triggered = False

serial_buffer = "" # <--- TAMBAHKAN INI

# ================= LOOP UTAMA =================
active_mode = 1 
gui_emergency_lock = False # Pengunci jika tombol GUI ditekan

while True:
    current_time = time.time()
    latest_data = None
    
    try:
        while True:
            data_udp, addr = sock_recv.recvfrom(1024)
            latest_data = data_udp
    except BlockingIOError:
        pass 

    if latest_data is not None:
        command_str = latest_data.decode('utf-8')
        
        # 1. Cek apakah ini instruksi langsung dari GUI Dashboard
        if command_str == "GUI_EMERGENCY_ON":
            gui_emergency_lock = True
            active_mode = 3
        elif command_str == "GUI_EMERGENCY_OFF":
            gui_emergency_lock = False
            active_mode = 1
            
        # 2. Jika ini adalah data normal dari Gamepad/Keyboard
        else:
            parts = command_str.split(',')
            # Format baru (10 field): s,y,r,t,heave,tiltArm,gripper,mode,depth_hold,relevel
            if len(parts) >= 10:
                try:
                    gamepad_mode = int(parts[7])
                    if gui_emergency_lock:
                        active_mode = 3 # Abaikan gamepad jika GUI mengunci
                    else:
                        active_mode = gamepad_mode
                except ValueError:
                    pass

                last_command_time = current_time
                if watchdog_triggered:
                    watchdog_triggered = False

                if (current_time - last_teensy_write_time) >= TEENSY_WRITE_INTERVAL:
                    # KUNCI GANDA: Jika Mode 3, paksa thruster 1500 di Jetson & kirim Mode 3 ke Teensy
                    # depth_hold & relevel dipaksa 0 saat emergency demi keamanan.
                    if active_mode == 3:
                        teensy_cmd = f"M:1500,1500,1500,1500,1500,90,90,3,0,0\n"
                    else:
                        teensy_cmd = (
                            f"M:{parts[0]},{parts[1]},{parts[2]},{parts[3]},{parts[4]},"
                            f"{parts[5]},{parts[6]},{active_mode},{parts[8]},{parts[9]}\n"
                        )
                    
                    ser.write(teensy_cmd.encode('utf-8'))
                    ser.flush()
                    last_teensy_write_time = current_time

    # 3. WATCHDOG FAILSAFE
    if (current_time - last_command_time) > WATCHDOG_TIMEOUT and not watchdog_triggered:
        ser.write(b'M:1500,1500,1500,1500,1500,90,90,3,0,0\n') # Anggap terputus sebagai darurat (Mode 3)
        ser.flush()
        watchdog_triggered = True

    # 4. BACA SENSOR TEENSY
    try:
        while ser.in_waiting > 0:
            # Baca semua karakter yang tersedia di kabel USB saat ini
            chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            serial_buffer += chunk
            
            # Selama ada karakter Enter (\n) di dalam buffer, proses datanya!
            while '\n' in serial_buffer:
                # Pecah buffer menjadi 1 baris utuh, dan sisanya simpan lagi
                line, serial_buffer = serial_buffer.split('\n', 1)
                line = line.strip()
                
                # Tembakkan ke GCS (TAMBAHKAN CMD DI SINI!)
                if line.startswith("P:") or line.startswith("PWM") or line.startswith("CMD"):
                    sock_send.sendto(line.encode('utf-8'), (GCS_IP, TELEMETRY_PORT))
    except Exception: 
        pass

    # 5. KIRIM STATUS MODE KE GCS
    if (current_time - last_mode_send_time) > 0.5:
        if watchdog_triggered: status_mode = "FAILSAFE (Terputus!)"
        elif active_mode == 3: status_mode = "EMERGENCY (KILL SWITCH)"
        elif active_mode == 2: status_mode = "DEPTH LOCK"   # <--- UBAH BAGIAN INI
        else: status_mode = "MANUAL (Remote)"
                
        sock_send.sendto(f"MODE:{status_mode}".encode('utf-8'), (GCS_IP, TELEMETRY_PORT))
        last_mode_send_time = current_time

    time.sleep(0.001)