#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tkinter as tk
from tkinter import messagebox
import threading
import time
import csv
import os

# QoS disamakan dengan sistem telemetry_receiver_node.py (BEST_EFFORT)
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

class LoggerNode(Node):
    def __init__(self):
        super().__init__('dataset_logger_node')
        self.create_subscription(Int32MultiArray, '/rov/thruster_pwm', self.pwm_cb, SENSOR_QOS)
        
        self.is_recording = False
        self.start_time = 0.0
        self.temp_pwm = []
        self.current_surge_pwm = 1500.0

    def pwm_cb(self, msg):
        # Struktur index berdasarkan telemetry_receiver_node.py:
        # 0:DKIRI, 1:TKIRI, 2:BKIRI, 3:DKANAN, 4:TKANAN, 5:BKANAN
        if len(msg.data) >= 6:
            tkiri = msg.data[1]
            tkanan = msg.data[4]
            # Rata-rata PWM propulsi utama (surge)
            self.current_surge_pwm = (tkiri + tkanan) / 2.0
            
            if self.is_recording:
                self.temp_pwm.append(self.current_surge_pwm)

class LoggerGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.root.title("ROV Trajectory Data Logger")
        self.root.geometry("400x350")
        self.node = ros_node
        
        self.dataset = [] # Menyimpan: [Run_ID, Avg_PWM, Distance, Time, Velocity]
        self.run_count = 1

        # --- UI Elements ---
        tk.Label(root, text="Target Distance (meters):", font=("Courier", 12)).pack(pady=10)
        self.entry_dist = tk.Entry(root, font=("Courier", 12), justify="center")
        self.entry_dist.insert(0, "10.0")
        self.entry_dist.pack(pady=5)

        self.lbl_status = tk.Label(root, text="Status: IDLE", font=("Courier", 12, "bold"), fg="orange")
        self.lbl_status.pack(pady=15)

        self.btn_start = tk.Button(root, text="▶ START RUN", font=("Courier", 12, "bold"), bg="green", fg="white", command=self.start_run)
        self.btn_start.pack(pady=5, fill="x", padx=50)

        self.btn_stop = tk.Button(root, text="⏹ STOP RUN", font=("Courier", 12, "bold"), bg="red", fg="white", command=self.stop_run, state="disabled")
        self.btn_stop.pack(pady=5, fill="x", padx=50)

        self.btn_export = tk.Button(root, text="💾 EXPORT DATASET", font=("Courier", 12, "bold"), bg="blue", fg="white", command=self.export_data)
        self.btn_export.pack(pady=20, fill="x", padx=50)

    def start_run(self):
        try:
            float(self.entry_dist.get())
        except ValueError:
            messagebox.showerror("Error", "Masukkan jarak dalam angka!")
            return

        self.node.temp_pwm = []
        self.node.start_time = time.time()
        self.node.is_recording = True
        
        self.lbl_status.config(text="Status: RECORDING...", fg="red")
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.entry_dist.config(state="disabled")

    def stop_run(self):
        if not self.node.is_recording: return
        
        self.node.is_recording = False
        end_time = time.time()
        
        delta_t = end_time - self.node.start_time
        dist = float(self.entry_dist.get())
        
        if len(self.node.temp_pwm) == 0:
            avg_pwm = self.node.current_surge_pwm
        else:
            avg_pwm = sum(self.node.temp_pwm) / len(self.node.temp_pwm)
            
        velocity = dist / delta_t if delta_t > 0 else 0.0

        # Simpan ke dataset memory
        self.dataset.append([self.run_count, round(avg_pwm, 1), dist, round(delta_t, 3), round(velocity, 4)])
        
        self.lbl_status.config(text=f"Run {self.run_count} Saved! (v={velocity:.2f} m/s)", fg="green")
        self.run_count += 1
        
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        self.entry_dist.config(state="normal")

    def export_data(self):
        if not self.dataset:
            messagebox.showwarning("Warning", "Dataset kosong. Lakukan Start/Stop Run dulu.")
            return
            
        filename = "rov_velocity_dataset.csv"
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Run_ID", "Avg_PWM", "Distance_m", "Time_s", "Velocity_m_s"])
            writer.writerows(self.dataset)
            
        messagebox.showinfo("Success", f"Dataset berhasil diekspor ke:\n{os.path.abspath(filename)}")

def ros_spin_thread(node):
    rclpy.spin(node)

if __name__ == '__main__':
    rclpy.init()
    node = LoggerNode()
    
    # Jalankan ROS spin di thread terpisah agar GUI Tkinter tidak freeze
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()
    
    root = tk.Tk()
    app = LoggerGUI(root, node)
    root.mainloop()
    
    node.destroy_node()
    rclpy.shutdown()