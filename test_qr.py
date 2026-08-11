#!/usr/bin/env python3
"""
test_qr_webcam.py
=================
Program mandiri (standalone) tanpa ROS untuk mengetes fungsionalitas 
pyzbar dan OpenCV dalam mendeteksi dan membaca QR code menggunakan webcam.
"""

import cv2
import numpy as np
from pyzbar.pyzbar import decode

def main():
    # Buka koneksi ke webcam (Index 0 biasanya adalah kamera utama/bawaan laptop)
    cap = cv2.VideoCapture(6)

    if not cap.isOpened():
        print("Error: Tidak dapat membuka webcam.")
        return

    print("Scanner QR Mandiri Aktif!")
    print("Arahkan QR code ke kamera. Tekan tombol 'q' pada keyboard untuk keluar.")

    last_qr = ""

    while True:
        # Baca frame dari webcam
        ret, frame = cap.read()
        if not ret:
            print("Gagal mengambil frame dari kamera.")
            break

        # Konversi ke grayscale agar pyzbar lebih mudah/cepat membaca QR
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Proses decoding QR Code
        decoded_objects = decode(gray)

        for obj in decoded_objects:
            # Decode data QR ke string
            text = obj.data.decode('utf-8')
            
            # Jika QR code baru/berbeda dengan sebelumnya, print di terminal
            if text != last_qr:
                print(f"[BERHASIL] QR Terdeteksi: {text}")
                last_qr = text

            # -- Bagian Visual (Opsional): Menggambar kotak hijau di sekitar QR Code --
            pts = obj.polygon
            if len(pts) > 0:
                pts = np.array(pts, np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], True, (0, 255, 0), 3)

            # Tampilkan teks hasil scan di atas QR code pada layar
            rect = obj.rect
            cv2.putText(frame, text, (rect.left, rect.top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Tampilkan jendela video
        cv2.imshow("Test QR Scanner", frame)

        # Keluar dari loop jika tombol 'q' ditekan
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Bersihkan memory dan tutup kamera
    cap.release()
    cv2.destroyAllWindows()
    print("Program selesai.")

if __name__ == '__main__':
    main()