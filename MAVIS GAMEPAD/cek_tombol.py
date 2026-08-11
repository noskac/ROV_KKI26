import pygame
import time

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Gamepad tidak terdeteksi.")
    exit()

joy = pygame.joystick.Joystick(0)
joy.init()

print(f"Controller: {joy.get_name()}")
print(f"Total Buttons: {joy.get_numbuttons()}")
print(f"Total Axes (Analog/Trigger): {joy.get_numaxes()}")
print("Silakan tekan tombol atau gerakkan analog di gamepad Anda! (Tekan Ctrl+C untuk keluar)\n")

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"Tombol ditekan: B{event.button}")
                
            elif event.type == pygame.JOYHATMOTION:
                print(f"D-Pad (Hat) ditekan: Hat {event.hat} | Nilai {event.value}")
                
            elif event.type == pygame.JOYAXISMOTION:
                # Mengabaikan pergerakan sangat kecil (di bawah 0.1) agar tidak spam (Deadzone)
                if abs(event.value) > 0.1:
                    print(f"Analog digerakkan: Axis {event.axis} | Nilai: {event.value:.2f}")
                    
        time.sleep(0.05)
        
except KeyboardInterrupt:
    pygame.quit()