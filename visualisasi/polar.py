import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

# Inisialisasi koneksi serial
ser = serial.Serial('COM6', 115200, timeout=1)

# Inisialisasi deque untuk menyimpan data jarak, derajat, dan arah
distance_data = deque(maxlen=500)
degree_data = deque(maxlen=500)
direction_data = deque(maxlen=500)

# Setup plot dalam koordinat polar
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, polar=True)

# Fungsi update untuk animasi
def update(frame):
    line = ser.readline().decode('utf-8').strip()
    if line:
        try:
            distance, degree, direction = map(float, line.split(','))
            
            if direction == 1:
                degree = 180 - degree  # Membalikkan derajat untuk arah = 1
            
            distance_data.append(distance)
            degree_data.append(np.deg2rad(degree))  # Konversi derajat ke radian
            direction_data.append(direction)
            
            ax.clear()
            ax.plot(degree_data, distance_data, 'bo-', label='Distance (cm)')
            ax.set_rmax(max(distance_data) + 10)
            ax.set_rticks([10, 20, 30, 40, 50, 60, 70, 80])  # Mengatur jarak
            ax.set_rlabel_position(-22.5)  # Sudut label jarak
            ax.grid(True)
            ax.set_theta_zero_location('N')
            ax.set_theta_direction(-1)
            ax.set_title("Real-Time Polar Plot of Distance vs Degree")
            ax.legend(loc='upper right')
        except ValueError:
            pass

# Membuat animasi
ani = animation.FuncAnimation(fig, update, interval=100)

plt.show()

# Menutup koneksi serial saat keluar
ser.close()
