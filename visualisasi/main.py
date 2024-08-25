import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Inisialisasi koneksi serial
ser = serial.Serial('COM6', 115200, timeout=1)

# Inisialisasi deque untuk menyimpan data jarak, derajat, dan arah
distance_data = deque(maxlen=100)
degree_data = deque(maxlen=100)
direction_data = deque(maxlen=100)

# Setup plot
fig, ax = plt.subplots(2, 1, figsize=(10, 8))

# Fungsi update untuk animasi
def update(frame):
    line = ser.readline().decode('utf-8').strip()
    if line:
        try:
            distance, degree, direction = map(float, line.split(','))
            distance_data.append(distance)
            degree_data.append(degree)
            direction_data.append(direction)
            
            ax[0].cla()
            ax[0].plot(degree_data, distance_data, 'b-o')
            ax[0].set_ylabel('Distance (cm)')
            ax[0].set_xlabel('Degree')
            ax[0].set_title('Real-Time Distance vs Degree')
            
            ax[1].cla()
            ax[1].plot(degree_data, direction_data, 'r-o')
            ax[1].set_ylabel('Direction')
            ax[1].set_xlabel('Degree')
            ax[1].set_title('Real-Time Direction vs Degree')
        except ValueError:
            pass

# Membuat animasi
ani = animation.FuncAnimation(fig, update, interval=100)

plt.tight_layout()
plt.show()

# Menutup koneksi serial saat keluar
ser.close()
