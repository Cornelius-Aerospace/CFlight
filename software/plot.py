# Connects to arduino streaming MPU6050 data and plots it in real time
# data format: "a/g: [ax, ay, az, gx, gy, gz]"" with tab delimiter (a/g: is static text slash header)
import numpy as np
import matplotlib.pyplot as plt
import time
import csv
import numpy as np

# Generate some sample data for altitude (alt) and velocity (vel)

time = []
altitude = []
velocity = []
with open("flight.csv", "r") as file:
  #  try:
    csv_reader = csv.reader(file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        if line_count != 0:
            time.append(int(row[0]))
            altitude.append(float(row[1]))
            velocity.append(float(row[2]))
        line_count += 1
# Create a new window with two separate subplots
fig, (ax1, ax2) = plt.subplots(2, 1)

# Plot the altitude data on the first subplot
ax1.plot(time, altitude, label="Altitude")
ax1.set_xlabel("Time")
ax1.set_ylabel("Altitude")
ax1.legend()

# Plot the velocity data on the second subplot
ax2.plot(time, velocity, label="Velocity", color='orange')
ax2.set_xlabel("Time")
ax2.set_ylabel("Velocity")
ax2.legend()

# Adjust spacing between subplots
plt.tight_layout()

# Show the plots in a new window
plt.show()
