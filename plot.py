# Connects to arduino streaming MPU6050 data and plots it in real time
# data format: "a/g: [ax, ay, az, gx, gy, gz]"" with tab delimiter (a/g: is static text slash header)
import serial
import numpy as np
import matplotlib.pyplot as plt
import time

PORT = '/dev/ttyUSB0'
BAUD = 115200
TIMEOUT = 1
HISTORY = 100 # Number of data points to plot
# Set up serial connection
ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
# Set up plot
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
fig.show()
fig.canvas.draw()
# Set up data arrays
axs = []
ays = []
azs = []
gxs = []
gys = []
gzs = []
# Set up plot
ax.set_xlim(0, 100)
# ax.set_ylim(-180, 180)
ax.set_xlabel('Time')
ax.set_ylabel('Angle')
ax.set_title('MPU6050 Data')
# Set up plot lines
#axs_line, = ax.plot(axs, label='ax')
#ays_line, = ax.plot(ays, label='ay')
#azs_line, = ax.plot(azs, label='az')
gxs_line, = ax.plot(gxs, label='gx')
gys_line, = ax.plot(gys, label='gy')
gzs_line, = ax.plot(gzs, label='gz')
ax.legend()

# Main loop
while True:
    try:
        data_raw = ser.readline().decode('utf-8').strip()
        data_raw = data_raw.split(',')
        if (len(data_raw) == 15):
            print(data_raw)
        #    axs.append(float(data_raw[1]))
        #    ays.append(float(data_raw[2]))
        #    azs.append(float(data_raw[3]))
            gxs.append(float(data_raw[1]))
            gys.append(float(data_raw[2]))
            gzs.append(float(data_raw[3]))
            if (len(gxs) > HISTORY):
               # axs.pop(0)
              #  ays.pop(0)
             #   azs.pop(0)
                gxs.pop(0)
                gys.pop(0)
                gzs.pop(0)
            
          #  axs_line.set_xdata(np.arange(len(axs)))
          #  axs_line.set_ydata(axs)
          #  ays_line.set_xdata(np.arange(len(ays)))
           # ays_line.set_ydata(ays)
           # azs_line.set_xdata(np.arange(len(azs)))
           # azs_line.set_ydata(azs)
            x = np.arange(len(gxs))
            gxs_line.set_xdata(x)
            gxs_line.set_ydata(gxs)
            gys_line.set_xdata(x)
            gys_line.set_ydata(gys)
            gzs_line.set_xdata(x)
            gzs_line.set_ydata(gzs)
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw()
            fig.canvas.flush_events()
        else:
            print('Invalid data: ')
            print(data_raw)
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(e)
        continue

ser.close()
    
