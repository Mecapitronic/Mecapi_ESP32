import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from serial import Serial

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(0, 0, 0, linestyle="", marker="o")
plt.draw()
ax.cla()

with Serial('COM14', 921600) as ser:
        while True:
            line = ser.readline().decode("utf-8")
            #line = "x:y:z"
            line_splitted = line.replace("\r\n","").split(':')
            sphere1rad = 0.5
            #sphere1num = line_splitted[0]
            sphere1x = int(line_splitted[0])
            sphere1y = int(line_splitted[1])
            sphere1z = int(line_splitted[2])/10
            ax.plot(sphere1x, sphere1y, sphere1z, linestyle="", marker="o")
            plt.draw()
            #plt.pause(0.001)
            #ax.cla()