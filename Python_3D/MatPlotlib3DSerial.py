import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from serial import Serial

plot_window = 0
sphere1x = np.array(np.zeros([plot_window]))
sphere1y = np.array(np.zeros([plot_window]))
sphere1z = np.array(np.zeros([plot_window]))

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#l,ys = ax.plot(var)

with Serial('COM14', 921600) as ser:
        while True:
                line = ser.readline().decode("utf-8")
                if not line == "*********************************************************":
                    #line = "x:y:z"
                    line_splitted = line.replace("\n","").replace("\r","").split(' ')
                    sphere1x = np.append(sphere1x, line_splitted[0])
                    sphere1y = np.append(sphere1y, line_splitted[1])
                    sphere1z = np.append(sphere1z, line_splitted[2])



                    #sphere1rad = 0.2
                    #sphere1x = int(line_splitted[0])
                    #sphere1y = int(line_splitted[1])
                    #sphere1z = int(line_splitted[2])
                    #ax.plot(sphere1x, sphere1y, sphere1z, linestyle="", marker="o")
                #else:
                    ax.scatter3D(sphere1x, sphere1y, sphere1z, marker="o")
                    ax.relim()
                    ax.autoscale_view()
                    #fig.canvas.draw()
                    #fig.canvas.flush_events()
                    plt.draw()
                    #plt.pause(0.001)
                    #ax.cla()