import matplotlib.pyplot as plt
import numpy as np

from serial import Serial
import socket

teleplotAddr = ("teleplot.fr",47269)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

with Serial('COM6', 921600) as ser:
    while True:
        line = ser.readline().decode("utf-8")
        # print(line)

        sock.sendto(line.encode(), teleplotAddr)



----------------------------


import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation
import pandas as pd


a = np.random.rand(2000, 3)*10
t = np.array([np.ones(100)*i for i in range(20)]).flatten()
df = pd.DataFrame({"time": t ,"x" : a[:,0], "y" : a[:,1], "z" : a[:,2]})

def update_graph(num):
    data=df[df['time']==num]
    graph.set_data (data.x, data.y)
    graph.set_3d_properties(data.z)
    title.set_text('3D Test, time={}'.format(num))
    return title, graph,


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
title = ax.set_title('3D Test')

data=df[df['time']==0]
graph, = ax.plot(data.x, data.y, data.z, linestyle="", marker="o")

ani = matplotlib.animation.FuncAnimation(fig, update_graph, 19,
                               interval=40, blit=True)

plt.show()