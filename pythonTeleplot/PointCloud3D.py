import matplotlib.pyplot as plt
import numpy as np

from serial import Serial
import socket

teleplotAddr = ("teleplot.fr",47269)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

plt.style.use('_mpl-gallery')

# Plot
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

ax.set(xticklabels=[],
       yticklabels=[],
       zticklabels=[])

plt.show()

with Serial('COM6', 921600) as ser:
    while True:
        line = ser.readline().decode("utf-8")
        x = line.sub
        ax.scatter(xs, ys, zs)
        
        # print(line)

        # sock.sendto(line.encode(), teleplotAddr)

# Make data
np.random.seed(19680801)
n = 100
rng = np.random.default_rng()
xs = rng.uniform(23, 32, n)
ys = rng.uniform(0, 100, n)
zs = rng.uniform(-50, -25, n)

ax.scatter(xs, ys, zs)

ax.set(xticklabels=[],
       yticklabels=[],
       zticklabels=[])

