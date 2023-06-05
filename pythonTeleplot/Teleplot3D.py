import time

from serial import Serial
import socket

teleplotAddr = ("teleplot.fr",18949)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

with Serial('COM6', 921600) as ser:
        while True:
            line = ser.readline().decode("utf-8")
            #line = "0:3:3:3"
            line_splitted = line.split(':')
            sphere1rad = 0.1
            sphere1num = line_splitted[0]
            sphere1x = line_splitted[1]
            sphere1y = line_splitted[2]
            sphere1z = line_splitted[3]
            msg = '3D|sphere'+sphere1num+',widget0:S:sphere:RA:'+ str(sphere1rad)+':P:'+ str(sphere1x) +':'+ str(sphere1y) +':'+ str(sphere1z) + ':C:black:O:1'
            sock.sendto(msg.encode(), teleplotAddr)
            time.sleep(0.01)

