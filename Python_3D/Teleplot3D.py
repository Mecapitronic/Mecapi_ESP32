import time

from serial import Serial
import socket

teleplotAddr = ("teleplot.fr",18949)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

with Serial('COM14', 921600) as ser:
        while True:
            line = ser.readline().decode("utf-8")
            #line = "x:y:z"
            line_splitted = line.replace("\r\n","").split(':')
            sphere1rad = 0.5
            #sphere1num = line_splitted[0]
            sphere1x = line_splitted[0]
            sphere1y = line_splitted[1]
            sphere1z = int(line_splitted[2])/10
            msg = '3D|p'+str(sphere1x)+"_"+str(sphere1y)+',widget0:S:sphere:RA:'+ str(sphere1rad)+':P:'+ str(sphere1x) +':'+ str(sphere1y) +':'+ str(sphere1z) + ':C:black:O:1'
            sock.sendto(msg.encode(), teleplotAddr)
            time.sleep(0.01)

