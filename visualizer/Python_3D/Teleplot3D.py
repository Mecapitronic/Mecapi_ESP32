import time

from serial import Serial
import socket

teleplotAddr = ("teleplot.fr",39627)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

with Serial('COM16', 921600) as ser:
        start = 0
        while True:
            try:
                line = ser.readline().decode("utf-8").replace("\r\n","")
                #line = "x y z rgb"
                if line == '***':
                    start=1
                if line != '***' and line != '---' and start == 1:
                    line_splitted = line.split(' ')
                    if len(line_splitted) > 3:
                        sphere1rad = 1
                        #sphere1num = line_splitted[0]
                        sphere1x = int(line_splitted[0])/10
                        sphere1y = int(line_splitted[1])/10
                        sphere1z = int(line_splitted[2])/10
                        msg = '3D|p'+str(sphere1x)+"_"+str(sphere1y)+',widget0:S:sphere:RA:'+ str(sphere1rad)+':P:'+ str(sphere1x) +':'+ str(sphere1y) +':'+ str(sphere1z) + ':C:black:O:1'
                        sock.sendto(msg.encode(), teleplotAddr)
                        time.sleep(0.01)
            except:
                continue
