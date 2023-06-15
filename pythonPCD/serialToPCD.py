import serial
import time

ser = serial.Serial('COM6', 921600, timeout=0)
ser.flushInput()

with open("test3D.pcd","w") as f:
    f.write("VERSION .7\r\n")
    f.write("FIELDS x y z rgb\r\n")
    f.write("SIZE 8 8 8 8\r\n")
    f.write("TYPE I I I I\r\n")
    f.write("COUNT 1 1 1 1\r\n")
    f.write("WIDTH 25\r\n")
    f.write("HEIGHT 25\r\n")
    f.write("VIEWPOINT 0 0 0 1 0 0 0\r\n")
    f.write("POINTS 626\r\n")
    f.write("DATA ascii\r\n")

while True:
    try:
        data = ser.read().decode("utf-8")
        with open("test3D.pcd","a") as f:
            f.write(data)
    except:
        print("exception")
        break