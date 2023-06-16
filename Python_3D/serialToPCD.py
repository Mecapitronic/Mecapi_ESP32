import serial
import time

ser = serial.Serial('COM6', 230400, timeout=0)
ser.flushInput()

with open("test3D.pcd","w") as f:
    f.write("VERSION .7\n")
    f.write("FIELDS x y z rgb\n")
    f.write("SIZE 8 8 8 8\n")
    f.write("TYPE I I I I\n")
    f.write("COUNT 1 1 1 1\n")
    f.write("WIDTH 25\n")
    f.write("HEIGHT 25\n")
    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    f.write("POINTS 626\n")
    f.write("DATA ascii\n")

while True:
    try:
        data = ser.readline().decode("utf-8")
        with open("test3D.pcd","a") as f: 
            f.write(data)
    except:
        print("exception")
        break