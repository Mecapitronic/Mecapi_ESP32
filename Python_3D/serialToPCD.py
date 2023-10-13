import serial
import subprocess, os, platform

ser = serial.Serial('COM16', 921600)
ser.flushInput()
ser.write(("Starting\n").encode())

filename = "C:\\Users\\Public\\test3D.pcd"

with open(filename,"wt") as f:
    f.write("# .PCD v.7 - Point Cloud Data file format\n")
    f.write("VERSION .7\n")
    f.write("FIELDS x y z rgb\n")
    f.write("SIZE 4 4 4 4\n")
    f.write("TYPE I I I I\n")
    f.write("COUNT 1 1 1 1\n")
    f.write("WIDTH 50\n")
    f.write("HEIGHT 50\n")
    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    f.write("POINTS 2500\n")
    f.write("DATA ascii\n")
    f.close()

#FIXME: en copiant les données dans un autre fichier PCD du dossier Point_Cloud ça fonctionne !

start = "***"
end = "---"
data = ""
str = ""

try:
    with open(filename,"at") as f:
        #data = ser.readline().decode("utf-8")
        while start not in str:
            data = ser.read_until(b'\n').decode("utf-8")
            str = data.replace('\n', '').replace('\r', '')

        #data = ser.readline().decode("utf-8")
        while end not in str:
            data = ser.read_until(b'\n').decode("utf-8")
            str = data.replace('\n', '').replace('\r', '')
            if str != end:
                f.write(str+'\n')
except:
    print("exception")

ser.close()
f.close()

os.startfile(filename, 'open')
