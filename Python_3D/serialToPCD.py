import serial

ser = serial.Serial('COM6', 921600)
ser.flushInput()
ser.write(("Starting\n").encode())

with open("test3D.pcd","wt") as f:
    f.write("# .PCD v.7 - Point Cloud Data file format\n")
    f.write("VERSION .7\n")
    f.write("FIELDS x y z rgb\n")
    f.write("SIZE 8 8 8 8\n")
    f.write("TYPE F F F F\n")
    f.write("COUNT 1 1 1 1\n")
    f.write("WIDTH 25\n")
    f.write("HEIGHT 25\n")
    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    f.write("POINTS 625\n")
    f.write("DATA ascii\n")
    f.close()


start = "***"
end = "---"
data = ""
str = ""

try:
    with open("test3D.pcd","at") as f:
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