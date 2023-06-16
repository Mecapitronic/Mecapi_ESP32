import open3d as o3d
import numpy as np
import time
import serial

ser = serial.Serial('COM6', 230400, timeout=0)
ser.flushInput()

# create visualizer and window.
vis = o3d.visualization.Visualizer()
vis.create_window(height=480, width=640)

# initialize pointcloud instance.
pcd = o3d.geometry.PointCloud()
# *optionally* add initial points
points = np.random.rand(1, 3)
pcd.points = o3d.utility.Vector3dVector(points)

# include it in the visualizer before non-blocking visualization.
vis.add_geometry(pcd)

# to add new points each dt secs.
dt = 0.01
# number of points that will be added
n_new = 10

previous_t = time.time()

# run non-blocking visualization. 
# To exit, press 'q' or click the 'x' of the window.
keep_running = True
while keep_running:
    
    #if time.time() - previous_t > dt:
        try:
            l = ser.readline()
            line=l.decode("utf-8")
            line_splitted = line.replace("\n","").replace("\r","").split(' ')
            # Options (uncomment each to try them out):
            # 1) extend with ndarrays.
            t2=np.array([int(line_splitted[0],10)/1000, int(line_splitted[1],10)/1000, int(line_splitted[2],10)/1000]) 
            t=np.fromstring(line, dtype=int, sep=' ')
            t3=np.array([t2.tolist()])
            pcd.points.extend(t3)
            
            # 2) extend with Vector3dVector instances.
            # pcd.points.extend(
            #o3d.utility.Vector3dVector(np.random.rand(n_new, 3))
            
            # 3) other iterables, e.g
            #pcd.points.extend(np.random.rand(n_new, 3).tolist())
            
            vis.update_geometry(pcd)
            previous_t = time.time()
        except:
            continue
        keep_running = vis.poll_events()
        vis.update_renderer()

vis.destroy_window()