import open3d as o3d
import numpy as np
import serial

ser = serial.Serial('COM6', 230400, timeout=0)
ser.flushInput()

# create visualizer and window.
vis = o3d.visualization.Visualizer()
vis.create_window(height=480, width=640)

# initialize pointcloud instance.
pcd = o3d.geometry.PointCloud()

# *optionally* add initial points
#draw axes and center
points = np.array([np.array([0, 0, 0]).tolist(), np.array([1, 0, 0]).tolist(),np.array([0, 1, 0]).tolist(),np.array([0, 0, 1]).tolist()])

pcd.points = o3d.utility.Vector3dVector(points)

# include it in the visualizer before non-blocking visualization.
vis.add_geometry(pcd)

opt = vis.get_render_option()
opt.show_coordinate_frame = True
#opt.background_color = np.asarray([0.5, 0.5, 0.5])


# run non-blocking visualization.
# To exit, press 'q' or click the 'x' of the window.
keep_running = True
while keep_running:

        try:
            if ser.in_waiting:
                l = ser.readline()
                line=l.decode("utf-8")
                line_splitted = line.replace("\n","").replace("\r","").split(' ')
                # Options (uncomment each to try them out):
                # 1) extend with ndarrays.
                x = int(line_splitted[0],10) / 1000 + 0.5
                y = int(line_splitted[1],10) / 1000
                z = int(line_splitted[2],10) / 1000 + 0.5
                t1=np.array([x, y, z])
                #t=np.fromstring(line, dtype=int, sep=' ')
                t2=np.array([t1.tolist()])
                pcd.points.extend(t2)

            # 2) extend with Vector3dVector instances.
            # pcd.points.extend(
            #o3d.utility.Vector3dVector(np.random.rand(n_new, 3))

            # 3) other iterables, e.g
            #pcd.points.extend(np.random.rand(n_new, 3).tolist())

            vis.update_geometry(pcd)
        except:
            continue
        keep_running = vis.poll_events()
        vis.update_renderer()

vis.destroy_window()