import open3d as o3d
import numpy as np
import serial

#TODO : see examples: https://snyk.io/advisor/python/open3d/functions/open3d.geometry.PointCloud

ser = serial.Serial('COM16', 921600, timeout=0)
ser.flushInput()

# create visualizer and window.
vis = o3d.visualization.Visualizer()
vis.create_window(height=800, width=800)

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
opt.background_color = np.asarray([0.5, 0.5, 0.5])

# Creating a mesh of the XYZ axes Cartesian coordinates frame.
# This mesh will show the directions in which the X, Y & Z-axes point,
# and can be overlaid on the 3D mesh to visualize its orientation in
# the Euclidean space.
# X-axis : Red arrow
# Y-axis : Green arrow
# Z-axis : Blue arrow


# run non-blocking visualization.
# To exit, press 'q' or click the 'x' of the window.
keep_running = True
while keep_running:

        try:
            if ser.in_waiting:
                #TODO : enregistrer et afficher tous les points d'une trame d'un coup => au bout de qq secondes ça ne répond plus...
                for i in range(2500):
                    l = ser.readline()
                    line=l.decode("utf-8")
                    line_splitted = line.replace("\n","").replace("\r","").split(' ')
                    # Options (uncomment each to try them out):
                    # 1) extend with ndarrays.
                    x = int(line_splitted[0],10) / 1000
                    y = int(line_splitted[1],10) / 1000
                    z = int(line_splitted[2],10) / 1000

                    t1=np.asarray([x, z, y])
                    #t=np.fromstring(line, dtype=int, sep=' ')
                    t2=np.array([t1.tolist()])
                    pcd.points.extend(o3d.utility.Vector3dVector(t2))

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
