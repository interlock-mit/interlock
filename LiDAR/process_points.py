from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d

def visualize(filename):
    cloud = o3d.io.read_point_cloud(filename) # Read the point cloud
    o3d.visualization.draw_geometries([cloud]) # Visualize the point cloud

def in_car_lane(point):
    x = float(point[0])
    return x > -0.5 and x < 0.5

filename = 'LiDAR/lidar002310.ply'
with open(filename, 'r') as f:
    points_in_lane = []
    while True:
        line = f.readline()
        if not line:
            break
        line = line.split(" ")
        if len(line) > 4 and in_car_lane(line):
            line_text = " ".join(line)
            points_in_lane.append(line_text)
    points_in_lane = np.array(points_in_lane)

# TODO: write numpy array of filtered points to new .ply file
new_filename = 'LiDAR/filtered-lidar.ply'
el = PlyElement.describe(points_in_lane, 'name')
PlyData([el]).write(new_filename)