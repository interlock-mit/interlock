from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d

def visualize(filename):
    cloud = o3d.io.read_point_cloud(filename) # Read the point cloud
    o3d.visualization.draw_geometries([cloud]) # Visualize the point cloud

def in_car_lane(point):
    x = float(point[0])
    return x > -6 and x < 6

def create_filtered_point_cloud(filename, new_filename):
    with open(filename, 'r') as f:
        points_in_lane = []
        while True:
            line = f.readline()
            if not line:
                break
            line = line.strip("\n").split(" ")
            if len(line) > 4 and in_car_lane(line):
                line = [float(num) for num in line]
                points_in_lane.append(line[:3])
    points_in_lane = np.array(points_in_lane)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_in_lane)
    o3d.io.write_point_cloud(new_filename, pcd)

filename = 'LiDAR/lidar002310.ply'
visualize(filename)

new_filename = 'LiDAR/filtered-lidar.ply'
create_filtered_point_cloud(filename, new_filename)
visualize(new_filename)