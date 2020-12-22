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
    return points_in_lane


def interlock(points, velocities, ego_speed, timestep_delta, max_decel = 9): # SI units
    ego_stopping_dist = (ego_speed**2)/(2*max_decel) + timestep_delta * ego_speed
    # print('ego stops at: ', ego_stopping_dist)
    cur_min = float('inf')
    for i, point in enumerate(points):
        point_stop_y = min(point[1] + (velocities[i][0]**2)/(2*max_decel), point[1] + (velocities[i][0] * ego_speed / max_decel))
        # print(point_stop_y)
        
        if point[1] < cur_min:
            cur_min = point[1]
        if point_stop_y < ego_stopping_dist:
            if point[1] + (velocities[i][0]**2)/(2*max_decel) <= point[1] + (velocities[i][0] * ego_speed / max_decel):
                print('case forward', velocities[i])
            else:
                print('case backward', velocities[i])
            return False, cur_min
    return True, cur_min

# def is_safe(point, stopping_distance, vel, max_decel):
#     return (point[1] ) or (point[1] > stopping_distance)
if __name__ == "__main__":
    filename = 'LiDAR/lidar002310.ply'
    visualize(filename)

    new_filename = 'LiDAR/filtered-lidar.ply'
    points = create_filtered_point_cloud(filename, new_filename)
    visualize(new_filename)

    print(interlock(points, 0, -15))