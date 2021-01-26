# from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d
from collections import defaultdict
import math


def visualize(filename):
    cloud = o3d.io.read_point_cloud(filename)
    o3d.visualization.draw_geometries([cloud])


def dist(point_a, point_b):
    x_a, y_a, z_a = point_a
    x_b, y_b, z_b = point_b
    return ((x_a - x_b)**2 + (y_a - y_b)**2 + (z_a - z_b)**2)**0.5


def check_traversal_order(points, traversal_orders, pos_threshold):
    for obj, traversal_order in traversal_orders.items():
        if traversal_order is None:
            continue
        seen = {traversal_order[0][0]}
        for (point_a, point_b) in traversal_order[1:]:
            if point_b not in seen:
                print("The traversal order is not valid, because we have not yet seen the point{}".format(
                    point_b))
                return False
            else:
                seen.add(point_a)
                d = dist(points[obj][point_a], points[obj][point_b])
                if d > pos_threshold:
                    print("Points {} and {} are {} apart, which is too far away".format(
                        point_a, point_b, d))
                    return False
    return True


def avg_velocity(obj):
    sum_v_x, sum_v_y, sum_v_z = 0, 0, 0
    for point_vel in obj:
        v_x, v_y, v_z = point_vel
        sum_v_x += v_x
        sum_v_y += v_y
        sum_v_z += v_z
    return sum_v_x/len(obj), sum_v_y/len(obj), sum_v_z/len(obj)


def check_same_velocity(obj_velocities, vel_threshold):
    for obj, vels in obj_velocities.items():
        if vels is None:
            continue
        # print(vels)
        avg_v_x, avg_v_y, avg_v_z = avg_velocity(vels)
        for point_vel in vels:
            v_x, v_y, v_z = point_vel
            msg = f"The velocity {point_vel} is too different from the average velocity, which is {(avg_v_x, avg_v_y, avg_v_z)}"
            if abs(v_x - avg_v_x) > vel_threshold[0]:
                print(msg)
                return False
            if abs(v_y - avg_v_y) > vel_threshold[1]:
                print(msg)
                return False
            if abs(v_z - avg_v_z) > vel_threshold[2]:
                print(msg)
                return False
    return True


def bounding_box(rgb_pts):
    min_x = min([pt[0] for pt in rgb_pts])
    max_x = max([pt[0] for pt in rgb_pts])
    min_y = min([pt[1] for pt in rgb_pts])
    max_y = max([pt[1] for pt in rgb_pts])
    return min_x, max_x, min_y, max_y


def cell_to_rgb(rgb_pts, bb_size):
    min_x, max_x, min_y, max_y = bounding_box(rgb_pts)
    num_cells_x = math.floor((max_x - min_x)/bb_size) + 1
    num_cells_y = math.floor((max_y - min_y)/bb_size) + 1
    index_map = {(i, j): set() for i in range(num_cells_x)
                 for j in range(num_cells_y)}
    print(index_map)
    for pt in rgb_pts:
        x, y = pt
        i = int((x - min_x)/bb_size)
        j = int((y - min_y)/bb_size)
        index_map[(i, j)].add(pt)
    cell_map = {}
    for key, val in cell_map.items():
        i, j = key
        x = min_x + i*bb_size
        y = min_y + j*bb_size
        cell_map[(x, x+bb_size, y, y+bb_size)] = val
    return cell_map


def cell_to_lidar(rgb_pts, lidar_pts, bb_size):
    min_x, max_x, min_y, max_y = bounding_box(rgb_pts)
    num_cells_x = math.ceil((max_x - min_x)/bb_size)
    num_cells_y = math.ceil((max_y - min_y)/bb_size)
    index_map = {(i, j): set() for i in range(num_cells_x)
                 for j in range(num_cells_y)}
    for pt in lidar_pts:
        x, y = pt
        i = int((x - min_x)/bb_size)
        j = int((y - min_x)/bb_size)
        index_map[(i, j)].add(pt)
    cell_map = {}
    for key, val in cell_map.items():
        i, j = key
        x = min_x + i*bb_size
        y = min_y + j*bb_size
        cell_map[(x, x+bb_size, y, y+bb_size)] = val
    return cell_map


def check_density_spread(rgb_pts, lidar_pts, bb_size):
    rgb_cells = cell_to_rgb(rgb_pts, bb_size)
    cells_with_rgb = filter(lambda x: len(rgb_cells[x]) > 0, rgb_cells)

    lidar_cells = cell_to_lidar(rgb_pts, lidar_pts, bb_size)
    cell_to_lidar_map = filter(lambda x: len(lidar_cells[x]) > 0, lidar_cells)

    for cell in cells_with_rgb:
        if cell not in cell_to_lidar_map:
            x_min, x_max, y_min, y_max = cell
            print("The bounding box from x={} to {} and y={} to {} contains an RGB point but not a lidar point".format(
                x_min, x_max, y_min, y_max))
            return False
    return True


def check_density_spread_all_objs(image_pos, image, image_scale_factor):
    density = [[set() for i in range(image.shape[1])] for j in range(image.shape[0])]
    print(len(image_pos))
    for obj in image_pos:
        for point in image_pos[obj]:
            density[point[0]//image_scale_factor][point[1]//image_scale_factor].add(obj)
    
    for i in range(len(density)):
        for j in range(len(density[i])):
            if image[i][j] not in density[i][j]:
                return False
    # print(density)
    return True


def check_ground_pts_on_ground(points, ground_id, height_threshold = 0.5):
    ground_points = points[ground_id]
    for pt in ground_points:
        z = pt[2]
        if z > height_threshold:
            print("The point {} is not on the ground".format(pt))
            return False
    return True


def interlock(obj_info, ground_id, traversal_orders, image, vel_threshold, image_scale_factor):
    """
    obj_info: dictionary where each key/value pair is an object that maps object ID
            to a list of points corresponding to that object.
            each point is a list of the form [loc, vel, image_pos]
    loc: numpy array of the form (x, y, z)
    vel: numpy array of the form (v_x, v_y, vg_z)
    image_pos: list of the form (x, y)

    ground_id: object ID corresponding to the ground object

    traversal_orders: list of traversal_orders
    traversal_order: list of tuples, each of the form (point_a, point_b)
            where point_a is the next point in the traversal order
            and point_b is the point close to point_a
    point: tuple (x, y, z) is the position of the point

    vel_threshold: maximum difference in velocity a point can have
            from the average velocity of that object for it to be "the same"

    image_scale_factor: the scale at which the resolution of the image is decreased
    """
    from traversal import cell_size
    pos_threshold = cell_size * 2 * (3 ** .5)
    points, vels, image_pos = {}, {}, {}
    print(len(obj_info))
    for obj in obj_info:
        if obj_info[obj]:
            points[obj], vels[obj], image_pos[obj] = zip(*obj_info[obj])

    if not check_traversal_order(points, traversal_orders, pos_threshold):
        print("Traversal order check failed")
        return False

    if not check_same_velocity(vels, vel_threshold):
        print("Same velocity check failed")
        return False

    if not check_density_spread_all_objs(image_pos, image, image_scale_factor):
        print("Density/spread check failed")
        return False

    if not check_ground_pts_on_ground(points, ground_id):
        print("Ground check failed")
        return False

    return True


# if __name__ == "__main__":
#     filename = 'LiDAR/lidar002310.ply'
#     visualize(filename)

#     new_filename = 'LiDAR/filtered-lidar.ply'
#     points = create_filtered_point_cloud(filename, new_filename)
#     visualize(new_filename)

#     print(interlock(points, 0, -15))
