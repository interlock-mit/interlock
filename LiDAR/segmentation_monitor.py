# from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d
from collections import defaultdict
import math


MAX_DECEL = 15
MIN_DIST = 10
DEFAULT_TIMESTEP = .2


def visualize(filename):
    cloud = o3d.io.read_point_cloud(filename)
    o3d.visualization.draw_geometries([cloud])


def dist(point_a, point_b=(0, 0, 0)):
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
                msg = f"The traversal order is not valid, because we have not yet seen the point {point_b}"
                print(msg)
                return False
            else:
                seen.add(point_a)
                d = dist(points[obj][point_a], points[obj][point_b])
                if d > pos_threshold:
                    msg = f"Points {point_a} and {point_b} are {d} apart, which is too far away"
                    print(message)
                    return False, obj
    return True, None


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
            if abs(v_x - avg_v_x) > vel_threshold[0] or abs(v_y - avg_v_y) > vel_threshold[1] or abs(v_z - avg_v_z) > vel_threshold[2]:
                msg = f"The velocity {point_vel} is too different from the average velocity, which is {(avg_v_x, avg_v_y, avg_v_z)}"
                print(msg)
                return False, obj
    return True, None


def check_density_spread_all_objs(image_pos, image, image_scale_factor):
    density = [[set() for i in range(image.shape[1])] for j in range(image.shape[0])]
    print(len(image_pos))
    for obj in image_pos:
        for point in image_pos[obj]:
            density[point[0]//image_scale_factor][point[1]//image_scale_factor].add(obj)
    
    bad_objs = set()
    for i in range(len(density)):
        for j in range(len(density[i])):
            if image[i][j] not in density[i][j]:
                bad_objs.add(image[i][j])
                return False, image[i][j]
    # if bad_objs:
    #     return False, list(bad_objs)
    # # print(density)
    return True, None


def check_ground_pts_on_ground(points, ground_id, height_threshold=0.5):
    ground_points = points[ground_id]
    for pt in ground_points:
        z = pt[2]
        if z > height_threshold:
            msg = f"The point {pt} is not on the ground"
            print(msg)
            return False
    return True


def is_safe(ego_vel, other_pos, other_vel, timestep=DEFAULT_TIMESTEP, min_dist=MIN_DIST, max_decel=MAX_DECEL):
    cur_speed = dist(ego_vel)
    stopping_time = cur_speed/max_decel

    def decel(vel, rate):
        x, y, z = vel
        helper = lambda x: max(x - rate, 0) if x > 0 else min(x + rate, 0)
        return (helper(x), helper(y), helper(z))

    def helper(ego_accel, other_accel):
        cur_time = 0
        cur_pos = (0, 0, 0)
        cur_vel = ego_vel
        other_cur_pos, other_cur_vel = other_pos, other_vel
        while cur_time < stopping_time:
            if dist(cur_pos, other_cur_pos) < min_dist:
                return False
            cur_vel = decel(cur_vel, ego_accel * timestep)
            other_cur_vel = decel(other_cur_vel, other_accel * timestep)
            cur_pos = add(mult(cur_vel, timestep), cur_pos)
            other_cur_pos = add(mult(other_cur_vel, timestep), other_cur_pos)
            cur_time += timestep
        return True

    return helper(max_decel, 0) and helper(max_decel, max_decel)


def check_predicates(points, vels, ego_vel):
    """
    For now assuming a straight path but eventually we can incorporate curved or more complex paths
    """
    point_in_path = lambda x: (-2 < x[0] < 2) and x[1] > 0 and x[2] > -1.2
    obj_in_path = lambda x: any(point_in_path(p) for p in points[x])
    object_ids_in_path = list(filter(obj_in_path, points))

    for obj_id in object_ids_in_path:
        closest_pt = min(points[obj_id], key=lambda p: dist(p))
        obj_vel = avg_velocity(vels[obj_id])
        if not is_safe(ego_vel, closest_pt, obj_vel):
            msg = f"The object with ID {obj_id} is not safe"
            print(msg)
            return False

    return True


def interlock(obj_info, ground_id, traversal_orders, image, vel_threshold, image_scale_factor, ego_vel):
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

    ego_vel: tuple (v_x, v_y, v_z) of the ego vehicle's current velocity
    """
    from traversal import cell_size
    pos_threshold = cell_size * 2 * (3 ** .5)
    points, vels, image_pos = {}, {}, {}
    print(len(obj_info))
    for obj in obj_info:
        if obj_info[obj]:
            points[obj], vels[obj], image_pos[obj] = zip(*obj_info[obj])

    def display_point_cloud(wrong_obj):
        import open3d
        pcd = open3d.geometry.PointCloud()
        # for wrong_obj in wrong_objs:
        pcd.points = open3d.utility.Vector3dVector(np.array(points[wrong_obj]))
        open3d.visualization.draw_geometries([pcd])

        pcd = open3d.geometry.PointCloud()
        pcd_points = None
        for obj in points:
            np_points = np.array(points[obj])
            pcd_points = np.concatenate((pcd_points, np_points), axis=0) if pcd_points is not None else np_points
        pcd.points = open3d.utility.Vector3dVector(pcd_points)
        open3d.visualization.draw_geometries([pcd])

        # for wrong_obj in wrong_objs:
        cur_img = np.zeros((600, 800, 3))
        for obj in points:
            if obj == wrong_obj:
                color = [1, 0, 0]
            else:
                color = [obj/40, obj/40, obj/40]
            for x, y in image_pos[obj]:
                cur_img[x][y] = color
        import matplotlib.pyplot as plt
        plt.imshow(cur_img)
        plt.show()

    result = check_traversal_order(points, traversal_orders, pos_threshold)
    if not result[0]:
        print("Traversal order check failed")
        display_point_cloud(result[1])
        return False

    result = check_same_velocity(vels, vel_threshold)
    if not result[0]:
        print("Same velocity check failed")
        display_point_cloud(result[1])
        return False

    result = check_density_spread_all_objs(image_pos, image, image_scale_factor)
    if not result[0]:
        print("Density/spread check failed", result[1])
        display_point_cloud(result[1])
        return False

    if not check_ground_pts_on_ground(points, ground_id):
        print("Ground check failed")
        return False

    if not check_predicates(points, vels, ego_vel):
        print("Predicates check failed")
        return False

    return True


# if __name__ == "__main__":
#     filename = 'LiDAR/lidar002310.ply'
#     visualize(filename)

#     new_filename = 'LiDAR/filtered-lidar.ply'
#     points = create_filtered_point_cloud(filename, new_filename)
#     visualize(new_filename)

#     print(interlock(points, 0, -15))
