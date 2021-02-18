# from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d
from collections import defaultdict
import math


MAX_DECEL = 15
MIN_DIST = 10
DEFAULT_TIMESTEP = .2


def add(vec1, vec2):
    return (vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2])
def mult(vec, factor):
    return (vec[0] * factor, vec[1] * factor, vec[2] * factor)
def dist(vec1, vec2):
    return ((vec1[0]-vec2[0])**2 + (vec1[1]-vec2[1])**2 + (vec1[2]-vec2[2])**2)**.5
def loc(pos, vel, t):
    return (pos[0]+t*vel[0], pos[1]+t*vel[1], pos[2]+t*vel[2])

def visualize(filename):
    cloud = o3d.io.read_point_cloud(filename)
    o3d.visualization.draw_geometries([cloud])


def dist(point_a, point_b=(0, 0, 0)):
    x_a, y_a, z_a = point_a
    x_b, y_b, z_b = point_b
    return ((x_a - x_b)**2 + (y_a - y_b)**2 + (z_a - z_b)**2)**0.5


def get_points(obj_id, grid):
    cells = [cell for row in grid for cell in row if cell[0] == obj_id]
    points = [pt for cell in cells for pt in cell[1]]
    return points


def check_traversal_order(test, grid, traversal_orders):
    from LiDAR.v2_traversal import cell_size
    pos_threshold = cell_size * 2 * (3 ** .5)
    for (obj_id, traversal_order) in traversal_orders:
        if traversal_order is None or len(traversal_order) == 0:
            continue
        seen = {traversal_order[0][0]}
        for (point_a, point_b) in traversal_order[1:]:
            if point_b not in seen:
                test["success"] = False
                test["error_msg"] = f"The traversal order is not valid, because we have not yet seen the point {point_b}"
                return
            else:
                seen.add(point_a)
                i_a, j_a, k_a = point_a
                i_b, j_b, k_b = point_b
                d = dist(grid[j_a][i_a][1][k_a][0], grid[j_b][i_b][1][k_b][0])
                if d > pos_threshold:
                    test["success"] = False
                    test["error_msg"] = f"Points {point_a} and {point_b} are {d} apart, which is too far away"
                    # TODO: should add point_a and point_b to bad_points, but we need to add 2d x,y coords to do that


def avg_velocity(obj):
    sum_v_x, sum_v_y, sum_v_z = 0, 0, 0
    for point_vel in obj:
        v_x, v_y, v_z = point_vel
        sum_v_x += v_x
        sum_v_y += v_y
        sum_v_z += v_z
    return sum_v_x/len(obj), sum_v_y/len(obj), sum_v_z/len(obj)


def check_same_velocity(test, obj_velocities, vel_threshold, image_pos):
    for obj, vels in obj_velocities.items():
        if vels is None:
            continue
        avg_v_x, avg_v_y, avg_v_z = avg_velocity(vels)
        for point_vel in vels:
            v_x, v_y, v_z = point_vel
            if abs(v_x - avg_v_x) > vel_threshold[0] or abs(v_y - avg_v_y) > vel_threshold[1] or abs(v_z - avg_v_z) > vel_threshold[2]:
                test["success"] = False
                test["error_msg"] = f"The velocity {point_vel} is too different from the average velocity, which is {(avg_v_x, avg_v_y, avg_v_z)}"
                test["bad_points"].extend(image_pos[obj])


def check_density_spread_all_objs(test, image_pos, image, image_scale_factor):
    from LiDAR.lidar_density_location import LIDAR_GRID

    density = [[set() for i in range(image.shape[1])] for j in range(image.shape[0])]
    for obj in image_pos:
        for point in image_pos[obj]:
            density[point[0]//image_scale_factor][point[1]//image_scale_factor].add(obj)
    
    bad_objs = set()
    for i in range(len(LIDAR_GRID)):
        for j in range(len(LIDAR_GRID[i])):
            if LIDAR_GRID[i][j] == 1 and image[i][j] not in density[i][j]:
                test["success"] = False
                # TODO: error msg
                # TODO: should also add points to bad_points
                bad_objs.add(image[i][j])
    # if bad_objs:
    #     return False, list(bad_objs)
    # # print(density)
    return True, []


def check_ground_pts_on_ground(test, grid, ground_ids, height_threshold=-0.5):
    bad_points = []
    for ground_id in ground_ids:
        ground_points = get_points(ground_id, grid)
        for pt in ground_points:
            z = pt[0][2]
            if z > height_threshold:
                test["success"] = False
                test["error_msg"] = f"The point {pt} is not on the ground"
                test["bad_points"].append(pt[2])


def is_safe(ego_vel, other_pos, other_vel, timestep=DEFAULT_TIMESTEP, min_dist=MIN_DIST, max_decel=MAX_DECEL):
    cur_speed = dist(ego_vel)
    stopping_time = cur_speed/max_decel

    def decel(vel, rate):
        x, y, z = vel
        helper = lambda x: max(x - rate, 0) if x > 0 else min(x + rate, 0)
        return (helper(x), helper(y), helper(z))

    def helper(ego_accel, other_accel):
        x_e, y_e, z_e = (0, 0, 0) # ego position
        x_o, y_o, z_o = other_pos # other position
        v_x_e, v_y_e, v_z_e = ego_vel # ego velocity
        v_x_o, v_y_o, v_z_o = other_vel # other velocity
        a_x_e, a_y_e, a_z_e = ego_accel # ego acceleration
        a_x_o, a_y_o, a_z_o = other_accel # other acceleration
        
        # polynomial equation of the form at^2 + bt + c = 0, minimized when t = -b/2a
        t_closest_x = abs((v_x_e - v_x_o)/(a_x_e - a_x_o))
        if t_closest_x > stopping_time: # enough time to stop
            return True
        else:
            closest_x_dist = 0.5*(a_x_e - a_x_o)*t_closest_x**2 + t_closest_x*(v_x_e - v_x_o) + x_e - x_o
            if closest_x_dist > min_dist: # closest x distance is large enough
                return True

        t_closest_y = abs((v_y_e - v_y_o)/(a_y_e - a_y_o))
        if t_closest_y > stopping_time: # enough time to stop
            return True
        else:
            closest_y_dist = 0.5*(a_y_e - a_y_o)*t_closest_y**2 + t_closest_y*(v_y_e - v_y_o) + y_e - y_o
            if closest_y_dist > min_dist: # closest y distance is large enough
                return True

        t_closest_z = abs((v_z_e - v_z_o)/(a_z_e - a_z_o))
        if t_closest_z > stopping_time: # enough time to stop
            return True
        else:
            closest_z_dist = 0.5*(a_z_e - a_z_o)*t_closest_z**2 + t_closest_z*(v_z_e - v_z_o) + z_e - z_o
            if closest_z_dist > min_dist: # closest y distance is large enough
                return True
    
        return False

    return helper(3*(max_decel,), 3*(0,)) and helper(3*(max_decel,), 3*(max_decel,))

def check_predicates(test, grid, vels, ego_vel, ground_ids, image_pos):
    """
    For now assuming a straight path but eventually we can incorporate curved or more complex paths
    """
    point_in_path = lambda x: (-2 < x[0][0] < 2) and x[0][1] > 0 and x[0][2] > -1.2
    obj_in_path = lambda x: any(point_in_path(p) for p in get_points(x, grid))
    obj_ids = {cell[0] for row in grid for cell in row}
    object_ids_in_path = list(filter(obj_in_path, obj_ids))
    for obj_id in object_ids_in_path:
        if obj_id in ground_ids:
            continue
        closest_pt = min(get_points(obj_id, grid), key=lambda p: dist(p[0]))[0]
        obj_vel = avg_velocity(vels[obj_id])
        if not is_safe(ego_vel, closest_pt, obj_vel):
            test["success"] = False
            test["error_msg"] = f"The object with ID {obj_id} is not safe"
            test["bad_points"].extend(image_pos[obj_id])
            return


def interlock(grid, ground_ids, traversal_orders, image, vel_threshold, image_scale_factor, ego_vel):
    """
    grid: 2d array of cells where each value is a tuple of the form (obj_id, lidar_points)
        where obj_id is the unique id corresponding to the object in that cell
        and lidar_points is a list of 3d lidar points that transform onto that 2d cell
        where each lidar point is a list of the form [loc, vel, image_pos]
    loc: numpy array of the form (x, y, z)
    vel: numpy array of the form (v_x, v_y, vg_z)
    image_pos: list of the form (x, y)

    ground_id: object ID corresponding to the ground object

    traversal_orders: list of tuples, each of the form (obj_id, traversal_order)
    traversal_order: list of tuples, each of the form (point_a, point_b)
            where point_a is the next point in the traversal order
            and point_b is the point close to point_a
    point: tuple (i, j, k) are the indices such that grid[j][i][1][k] gives the point

    vel_threshold: maximum difference in velocity a point can have
            from the average velocity of that object for it to be "the same"

    image_scale_factor: the scale at which the resolution of the image is decreased

    ego_vel: tuple (v_x, v_y, v_z) of the ego vehicle's current velocity
    """
    points, vels, image_pos = defaultdict(list), defaultdict(list), defaultdict(list)
    for j in range(len(grid)):
        for i in range(len(grid[j])):
            obj_id, lidar_pts = grid[j][i]
            for pt in lidar_pts:
                points[obj_id].append(pt[0])
                vels[obj_id].append(pt[1])
                image_pos[obj_id].append(pt[2])


    def display_point_cloud(wrong_obj):
        import open3d
        pcd = open3d.geometry.PointCloud()
        # for wrong_obj in wrong_objs:
        wrong_points = [point[0] for point in get_points(wrong_obj, grid)]
        pcd.points = open3d.utility.Vector3dVector(np.array(wrong_points))
        open3d.visualization.draw_geometries([pcd])

        obj_ids = {cell[0] for row in grid for cell in row}

        pcd = open3d.geometry.PointCloud()
        pcd_points = None
        for obj in obj_ids:
            np_points = np.array([point[0] for point in get_points(obj, grid)])
            pcd_points = np.concatenate((pcd_points, np_points), axis=0) if pcd_points is not None else np_points
        pcd.points = open3d.utility.Vector3dVector(pcd_points)
        open3d.visualization.draw_geometries([pcd])

        # for wrong_obj in wrong_objs:
        cur_img = np.zeros((600, 800, 3))
        for obj in obj_ids:
            if obj == wrong_obj:
                color = [1, 0, 0]
            else:
                color = [obj/40, obj/40, obj/40]
            for x, y in image_pos[obj]:
                cur_img[x][y] = color
        import matplotlib.pyplot as plt
        plt.imshow(cur_img)
        plt.show()

    test_suite = {
        "Spatial Check": {
            "function": lambda test: check_traversal_order(test, grid, traversal_orders),
            "success": True,
            "error_msg": "",
            "bad_points": []
        },
        "Velocity Check": {
            "function": lambda test: check_same_velocity(test, vels, vel_threshold, image_pos),
            "success": True,
            "error_msg": "",
            "bad_points": []
        },
        "Density Check": {
            "function": lambda test: check_density_spread_all_objs(test, image_pos, image, image_scale_factor),
            "success": True,
            "error_msg": "",
            "bad_points": []
        },
        "Ground Check": {
            "function": lambda test: check_ground_pts_on_ground(test, grid, ground_ids),
            "success": True,
            "error_msg": "",
            "bad_points": []
        },
        "Collision Check": {
            "function": lambda test: check_predicates(test, grid, vels, ego_vel, ground_ids, image_pos),
            "success": True,
            "error_msg": "",
            "bad_points": []
        },
    }
    for check in test_suite:
        test = test_suite[check]
        test["function"](test)
    return test_suite

    """result = check_traversal_order(grid, traversal_orders)
    success = True
    bad_points = []
    if not result[0]:
        print("Traversal order check failed")
        success = False
        bad_points.extend(result[1])

    result = check_same_velocity(vels, vel_threshold)
    if not result[0]:
        print("Same velocity check failed")
        success = False
        bad_points.extend(result[1])

    result = check_density_spread_all_objs(image_pos, image, image_scale_factor)
    if not result[0]:
        print("Density/spread check failed", result[1])
        success = False
        bad_points.extend(result[1])

    if not check_ground_pts_on_ground(grid, ground_id):
        print("Ground check failed")
        success = False
        bad_points.extend(result[1])

    if not check_predicates(grid, vels, ego_vel):
        print("Predicates check failed")
        success = False
    
    print("Success: ", success)
    return success, bad_points"""


# if __name__ == "__main__":
#     filename = 'LiDAR/lidar002310.ply'
#     visualize(filename)

#     new_filename = 'LiDAR/filtered-lidar.ply'
#     points = create_filtered_point_cloud(filename, new_filename)
#     visualize(new_filename)

#     print(interlock(points, 0, -15))
