# from plyfile import PlyData, PlyElement
import numpy as np
from collections import defaultdict

import datetime

MAX_DECEL = -4.5 # m/s^2
MIN_DIST = 0 # m
LANE_WIDTH = 3.5 # m
imag_threshold = 1e-5 # cutoff for imaginary/real roots

VELOCITY_THRESHOLD = (2, 2, 2)
THRESHOLD = 3
SCOPE = 30

IGNORE_CELL_CUTOFF = 5
IGNORE_LIDAR_POINTS_CUTOFF = 10

POINT_IN_PATH = lambda x: (-LANE_WIDTH/2 < x[1] < LANE_WIDTH/2) and x[0] > 0 and x[2] > -1.2 and x[0] < SCOPE
POINT_COORD = lambda p, grid: grid[p[0]][p[1]][1][p[2]][0]

def add(vec1, vec2):
    return (vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2])
def mult(vec, factor):
    return (vec[0] * factor, vec[1] * factor, vec[2] * factor)
def dist(vec1, vec2):
    return ((vec1[0]-vec2[0])**2 + (vec1[1]-vec2[1])**2 + (vec1[2]-vec2[2])**2)**.5
def loc(pos, vel, t):
    return (pos[0]+t*vel[0], pos[1]+t*vel[1], pos[2]+t*vel[2])

def dist(point_a, point_b=(0, 0, 0)):
    x_a, y_a, z_a = point_a
    x_b, y_b, z_b = point_b
    return ((x_a - x_b)**2 + (y_a - y_b)**2 + (z_a - z_b)**2)**0.5


def get_points(obj_id, grid):
    cells = [cell for row in grid for cell in row if cell[0] == obj_id]
    points = [pt for cell in cells for pt in cell[1]]
    return points


def check_traversal_order(test, grid, traversal_orders, sky_ids, image_pos, debug=False):
    for (obj_id, traversal_order) in traversal_orders:
        if obj_id in sky_ids or traversal_order is None or len(traversal_order) == 0:
            continue
        
        first_elem = traversal_order[0][0]
        seen = {first_elem: 1}
        connected_components = {1: {first_elem}}
        connected_components_size = {1: 1 if POINT_IN_PATH(POINT_COORD(first_elem, grid)) else 0}
        for (point_a, point_b) in traversal_order[1:]:
            if point_b not in seen:
                test["success"] = False
                test["error_msg"] = f"The traversal order is not valid, because we have not yet seen the point {point_b}"
                return
            else:
                d = dist(POINT_COORD(point_a, grid), POINT_COORD(point_b, grid))
                in_path = 1 if POINT_IN_PATH(POINT_COORD(point_a, grid)) else 0
                component = None
                if d > THRESHOLD:
                    component = len(connected_components)+1
                    connected_components[component] = {point_a}
                    connected_components_size[component] = in_path
                else:
                    component = seen[point_b]
                    connected_components[component].add(point_a)
                    connected_components_size[component] += in_path
                seen[point_a] = component

        if sum(connected_components_size[x] > 0 for x in connected_components_size) > 1:
            min_dist = float('inf')
            lengths = [(connected_components_size[x], x) for x in connected_components_size]
            max_num, max_indx = max(lengths, key= lambda x: x[0])
            rest_num = sum(connected_components_size[x] for x in connected_components if x != max_indx)
            if rest_num <= IGNORE_LIDAR_POINTS_CUTOFF:
                return
            print("Spatial Object Error. Right # of lidar pts: ", max_num, ". Wrong # of lidar pts: ", rest_num)
            test["success"] = False
            for indx in connected_components:
                if indx != max_indx:
                    for i,j,k in connected_components[indx]:
                        pt = grid[i][j][1][k][0]
                        if dist(pt) < min_dist:
                            min_dist = dist(pt)
                        test["bad_points"].append(grid[i][j][1][k][2])
                else:
                    for i,j,k in connected_components[indx]:
                        test["good_points"].append(grid[i][j][1][k][2])
        else:
            pass

def avg_velocity(obj):
    sum_v_x, sum_v_y, sum_v_z = 0, 0, 0
    for point_vel in obj:
        v_x, v_y, v_z = point_vel
        sum_v_x += v_x
        sum_v_y += v_y
        sum_v_z += v_z
    return sum_v_x/len(obj), sum_v_y/len(obj), sum_v_z/len(obj)


def check_same_velocity(test, points, obj_velocities, image_pos):
    for obj, vels in obj_velocities.items():
        if vels is None:
            continue
        positions = points[obj]
        bad_points = []
        avg_v_x, avg_v_y, avg_v_z = avg_velocity(vels)
        for pt, point_vel in zip(positions, vels):
            v_x, v_y, v_z = point_vel
            if POINT_IN_PATH(pt) and (abs(v_x - avg_v_x) > VELOCITY_THRESHOLD[0] or abs(v_y - avg_v_y) > VELOCITY_THRESHOLD[1] or abs(v_z - avg_v_z) > VELOCITY_THRESHOLD[2]):
                bad_points.append(point_vel)
        if len(bad_points) > IGNORE_LIDAR_POINTS_CUTOFF:
            test["success"] = False
            test["bad_points"].extend(image_pos[obj])


def check_density_spread_all_objs(test, grid, image_pos, image, image_scale_factor):
    from Segmentation.lidar_density_location import LIDAR_GRID
    missing_cells = []
    for i in range(len(LIDAR_GRID)):
        for j in range(len(LIDAR_GRID[i])):
            if LIDAR_GRID[i][j] == 1 and len(grid[i][j][1]) == 0 and len(LIDAR_GRID[i])//3 <= j <= len(LIDAR_GRID[i]) - len(LIDAR_GRID[i])//3:
                missing_cells.append((i,j))
    if len(missing_cells) > IGNORE_CELL_CUTOFF:
        test["success"] = False
    return True, []


def check_ground_pts_on_ground(test, grid, ground_ids, height_threshold=0):
    bad_points = []
    for ground_id in ground_ids:
        ground_points = get_points(ground_id, grid)
        for pt in ground_points:
            z = pt[0][2]
            if z > height_threshold:
                test["success"] = False
                test["error_msg"] = f"The point {pt} is not on the ground"
                test["bad_points"].append(pt[2])


def is_safe(ego_vel, other_pos, other_vel, min_dist=MIN_DIST, max_decel=MAX_DECEL):
    cur_speed = dist(ego_vel)
    stopping_time = cur_speed/abs(max_decel)

    def helper(ego_accel, other_accel):
        x_e, y_e, z_e = (0, 0, 0) # ego position
        x_o, y_o, z_o = other_pos # other position
        v_x_e, v_y_e, v_z_e = ego_vel # ego velocity
        v_x_o, v_y_o, v_z_o = other_vel # other velocity
        a_e = ego_accel # ego acceleration
        a_o = other_accel # other acceleration
        
        d_x, d_y, d_z = x_e - x_o, y_e - y_o, z_e - z_o
        d_v_x, d_v_y, d_v_z = v_x_e - v_x_o, v_y_e - v_y_o, v_z_e - v_z_o
        d_a = a_e - a_o

        def is_real(t):
            return abs(np.imag(t)) < imag_threshold

        def solve_quadratic(a, b, c):
            """ Returns a list of all real t values that solve at^2 + bt + c = 0 """
            return [t for t in np.roots([a, b, c]) if is_real(t) and t >= 0]

        def solve_cubic(a, b, c, d):
            """ Returns a list of all real t values that solve at^3 + bt^2 + ct + d = 0 """
            return [t for t in np.roots([a, b, c, d]) if is_real(t) and t >= 0]

        def solve_quartic(a, b, c, d, e):
            """ Returns a list of all real t values that solve at^4 + bt^3 + ct^2 + dt + e = 0 """
            return [t for t in np.roots([a, b, c, d, e]) if is_real(t) and t >= 0]

        def eval(t, a, b, c, d, e):
            return (a*t**4 + b*t**3 + c*t**2 + d*t + e)**0.5

        # distance between ego and other as a function of time: d(t) = sqrt(at^4 + bt^3 + ct^2 + dt + e)
        a = (3/4)*(d_a**2)
        b = (d_a/2)*(d_v_x + d_v_y + d_v_z)
        c = (d_a/2)*(d_x + d_y + d_z) + d_v_x**2 + d_v_y**2 + d_v_z**2
        d = 2*(d_x*d_v_x + d_y*d_v_y + d_z*d_v_z)
        e = d_x**2 + d_y**2 + d_z**2 

        # unsafe if d(0) <= min_dist
        if eval(0, a, b, c, d, e) <= min_dist:
            return False
        
        # unsafe if d(stopping_time) <= min_dist
        if eval(stopping_time, a, b, c, d, e) <= min_dist:
            return False

        def return_ts(a, b, c, d, e):
            if a == 0:
                if b == 0:
                    if c == 0:
                        if d == 0:
                            return []
                        else:
                            return [(min_dist**2 - e)/d]
                    else:
                        return solve_quadratic(c, d, e)
                else:
                    return solve_cubic(b, c, d, e)
            else:
                return solve_quartic(a, b, c, d, e)

        # unsafe if d(t) <= min_dist for some t <= stopping_time
        # d(t) minimized when 4at^3 + 3bt^2 + 2ct + d = 0
        for t in return_ts(0, 4*a, 3*b, 2*c, d):
            if 0 <= t <= stopping_time:
                if eval(t, a, b, c, d, e) <= min_dist:
                    return False

        # unsafe if d(t) = min_dist for some t <= stopping_time
        for t in return_ts(a, b, c, d, e - min_dist**2):
            if 0 <= t <= stopping_time:
                return False

        # unsafe if d(t) = 0 for some t <= stopping_time
        for t in return_ts(a, b, c, d, e):
            if 0 <= t <= stopping_time:
                return False

        return True

    if all([other_vel[i] == 0] for i in range(3)): # stationary object
        return helper(max_decel, 0)
    else: # moving object
        return helper(max_decel, 0) and helper(max_decel, max_decel)

def check_predicates(test, grid, vels, ego_vel, ground_ids, image_pos):
    """
    For now assuming a straight path but eventually we can incorporate curved or more complex paths
    """
    obj_in_path = lambda x: any(POINT_IN_PATH(p[0]) for p in get_points(x, grid))
    obj_ids = {cell[0] for row in grid for cell in row}
    object_ids_in_path = list(filter(obj_in_path, obj_ids))
    for obj_id in object_ids_in_path:
        if obj_id in ground_ids:
            continue
        closest_pt = min(get_points(obj_id, grid), key=lambda p: dist(p[0]))[0]
        obj_vel = avg_velocity(vels[obj_id])
        if not is_safe(ego_vel, closest_pt, obj_vel):
            test["success"] = False
            #test["error_msg"] = f"The object with ID {obj_id} is not safe"
            test["bad_points"].extend(image_pos[obj_id])


def interlock(grid, ground_ids, sky_ids, traversal_orders, image, image_scale_factor, ego_vel):
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

    test_suite = {
        "Spatial Check": {
            "function": lambda test: check_traversal_order(test, grid, traversal_orders, sky_ids, image_pos),
            "success": True,
            "error_msg": "",
            "bad_points": []
        },
        "Velocity Check": {
            "function": lambda test: check_same_velocity(test, points, vels, image_pos),
            "success": True,
            "error_msg": "",
            "bad_points": []
        },
        "Density Check": {
            "function": lambda test: check_density_spread_all_objs(test, grid, image_pos, image, image_scale_factor),
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
        test["good_points"] = []
        time = datetime.datetime.now()
        test["function"](test)
        print("Finished check ", check, ".  Time: ", (datetime.datetime.now()-time).total_seconds())
    return test_suite
