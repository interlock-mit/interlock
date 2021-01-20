from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d


def visualize(filename):
    cloud = o3d.io.read_point_cloud(filename)
    o3d.visualization.draw_geometries([cloud])


def dist(point_a, point_b):
    x_a, y_a, z_a = point_a[0]
    x_b, y_b, z_b = point_b[0]
    return ((x_a - x_b)**2 + (y_a - y_b)**2 + (z_a - z_b)**2)**0.5


def traversal_order_check(objects, pos_threshold):
    for obj in objects:
        seen = {obj[0][0]}
        for (point_a, point_b) in obj:
            if point_b not in seen:
                print("The traversal order is not valid, because we have not yet seen the point{}".format(
                    point_b))
                return False
            else:
                seen.add(point_a)
                d = dist(point_a, point_b)
                if d > threshold:
                    print("Points {} and {} are {} apart, which is too far away".format(
                        point_a, point_b, d))
                    return False
    return True


def avg_velocity(obj):
    sum_v_x, sum_v_y, sum_v_z = 0, 0, 0
    for (point, _) in obj:
        v_x, v_y, v_z = point[1]
        sum_v_x += v_x
        sum_v_y += v_y
        sum_v_z += v_z
    return sum_v_x/len(obj), sum_v_y/len(obj), sum_v_z/len(obj)


def same_velocity(objects, vel_threshold):
    for obj in objects:
        avg_v_x, avg_v_y, avg_v_z = avg_velocity(obj)
        for (point, _) in obj:
            v_x, v_y, v_z = point[1]
            msg = "The velocity of point {} is too different from the average velocity, which is {}".format(
                point, (avg_v_x, avg_v_y, avg_v_z))
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


def interlock(objects, pos_threshold, vel_threshold):
    """
    objects: list of objects
    object: list of tuples, each of the form (point_a, point_b)
            where point_a is the next point in the traversal order
            and point_b is the point close to point_a
    point:  tuple of (pos, vel)
            where pos = (x, y, z) is the position of the point
            and vel = (v_x, v_y, v_z) is the velocity of the point
    pos_threshold: maximum distance between two points for them to be "close"
    vel_threshold: maximum difference in velocity a point can have
            from the average velocity of that object for it to be "the same"
    """
    if not traversal_order_check(objects, pos_threshold):
        print("Traversal order check failed")
        return False
    if not same_velocity(objects, vel_threshold):
        print("Same velocity check failed")
        return False


# if __name__ == "__main__":
#     filename = 'LiDAR/lidar002310.ply'
#     visualize(filename)

#     new_filename = 'LiDAR/filtered-lidar.ply'
#     points = create_filtered_point_cloud(filename, new_filename)
#     visualize(new_filename)

#     print(interlock(points, 0, -15))
