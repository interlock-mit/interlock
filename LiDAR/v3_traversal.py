from LiDAR.segmentation_monitor import THRESHOLD
from collections import defaultdict

def distance_squared(grid, pointA, pointB):
    i_a, j_a, k_a = pointA
    i_b, j_b, k_b = pointB
    (x_a,y_a,z_a), *_ = grid[i_a][j_a][1][k_a]
    (x_b,y_b,z_b), *_ = grid[i_b][j_b][1][k_b]
    return (x_a-x_b)**2 + (y_a-y_b)**2 + (z_a-z_b)**2

def hash_coords(grid, point):
    i,j,k = point
    (x,y,z), velocity, rgb_coord = grid[i][j][1][k]
    return (int(x), int(y), int(z))


def get_traversal_orders(grid):
    hashed_coords = {}
    objects_left = {}

    for i in range(len(grid)):
        for j in range(len(grid[i])):
            for k in range(len(grid[i][j][1])):
                object_id = grid[i][j][0]
                if object_id not in objects_left:
                    objects_left[object_id] = set()
                    hashed_coords[object_id] = defaultdict(set)
                hashed = hash_coords(grid, (i,j,k))
                hashed_coords[object_id][hashed].add((i,j,k))
                objects_left[object_id].add((i,j,k))
    
    traversals = []
    while len(objects_left) > 0:
        object_id, lidar_points_left = objects_left.popitem()
        object_traversal = []

        first_point = dummy_point =  lidar_points_left.pop()
        hashed_coords[object_id][hash_coords(grid, first_point)].remove(first_point)
        dfs_queue = [(first_point, None)] # pairs of (new_point, seen_point)
        while len(dfs_queue) > 0 or len(lidar_points_left) > 0:
            if len(dfs_queue) == 0: # we found another cluster of points, have to start over
                first_point = lidar_points_left.pop()
                hashed_coords[object_id][hash_coords(grid, first_point)].remove(first_point)
                dfs_queue = [(first_point, dummy_point)] # pairs of (new_point, seen_point)
            # find all hashed points close to new_point
            new_point, seen_point = dfs_queue.pop()
            i,j,k = new_point
            (x,y,z), velocity, rgb_coord = grid[i][j][1][k]
            for p_x in range(int(x-THRESHOLD), int(x+THRESHOLD)+2):
                for p_y in range(int(y-THRESHOLD), int(y+THRESHOLD)+2):
                    for p_z in range(int(z-THRESHOLD), int(z+THRESHOLD)+2):
                        start_indx = len(dfs_queue)
                        for nearby_point in hashed_coords[object_id][(p_x, p_y, p_z)]:
                            if distance_squared(grid, new_point, nearby_point) <= THRESHOLD**2:
                                dfs_queue.append((nearby_point, new_point))
                        # remove new points
                        for indx in range(start_indx, len(dfs_queue)):
                            seen_pt, _ = dfs_queue[indx]
                            lidar_points_left.remove(seen_pt)
                            hashed_coords[object_id][(p_x, p_y, p_z)].remove(seen_pt)
            object_traversal.append((new_point, seen_point))
        traversals.append([object_id, object_traversal])
    return traversals