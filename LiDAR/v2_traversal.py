import itertools
from collections import defaultdict

cell_size = .5

def get_traversal_orders(grid):
    """
    grid: 2d array of cells where each cell is a tuple of the form (obj_id, lidar_points)
        where obj_id is the unique id corresponding to the object in that cell
        and lidar_points is a list of 3d lidar points that transform onto that 2d cell
        where each lidar point is a list of the form [loc, vel, image_pos]
    loc: numpy array of the form (x, y, z)
    vel: numpy array of the form (v_x, v_y, vg_z)
    image_pos: list of the form (x, y)

    output: a list of tuples, each of the form (obj_id, traversal_order)
    traversal_order: list of tuples, each of the form (pt_1, pt_2)
    pt_1, pt_2: grid locations, which are tuples of the form (i, j, k)
    i: row of the point's grid cell
    j: column of the point's grid cell
    k: the index of the list lidar_points at which the point appears

    the first point must be close to the second point, and the second point must be a point that we have seen earlier on in the list;
    the first tuple's second value is None

    Finds a traversal order for determining the spread of a collection of 3D LiDAR points given that points in that collection
    are at most cell_size distance apart
    """
    if not len(grid): # empty grid given
        return 

    def get_min(idx, obj_id):
        min_row_pts = []
        for row in grid:
            idx_pts = [pt[0][idx] for cell in row for pt in cell[1] if cell[0] == obj_id]
            if any(idx_pts): min_row_pts.append(min(idx_pts))
        return min(min_row_pts) if any(min_row_pts) else None

    def get_max(idx, obj_id):
        max_row_pts = []
        for row in grid:
            idx_pts = [pt[0][idx] for cell in row for pt in cell[1] if cell[0] == obj_id]
            if any(idx_pts): max_row_pts.append(max(idx_pts))
        return max(max_row_pts) if any(max_row_pts) else None

    def get_len(idx, obj_id, min_vals, max_vals):
        return int((max_vals[idx] - min_vals[idx])/cell_size) + 1

    obj_ids = {cell[0] for row in grid for cell in row}

    mins = {obj_id: tuple(get_min(i, obj_id) for i in range(3)) for obj_id in obj_ids}
    maxes = {obj_id: tuple(get_max(i, obj_id) for i in range(3)) for obj_id in obj_ids}

    lens = {}
    for obj_id in obj_ids:
        min_vals, max_vals = mins[obj_id], maxes[obj_id]
        if max_vals[0] is not None:
            lens[obj_id] = tuple(get_len(i, obj_id, min_vals, max_vals) for i in range(3))

    # dictionary mapping obj_id to a 3D array representing 3D grid-space for that object
    # 3D array represented as a dictionary mapping density indices -> grid index (i,j,k)
    density = defaultdict(dict)

    def get_cell_idx(obj_id, point, idx):
        return int((point[idx] - mins[obj_id][idx])/cell_size)

    for j in range(len(grid)):
        row = grid[j]
        for i in range(len(row)):
            obj_id, pts = row[i]
            for k in range(len(pts)):
                # populates a cell with a point if the point is in the cell, max one point per cell
                idxs = tuple(get_cell_idx(obj_id, pts[k][0], i) for i in range(3))
                density[obj_id][idxs] = (i,j,k)


    # yields the grid indices near the given index by examining neighboring cells
    # idxs: density indices (density[obj_id][idxs] gives the set of points in that 3D cell)
    # obj_id: id of the object that the point is part of
    def get_neighbors(idxs, obj_id):
        for idx in range(3):
            alts = [0, 0, 0]
            if idxs[idx] > 0:
                alts[idx] = -1
                indices = (idxs[0] + alts[0], idxs[1] + alts[1], idxs[2] + alts[2])
                if indices in density[obj_id]:
                    if density[obj_id][indices] is not None:
                        yield indices
            if idxs[idx] < lens[obj_id][idx] - 1:
                alts[idx] = 1
                indices = (idxs[0] + alts[0], idxs[1] + alts[1], idxs[2] + alts[2])
                if indices in density[obj_id]:
                    if density[obj_id][indices] is not None:
                        yield indices


    # BFS to find traversal order 
    def bfs(obj_id):
        if len(density[obj_id].keys()) == 0: return []
        dens_idx = list(density[obj_id].keys())[0]
        start = density[obj_id][dens_idx]
        ordering = [(start, None)]
        queue = [dens_idx]
        seen = {dens_idx}
        while queue:
            new_queue = []
            for dens_idx in queue:
                for neighbor_cell in get_neighbors(dens_idx, obj_id):
                    if neighbor_cell not in seen:
                        ordering.append((density[obj_id][neighbor_cell], density[obj_id][dens_idx]))
                        seen.add(neighbor_cell)
                        new_queue.append(neighbor_cell)
            queue = new_queue
        return ordering

    orderings = [(obj_id, bfs(obj_id)) for obj_id in obj_ids]
    return orderings


def generate_points(lower, upper, is_rand = False):
    if is_rand: 
        import numpy as np 
        sampl = np.random.uniform(low=lower, high=upper, size=(5,3))
        return [tuple(point) for point in sampl]
    pts = [(i,j,k) for i in range(lower, upper) for j in range(lower, upper) for k in range(lower, upper)]
    return pts


def process_and_remove_outliers(obj_info):
    axis = 1
    filter_dist = 3
    for obj in obj_info:
        if not obj_info[obj]:
            continue
        # print(obj_info[obj])
        points = sorted(obj_info[obj], key=lambda x: x[0][axis])
        median = points[len(points)//2][0][axis]
        obj_info[obj] = list(filter(lambda x: abs(x[0][axis] - median) < filter_dist, points))
        # print(obj_info[obj])




if __name__ == "__main__":

    import pickle
    import numpy as np
    import open3d
    pcd = open3d.geometry.PointCloud()
    pcd_points = None
    with open("car_106761.pkl", 'rb') as pklfile: 
        # keys: [1, 2, 7, 123, 155, 163, 222, 246, 261, 309, 335, 398, 423, 499, 500, 510, 518, 519, 521, 531, 568, 569, 573, 575, 580, 586, 593, 608, 632, 634, 684, 689, 692, 693, 694, 695]
        obj = pickle.load(pklfile)
        print(obj.keys())
        for key in obj:
            print(key)
            pos_vels = obj[1024]
            pos = process_and_remove_outliers(pos_vels)
            points = [tuple(pos) for pos in pos]
            np_points = np.array([pos for pos in pos]) 
            if len(points):
                pcd_points = np.concatenate((pcd_points, np_points), axis=0) if pcd_points is not None else np_points
            # print(np_points.shape, pcd_points.shape)
                cur_pcd = open3d.geometry.PointCloud()
                cur_pcd.points = open3d.utility.Vector3dVector(np_points)
                open3d.visualization.draw_geometries([cur_pcd])


                print(get_traversal_order(points))
            break


    print(np_points.shape)
    pcd.points = open3d.utility.Vector3dVector(pcd_points)
    open3d.visualization.draw_geometries([pcd])

        
