cell_size = .5

def get_traversal_order(points):
    """
    input: points - a list of tuples, where each tuple corresponds to a point in 3D space
    output: a list of tuples, where each tuple contains two 3D points; the first point must be close to the second point, 
    and the second point must be a point that we have seen earlier on in the list; the first tuple's second value is None

    Finds a traversal order for determining the spread of a collection of 3D LiDAR points given that points in that collection
    are at most cell_size distance apart
    """
    # print(points)
    if not len(points): # empty list of points given
        return None
    def get_min(idx):
        return min(points, key=lambda x: x[idx])[idx]
    def get_max(idx):
        return max(points, key=lambda x: x[idx])[idx]

    mins = get_min(0), get_min(1), get_min(2)
    maxes = get_max(0), get_max(1), get_max(2)

    def get_len(idx):
        return int((maxes[idx]-mins[idx])/cell_size) + 1
    lens = get_len(0), get_len(1), get_len(2)

    # 3D array representing 3D grid-space
    density = [[[None] * lens[2] for j in range(lens[1])] for i in range(lens[0])]
    
    def get_cell_idx(val, idx):
        return int((val - mins[idx])/cell_size)

    for i in range(len(points)):
        idxs = get_cell_idx(points[i][0], 0), get_cell_idx(points[i][1], 1), get_cell_idx(points[i][2], 2)
        # populates a cell with a point if the point is in the cell; max one point per cell
        density[idxs[0]][idxs[1]][idxs[2]] = i 
    # print(density)

    def get_neighbors(point):
        # yields the points near the given point by examining neighboring cells
        idxs = get_cell_idx(point[0], 0), get_cell_idx(point[1], 1), get_cell_idx(point[2], 2)
        neighbors = []

        for idx in range(3):
            alts = [0, 0, 0]
            if idxs[idx] > 0:
                alts[idx] = -1
                neighbor = density[idxs[0] + alts[0]][idxs[1] + alts[1]][idxs[2] + alts[2]]
                if neighbor is not None:
                    yield neighbor
            if idxs[idx] < lens[idx] - 1:
                alts[idx] = 1
                neighbor = density[idxs[0] + alts[0]][idxs[1] + alts[1]][idxs[2] + alts[2]]
                if neighbor is not None:
                    yield neighbor

    # BFS to find traversal order 
    start = 0
    ordering = [(start, None)]
    queue = [start]
    seen = {start}
    while queue:
        new_queue = []
        for point_idx in queue:
            for neighbor in get_neighbors(points[point_idx]):
                if neighbor not in seen:
                    ordering.append((neighbor, point_idx))
                    seen.add(neighbor)
                    new_queue.append(neighbor)
        queue = new_queue
    return ordering
                    

    
# points = [(1.1, 2.2, 3.3), (3.4, 5.2, -1.2), (2.3, 3.5, 4.3)]

def generate_points(lower, upper, is_rand = False):
    if is_rand: 
        import numpy as np 
        sampl = np.random.uniform(low=lower, high=upper, size=(5,3))
        return [tuple(point) for point in sampl]
    pts = []
    for i in range(lower, upper):
        for j in range(lower, upper):
            for k in range(lower, upper):
                pts.append((i,j,k))
    return pts


def process_and_remove_outliers(pos_vels, remove=True):
    axis = 1
    filter_dist = 3
    points = sorted([tuple(pos) for pos, vel, pic_pos in pos_vels], key=lambda x: x[axis])
    if remove:
        median = points[len(points)//2][axis]
        return list(filter(lambda x: abs(x[axis] - median) < filter_dist, points))
    return points



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

        
