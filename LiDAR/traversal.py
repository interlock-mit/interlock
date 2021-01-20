cell_size = 3

def get_traversal_order(points):
    def get_min(idx):
        return min(points, key=lambda x: x[idx])[idx]
    def get_max(idx):
        return max(points, key=lambda x: x[idx])[idx]

    mins = get_min(0), get_min(1), get_min(2)
    maxes = get_max(0), get_max(1), get_max(2)

    def get_len(idx):
        return int((maxes[idx]-mins[idx])/cell_size) + 1
    lens = get_len(0), get_len(1), get_len(2)

    density = [[[None] * lens[2] for j in range(lens[1])] for i in range(lens[0])]
    
    def get_cell_idx(val, idx):
        return int((val - mins[idx])/cell_size)

    for point in points:
        idxs = get_cell_idx(point[0], 0), get_cell_idx(point[1], 1), get_cell_idx(point[2], 2)
        density[idxs[0]][idxs[1]][idxs[2]] = point
    print(density)

    def get_neighbors(point):
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

    start = points[0]
    ordering = [(start, None)]
    queue = [start]
    seen = {start}
    while queue:
        new_queue = []
        for point in queue:
            for neighbor in get_neighbors(point):
                if neighbor not in seen:
                    ordering.append((neighbor, point))
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

# points = generate_points(0, 3)
# print(points)
# print(get_traversal_order(points))


import pickle
import numpy as np
import open3d
pcd = open3d.geometry.PointCloud()
pcd_points = None
with open("scene_449.pkl", 'rb') as pklfile: 
    # keys: [1, 2, 7, 123, 155, 163, 222, 246, 261, 309, 335, 398, 423, 499, 500, 510, 518, 519, 521, 531, 568, 569, 573, 575, 580, 586, 593, 608, 632, 634, 684, 689, 692, 693, 694, 695]
    obj = pickle.load(pklfile)
    print(obj.keys())
    for key in obj:
        print(key)
        pos_vels = obj[2]
        points = [tuple(pos) for vel, pos in pos_vels]
        print(len(points))
        np_points = np.array([pos for vel, pos in pos_vels]) 
        if len(points):
            pcd_points = np.concatenate((pcd_points, np_points), axis=0) if pcd_points is not None else np_points
        # print(np_points.shape, pcd_points.shape)

        print(get_traversal_order(points))
        break
print(np_points.shape)
pcd.points = open3d.utility.Vector3dVector(pcd_points)
open3d.visualization.draw_geometries([pcd])

        
