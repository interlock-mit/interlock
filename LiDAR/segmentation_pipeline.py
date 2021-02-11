# from traversal import get_traversal_order, cell_size, process_and_remove_outliers
from v2_traversal import get_traversal_orders, cell_size, process_and_remove_outliers
from segmentation_monitor import interlock

import pickle
import numpy as np
import open3d


def combine_img_lidar(obj_info, image, scale_factor):
    cell_to_lidar = {}
    for obj_id in obj_info:dar = {}
    for obj_id in obj_info:
        obj = obj_info[obj_id]
        for point in obj:
            x, y = point[2]
            cell = (x//10, y//10)
            if cell in cell_to_lidar:
                cell_to_lidar[cell].append(point)
            else:
                cell_to_lidar[cell] = [point]

    grid = []
    for j in range(len(image)):
        row = []
        for i in range(len(image[j])):
            if (i, j) in cell_to_lidar:
                lidar_pts = cell_to_lidar[(i, j)]
            else:
                lidar_pts = []
            obj_id = image[j][i]
            cell_info = (obj_id, lidar_pts)
            row.append(cell_info)
        grid.append(row)

    return grid


# def get_traversal_orders(obj_info):
#     return {obj: get_traversal_order([point_info[0] for point_info in obj_info[obj]]) for obj in obj_info}


pcd = open3d.geometry.PointCloud()
pcd_points = None
with open("pkl_files/car_530.pkl", 'rb') as pklfile: 
    # keys: [1, 5, 6, 7, 8, 9, 11, 13, 15, 16, 17, 18, 22, 24, 25, 27, 33, 35, 36, 38, 39, 40]
    data = pickle.load(pklfile)
    obj_info = data["object_pts"]
    image = data["labeled_image"]
    scale_factor = data["factor"]
    # process_and_remove_outliers(obj_info)
    # traversal_orders = get_traversal_orders(obj_info)
    grid = combine_img_lidar(obj_info, image, scale_factor)
    traversal_orders = get_traversal_orders(grid)
    interlock(grid, None, traversal_orders, image, (10,10,10), scale_factor, None)

    # for key in obj:
    #     print(key)
    #     pos_vels = obj[1024]
    


        # projection_map = {tuple(pos): pic_pos for pos, vel, pic_pos in pos_vels}
        # pos = process_and_remove_outliers(pos_vels, False)
        # points = [tuple(pos) for pos in pos]
        # print(len(points))
        # np_points = np.array([pos for pos in pos]) 
        # if len(points):
        #     pcd_points = np.concatenate((pcd_points, np_points), axis=0) if pcd_points is not None else np_points
        # # print(np_points.shape, pcd_points.shape)
        #     cur_pcd = open3d.geometry.PointCloud()
        #     cur_pcd.points = open3d.utility.Vector3dVector(np_points)
        #     open3d.visualization.draw_geometries([cur_pcd])


        #     traversal_order = get_traversal_order(points)
             
        #     # print(traversal_order)
        #     print(check_traversal_order([traversal_order], cell_size *2 *  (3 ** .5)))
        #     traversal_pts = [pt for pt, other_pt in traversal_order]
        #     rgb_pts = [tuple(projection_map[pos]) for pos in traversal_pts]
        #     print(check_density_spread(rgb_pts, traversal_pts, 10))
        # break


# print(np_points.shape)
# pcd.points = open3d.utility.Vector3dVector(pcd_points)
# open3d.visualization.draw_geometries([pcd])
