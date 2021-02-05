from traversal import get_traversal_order, cell_size, process_and_remove_outliers
from segmentation_monitor import interlock

import pickle
import numpy as np
import open3d


def get_traversal_orders(obj_info):
    return {obj: get_traversal_order([point_info[0] for point_info in obj_info[obj]]) for obj in obj_info}


pcd = open3d.geometry.PointCloud()
pcd_points = None
with open("pkl_files/car_530.pkl", 'rb') as pklfile: 
    # keys: [1, 2, 7, 123, 155, 163, 222, 246, 261, 309, 335, 398, 423, 499, 500, 510, 518, 519, 521, 531, 568, 569, 573, 575, 580, 586, 593, 608, 632, 634, 684, 689, 692, 693, 694, 695]
    data = pickle.load(pklfile)
    obj_info = data["object_pts"]
    print(obj_info.keys())
    image = data["labeled_image"]
    scale_factor = data["factor"]
    print(image)
    print(type(image))
    nums = set()
    for i in range(len(image)):
        for j in range(len(image[i])):
            nums.add(image[i][j])
    print(nums)
    # process_and_remove_outliers(obj_info)
    traversal_orders = get_traversal_orders(obj_info)
    interlock(obj_info, None, traversal_orders, image, (10,10,10), scale_factor, None)

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
