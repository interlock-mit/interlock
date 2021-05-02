# from traversal import get_traversal_order, cell_size, process_and_remove_outliers
from LiDAR.v3_traversal import get_traversal_orders
from LiDAR.segmentation_monitor import interlock

import pickle
import numpy as np
import open3d
import datetime


def pipeline(grid, image, scale_factor, ego_vel, ground_ids, sky_ids):
    print("Number of lidar points: ", sum(len(x[1]) for row in grid for x in row))
    time = datetime.datetime.now()
    traversal_orders = get_traversal_orders(grid)
    print("created traversal orders. Time: ", (datetime.datetime.now()-time).total_seconds())
    #traversal_orders = None
    time = datetime.datetime.now()
    result = interlock(grid, ground_ids, sky_ids, traversal_orders, image, scale_factor, ego_vel)
    print("finished interlock pipeline. Time: ", (datetime.datetime.now()-time).total_seconds())
    return result
