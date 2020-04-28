from scipy.spatial import cKDTree
import numpy as np

###### parameters, in meters, tuned for 1/10-scale remote-controlled car ######

min_dist_away = 1 # defines what's too close for a lidar point
rect_dist = 1 # distance in front of scanner of the lane rectangle

# the below depend on `rect_dist`
dist_bt_rows = 0.0345 # distance between lidar scan rows when scanning a flat 
                      # (perpendicular) wall `rect_dist` away from the scanner.
                      # Calibrated based on Velodyne Puck.
lane_left = -0.25
lane_right = 0.25
lane_bot = 0.0
lane_top = 0.1
max_xpt_separation = 0.01 # horizontally within every max_xpt_separation meters,
                          # there's at least one certificate point (i.e. there
                          # is no obstacle with this width at distance `rect_dist`)

################################################################################

class Certificate:
    def __init__(self, points, action):
        self.action = action
        self.points = points # np.ndarray of 3D points (shape ix3)


def distance(pt, pt2=(0,0,0)):
    """Distance of a 3D point from another 3D point, or the origin"""
    x, y, z = pt
    x2, y2, z2 = pt2
    return ((x-x2)**2 + (y-y2)**2 + (z-z2)**2)**0.5

################################################################################

def snow_removed(pts):
    num_neighbors = max(int(len(pts)/15), 1) # tunable
    beta_mult = 3. # tunable. (about 99.7% of a normal distribution
                   # would fall this many stdevs above the mean)

    tree = cKDTree(pts)
    neighbor_distances, neighbor_indices = tree.query(pts, k=num_neighbors)
    distance_sums = np.sum(neighbor_distances, axis=1)

    def get_average(n):
        return n/num_neighbors
    distances = get_average(distance_sums)
    total_mean = np.average(distances)
    total_std = np.std(distances)

    threshhold = total_mean + total_std * beta_mult

    return np.where(distances <= threshhold, True, False) # bool mask

def filtered(pts):
    is_not_snow = snow_removed(pts)

    def is_far_enough(pt):
        return distance(pt) >= min_dist_away
    is_far_enough_mask = np.apply_along_axis(is_far_enough, 1, pts) # bool mask

    return pts[np.logical_and(is_not_snow, is_far_enough_mask)]

################################################################################

def smallest_cert(pts):
    """
    Returns the smallest subset of pts, as a np.ndarray, such that,
    if possible, there is at most max_xpt_separation between each row,
    and all rows are maintained. (To best satisfy the interlock.)

    Uses a (negligibly non-optimal) greedy algorithm.
    """
    # 1. project the points
    def projected_flattened_pt(pt):
        mag = distance(pt)
        distance_wanted = rect_dist
        return  distance_wanted/mag*pt[1], \
                distance_wanted/mag*pt[2]

    flat_pts = np.array([[projected_flattened_pt(pt), pt] for pt in pts])

    # 2. identify rows of data
    data = {} 
    # maps height (from ground) to the "row" (scan) list of
    # horizontal-only (1D) data
    for pt, origpt in flat_pts:
        for row_height in data:
            if abs(pt[1]-row_height) < dist_bt_rows/2:
                data[row_height].append([pt[0], origpt])
                break
        else:
            data[pt[1]] = [[pt[0], origpt]]

    #3. build up final_data so each row has enough point x-density
    def get_left_i(pts, left_edge):
        # get last index <= left_edge
        for i, pt in enumerate(pts):
            if pt[0] > left_edge:
                return i - 1

    def get_right_i(pts, right_edge):
        # get first index >= right_edge
        i = len(pts)-1
        while True:
            pt = pts[i]
            if pt[0] < right_edge:
                return i + 1
            i -= 1

    final_data = []
    for _, subdata in data.items():
        subdata = sorted(subdata, key=lambda t: t[0])
        i = get_left_i(subdata, lane_left)
        end = get_right_i(subdata, lane_right)

        final_data = [subdata[i][1]]
        while i < end:
            next_i = get_left_i(subdata[i:], 
                                subdata[i][0]+max_xpt_separation)
            if next_i == 0:
                return False
            i += next_i
            final_data.append(subdata[i][1])

    return final_data

################################################################################

def controller(lidar_points):
    """
    Produces a Certificate of points if deemed safe to proceed forward;
    otherwise produces an empty Certificate with action False.

    lidar_points is a numpy array of 3D points
    """
    # Project points onto a "rectangle" plane `rect_dist` in front of scanner
    proj_lidar_points = rect_dist * lidar_points / np.linalg.norm(lidar_points, axis=1)[:,None]

    # Reduce to the points within the lane ahead
    lidar_points = lidar_points[np.multiply(
                                 np.multiply(
                                  np.logical_and(lane_left <= proj_lidar_points[:,1], 
                                                 proj_lidar_points[:,1] <= lane_right), 
                                  proj_lidar_points[:,0] >= 0
                                 ), 
                                 np.logical_and(lane_bot <= proj_lidar_points[:,2],
                                                proj_lidar_points[:,2]  <= lane_top)
                                )
                               ]

    npoints_before_filtering = len(lidar_points)
    lidar_points = filtered(lidar_points)

    # Simple check (to be replaced by production neural net code): 
    # at least 80% of points weren't snow outliers or too close
    if len(lidar_points) < 0.8*npoints_before_filtering:
        return Certificate(lidar_points, False)

    # certificate contains the min number of far enough, 
    # non-snow points
    cert_pts = smallest_cert(lidar_points)
    return Certificate(cert_pts, bool(cert_pts))
