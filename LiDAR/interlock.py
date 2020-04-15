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

def distance(pt, pt2=(0,0,0), horz=False):
    """Distance of a 3D point from another 3D point, or the origin.
    If `horz`, returns the 2D distance ignoring the z coordinate"""
    x, y, z = pt
    x2, y2, z2 = pt2
    if horz:
        return ((x-x2)**2 + (y-y2)**2)**0.5
    return ((x-x2)**2 + (y-y2)**2 + (z-z2)**2)**0.5

################################################################################

def interlock(certificate):
    """Returns whether the controller is allowed to keep going, given the
       certificate object.
       Specially designed for the non-uniform nature of lidar scanner.
    """
    if not certificate.action:
        return False

    pts = certificate.points

    # 0. check no point is too close
    for pt in pts:
        if distance(pt) < min_dist_away:
            return False

    # 1. project the points
    def projected_flattened_pt(pt):
        mag = distance(pt)
        distance_wanted = rect_dist 
        return  distance_wanted/mag*pt[1], \
                distance_wanted/mag*pt[2]

    flat_pts = [projected_flattened_pt(row) for row in pts]

    # 2. identify rows of data
    data = {} # maps height (from ground) to the "row" (scan) list 
              # of horizontal-only (1D) data
    for pt in flat_pts:
        for row_height in data:
            if abs(pt[1]-row_height) < dist_bt_rows/2:
                data[row_height].append([pt[0]])
                break
        else:
            data[pt[1]] = [[pt[0]]]

    #3. check there are enough rows (that span the lane)
    if not len(data) >= int((lane_top-lane_bot) / dist_bt_rows)-2:
        return False 

    #4. check each row has enough x-density of points
    for _, subdata in data.items():
        subdata = sorted(subdata)
        for pt1, pt2 in zip(subdata, subdata[1:]):
            if pt1[0] < lane_left or pt2[0] > lane_right: 
                # outside the lane
                continue
            if distance(pt1, pt2, horz=True) > max_xpt_separation:
                return False

    return True
