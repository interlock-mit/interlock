####### parameters, in meters, tuned for 1/10-scale remote-controlled car ######
#
#min_dist_away = 1 # defines what's too close for a lidar point
#rect_dist = 1 # distance in front of scanner of the lane rectangle
#
## the below depend on `rect_dist`
#dist_bt_rows = 0.0345 # distance between lidar scan rows when scanning a flat 
#                      # (perpendicular) wall `rect_dist` away from the scanner.
#                      # Calibrated based on Velodyne Puck.
#lane_left = -0.25
#lane_right = 0.25
#lane_bot = 0.0
#lane_top = 0.1
#max_xpt_separation = 0.01 # horizontally within every max_xpt_separation meters,
#                          # there's at least one certificate point (i.e. there
#                          # is no obstacle with this width at distance `rect_dist`)
#
#################################################################################
#
#def distance(pt, pt2=(0,0,0)):
#    """Distance of a 3D point from another 3D point, or the origin"""
#    x, y, z = pt
#    x2, y2, z2 = pt2
#    return ((x-x2)**2 + (y-y2)**2 + (z-z2)**2)**0.5
#
#################################################################################
#
#def interlock(certificate):
#    """Returns whether the controller is allowed to keep going, given the
#       certificate object.
#       Specially designed for the non-uniform nature of lidar scanner.
#    """
#    if not certificate.action:
#        return False
#
#    pts = certificate.points
#
#    # 0. check no point is too close
#    for pt in pts:
#        if distance(pt) < min_dist_away:
#            return False
#
#    # 1. project the points
#    def projected_flattened_pt(pt):
#        mag = distance(pt)
#        distance_wanted = rect_dist 
#        return  distance_wanted/mag*pt[1], \
#                distance_wanted/mag*pt[2]
#
#    flat_pts = [projected_flattened_pt(row) for row in pts]
#
#    # 2. identify rows of data
#    data = {} # maps height (from ground) to the "row" (scan) list 
#              # of horizontal-only (1D) data
#    for pt in flat_pts:
#        for row_height in data:
#            if abs(pt[1]-row_height) < dist_bt_rows/2:
#                data[row_height].append([pt[0]])
#                break
#        else:
#            data[pt[1]] = [[pt[0]]]
#
#    #3. check there are enough rows (that span the lane)
#    if not len(data) >= int((lane_top-lane_bot) / dist_bt_rows)-2:
#        return False 
#
#    #4. check each row has enough x-density of points
#    for _, subdata in data.items():
#        subdata = sorted(subdata)
#        for pt1, pt2 in zip(subdata, subdata[1:]):
#            if pt1[0] < lane_left or pt2[0] > lane_right: 
#                # outside the lane
#                continue
#            if abs(pt1[0]-pt2[0]) > max_xpt_separation:
#                return False
#
#    return True

###############
### revised
################

def interlock( … ):
    """
    All distances in metres, measured from centerpoint
    A LiDAR point is a tuple (fb, rl, ud), with each coordinate in meters, with (0, 0, 0) centered on the LiDAR scanner (forward/backward, right/left, up/down: positive in direction of first of each)
    min_forward_dist: minimum distance away of all points
    lane_left, lane_right: displacement of left and right lane edges at distance min_dist
    lane_up, lane_down: required spread of points vertically
    max_rl_diff: maximum horizontal spacing of points within each row
    max_ud_diff: maximum vertical spacing of rows
    max_row_dev: maximum vertical deviation of point in row from row vertical coordinate
    row_heights: array of vertical coordinates for rows, row_heights[i] corresponding to rows[i]
    rows: array of rows, each being array of points

    Specification (fix array index bounds later)
    Return conjunction of
    Min forward dist conformance
            all r: rows | all i | r[i].fb >= min_forward_dist
    Row height conformance
    all p in row [i] | abs(p.ud -  row_heights[i]) <= max_row_dev
    Row height separation
            all i | abs(row_heights[i] - row_heights[i+1]) <= max_ud_diff
    Horizontal density
            all r: rows | all i | abs(r[i].rl - r[i+1].rl) <= max_rl_diff
    Horizontal spread
            all r: rows | r[0].rl <= lane_left and r[n].rl >= lane_right
    Vertical spread (assumes row heights ordered from top to bottom)
            row_heights[0] >= lane_up and row_heights[n] <= lane_down
    """
    def ud (p):
      return p[2]
    def rl (p):
      return p[1]
    def fb (p):
      return p[0]

    # return a point corresponding to projection onto plane at min_forward_dist
    def proj(p):
      k = min_forward_dist / fb(p)
      return (min_dist, rl(p) * k, ud(p) * k)

    result = True

    # vertical spread
    result = result && row_heights[0] >= lane_up
    result = result && row_heights[len(row_heights)-1] <= lane_down


    i = 0
    while (i < len(rows)):
        # horizontal spread
        result = result && rl(proj (rows[i][0])) <= lane_left
        result = result && rl(proj (rows[i][-1])) >= lane_right

        j = 0

        while (j < len(rows[i])):
            # row height conformance
            dev = abs( ud(proj(rows[i][j])) - row_heights[i] )
            result = result && dev <= max_row_dev

            # horizontal density
            if j+1 < len(rows[i]):
                rl_diff = abs( rl(proj(rows[i][j])) - rl(proj(rows[i][j+1])))
                result = result && rl_diff <= max_rl_diff

            # Min forward dist conformance
            result = result && fb(rows[i][j]) >= min_forward_dist
            j++

    # row height separation
    if i+1 < len(rows):
    ud_diff = abs(row_heights[i] - row_heights[i+1])
    result = result && ud_diff <= max_ud_diff

    i++
    return result

