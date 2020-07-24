def interlock(min_forward_dist, lane_left, lane_right, lane_up, lane_down,
        max_rl_diff, max_ud_diff, max_row_dev, row_heights, rows):
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

