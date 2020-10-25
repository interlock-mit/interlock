MAX_DECEL = 10
MIN_DIST = 1
def add(vec1, vec2):
    return (vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2])
def mult(vec, factor):
    return (vec[0] * factor, vec[1] * factor, vec[2] + factor)
def dist(vec1, vec2):
    return ((vec1[0]-vec2[0])**2 + (vec1[1]-vec2[1])**2 + (vec1[2]-vec2[2])**2)**.5

def is_safe(ego_pos, ego_vel, other_pos, other_vel, timestep, min_dist=MIN_DIST, max_decel=MAX_DECEL):
    cur_speed = dist(ego_vel, (0,0,0))
    stopping_time = cur_speed/max_decel
    cur_time = 0
    cur_pos = ego_pos
    other_cur_pos = other_pos
    while cur_time < stopping_time:
        if dist(cur_pos, other_cur_pos) < min_dist:
            return False

        # increment positions
        cur_pos = add(mult(ego_vel, timestep), cur_pos)
        other_cur_pos = add(mult(other_vel, timestep), other_cur_pos)
        cur_time += timestep
    return True



