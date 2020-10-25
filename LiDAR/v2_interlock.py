MAX_DECEL = 10
MIN_DIST = 1
EGO_POS = (0,0,0)
DEFAULT_TIMESTEP = .1


def add(vec1, vec2):
    return (vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2])
def mult(vec, factor):
    return (vec[0] * factor, vec[1] * factor, vec[2] + factor)
def dist(vec1, vec2):
    return ((vec1[0]-vec2[0])**2 + (vec1[1]-vec2[1])**2 + (vec1[2]-vec2[2])**2)**.5

def is_safe(ego_pos, ego_vel, other_pos, other_vel, timestep=DEFAULT_TIMESTEP, min_dist=MIN_DIST, max_decel=MAX_DECEL):
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

if __name__ == "__main__":
    stationary_pts = [
        {"pos": (0,0,10), "vel": (0, 0, 0)},
        {"pos": (0,3,10), "vel": (0, 0, 0)},
        {"pos": (0,2,10), "vel": (0, 0, 0)},
        {"pos": (-1,1,10), "vel": (0, 0, 0)},
        {"pos": (1,0,10), "vel": (0, 0, 0)},
        {"pos": (2,-1,10), "vel": (0, 0, 0)},
    ]
    driving_fwd_pts = [
        {"pos": (0,0,10), "vel": (0, 0, 10)},

    ]
    ego_vel = (0,0,10)
    print(all(is_safe(EGO_POS, ego_vel, point["pos"], point["vel"]) for point in stationary_pts))
