MAX_DECEL = 10
MIN_DIST = 1

# MAX_RL = 0.1
# MAX_UD = 0.5
# MAX_ROW_DEV = 0.1

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

    def decel(vel, rate):
        x, y, z = vel
        def helper(val):
            result = max(val - rate, 0) if val > 0 else min(val + rate, 0):
            return result
        return (helper(x), helper(y), helper(z))

    def helper(ego_accel, other_accel):
        cur_time = 0
        cur_pos, cur_vel = ego_pos, ego_vel
        other_cur_pos, other_cur_vel = other_pos, other_vel
        while cur_time < stopping_time:
            if dist(cur_pos, other_cur_pos) < min_dist:
                return False

            # increment velocities then positions 
            cur_vel = decel(cur_vel, ego_accel)
            other_cur_vel = decel(other_cur_vel, other_accel)
            cur_pos = add(mult(cur_vel, timestep), cur_pos)
            other_cur_pos = add(mult(other_cur_vel, timestep), other_cur_pos)
            cur_time += timestep
        return True
    if not helper(MAX_DECEL, 0):
        return False
    if not helper(MAX_DECEL, MAX_DECEL):
        return False
    return True

def interlock(ego_pos, ego_vel, points, timestep=DEFAULT_TIMESTEP, min_dist=MIN_DIST, max_decel=MAX_DECEL):
    """
    ego_pos   : position of the vehicle (x_pos, y_pos, z_pos)
    ego_vel   : velocity of the vehicle (x_vel, y_vel, z_vel)
    points    : a list of points and velocities given by the certificate,
                each of the form {"pos": (x_pos,y_pos,z_pos), "vel": (x_vel,y_vel,z_vel)}
    timestep  : the time increment to consider
    min_dist  : min distance we can be away from a point
    max_decel : max deceleration of the vehicle

    Returns True if the vehicle is at a safe distance from all points, False otherwise
    """
    for point in points:
        other_pos, other_vel = point["pos"], point["vel"]
        if not is_safe(ego_pos, ego_vel, other_pos, other_vel, timestep, min_dist, max_decel):
            return False
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
    print(interlock(EGO_POS, ego_vel, stationary_pts))
