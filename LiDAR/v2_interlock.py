MAX_DECEL = 15
MIN_DIST = 10

# MAX_RL = 0.1
# MAX_UD = 0.5
# MAX_ROW_DEV = 0.1

EGO_POS = (0,0,0)
DEFAULT_TIMESTEP = .2


def add(vec1, vec2):
    return (vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2])
def mult(vec, factor):
    return (vec[0] * factor, vec[1] * factor, vec[2] * factor)
def dist(vec1, vec2):
    return ((vec1[0]-vec2[0])**2 + (vec1[1]-vec2[1])**2 + (vec1[2]-vec2[2])**2)**.5
def loc(pos, vel, t):
    return (pos[0]+t*vel[0], pos[1]+t*vel[1], pos[2]+t*vel[2])

def loc_range(pos, vel, t, max_accel, max_decel):
    x = pos[0] + t*vel[0]
    x_range = (x + 0.5*max_accel[0]*(t**2), x + 0.5*max_decel[0]*(t**2))
    y = pos[1] + t*vel[1]
    y_range = (y + 0.5*max_accel[1]*(t**2), y + 0.5*max_decel[1]*(t**2))
    z = pos[2] + t*vel[2]
    z_range = (z + 0.5*max_accel[2]*(t**2), z + 0.5*max_decel[2]*(t**2))
    return x_range, y_range, z_range

def avg_vel(object_points):
    sum_vel = (0, 0, 0)
    for (pos, vel) in object_points:
        sum_vel = (sum_vel[0] + vel[0], sum_vel[1] + vel[1], sum_vel[2] + vel[2])
    n = len(object_points)
    return (sum_vel[0]/n, sum_vel[1]/n, sum_vel[2]/n)

def max_diff_vel(object_points):
    min_x, max_x, min_y, max_y, min_z, max_z = (None)*6
    for (pos, vel) in object_points:
        if min_x is None or vel[0] < min_x:
            min_x = vel[0]
        if max_x is None or vel[0] > max_x:
            max_x = vel[0]
        if min_y is None or vel[1] < min_y:
            min_y = vel[1]
        if max_y is None or vel[1] > max_y:
            max_y = vel[1]
        if min_z is None or vel[2] < min_z:
            min_z = vel[2]
        if max_z is None or vel[2] > max_z:
            max_z = vel[2]
    return (max_x - min_x, max_y - min_y, max_z - min_z)

def object_same_speed(object_points, epsilon):
    ''' epsilon is the maximum variability in velocity we are willing to tolerate '''
    (x_diff, y_diff, z_diff) = max_diff_vel(object_points)
    return x_diff < epsilon and y_diff < epsilon and z_diff < epsilon

def within_bounding_box(pos, bounding_box):
    (x_min, x_max, y_min, y_max, z_min, z_max) = bounding_box
    return x_min <= pos[0] <= x_max and y_min <= pos[1] <= y_max and z_min <= pos[2] <= z_max

def get_object_points(all_lidar_points, bounding_box):
    return list(filter(all_lidar_points, lambda (pos, vel): within_bounding_box(pos, bounding_box)))

def is_safe(ego_pos, ego_vel, other_pos, other_vel, timestep=DEFAULT_TIMESTEP, min_dist=MIN_DIST, max_decel=MAX_DECEL):
    cur_speed = dist(ego_vel, (0,0,0))
    stopping_time = cur_speed/max_decel

    def decel(vel, rate):
        x, y, z = vel
        def helper(val):
            result = max(val - rate, 0) if val > 0 else min(val + rate, 0)
            return result
        # print('vel rate', vel, rate)
        # print('new vel', (helper(x), helper(y), helper(z)))
        return (helper(x), helper(y), helper(z))

    def helper(ego_accel, other_accel):
        cur_time = 0
        cur_pos, cur_vel = ego_pos, ego_vel
        other_cur_pos, other_cur_vel = other_pos, other_vel
        while cur_time < stopping_time:
            if dist(cur_pos, other_cur_pos) < min_dist:
                return False
            # print(cur_pos, other_cur_pos)
            # increment velocities then positions 
            # print('ego vel', cur_vel)
            # print('other vel', other_cur_vel)
            cur_vel = decel(cur_vel, ego_accel * timestep)
            other_cur_vel = decel(other_cur_vel, other_accel * timestep)
            cur_pos = add(mult(cur_vel, timestep), cur_pos)
            other_cur_pos = add(mult(other_cur_vel, timestep), other_cur_pos)
            cur_time += timestep
        return True
    if not helper(MAX_DECEL, 0):
        return False
    # print('now doing both decel')
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
        {"pos": (0,0,15), "vel": (0, 0, 0)},
        {"pos": (0,3,15), "vel": (0, 0, 0)},
        {"pos": (0,2,15), "vel": (0, 0, 0)},
        {"pos": (-1,1,15), "vel": (0, 0, 0)},
        {"pos": (1,0,15), "vel": (0, 0, 0)},
        {"pos": (2,-1,15), "vel": (0, 0, 0)},
    ]
    driving_fwd_pts = [
        {"pos": (0,0,15), "vel": (0, 0, 60)},
        {"pos": (0,3,15), "vel": (0, 0, 60)},
        {"pos": (0,2,15), "vel": (0, 0, 60)},
        {"pos": (-1,1,15), "vel": (0, 0, 60)},
        {"pos": (1,0,15), "vel": (0, 0, 60)},
        {"pos": (2,-1,15), "vel": (0, 0, 60)},
    ]
    driving_fwd_close_pts = [
        {"pos": (0,0,5), "vel": (0, 0, 60)},
        {"pos": (0,3,5), "vel": (0, 0, 60)},
        {"pos": (0,2,5), "vel": (0, 0, 60)},
        {"pos": (-1,1,5), "vel": (0, 0, 60)},
        {"pos": (1,0,5), "vel": (0, 0, 60)},
        {"pos": (2,-1,5), "vel": (0, 0, 60)},
    ]
    ego_vel = (0,0,60)
    print(interlock(EGO_POS, ego_vel, stationary_pts))
    print(interlock(EGO_POS, ego_vel, driving_fwd_pts))
    print(interlock(EGO_POS, ego_vel, driving_fwd_close_pts))

