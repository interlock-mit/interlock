import numpy as np
import random

class GroundDetector:
    def __init__(self, data, n_ransac_trials=20, closeness_delta=0.1, max_height=0.25, max_tilt=0.25):
        """data is a np array of shape (n_points, 3)"""
        self.n_ransac_trials = n_ransac_trials
        self.closeness_delta = closeness_delta
        self.max_height = max_height # meters below which the three points in the plane must be
        self.max_tilt = max_tilt
        self.ground_plane = self.get_ground_plane(data)

    def on_ground(self, point):
        return self.ground_plane.contains(point, self.closeness_delta)

    def get_ground_plane(self, data):
        """Simple implementation of RANSAC algorithm, subject to necessary conditions
           for a "ground-like" plane"""
        max_points_fit = 0
        best_plane = None
        num_trials = 0
        while num_trials < self.n_ransac_trials:
            # three random points
            p1, p2, p3 = [data[random.randrange(np.shape(data)[0])] for _ in range(3)]
           
            if not all(p[2]<self.max_height for p in [p1, p2, p3]):
                # plane are too high
                continue
            plane = Plane(p1, p2, p3)
            if not plane.small_angle(self.max_tilt):
                # plane is too tilted
                continue

            # plane is ground-like--compute how many points it contains
            num_trials += 1
            points_fit = sum([plane.contains(pt, self.closeness_delta) for pt in data])
            if points_fit > max_points_fit:
                max_points_fit = points_fit
                best_plane = plane
        
        return best_plane

class Plane:
    def __init__(self, p1, p2, p3):
        p1 = np.column_stack(p1)
        p2 = np.column_stack(p2)
        p3 = np.column_stack(p3)
        vec1 = p2 - p1
        vec2 = p3 - p1
        self.normal_vector = np.cross(vec1, vec2)
        self.point = p1

    def small_angle(self, threshhold):
        """Returns whether or not this plane has vertical angle 
        less than some threshhold. (We don't compute the 
        actual angle, but use the magnitude of the x and y
        components as a proxy). Low threshholds require more 'flat'
        planes. """
        normalized_vec = self.normal_vector/np.linalg.norm(self.normal_vector)
        x, y = normalized_vec[0][0], normalized_vec[0][1]
        mag = (x**2+y**2)**0.5
        return mag < threshhold

    def contains(self, p, closeness_delta=0.1):
        p = np.column_stack(p)
        p_from_point_in_plane = p - self.point
        try:
            v_dot_n = np.dot(p_from_point_in_plane[0], self.normal_vector[0])
            n_dot_n = 1.0*np.dot(self.normal_vector[0], self.normal_vector[0])
            proj_onto_normal_vector = v_dot_n/n_dot_n * self.normal_vector
        except ZeroDivisionError:
            # normal vector is zero -- 3 points weren't coplanar
            return False

        return np.linalg.norm(proj_onto_normal_vector) <= closeness_delta
     