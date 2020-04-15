#!/usr/bin/env python

"""
Racecar driver. 
Includes logic to transform between LiDAR and image coordinates.
Must be run in same directory as a controller and interlock
implementation.
"""

import rospy
import time
from sensor_msgs.msg import PointCloud2, PointField, LaserScan, CompressedImage
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import utils
import numpy as np
import yaml
import cv2

import controller
import interlock
import math

def rad(deg):
    return deg*math.pi/180

im_theta_y = rad(16)
im_theta_x = rad(33) # degrees
lidar_theta_x = rad(180) # degrees
lidar_theta_y = rad(15) # degrees
LIDAR_ANGLE_SPANS = (lidar_theta_x, lidar_theta_y)

def pt_to_lidar_angle(im_point, im_dims):
    """
    TODO

    im_point : (x, y) where x is the number of pixels to the
                right of the image's top left, y is the number
                of pixels from the image's top--(0,0) is TL corner
    im_dims : (width, height) in number of pixels
    """
    x, y = im_point
    width, height = im_dims

    def get_angle(half_dim, pt, im_angle):
        """TODO"""
        dim_ratio = float(pt - half_dim)/half_dim
        return math.atan2(dim_ratio * math.tan(im_angle), 1)

    return get_angle(width/2.0, x, im_theta_x), \
           get_angle(height/2.0, y, im_theta_y)
           

def lidar_angle_to_dist(theta_x, theta_y, angle_spans, lidar):
    x_span, y_span = angle_spans

    def angle_to_bucket(theta, n_buckets, angle_span):
        """
        Finds closest bucket that this angle falls into. 
        'Bucket' is a row or column. Returns 0 for the 
        top or leftmost bucket.

        angle_span is the angle, in degrees, away from 
        the center point to which the n_buckets buckets 
        are equally spaced. In other words, the angle
        from the center of the outermost bucket.
        """
        return int(round((angle_span + theta) * (n_buckets) / (2.0 * angle_span)))

    row = min(len(lidar) - 1, 
              max(0, len(lidar) - angle_to_bucket(theta_y, 
                                                  len(lidar), 
                                                  y_span)))

    def get_center_i(row):
        # get the index of the lidar point IN FRONT OF YOU (x>0),
        # whose y value is smallest (the actual center)
        center_y, center = float('inf'), 0
        for i, (x, y, _) in enumerate(row):
            if abs(y) < abs(center_y) and x > 0:
                center_y = y
                center = i
        return center

    center_i = get_center_i(lidar[row])
    shift = int(center_i - (len(lidar[row])-1)/2.0)

    col = angle_to_bucket(theta_x, len(lidar[row]), x_span) + shift

    return lidar[row][col]

def transform(pt, im_dims, lidar):
    temp = pt_to_lidar_angle(pt, im_dims)
    return lidar_angle_to_dist(temp[0], temp[1], LIDAR_ANGLE_SPANS, lidar)

class VeloscanProcessor:
    def __init__(self):
        self.velodyne_sub = rospy.Subscriber("/racecar/velodyne_points", PointCloud2, self.velodyne_cb, queue_size=1)
        self.cert_pub = rospy.Publisher("/racecar/cert_points", Marker, queue_size=1)
        self.snow_pub = rospy.Publisher("/racecar/snow_points", Marker, queue_size=1)
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.first_scan = False
        self.scan_data = None
        self.last_scan_t = None
        self.centroid = (int(.5*320), int(.5*240))
        self.image_np = None
        self.np_arr = None
        self.hsv = None
        self.mask = None

    def image_to_laser(self, centroid, laser_data):
        laser_vertical_lower = -15.0 * np.pi / 180.0
        laser_vertical_upper = -laser_vertical_lower
        laser_fov_left = -30.0 * np.pi / 180.0
        laser_fov_right = -laser_fov_left
        h = 480.0
        w = 640.0
        nearest_ring = laser_vertical_upper - 30.0*np.pi/180.0 * (centroid[1] / h) 

    def velodyne_cb(self, msg):
        rings = {x:[] for x in range(0,16)}
        for p in pc2.read_points(msg, field_names = ("x", "y", "z", "ring"), skip_nans=True):
            rings[p[3]].append(list(p[0:3]))
        pt = transform(self.centroid, (640, 480), rings)

        m = Marker()
        m.header = utils.make_header("velodyne")
        m.type = 8
        m.color.r = 1.
        m.color.g = 0.
        m.color.b = 0.
        m.color.a = 1.
        m.scale.x = .05
        m.scale.y = .05
        
        m.points = [Point(pt[0], pt[1], pt[2])]
        self.cert_pub.publish(m)

    return
        c = controller.controller(data)
        print ""
        print c.action
        r = interlock.interlock(c)
        print ""
        pts = []

        m = Marker()
        m.header = utils.make_header("velodyne")
        m.type = 8
        m.color.r = 1. if not r else 0.
        m.color.g = 0. if not r else 1.
        m.color.b = 0.
        m.color.a = 1.
        m.scale.x = .02
        m.scale.y = .02
        
        m.points = [Point(x[0], x[1], x[2]) for x in c.points]
        self.cert_pub.publish(m)

        snow_pts = []
        n = Marker()
        n.header = utils.make_header("velodyne")
        n.type = 8
        n.color.r = .75
        n.color.g = 0. 
        n.color.b = 0.75
        n.color.a = 1.
        n.scale.x = .02
        n.scale.y = .02
        
        n.points = [Point(x[0], x[1], x[2]) for x in c.snow_points]
        self.snow_pub.publish(n)

    def run(self):
        cap = cv2.VideoCapture(1)
        while (not rospy.is_shutdown()):
            ret, self.image_np = cap.read()
            t = time.time()
            self.hsv = cv2.cvtColor(self.image_np, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([90, 30, 90])
            upper_blue = np.array([250, 230, 250])
            self.mask = cv2.inRange(self.hsv, lower_blue, upper_blue)
            self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            M = cv2.moments(self.mask)
            try:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
            except ZeroDivisionError:
                cx = 0
                cy = 0
        self.centroid = (cx, cy)
        cv2.circle(self.image_np, self.centroid, 5, (0, 0, 255), thickness=-1) 
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', self.image_np)[1]).tostring()
            self.image_pub.publish(msg)
        cap.release()
        

if __name__ == "__main__":
    rospy.init_node("veloscan_processor")
    s = VeloscanProcessor()
    s.run()
    rospy.spin()
