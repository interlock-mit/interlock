import carla
import random
from carla_scripts.carla_painter import CarlaPainter
from LiDAR.process_points import interlock
import math
import numpy as np
import open3d as o3d
import datetime
from LiDAR.segmentation_pipeline import pipeline
from collections import defaultdict

from queue import Queue
from queue import Empty
from matplotlib import cm
from PIL import Image
import cv2
import time
import pickle
from skimage.measure import label as label_objects
from scipy import stats
import matplotlib.pyplot as plt
from statistics import mode

VIRIDIS = np.array(cm.get_cmap('viridis').colors)
VID_RANGE = np.linspace(0.0,1.0, VIRIDIS.shape[0])

FILTER = True
RECORD = True
SAVE_RATE = 1
TIMESTEP_DELTA = .05

TAG = {"unlabeled": 0,
        "building": 1,
        "fence": 2,
        "other": 3,
        "pedestrian": 4,
        "pole": 5,
        "roadline": 6,
        "road": 7,
        "sidewalk": 8,
        "vegetation": 9,
        "vehicles": 10,
        "wall": 11,
        "trafficsign": 12,
        "sky": 13,
        "ground": 14,
        "bridge": 15,
        "railtrack": 16,
        "guardrail": 17,
        "trafficlight": 18,
        "static": 19,
        "dynamic": 20,
        "water": 21,
        "terrain": 22}

TAG_ID = {TAG[key]: key for key in TAG}

# must be a common factor of camera width/height. For 400x1200 can be: 2, 4, 5, 8, 10, 16, 20 ...
DOWNSAMPLE_FACTOR = 10

class Lidarcamera:
    def __init__(self):
        self.world = None
        self.ego_vehicle = None
        self.lidar = None
        self.camera = None
        self.blueprint_camera = None
        self.image_camera = None
        self.obstacle = None
        self.client = None
        self.scanned_angle = 0
        self.point_idx = 0
        self.print_once = False
        self.blueprint_lidar = None
        self.lidar_certificate = None
        self.certificate_result = True

        # new
        self.tm = None
        self.vehicles = []
        self.closest_point = float('inf')
        self.count = 0
        self.image_queue = Queue()
        self.lidar_queue = Queue()
        self.camera_segmentation_queue = Queue()
        self.frame = 0
        self.camera_segmentation = None

        # degrees
        self.certificate_angular_bounds = {"up": 5,
                "down": -4,
                "left": -7,
                "right": 7}
        # meters
        self.certificate_distance = 20
        # in coordinates relative to lidar origin
        self.lane_bounds = {'left': -1, 'right': 1, 'down':-1.5, 'up':1.5}
        self.diffs = {"rl": 0.1, "ud": 0.5, "row_dev": 0.1}
        self.row_heights = []

    def d_to_rad(self, deg):
        return deg*np.pi/180.0

    def display_points(self, pts, origin):
        disp_pts = pts.copy()
        disp_pts[:,0] += origin.x
        disp_pts[:,1] += origin.y
        disp_pts[:,2] += origin.z
        #translate = disp_pts + 0.05
        #out = np.stack((disp_pts, translate), axis=1).tolist()
        self.painter.draw_points(disp_pts.tolist())
   
    def angle(self, pt, origin=0):
        if isinstance(pt, carla.Vector3D):
            return math.atan2(pt.y, pt.x)
        else:
            return math.atan2(pt[1], pt[0])

    def camera_listener(self, data):
        im_array = np.copy(np.frombuffer(data.raw_data, dtype=np.dtype("uint8")))
        im_array = np.reshape(im_array, (data.height, data.width, 4))
        im_array = im_array[:, :, :3][:, :, ::-1]
        self.image_queue.put(im_array)
    
    def camera_segmentation_listener(self, data):
        #data.convert(carla.ColorConverter.CityScapesPalette)
        im_array = np.copy(np.frombuffer(data.raw_data, dtype=np.dtype("uint8")))
        im_array = np.reshape(im_array, (data.height, data.width, 4))
        im_array = im_array[:, :, :3][:, :, ::-1][:, :, 0]
        self.camera_segmentation_queue.put(im_array)

    def lidar_listener(self, data):
        channel = 0
        points_seen = 0
        points = data.get_point_count(channel)
        last_theta = None
        last_phi = 0
        new_data = []
        for pt in data:
            new_data.append([pt.point.x, pt.point.y, pt.point.z, pt.object_idx, pt.object_tag])
            points_seen += 1
            theta = math.degrees(math.atan(pt.point.y/(pt.point.x+1e-20)))
            phi = math.degrees(math.atan(math.sqrt(pt.point.x**2 + pt.point.y**2)/pt.point.z))
            if last_theta is None:
                diff_theta = 0
            else:
                diff_theta = theta-last_theta
            diff_phi = abs(abs(phi)-abs(last_phi))
            #print("horizontal: ", diff_theta)
            #if diff_phi > 1e-2:
            #    print("vertical: ", diff_phi)
            MIN_DIFF = 0.4
            if diff_theta > MIN_DIFF:
                R = 10000
                for shift in range(1, int(diff_theta/MIN_DIFF) + 1):
                    new_theta = last_theta + (shift * MIN_DIFF)
                    x = R * math.sin(math.radians(phi)) * math.cos(math.radians(new_theta))
                    y = R * math.sin(math.radians(phi)) * math.sin(math.radians(new_theta))
                    z = R * math.cos(math.radians(phi))
                    new_data.append([x, y, z, 0, TAG["sky"]])
            last_theta = theta
            last_phi = phi
            if points == points_seen:
                channel += 1
                points = data.get_point_count(channel)
                points_seen = 0
                last_theta = None
        self.lidar_queue.put(np.array(new_data))
    def get_rgb_point_velocities(self, points):
        # see carla/PythonAPI/examples/lidar_to_camera.py for detailed documentation
        
        xyz_pts = np.copy(points)
        #xyz_pts[:,[0,1]] = xyz_pts[:,[1,0]]
        velocities = self.get_lidar_point_velocities(xyz_pts)
        tags = xyz_pts[:,4]
        
        imageW = self.blueprint_camera.get_attribute("image_size_x").as_int()
        imageH = self.blueprint_camera.get_attribute("image_size_y").as_int()
        imageFOV = self.blueprint_camera.get_attribute("fov").as_float()
        focal = imageW / (2.0 * np.tan(imageFOV * np.pi / 360.0))
        K = np.identity(3)
        K[0,0] = K[1,1] = focal
        K[0,2] = imageW / 2.0
        K[1,2] = imageH / 2.0

        lidar_points = np.array(points[:,:3]).T
        lidar_points = np.r_[lidar_points, [np.ones(lidar_points.shape[1])]]

        lidar_to_world = self.lidar.get_transform().get_matrix()
        world_points = np.dot(lidar_to_world, lidar_points)

        world_to_camera = np.array(self.camera.get_transform().get_inverse_matrix())
        sensor_points = np.dot(world_to_camera, world_points)

        points_in_camera_coords = np.array([sensor_points[1], -sensor_points[2], sensor_points[0]])

        points_2D_unnormalized = np.dot(K, points_in_camera_coords)
        points_2D = np.array([
            points_2D_unnormalized[0,:] / (points_2D_unnormalized[2,:]+0.0001),
            points_2D_unnormalized[1,:] / (points_2D_unnormalized[2,:]+0.0001),
            points_2D_unnormalized[2,:]])
        points_2D = points_2D.T

        points_in_range = (points_2D[:,0] > 0.0) & (points_2D[:,0] < imageW) & (points_2D[:,1] > 0.0) & (points_2D[:,1] < imageH) & (points_2D[:,2] > 0.0)
        original_pts = xyz_pts[points_in_range]
        points_2D = points_2D[points_in_range]
        velocities = velocities[points_in_range]
        tags = tags[points_in_range]
        
        points_2D[:,[0,1]] = points_2D[:,[1,0]]
        return points_2D, velocities, original_pts, tags

    def get_lidar_point_velocities(self, points):
        vehicle_actor_ids_to_velocity = {x.actor_id: self.world.get_actor(x.actor_id).get_velocity() for x in self.vehicles}
        velocities = np.zeros((points.shape[0], 3))
        
        for i, point in enumerate(points):
            if point[3] in vehicle_actor_ids_to_velocity:
                velocity = vehicle_actor_ids_to_velocity[point[3]]
                velocities[i] = [velocity.x, velocity.y, velocity.z]
            else:
                velocities[i] = [0,0,0]
        return velocities

    def image_to_grid(self, lidar_cloud, segmented_image, factor):

        def label_image(image):
            # label roadline and road as same object
            road_indices = (image == TAG["roadline"]) | (image == TAG["road"])
            image[road_indices] = TAG["road"]
            return label_objects(image, connectivity=1)

        labeled_image = label_image(segmented_image)
        rgb_pts, velocities, lidar_pts, tags = self.get_rgb_point_velocities(lidar_cloud)
        # grid of cells with value [label, list_of_lidar_points]
        grid = [[[None, []] for i in range(len(labeled_image[0])//factor)] for j in range(len(labeled_image)//factor)]

        # list mapping object id's to the spegmented tag
        object_id_to_tag = defaultdict(int)
        # place lidar points on grid
        for i in range(len(rgb_pts)):
            x,y,_ = rgb_pts[i].astype(np.int)
            object_id = labeled_image[x,y]
            segmented_tag_truth = tags[i]
            segmented_tag_projection = segmented_image[x,y]
            # some lidar points are projected onto an incorrect object (due to rounding/resolution issues)
            if segmented_tag_truth != segmented_tag_projection:
                object_id = None
                min_dist = float('inf')
                # if we can find the object nearby, assign the lidar point to the closest object. if not, skip the lidar point
                for s_x in range(max(0, x - DOWNSAMPLE_FACTOR), min(x + DOWNSAMPLE_FACTOR + 1, segmented_image.shape[0])):
                    for s_y in range(max(0, y - DOWNSAMPLE_FACTOR), min(y + DOWNSAMPLE_FACTOR + 1, segmented_image.shape[1])):
                        dist = (x-s_x)**2 + (y-s_y)**2
                        if segmented_image[s_x, s_y] == segmented_tag_truth and dist < min_dist:
                            min_dist = dist
                            object_id = labeled_image[s_x, s_y]
                if object_id is None:
                    #print((x,y), "Lidar Truth: ", TAG_ID[segmented_tag_truth], "Predicted: ",TAG_ID[segmented_tag_projection])
                    continue
            object_id_to_tag[object_id] = segmented_tag_truth
            grid[x//factor][y//factor][1].append([lidar_pts[i][:3], velocities[i], [x,y], object_id])

        # assign labels to each cell in grid and removing lidar pts not corresponding to that label
        for row in range(len(grid)):
            for column in range(len(grid[row])):
                cell = grid[row][column]
                label_to_points = defaultdict(list)
                for point in cell[1]:
                    label = point.pop()
                    label_to_points[label].append(point)
                # no lidar points, so set label to the id of middle pixel inside cell
                if len(label_to_points) == 0:
                    cell[0] = labeled_image[row*factor + factor//2, column*factor + factor//2]
                # object with most lidar points gets the cell labelled
                else:
                    label = max(label_to_points, key=lambda k: len(label_to_points[k]))
                    cell[0] = label
                    cell[1] = label_to_points[label]
        
        # new labeled image consists of the labels of the grid we made
        labeled_image = np.array([[label for label,_ in row] for row in grid])

        #print([[int(len(x[1]) > 0) for x in row] for row in grid])
        return grid, labeled_image, object_id_to_tag
        
    def extract_lane_points(self, points):
        valid_lane_points = ((points[:, 0] > -2) & (points[:,0] < 2) & (points[:,1] > 0) & (points[:,2] > -1.2))
        lane_points = points[valid_lane_points]
        ego_dist_to_front = self.ego_vehicle.bounding_box.extent.y
        lane_points[:,1] -= ego_dist_to_front
        return lane_points

    def sensor_processor(self):
        try:
            # sensors send data asynchronously, but they capture it at the same rate. Wait to receive data from all sensors
            rgb_image = self.image_queue.get(True, 1.0)
            lidar_cloud = self.lidar_queue.get(True, 1000.0)
            segmentation_image = self.camera_segmentation_queue.get(True, 1.0)

        except Empty:
            print("Warning: Sensor data missed")
            return

        grid, labeled_image, object_id_to_tag = self.image_to_grid(lidar_cloud, segmentation_image, DOWNSAMPLE_FACTOR)

        ground_ids = [object_id for object_id in object_id_to_tag if object_id_to_tag[object_id] == TAG["road"]]
        sky_ids = [object_id for object_id in object_id_to_tag if object_id_to_tag[object_id] == TAG["sky"]]
        ego_vel = self.ego_vehicle.get_velocity()
        result = pipeline(grid, labeled_image, DOWNSAMPLE_FACTOR, [ego_vel.x, ego_vel.y, ego_vel.z], ground_ids, sky_ids)
        self.count += 1

        # saving frame to debug..
        if self.count % SAVE_RATE == 0 and RECORD:
            check_img = np.zeros(rgb_image.shape, dtype=np.uint8)
            id_to_color = {}
            delta = 1
            rgb_image_copy = np.copy(rgb_image)
            segmented_image_copy = np.copy(segmentation_image)
            text_X = 10
            text_Y = 30
            colors = {
                "Spatial Check": [255,0,0],
                "Velocity Check": [0,0,255],
                "Density Check": [128,0,128],
                "Ground Check": [255,165,0],
                "Collision Check": [0,0,0]
            }
            bad_squares = set()
            for test in result:
                if result[test]["success"]:
                    color = [0,255,0]
                else:
                    color = colors[test]
                cv2.putText(rgb_image_copy, test, (text_X,text_Y), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                text_Y += 30
                for pt in result[test]["bad_points"]:
                    x,y = pt
                    for dx in range(x-delta, x+delta+1):
                        for dy in range(y-delta, y+delta+1):
                            rgb_image_copy[max(min(dx,rgb_image.shape[0]-1),0),max(min(dy,rgb_image.shape[1]-1),0)] = color
                            bad_squares.add((dx//DOWNSAMPLE_FACTOR, dy//DOWNSAMPLE_FACTOR))
                for pt in result[test]["good_points"]:
                    x,y = pt
                    for dx in range(x-delta, x+delta+1):
                        for dy in range(y-delta, y+delta+1):
                            rgb_image_copy[max(min(dx,rgb_image.shape[0]-1),0),max(min(dy,rgb_image.shape[1]-1),0)] = [0,0,255]
                            bad_squares.add((dx//DOWNSAMPLE_FACTOR, dy//DOWNSAMPLE_FACTOR))
            for j in range(len(grid)):
                for i in range(len(grid[j])):
                    cell = grid[j][i]
                    if len(cell[1]) == 0 or (j,i) in bad_squares:
                        continue
                    label, lidar_pt = cell
                    x = j * DOWNSAMPLE_FACTOR + DOWNSAMPLE_FACTOR//2
                    y = i * DOWNSAMPLE_FACTOR + DOWNSAMPLE_FACTOR//2
                    if label not in id_to_color:
                        id_to_color[label] = list(np.random.random(size=3) * 256)
                    for dx in range(x-delta, x+delta+1):
                        for dy in range(y-delta, y+delta+1):
                            rgb_image_copy[max(min(dx,rgb_image.shape[0]-1),0),max(min(dy,rgb_image.shape[1]-1),0)] = [255,255,255]
            pic =  np.vstack((rgb_image_copy, rgb_image))
            Image.fromarray(pic).save(f"lidar/pic_{self.frame}.png")

    def spawn_obstacle(self, index=0, dist=15):
        if self.obstacle is not None:
            self.obstacle.destroy()

        bp = self.world.get_blueprint_library().filter("vehicle.*")[index]
        forward_vec = self.ego_vehicle.get_transform().get_forward_vector()
        mag = math.sqrt(forward_vec.x**2 + forward_vec.y**2)
        dx = dist*forward_vec.x/mag
        dy = dist*forward_vec.y/mag

        s = self.ego_vehicle.get_transform()
        s.location.z += 2.0
        s.location.x += dx
        s.location.y += dy
        s.rotation.roll = 0.0
        s.rotation.pitch = 0.0
        return self.world.try_spawn_actor(bp, s)

    def main(self, case):
        try:
            # initialize one painter for CarlaViz connection
            try:
                self.painter = CarlaPainter('localhost', 8089)
            except Exception as e:
                print("NO PAINTER")
                self.painter = None

            # connect to Carla engine
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            previous_settings = self.world.get_settings()
            self.world.apply_settings(carla.WorldSettings(
                synchronous_mode=True,
                fixed_delta_seconds=1.0 / 20.0))
            self.tm = self.client.get_trafficmanager()
            self.tm.set_synchronous_mode(True)
            self.tm_port = self.tm.get_port()
            
            # create interlock starting scenario
            results = case(self.tm_port, self.client.apply_batch_sync, self.world)
            self.vehicles.extend(results[1:])
            if not results[0].error:
                self.ego_vehicle = self.world.get_actor(results[0].actor_id)
                self.ego_id = results[0].actor_id
            else:
                print('spawn ego error, exit')
                self.ego_vehicle = None
                return

            # attach a camera and a lidar to the ego vehicle
            self.blueprint_camera = self.world.get_blueprint_library().find('sensor.camera.rgb')
            self.blueprint_camera.set_attribute('image_size_x', '1200')
            self.blueprint_camera.set_attribute('image_size_y', '400')
            self.blueprint_camera.set_attribute('fov', '120')
            #self.blueprint_camera.set_attribute('sensor_tick', '0.1')
            transform_camera = carla.Transform(carla.Location(x=.0, z=1.8))
            self.camera = self.world.spawn_actor(self.blueprint_camera, transform_camera, attach_to=self.ego_vehicle)
            self.camera.listen(lambda data: self.camera_listener(data))

            self.blueprint_lidar = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            self.blueprint_lidar.set_attribute('range', '200000')
            self.blueprint_lidar.set_attribute('rotation_frequency', '20')
            #self.blueprint_lidar.set_attribute('channels', '64') # 0.6/0.4 resolution
            self.blueprint_lidar.set_attribute('channels', '192') # 0.2/0.2 resolution
            #self.blueprint_lidar.set_attribute('channels', '402')# 0.1/0.1 resolution
            self.blueprint_lidar.set_attribute('lower_fov', '-25')
            self.blueprint_lidar.set_attribute('upper_fov', '15')
            #self.blueprint_lidar.set_attribute('points_per_second', '1200000')# 0.6/0.4 resolution
            self.blueprint_lidar.set_attribute('points_per_second', '7200000')# 0.2/0.2 resolution
            #self.blueprint_lidar.set_attribute('points_per_second', '28860000')# 0.1/0.1 resolution
            transform_lidar = carla.Transform(carla.Location(x=0.0, z=1.8))
            self.lidar = self.world.spawn_actor(self.blueprint_lidar, transform_lidar, attach_to=self.ego_vehicle)
            self.lidar.listen(lambda data: self.lidar_listener(data))

            # attach semantic segmentation camera to ego
            blueprint_camera_segmentation = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            blueprint_camera_segmentation.set_attribute('image_size_x', str(self.blueprint_camera.get_attribute("image_size_x").as_int()))
            blueprint_camera_segmentation.set_attribute('image_size_y', str(self.blueprint_camera.get_attribute("image_size_y").as_int()))
            blueprint_camera_segmentation.set_attribute('fov', str(self.blueprint_camera.get_attribute("fov").as_float()))
            self.camera_segmentation = self.world.spawn_actor(blueprint_camera_segmentation, transform_camera, attach_to=self.ego_vehicle)
            self.camera_segmentation.listen(lambda data: self.camera_segmentation_listener(data))

            # tick to generate these actors in the game world
            self.world.tick()

            #self.obstacle = self.spawn_obstacle()

            while (True):
                self.frame += 1
                self.world.tick()
                self.sensor_processor()
                strs = []
                locs = []

                loc = self.ego_vehicle.get_location()
                strs.append("{:.2f}, ".format(loc.x) + "{:.2f}".format(loc.y) \
                        + ", {:.2f}".format(loc.z))
                locs.append([loc.x, loc.y, loc.z + 10.0])

                # strs.append( "{:.2f}, ".format(loc.x-self.certificate_distance) + "{:.2f}".format(loc.y) \
                #         + ", {:.2f}".format(loc.z))
                strs.append('closest point: '+ str(self.closest_point)[:4])

                locs.append([loc.x-self.certificate_distance, loc.y, loc.z + 10.0])

                strs.append("Certificate GOOD" if self.certificate_result else "Certificate BAD")
                if not self.certificate_result:
                    # print('brake')
                    # vehicle_actors = [self.world.get_actor(x.actor_id) for x in self.vehicles]

                    # print([v.get_velocity().x for v in vehicle_actors])
                    self.world.get_actor(self.ego_id).apply_control(carla.VehicleControl(brake=1.0))

                locs.append([loc.x-5, loc.y-5, loc.z + 20.0])
                self.painter.draw_texts(strs, locs, size=20)

        finally:
            if previous_settings is not None:
                self.world.apply_settings(previous_settings)
            if self.lidar is not None:
                self.lidar.stop()
                self.lidar.destroy()
            if self.camera is not None:
                self.camera.stop()
                self.camera.destroy()
            if self.camera_segmentation is not None:
                self.camera_segmentation.stop()
                self.camera_segmentation.destroy()
            if self.ego_vehicle is not None:
                self.ego_vehicle.destroy()
            if self.obstacle is not None:
                self.obstacle.destroy()
            self.client.apply_batch([carla.command.DestroyActor(x.actor_id) for x in self.vehicles])

def egoAndCarDrivingAutoPilot(tm_port, apply_batch, world):
    ego_transform = carla.Transform(carla.Location(x=120.07566833496, y=8.87075996, z=0.27530714869499207))
    vehicle_2_transform = carla.Transform(carla.Location(x=130.07566833496, y=8.87075996, z=0.27530714869499207))

    blueprints_vehicles = world.get_blueprint_library().filter("vehicle.*")
    blueprints_vehicles = [x for x in blueprints_vehicles if int(x.get_attribute('number_of_wheels')) == 4]
    # set ego vehicle's role name to let CarlaViz know this vehicle is the ego vehicle
    blueprints_vehicles[0].set_attribute('role_name', 'ego') # or set to 'hero'

    actor1 = carla.command.SpawnActor(blueprints_vehicles[0], ego_transform).then(carla.command.SetAutopilot(carla.command.FutureActor, True, tm_port))
    actor2 = carla.command.SpawnActor(blueprints_vehicles[1], vehicle_2_transform).then(carla.command.SetAutopilot(carla.command.FutureActor, True, tm_port))
    batch = [actor1, actor2]
    
    results = apply_batch(batch, True)
    return results

def egoCrashingIntoStationaryCar(tm_port, apply_batch, world):
    ego_transform = carla.Transform(carla.Location(x=110.07566833496, y=8.87075996, z=0.27530714869499207))
    vehicle_2_transform = carla.Transform(carla.Location(x=160.07566833496, y=8.87075996, z=0.27530714869499207))

    blueprints_vehicles = world.get_blueprint_library().filter("vehicle.*")
    blueprints_vehicles = [x for x in blueprints_vehicles if int(x.get_attribute('number_of_wheels')) == 4]
    # set ego vehicle's role name to let CarlaViz know this vehicle is the ego vehicle
    blueprints_vehicles[0].set_attribute('role_name', 'ego') # or set to 'hero'

    actor1 = carla.command.SpawnActor(blueprints_vehicles[0], ego_transform)
    actor2 = carla.command.SpawnActor(blueprints_vehicles[1], vehicle_2_transform)
    batch = [actor1, actor2]
    
    results = apply_batch(batch, True)
    world.get_actor(results[0].actor_id).set_target_velocity(carla.Vector3D(5,0,0))
    return results


def egoCrashingIntoWalkingPed(tm_port, apply_batch, world):
    ego_transform = carla.Transform(carla.Location(x=110.07566833496, y=8.87075996, z=0.27530714869499207))
    ped_transform = carla.Transform(carla.Location(x=160.07566833496, y=2.87075996, z=0.27530714869499207), carla.Rotation(0, 90, 0))

    blueprints = world.get_blueprint_library().filter("vehicle.*")
    blueprints_vehicles = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
    blueprints_vehicles[0].set_attribute('role_name', 'ego') # or set to 'hero'

    blueprints_peds = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 2]

    actor1 = carla.command.SpawnActor(blueprints_vehicles[0], ego_transform)
    actor2 = carla.command.SpawnActor(blueprints_peds[0], ped_transform)
    batch = [actor1, actor2]
    
    results = apply_batch(batch, True)
    world.get_actor(results[0].actor_id).set_target_velocity(carla.Vector3D(20,0,0))
    world.get_actor(results[1].actor_id).enable_constant_velocity(carla.Vector3D(2,0,0))

    return results

def otherLane(tm_port, apply_batch, world):
    ego_transform = carla.Transform(carla.Location(x=110.07566833496, y=8.87075996, z=0.27530714869499207))
    vehicle_2_transform = carla.Transform(carla.Location(x=160.07566833496, y=4.87075996, z=0.27530714869499207))

    blueprints = world.get_blueprint_library().filter("vehicle.*")
    blueprints_vehicles = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
    blueprints_vehicles[0].set_attribute('role_name', 'ego') # or set to 'hero'

    actor1 = carla.command.SpawnActor(blueprints_vehicles[0], ego_transform)
    actor2 = carla.command.SpawnActor(blueprints_vehicles[1], vehicle_2_transform)
    batch = [actor1, actor2]
    
    results = apply_batch(batch, True)
    world.get_actor(results[0].actor_id).set_target_velocity(carla.Vector3D(5,0,0))

    return results

def egoAndCarAtIntersection(tm_port, apply_batch, world):
    ego_transform = carla.Transform(carla.Location(x=210.07566833496, y=9.7075996, z=0.17530714869499207))
    vehicle_2_transform = carla.Transform(carla.Location(x=217.07566833496, y=9.7075996, z=0.17530714869499207))
    vehicle_3_transform = carla.Transform(carla.Location(x=230.07566833496, y=9.7075996, z=0.17530714869499207))

    blueprints_vehicles = world.get_blueprint_library().filter("vehicle.*")
    blueprints_vehicles = [x for x in blueprints_vehicles if int(x.get_attribute('number_of_wheels')) == 4]
    # set ego vehicle's role name to let CarlaViz know this vehicle is the ego vehicle
    blueprints_vehicles[0].set_attribute('role_name', 'ego') # or set to 'hero'

    actor1 = carla.command.SpawnActor(blueprints_vehicles[0], ego_transform)
    actor2 = carla.command.SpawnActor(blueprints_vehicles[1], vehicle_2_transform)
    actor3 = carla.command.SpawnActor(blueprints_vehicles[0], vehicle_3_transform).then(carla.command.SetAutopilot(carla.command.FutureActor, True, tm_port))
    batch = [actor1, actor2, actor3]
    
    results = apply_batch(batch, True)
    return results

if __name__ == "__main__":
    lidarcamera = Lidarcamera()
    lidarcamera.main(egoAndCarDrivingAutoPilot)
