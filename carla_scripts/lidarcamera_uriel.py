import carla
import random
from carla_scripts.carla_painter import CarlaPainter
from LiDAR.process_points import interlock
import math
import numpy as np
import open3d as o3d
import datetime

from queue import Queue
from queue import Empty
from matplotlib import cm
from PIL import Image
import cv2
import time
import pickle
from skimage.measure import label, regionprops
from scipy import stats
import matplotlib.pyplot as plt

VIRIDIS = np.array(cm.get_cmap('viridis').colors)
VID_RANGE = np.linspace(0.0,1.0, VIRIDIS.shape[0])

FILTER = True
RECORD = True
SAVE_RATE = 5
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

# must be a common factor of camera width/height. For 600x800 can be: 2, 4, 5, 8, 10, ...
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
        self.points = None
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

    def lidar_listener(self, data):
        data = np.array([[pt.point.x, pt.point.y, pt.point.z, pt.object_idx] for pt in data])
        self.lidar_queue.put(data)
    
    def camera_segmentation_listener(self, data):
        #data.convert(carla.ColorConverter.CityScapesPalette)
        im_array = np.copy(np.frombuffer(data.raw_data, dtype=np.dtype("uint8")))
        im_array = np.reshape(im_array, (data.height, data.width, 4))
        im_array = im_array[:, :, :3][:, :, ::-1][:, :, 0]
        self.camera_segmentation_queue.put(im_array)

    def get_rgb_point_velocities(self):
        # see carla/PythonAPI/examples/lidar_to_camera.py for detailed documentation
        
        xyz_pts = np.copy(self.points)
        xyz_pts[:,[0,1]] = xyz_pts[:,[1,0]]
        velocities, colors = self.get_lidar_point_velocities(xyz_pts)
        
        imageW = self.blueprint_camera.get_attribute("image_size_x").as_int()
        imageH = self.blueprint_camera.get_attribute("image_size_y").as_int()
        imageFOV = self.blueprint_camera.get_attribute("fov").as_float()
        focal = imageW / (2.0 * np.tan(imageFOV * np.pi / 360.0))
        K = np.identity(3)
        K[0,0] = K[1,1] = focal
        K[0,2] = imageW / 2.0
        K[1,2] = imageH / 2.0

        intensity = np.array(self.points[:,3])
        lidar_points = np.array(self.points[:,:3]).T
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
        intensity = intensity.T

        points_in_range = (points_2D[:,0] > 0.0) & (points_2D[:,0] < imageW) & (points_2D[:,1] > 0.0) & (points_2D[:,1] < imageH) & (points_2D[:,2] > 0.0)
        original_pts = xyz_pts[points_in_range]
        points_2D = points_2D[points_in_range]
        intensity = intensity[points_in_range]
        colors = colors[points_in_range] * 255
        velocities = velocities[points_in_range]
        
        points_2D[:,[0,1]] = points_2D[:,[1,0]]
        return points_2D, velocities, original_pts

    def get_lidar_point_velocities(self, points):
        vehicle_actor_ids_to_velocity = {x.actor_id: self.world.get_actor(x.actor_id).get_velocity() for x in self.vehicles}
        
        colors = np.zeros((points.shape[0], 3))
        velocities = np.zeros((points.shape[0], 3))
        
        for i, point in enumerate(points):
            if point[3] in vehicle_actor_ids_to_velocity:
                velocity = vehicle_actor_ids_to_velocity[point[3]]
                velocities[i] = [velocity.x, velocity.y, velocity.z]
                colors[i] = [1,0,0]
            else:
                velocities[i] = [0,0,0]
                colors[i] = [0,1,1]
        return velocities, colors

    def get_objects_with_lidar_pts(self, labeled_image, downscaled_segmented_img, segmented_img, factor):
        grid = [[[label] for label in row] for row in labeled_image]
        """object_to_lidar_pts = {}
        for obj in regionprops(labeled_image):
            if obj.area >= 500 / (factor * factor):
                object_to_lidar_pts[obj.label] = []"""
        rgb_pts, velocities, lidar_pts = self.get_rgb_point_velocities()
        for i in range(len(rgb_pts)):
            x,y,_ = rgb_pts[i].astype(np.int)
            true_segmented_label = segmented_img[x, y]
            segmented_label = downscaled_segmented_img[x//factor, y//factor]
            """
            object_label = labeled_image[x//factor,y//factor]
            if true_segmented_label == segmented_label and object_label in object_to_lidar_pts:
                object_to_lidar_pts[object_label].append([lidar_pts[i], velocities[i], [x,y]])
            """
            if true_segmented_label == segmented_label and len(grid[x//factor][y//factor]) == 1:
                grid[x//factor][y//factor].append([lidar_pts[i], velocities[i]])
        #print([[int(len(x) > 1) for x in row] for row in grid])
        return grid
        
    def extract_lane_points(self, points):
        valid_lane_points = ((points[:, 0] > -2) & (points[:,0] < 2) & (points[:,1] > 0) & (points[:,2] > -1.2))
        lane_points = points[valid_lane_points]
        ego_dist_to_front = self.ego_vehicle.bounding_box.extent.y
        lane_points[:,1] -= ego_dist_to_front
        return lane_points

    def downsample_image(self, image, width, height, factor):
        image = image.reshape(height//factor,factor,width//factor,factor).swapaxes(1,2).reshape(height//factor,width//factor,-1)
        im, count = stats.mode(image, axis=2)
        return im[:,:,0]

    def show_points(self, background_img, labeled_image, factor):
        plt.imshow(background_img)
        plt.show()
        objs = self.get_objects_with_lidar_pts(labeled_image, factor)
        for obj in objs:
            if len(objs[obj]) == 0:
                print("OBJ HAS NO LIDAR PTS")
            else:
                img = np.copy(background_img)
                for pos_3d, vel, pos_2d in objs[obj]:
                    x,y = pos_2d
                    img[x,y] = 250
                plt.imshow(img)
                plt.show()

    def label(self, segmentation_img):
        segmentation_img = np.copy(segmentation_img)
        ground_indices = (segmentation_img == TAG["roadline"]) | (segmentation_img == TAG["sidewalk"]) | (segmentation_img == TAG["road"])
        segmentation_img[ground_indices] = TAG["road"]
        return label(segmentation_img, connectivity=1)

    def sensor_processor(self, frame_number):
        try:
            rgb_image = self.image_queue.get(True, 1.0)
            lidar_cloud = self.lidar_queue.get(True, 1.0)
            segmentation_image = self.camera_segmentation_queue.get(True, 1.0)

        except Empty:
            print("Warning: Sensor data missed")
            return
        
        self.points = lidar_cloud

        down_sampled_segmentation_image = self.downsample_image(segmentation_image, 800, 600, DOWNSAMPLE_FACTOR)
        labeled_image = self.label(down_sampled_segmentation_image)
        grid = self.get_objects_with_lidar_pts(labeled_image, down_sampled_segmentation_image, segmentation_image, DOWNSAMPLE_FACTOR)
        #self.show_points(segmentation_image, labeled_image, DOWNSAMPLE_FACTOR)
        dic = {"grid": grid,
                "labeled_image": labeled_image,
                "factor": DOWNSAMPLE_FACTOR}

        self.count += 1
        if self.count % SAVE_RATE == 0 and RECORD:
            #fil = open(f"lidar/car_{frame_number}", 'ab')
            #pickle.dump(dic, fil)
            #fil.close()
            #Image.fromarray(labeled_image.astype(np.uint8)).save(f"lidar/labeled_image_{frame_number}.png")
            Image.fromarray(rgb_image).save(f"lidar/rgb_im_{frame_number}.png")
            Image.fromarray(segmentation_image).save(f"lidar/segmentation_{frame_number}.png")
            Image.fromarray(down_sampled_segmentation_image).save(f"lidar/downsampled_{frame_number}.png")
            rgb_pts, velocities, lidar_pts = self.get_rgb_point_velocities()
            for i in range(len(rgb_pts)):
                x,y,_ = rgb_pts[i].astype(np.int)
                segmentation_image[x,y] = 0
            Image.fromarray(segmentation_image).save(f"lidar/rgb_im_lidar_{frame_number}.png")
            pass
        
        # some weird transformations have to happen for the visualizer:
        # flip the x and y coords 
        """xyz_pts = np.copy(self.points)
        xyz_pts[:,[0,1]] = xyz_pts[:,[1,0]]
        xyz_pts = xyz_pts[:,:3]

        lane_points = self.extract_lane_points(xyz_pts)
        velocities, colors = self.get_lidar_point_velocities(lane_points)

        ego_vel = self.ego_vehicle.get_velocity() 
        ego_speed = (ego_vel.x ** 2 + ego_vel.y ** 2 + ego_vel.z ** 2) ** .5
        self.certificate_result, self.closest_point = interlock(lane_points, velocities, ego_speed, TIMESTEP_DELTA)"""
        self.points = None

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
            # initialize one painter
            try:
                self.painter = CarlaPainter('localhost', 8089)
            except Exception as e:
                print("NO PAINTER")
                self.painter = None

            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()

            # set synchronous mode
            previous_settings = self.world.get_settings()
            self.world.apply_settings(carla.WorldSettings(
                synchronous_mode=True,
                fixed_delta_seconds=1.0 / 20.0))
            self.tm = self.client.get_trafficmanager()
            self.tm.set_synchronous_mode(True)
            self.tm_port = self.tm.get_port()
            print('tm port is: ', self.tm_port)
            
            # create the specific case
            results = case(self.tm_port, self.client.apply_batch_sync, self.world)
            self.vehicles.append(results[1])
            if not results[0].error:
                self.ego_vehicle = self.world.get_actor(results[0].actor_id)
                self.ego_id = results[0].actor_id
            else:
                print('spawn ego error, exit')
                self.ego_vehicle = None
                return

            # attach a camera and a lidar to the ego vehicle
            self.blueprint_camera = self.world.get_blueprint_library().find('sensor.camera.rgb')
            #self.blueprint_camera.set_attribute('image_size_x', '640')
            #self.blueprint_camera.set_attribute('image_size_y', '480')
            self.blueprint_camera.set_attribute('fov', '110')
            #self.blueprint_camera.set_attribute('sensor_tick', '0.1')
            transform_camera = carla.Transform(carla.Location(x=.0, z=1.8))
            self.camera = self.world.spawn_actor(self.blueprint_camera, transform_camera, attach_to=self.ego_vehicle)
            self.camera.listen(lambda data: self.camera_listener(data))

            blueprint_lidar = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            # these specs follow the velodyne vlp32 spec
            blueprint_lidar.set_attribute('range', '200')
            #blueprint_lidar.set_attribute('range', '30')
            blueprint_lidar.set_attribute('rotation_frequency', '20')
            blueprint_lidar.set_attribute('channels', '32')
            blueprint_lidar.set_attribute('lower_fov', '-25')
            blueprint_lidar.set_attribute('upper_fov', '15')
            blueprint_lidar.set_attribute('points_per_second', '578560')
            #blueprint_lidar.set_attribute('atmosphere_attenuation_rate', '0.0')
            self.blueprint_lidar = blueprint_lidar
            transform_lidar = carla.Transform(carla.Location(x=0.0, z=1.8))
            self.lidar = self.world.spawn_actor(blueprint_lidar, transform_lidar, attach_to=self.ego_vehicle)
            self.lidar.listen(lambda data: self.lidar_listener(data))

            # attach semantic segmentation camera to ego
            blueprint_camera_segmentation = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            blueprint_camera_segmentation.set_attribute('fov', '110')
            transform_camera_segmentation = carla.Transform(carla.Location(x=.0, z=1.8))
            self.camera_segmentation = self.world.spawn_actor(blueprint_camera_segmentation, transform_camera_segmentation, attach_to=self.ego_vehicle)
            self.camera_segmentation.listen(lambda data: self.camera_segmentation_listener(data))

            # tick to generate these actors in the game world
            self.world.tick()

            # save vehicles' trajectories to draw in the frontend
            trajectories = [[]]
            self.obstacle = None

            #self.obstacle = self.spawn_obstacle()

            while (True):
                self.world.tick()
                self.frame += 1
                self.sensor_processor(self.frame)
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

if __name__ == "__main__":
    lidarcamera = Lidarcamera()
    lidarcamera.main(egoAndCarDrivingAutoPilot)
