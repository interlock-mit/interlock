import carla
import random
from carla_scripts.carla_painter import CarlaPainter
from LiDAR.interlock import interlock
import math
import numpy as np

FILTER = True

class Lidarcamera:
    def __init__(self):
        self.world = None
        self.ego_vehicle = None
        self.lidar = None
        self.camera = None
        self.obstacle = None
        self.client = None
        self.scanned_angle = 0
        self.points = None
        self.point_idx = 0
        self.print_once = False
        self.blueprint_lidar = None

        self.lidar_certificate = None
        self.certificate_result = True

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

    def generate_certificate(self):
        # assuming at this point the points are ordered such that the first
        # point in the list is at the rear of the car

        # calc which rows are needed
        upper_fov = self.blueprint_lidar.get_attribute('upper_fov').as_float()
        lower_fov = self.blueprint_lidar.get_attribute('lower_fov').as_float()

        n = self.blueprint_lidar.get_attribute('channels').as_int()
        deg_per_row = (upper_fov - lower_fov) / (n-1)
        deg_per_pt = 360 / len(self.points[0])

        bot_idx = 31-int((self.certificate_angular_bounds["down"]-lower_fov)/deg_per_row)
        top_idx = 31-(int((self.certificate_angular_bounds["up"]-lower_fov)/deg_per_row) + 1)
        left_idx = int((self.certificate_angular_bounds["left"]+180)/deg_per_pt)
        right_idx = int((self.certificate_angular_bounds["right"]+180)/deg_per_pt)+1

        # the height of the row at the minimum dist
        self.row_heights = [self.certificate_distance*math.tan(self.d_to_rad(upper_fov-deg_per_row*i)) for i in range(top_idx, bot_idx+1)]
        self.certificate = [self.points[i][left_idx:right_idx,:3] for i in range(top_idx, bot_idx+1)]
        # filter thigns that are too close
        if FILTER:
            for i in range(len(self.certificate)):
                self.certificate[i] = self.certificate[i][~(self.certificate[i][:,0]>-self.certificate_distance)]
        
        self.display_points(np.vstack(self.certificate), self.lidar.get_location())
        self.certificate_result = self.check_certificate()

    def check_certificate(self):
        # transform here will depend on spawn loc
        for i in range(len(self.certificate)):
            self.certificate[i][:,0] *= -1  # now forward is positive
            self.certificate[i][:,1] *= -1  # now right is positive
            self.certificate[i] = self.certificate[i].tolist()
        result = interlock(self.certificate_distance, self.lane_bounds["left"],
                self.lane_bounds["right"], self.lane_bounds["up"], 
                self.lane_bounds["down"], self.diffs["rl"], self.diffs["ud"],
                self.diffs["row_dev"], self.row_heights, self.certificate)
        return result


   

    def angle(self, pt, origin=0):
        if isinstance(pt, carla.Vector3D):
            return math.atan2(pt.y, pt.x)
        else:
            return math.atan2(pt[1], pt[0])

    def camera_listener(self, data):
        pass

    def lidar_listener(self, data):
        n = data.channels
        # the raw data contains info on the number of points per channel,
        # and points are ordered based on channel, so we can calculate the
        # ending indicies for all channels
        counts = [0]
        for i in range(n):
            counts.append(counts[i] + data.get_point_count(i))

        # massage the points into the right shape
        points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
        points = np.array(np.reshape(points, (int(points.shape[0] / 3),3)))

        # some weird transformations have to happen for the visualizer:
        # flip the x and y coords 
        points[:,[0,1]] = points[:,[1,0]]

        # mirror the y and z coords
        points[:,1] *= -1
        points[:,2] *= -1

        # calculate the angle of each point, so that we can make sure they're
        # ordered correctly
        points = np.hstack((points, np.arctan2(points[:,1], points[:,0]).reshape(points.shape[0], 1)))

        # translate to the lidar unit's location
        #loc = self.lidar.get_location()
        #points[:,0] += loc.x
        #points[:,1] += loc.y
        #points[:,2] += loc.z

        #TODO
        # right now the angles are in world coordinates centered about the 
        # lidar, we need to make the angles relative to the car's orientation


        # instantiate the points dict
        if self.points is None:
            self.points = {i:None for i in range(n)}

        # place the data by row into the points dict, using the indicies we
        # calculated earlier
        for i in range(n):
            if self.points[i] is None:
                self.points[i] = points[counts[i]:counts[i+1]].copy()
            else:
                self.points[i] = np.vstack((self.points[i], points[counts[i]:counts[i+1]]))

        # the lidar does not complete a full rotation every tick, so we keep
        # track of the scanned angle to know when it has finished one rotation
        self.scanned_angle += data.horizontal_angle
        if self.scanned_angle >= 2*math.pi:
            # calc angles and append

            # calc the forward angle of the car, then find the opposite of it
            # so that we know which angle the lidar scans should start at
            forward_vec = self.ego_vehicle.get_transform().get_forward_vector()
            forward_angle = self.angle(forward_vec)
            # rotate 180 degrees
            back_angle = forward_angle + np.pi
            if back_angle > np.pi:
                back_angle = -2*np.pi + back_angle
            elif back_angle < -np.pi:
                back_angle = 2*np.pi + back_angle

            for i in range(n):
                # sort each row by angle
                self.points[i] = self.points[i][np.argsort(self.points[i][:,3])]

                # now re-order the array so that the points start at the back
                # of the car and go around the front, from left to right

                # the first point in the array at this point in the code should
                # be at angle -pi. so, find the index at which we'd find the
                # back of the car
                num_pts = self.points[i].shape[0]
                rad_per_pt = 2*np.pi / num_pts
                back_index = abs(int((back_angle+np.pi) / rad_per_pt))
                indicies = np.array([(j+back_index) % num_pts for j in range(num_pts)])
                self.points[i] = self.points[i][indicies,:]

            #origin = self.lidar.get_location()
            #self.display_points(self.points[31][:300,:3], origin)
            self.generate_certificate()

            # draw the distance
            loc = self.ego_vehicle.get_location()
            line = [[[loc.x-self.certificate_distance, loc.y-5, loc.z],[loc.x-self.certificate_distance, loc.y+5, loc.z]]]
            self.painter.draw_polylines(line, width=0.2)

            self.scanned_angle = 0
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

    def main(self):
        try:
            # initialize one painter
            try:
                self.painter = CarlaPainter('localhost', 8089)
            except Exception as e:
                print("NO PAINTER")
                print(e)
                self.painter = None

            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()

            # set synchronous mode
            previous_settings = self.world.get_settings()
            self.world.apply_settings(carla.WorldSettings(
                synchronous_mode=True,
                fixed_delta_seconds=1.0 / 30.0))

            # spawn an ego vehicle
            spawn_points = self.world.get_map().get_spawn_points()
            blueprints_vehicles = self.world.get_blueprint_library().filter("vehicle.*")

            ego_transform = spawn_points[0]

            blueprints_vehicles = [x for x in blueprints_vehicles if int(x.get_attribute('number_of_wheels')) == 4]

            # set ego vehicle's role name to let CarlaViz know this vehicle is the ego vehicle
            blueprints_vehicles[0].set_attribute('role_name', 'ego') # or set to 'hero'
            batch = [carla.command.SpawnActor(blueprints_vehicles[0], ego_transform).then(carla.command.SetAutopilot(carla.command.FutureActor, False))]
            results = self.client.apply_batch_sync(batch, False)
            if not results[0].error:
                self.ego_vehicle = self.world.get_actor(results[0].actor_id)
            else:
                print('spawn ego error, exit')
                self.ego_vehicle = None
                return

            # attach a camera and a lidar to the ego vehicle
            #blueprint_camera = self.world.get_blueprint_library().find('sensor.camera.rgb')
            #blueprint_camera.set_attribute('image_size_x', '640')
            #blueprint_camera.set_attribute('image_size_y', '480')
            #blueprint_camera.set_attribute('fov', '110')
            #blueprint_camera.set_attribute('sensor_tick', '0.1')
            #transform_camera = carla.Transform(carla.Location(x=.3, z=1.8))
            #self.camera = self.world.spawn_actor(blueprint_camera, transform_camera, attach_to=self.ego_vehicle)
            #self.camera.listen(lambda data: self.camera_listener(data))

            blueprint_lidar = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            # these specs follow the velodyne vlp32 spec
            blueprint_lidar.set_attribute('range', '200')
            #blueprint_lidar.set_attribute('range', '30')
            blueprint_lidar.set_attribute('rotation_frequency', '10')
            blueprint_lidar.set_attribute('channels', '32')
            blueprint_lidar.set_attribute('lower_fov', '-25')
            blueprint_lidar.set_attribute('upper_fov', '15')
            blueprint_lidar.set_attribute('points_per_second', '578560')
            self.blueprint_lidar = blueprint_lidar
            transform_lidar = carla.Transform(carla.Location(x=0.0, z=1.8))
            self.lidar = self.world.spawn_actor(blueprint_lidar, transform_lidar, attach_to=self.ego_vehicle)
            self.lidar.listen(lambda data: self.lidar_listener(data))

            # tick to generate these actors in the game world
            self.world.tick()

            # save vehicles' trajectories to draw in the frontend
            trajectories = [[]]
            self.obstacle = None

            #self.obstacle = self.spawn_obstacle()

            while (True):
                self.world.tick()
                strs = []
                locs = []

                loc = self.ego_vehicle.get_location()
                strs.append("{:.2f}, ".format(loc.x) + "{:.2f}".format(loc.y) \
                        + ", {:.2f}".format(loc.z))
                locs.append([loc.x, loc.y, loc.z + 10.0])

                strs.append( "{:.2f}, ".format(loc.x-self.certificate_distance) + "{:.2f}".format(loc.y) \
                        + ", {:.2f}".format(loc.z))

                locs.append([loc.x-self.certificate_distance, loc.y, loc.z + 10.0])

                strs.append("Certificate GOOD" if self.certificate_result else "Certificate BAD")
                locs.append([loc.x-5, loc.y-5, loc.z + 20.0])
                self.painter.draw_texts(strs, locs, size=20)
                ##@trajectories[0].append([ego_location.x, ego_location.y, ego_location.z])

                ## draw trajectories
                #painter.draw_polylines(trajectories)

                ## draw ego vehicle's velocity just above the ego vehicle
                #ego_velocity = ego_vehicle.get_velocity()
                #velocity_str = "{:.2f}, ".format(ego_velocity.x) + "{:.2f}".format(ego_velocity.y) \
                #        + ", {:.2f}".format(ego_velocity.z)


        finally:
            if previous_settings is not None:
                self.world.apply_settings(previous_settings)
            if self.lidar is not None:
                self.lidar.stop()
                self.lidar.destroy()
            if self.camera is not None:
                self.camera.stop()
                self.camera.destroy()
            if self.ego_vehicle is not None:
                self.ego_vehicle.destroy()
            if self.obstacle is not None:
                self.obstacle.destroy()

if __name__ == "__main__":
    lidarcamera = Lidarcamera()
    lidarcamera.main()
