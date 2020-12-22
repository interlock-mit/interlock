import carla
import random
from carla_scripts.carla_painter import CarlaPainter
from LiDAR.process_points import interlock
import math
import numpy as np
import open3d as o3d

FILTER = True
RECORD = False
TIMESTEP_DELTA = .05

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
        self.timestep = None

        # new
        self.tm = None
        self.vehicles = []
        self.closest_point = float('inf')
        self.count = 0

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
        pass

    def lidar_listener(self, data):
        n = data.channels
        # the raw data contains info on the number of points per channel,
        # and points are ordered based on channel, so we can calculate the
        # ending indicies for all channels
        # counts = [0]
        # for i in range(n):
        #     counts.append(counts[i] + data.get_point_count(i))

        # massage the points into the right shape
        points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
        points = np.array(np.reshape(points, (int(points.shape[0] / 4),4)))

        # some weird transformations have to happen for the visualizer:
        # flip the x and y coords 
        points[:,[0,1]] = points[:,[1,0]]
        # points[:,1] *= -1
        # points[:,2] *= -1


        # calculate the angle of each point, so that we can make sure they're
        # ordered correctly
        # points = np.hstack((points, np.arctan2(points[:,1], points[:,0]).reshape(points.shape[0], 1)))

        # translate to the lidar unit's location
        # loc = self.lidar.get_location()
        # points[:,0] += loc.x
        # points[:,1] += loc.y
        # points[:,2] += loc.z

    

        # #TODO
        # # right now the angles are in world coordinates centered about the 
        # # lidar, we need to make the angles relative to the car's orientation
        if self.points is not None:
            self.points = np.append(self.points, points, axis=0)
        else:
            self.points = points

        # the lidar does not complete a full rotation every tick, so we keep
        # track of the scanned angle to know when it has finished one rotation
        self.scanned_angle += 1
        num_pts = 0
        for channel in range(n):
            num_pts += data.get_point_count(channel)
        # print('num pts: ', num_pts, ' shape of poitns is', points.shape, 'shape of self is', self.points.shape)

        if self.scanned_angle >= 2:
            self.count += 1
            xyz_pts = self.points[:,:3]
            # print(xyz_pts.shape)
            
            xyz_filtered = xyz_pts[((xyz_pts[:, 0] > -2) & (xyz_pts[:,0] < 2) & (xyz_pts[:,1] > 0) & (xyz_pts[:,2] > -1.2))]
            # testing other filters
            # xyz_filtered = xyz_pts[(xyz_pts[:,2] > -1)]
            # xyz_filtered = xyz_pts[((xyz_pts[:, 0] > -2) & (xyz_pts[:,0] < 2))]
            # xyz_filtered = xyz_pts[(xyz_pts[:,1] > 0)]
            vs = [self.world.get_actor(x.actor_id) for x in self.vehicles]
            # print(vs)
            

            # TODO: add z coordinate
            locs = [(v.get_location().x - v.bounding_box.extent.x,
                    v.get_location().x + v.bounding_box.extent.x,
                    v.get_location().y - v.bounding_box.extent.y,
                    v.get_location().y + v.bounding_box.extent.y) for v in vs]
            # print(locs)
            lidar_loc = self.lidar.get_location()

            # print('lidar at', lidar_loc)

            # print('filtered', xyz_filtered)


            colors = np.zeros(xyz_filtered.shape)
            velocities = np.zeros(xyz_filtered.shape)

            # velocities of every car relative to LiDAR
            car_velocities = [v.get_velocity() for v in vs]
            ego_vel = self.ego_vehicle.get_velocity() 
            ego_speed = (ego_vel.x ** 2 + ego_vel.y ** 2 + ego_vel.z ** 2) ** .5

            ego_dist_to_front = self.ego_vehicle.bounding_box.extent.y
            xyz_filtered[:,1] -= ego_dist_to_front
            for i, point in enumerate(xyz_filtered):
                # TODO: use vehicle transform to do non-jank rotation transform
                px = point[1] + lidar_loc.x
                py = point[0] + lidar_loc.y
                for j, (leftX, rightX, backY, frontY) in enumerate(locs):
                    if leftX - .3 <= px <= rightX + .3 and backY - .3 <= py <= frontY + .3:
                        colors[i] = [1, 0, 0]
                        # TODO: make this not 1d 
                        # velocities are relative to the stationary location of the car
                        velocities[i] = [car_velocities[j].x, car_velocities[j].y, car_velocities[j].z]
                        break
                    else:
                        colors[i] = [0, 1, 1]
                        velocities[i] = [0, 0, 0]

            if self.count % 10 == 0 and RECORD:

                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(xyz_pts.astype(np.float64))
                o3d.io.write_point_cloud(f"lidar/{data.frame}.ply", pcd)


                pcd = o3d.geometry.PointCloud()
                pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
                pcd.points = o3d.utility.Vector3dVector(xyz_filtered.astype(np.float64))
                o3d.io.write_point_cloud(f"lidar/{data.frame}_filtered.ply", pcd)
            
            self.certificate_result, self.closest_point = interlock(xyz_filtered, velocities, ego_speed, TIMESTEP_DELTA)

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

    def main(self, case):
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
                fixed_delta_seconds=TIMESTEP_DELTA))
            self.tm = self.client.get_trafficmanager()
            self.tm.set_synchronous_mode(True)
            self.tm_port = self.tm.get_port()
            print('tm port is: ', self.tm_port)
            
            # create the specific case
            results, callback = case(self.tm_port, self.client.apply_batch_sync, self.world)
            self.callback = callback
            print('results', results)
            self.vehicles.append(results[1])
            if not results[0].error:
                self.ego_vehicle = self.world.get_actor(results[0].actor_id)
                self.ego_id = results[0].actor_id
            else:
                print('spawn ego error, exit')
                self.ego_vehicle = None
                return
            self.timestep = 0

            # batch = [carla.command.SpawnActor(blueprints_vehicles[0], ego_transform).then(carla.command.SetAutopilot(carla.command.FutureActor, False))]
            # results = self.client.apply_batch_sync(batch, False)
            # if not results[0].error:
            #     self.ego_vehicle = self.world.get_actor(results[0].actor_id)
            # else:
            #     print('spawn ego error, exit')
            #     self.ego_vehicle = None
            #     return

            # attach a camera and a lidar to the ego vehicle
            blueprint_camera = self.world.get_blueprint_library().find('sensor.camera.rgb')
            #blueprint_camera.set_attribute('image_size_x', '640')
            #blueprint_camera.set_attribute('image_size_y', '480')
            blueprint_camera.set_attribute('fov', '110')
            #blueprint_camera.set_attribute('sensor_tick', '0.1')
            transform_camera = carla.Transform(carla.Location(x=.0, z=1.8))
            self.camera = self.world.spawn_actor(blueprint_camera, transform_camera, attach_to=self.ego_vehicle)
            self.camera.listen(lambda data: self.camera_listener(data))

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

                # strs.append( "{:.2f}, ".format(loc.x-self.certificate_distance) + "{:.2f}".format(loc.y) \
                #         + ", {:.2f}".format(loc.z))
                strs.append('closest point: '+ str(self.closest_point)[:4])

                locs.append([loc.x-self.certificate_distance, loc.y, loc.z + 10.0])

                strs.append("Certificate GOOD" if self.certificate_result else "Certificate BAD")
                if not self.certificate_result:
                    print('brake at ', self.timestep)
                    # vs = [self.world.get_actor(x.actor_id) for x in self.vehicles]

                    # print(self.world.get_actor(self.ego_id).get_velocity().x)
                    self.world.get_actor(self.ego_id).apply_control(carla.VehicleControl(brake=1.0))

                locs.append([loc.x-5, loc.y-5, loc.z + 20.0])
                self.painter.draw_texts(strs, locs, size=20)
                ##@trajectories[0].append([ego_location.x, ego_location.y, ego_location.z])

                ## draw trajectories
                #painter.draw_polylines(trajectories)

                ## draw ego vehicle's velocity just above the ego vehicle
                #ego_velocity = ego_vehicle.get_velocity()
                #velocity_str = "{:.2f}, ".format(ego_velocity.x) + "{:.2f}".format(ego_velocity.y) \
                #        + ", {:.2f}".format(ego_velocity.z)
                self.callback(self.world, self.timestep)
                self.timestep += 1
                if self.timestep == 100:
                    print('now')



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
            self.client.apply_batch([carla.command.DestroyActor(x.actor_id) for x in self.vehicles])


def do_nothing(world, timestep):
    return

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
    return results, do_nothing

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
    world.get_actor(results[0].actor_id).set_target_velocity(carla.Vector3D(15,0,0))
    return results, do_nothing


def egoCrashingIntoWalkingPed(tm_port, apply_batch, world):
    ego_transform = carla.Transform(carla.Location(x=110.07566833496, y=8.87075996, z=0.27530714869499207))
    ped_transform = carla.Transform(carla.Location(x=160.07566833496, y=3.87075996, z=0.27530714869499207), carla.Rotation(0, 77, 0))

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

    def callback(my_world, timestep):
        my_world.get_actor(results[1].actor_id).set_target_velocity(carla.Vector3D(2,0,0))

    return results, callback

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

    return results, do_nothing

def egoCrashingIntoBrakingCar(tm_port, apply_batch, world):
    ego_transform = carla.Transform(carla.Location(x=140.07566833496, y=8.87075996, z=0.27530714869499207))
    vehicle_2_transform = carla.Transform(carla.Location(x=160.07566833496, y=8.87075996, z=0.27530714869499207))


    blueprints_vehicles = world.get_blueprint_library().filter("vehicle.*")
    blueprints_vehicles = [x for x in blueprints_vehicles if int(x.get_attribute('number_of_wheels')) == 4]
    # set ego vehicle's role name to let CarlaViz know this vehicle is the ego vehicle
    blueprints_vehicles[0].set_attribute('role_name', 'ego') # or set to 'hero'

    actor1 = carla.command.SpawnActor(blueprints_vehicles[0], ego_transform)
    actor2 = carla.command.SpawnActor(blueprints_vehicles[1], vehicle_2_transform)
    batch = [actor1, actor2]
    
    results = apply_batch(batch, True)
    world.get_actor(results[0].actor_id).set_target_velocity(carla.Vector3D(15,0,0))
    world.get_actor(results[1].actor_id).set_target_velocity(carla.Vector3D(15,0,0))

    def callback(my_world, timestep):
        if timestep > 50:
            world.get_actor(results[1].actor_id).apply_control(carla.VehicleControl(brake=.8))

    return results, callback

def egoCrashingIntoPedInLane(tm_port, apply_batch, world):
    ego_transform = carla.Transform(carla.Location(x=110.07566833496, y=8.87075996, z=0.27530714869499207))
    ped_transform = carla.Transform(carla.Location(x=170.07566833496, y=8.87075996, z=1.27530714869499207))


    blueprints_vehicles = world.get_blueprint_library().filter("vehicle.*")
    blueprints_vehicles = [x for x in blueprints_vehicles if int(x.get_attribute('number_of_wheels')) == 4]
    # set ego vehicle's role name to let CarlaViz know this vehicle is the ego vehicle
    blueprints_vehicles[0].set_attribute('role_name', 'ego') # or set to 'hero'

    actor1 = carla.command.SpawnActor(blueprints_vehicles[0], ego_transform)

    blueprints_peds = world.get_blueprint_library().filter("walker.*")

    actor2 = carla.command.SpawnActor(blueprints_peds[0], ped_transform)
    batch = [actor1, actor2]
    
    results = apply_batch(batch, True)
    world.get_actor(results[0].actor_id).set_target_velocity(carla.Vector3D(15,0,0))
    world.get_actor(results[1].actor_id).set_target_velocity(carla.Vector3D(2,0,0))

    def callback(my_world, timestep):
        control = carla.WalkerControl()
        my_world.get_actor(results[1].actor_id).set_target_velocity(carla.Vector3D(2,0,0))

    return results, callback

def egoCrashingIntoBackwardCar(tm_port, apply_batch, world):
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
    world.get_actor(results[0].actor_id).set_target_velocity(carla.Vector3D(15,0,0))
    world.get_actor(results[1].actor_id).set_target_velocity(carla.Vector3D(-5,0,0))

    return results, do_nothing


if __name__ == "__main__":
    lidarcamera = Lidarcamera()
    lidarcamera.main(egoCrashingIntoWalkingPed)
