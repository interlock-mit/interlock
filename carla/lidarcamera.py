import carla
import random
from carla_painter import CarlaPainter
import math
import numpy as np

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

    def camera_listener(self, data):
        pass

    def lidar_listener(self, data):
        n = data.channels
        counts = []
        for i in range(n):
            if i == 0:
                counts.append(data.get_point_count(i))
            else:
                counts.append(counts[i-1] + data.get_point_count(i))

        points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
        points = np.array(np.reshape(points, (int(points.shape[0] / 3),3)))
        points[:,[0,1]] = points[:,[1,0]]
        loc = self.lidar.get_location()
        #loc = self.ego_vehicle.get_location()
        points[:,1] *= -1
        points[:,2] *= -1

        points[:,0] += loc.x
        points[:,1] += loc.y
        points[:,2] += loc.z
        points = points.tolist()
        if self.points is None:
            self.points = {i:[] for i in range(n)}
        for i in range(n):
            if i == 0:
                self.points[i]+=points[0:counts[i]]
            else:
                self.points[i]+=points[counts[i-1]:counts[i]]
        self.scanned_angle += data.horizontal_angle
        if self.scanned_angle > 2*math.pi:
            #self.painter.draw_points(self.points[self.point_idx%n])
            lines = self.points[self.point_idx%n]
            #for i in range(len(lines)):
            #    lines[i] = [lines[i], lines[i]]
            self.painter.draw_points(lines)
            self.points = None
            self.point_idx += 1
            self.scanned_angle = 0
        #self.painter.draw_polylines([points.tolist()], width=1)
        #origin = [ego_location.x, ego_location.y, ego_location.z + 10.0]
        #pts = []
        #for i in range(50):
        #    pts.append([origin[0]+random.random()*10,
        #            origin[1]+random.random()*10,
        #            origin[2]+random.random()*10])
        #painter.draw_points(pts)


    def spawn_obstacle(self, index=0, dist=10):
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
            self.painter = CarlaPainter('localhost', 8089)

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
            blueprint_camera = self.world.get_blueprint_library().find('sensor.camera.rgb')
            blueprint_camera.set_attribute('image_size_x', '640')
            blueprint_camera.set_attribute('image_size_y', '480')
            blueprint_camera.set_attribute('fov', '110')
            blueprint_camera.set_attribute('sensor_tick', '0.1')
            transform_camera = carla.Transform(carla.Location(x=.3, z=1.8))
            self.camera = self.world.spawn_actor(blueprint_camera, transform_camera, attach_to=self.ego_vehicle)
            self.camera.listen(lambda data: self.camera_listener(data))

            blueprint_lidar = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            blueprint_lidar.set_attribute('range', '30')
            blueprint_lidar.set_attribute('rotation_frequency', '10')
            blueprint_lidar.set_attribute('channels', '32')
            blueprint_lidar.set_attribute('lower_fov', '-30')
            blueprint_lidar.set_attribute('upper_fov', '30')
            blueprint_lidar.set_attribute('points_per_second', '56000')
            transform_lidar = carla.Transform(carla.Location(x=0.0, z=1.8))
            self.lidar = self.world.spawn_actor(blueprint_lidar, transform_lidar, attach_to=self.ego_vehicle)
            self.lidar.listen(lambda data: self.lidar_listener(data))

            # tick to generate these actors in the game world
            self.world.tick()

            # save vehicles' trajectories to draw in the frontend
            trajectories = [[]]
            self.obstacle = None

            self.obstacle = self.spawn_obstacle()

            while (True):
                self.world.tick()
                #ego_location = ego_vehicle.get_location()
                ##@trajectories[0].append([ego_location.x, ego_location.y, ego_location.z])

                ## draw trajectories
                #painter.draw_polylines(trajectories)

                ## draw ego vehicle's velocity just above the ego vehicle
                #ego_velocity = ego_vehicle.get_velocity()
                #velocity_str = "{:.2f}, ".format(ego_velocity.x) + "{:.2f}".format(ego_velocity.y) \
                #        + ", {:.2f}".format(ego_velocity.z)
                #painter.draw_texts([velocity_str],
                #            [[ego_location.x, ego_location.y, ego_location.z + 10.0]], size=20)


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
    l = Lidarcamera()
    l.main()
