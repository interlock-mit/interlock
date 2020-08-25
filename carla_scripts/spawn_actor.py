#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn NPCs into the simulation"""

import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('/home/interlock/carla_binary/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
import random
import json

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=0,
        type=int,
        help='number of vehicles (default: 0)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=0,
        type=int,
        help='number of walkers (default: 0)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Enanble')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enanble car lights')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.0)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)

        if args.sync:
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
                world.apply_settings(settings)
            else:
                synchronous_master = False

        blueprints = world.get_blueprint_library().filter(args.filterv)
        blueprintsWalkers = world.get_blueprint_library().filter(args.filterw)

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('t2')]
        
        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

############################# SPAWN ACTORS FOR TESTING  ##############################################################################################################
        print('Reference https://carla.readthedocs.io/en/latest/bp_library/ to see what actors you can spawn!')
        print('Rotations: Roll = Degrees around x-axis, Pitch = Degrees around y-axis, Yaw = Degrees around z-axis')
        print()
        print_bps = input('Do you want to view all the blueprints [y/n]? ')
        if print_bps.lower() == 'y' or print_bps.lower() == 'yes':
            print_atts = input('Print each blueprints attributes too [y/n]? ')
            blueprints = [bp for bp in world.get_blueprint_library().filter('*')]
            for blueprint in blueprints:
                print(blueprint.id)
                if print_atts.lower() == 'y' or print_atts.lower() == 'yes':
                    for attr in blueprint:
                        print('  - {}'.format(attr))
            print()
        print()

        load_preset = input('Load saved preset(s) [y/n]? ')
        if load_preset.lower() == 'y' or load_preset.lower() == 'yes':
            load_n = int(input('How many presets to load? '))
            for load_num in range(load_n):
                save_name = input('The name of preset '+str(load_num + 1)+' to load(ie. pack_snow or flying_trash)? ')
                with open('saves.json', 'r+') as fp:
                    saves = json.load(fp)
                    if save_name.startswith('pack'):
                        pack = saves[save_name]
                        for item in pack:
                            preset = pack[item]
                            this_bp = world.get_blueprint_library().filter(preset['name_tag'])[0]
                            bp_spawn = carla.Transform(carla.Location(x=preset['x'], y=preset['y'], z=preset['z']), carla.Rotation(pitch=preset['pitch'], yaw=preset['yaw'], roll=preset['roll']))
                            bp_type = preset['bp_type']
                            try:
                                attrs = preset['attributes']
                                for name, val in attrs.items():
                                    this_bp.set_attribute(name, val)
                            except KeyError:
                                print('Attributes dict does NOT exist. Please add one into this preset (even if its emtpy)')

                            if bp_type.lower() == 'v' or bp_type.lower() == 'vehicle':
                                vehicles_list.append(world.spawn_actor(this_bp, bp_spawn))
                                # so we can put at any pose any position and hold it there
                                vehicles_list[-1].set_simulate_physics(False)
                            elif bp_type.lower() == 'w' or bp_type.lower() == 'walker':
                                walkers_list.append(world.spawn_actor(this_bp, bp_spawn))
                    else:
                        preset = saves[save_name]
                        this_bp = world.get_blueprint_library().filter(preset['name_tag'])[0]
                        bp_spawn = carla.Transform(carla.Location(x=preset['x'], y=preset['y'], z=preset['z']), carla.Rotation(pitch=preset['pitch'], yaw=preset['yaw'], roll=preset['roll']))
                        bp_type = preset['bp_type']
                        try:
                            attrs = preset['attributes']
                            for name, val in attrs.items():
                                this_bp.set_attribute(name, val)
                        except KeyError:
                            print('Attributes dict does NOT exist. Please add one into the preset (even if its emtpy)')

                        if bp_type.lower() == 'v' or bp_type.lower() == 'vehicle':
                            vehicles_list.append(world.spawn_actor(this_bp, bp_spawn))
                            # so we can put at any pose any position and hold it there
                            vehicles_list[-1].set_simulate_physics(False)
                            print('Preset ' + str(load_num + 1) + ' has spawned in \n')
                        elif bp_type.lower() == 'w' or bp_type.lower() == 'walker':
                            walkers_list.append(world.spawn_actor(this_bp, bp_spawn))
                            print('Preset ' + str(load_num + 1) + ' has spawned in \n')
        ask_save = True
        ask_attr = True
        attributes = {}  # used to store attributes in a preset
        N = int(input('How many new actors would you like to spawn? '))
        for actor_num in range(N):
            cur_bp = input('Please enter a tag/name for actor ' + str(actor_num + 1) + ' from the blueprint library to spawn(ie. crossbike or model3): ')
            bp_type = input('Is actor ' + str(actor_num + 1) + ' a vehicle or walker type (check ref if unsure) [v/w]? ')
            coords = input('Please enter the spawn location of actor ' + str(actor_num + 1) + ' as a tuple (x, y, z): ')
            coords = eval(coords)

            rotations = input('Please enter the degrees of rotation as a tuple (roll, pitch, yaw): ')
            rotations = eval(rotations)

            this_bp = world.get_blueprint_library().filter(cur_bp)[0]
            bp_spawn = carla.Transform(carla.Location(x=coords[0], y=coords[1], z=coords[2]), carla.Rotation(pitch=rotations[1], yaw=rotations[2], roll=rotations[0]))
            
            if ask_attr:
                edit_attr = input('Do you want to edit this any of this actors attributes (MUST BE MODIFIABLE + check REF)[y/n]? ')
                if edit_attr.lower() == 'y' or edit_attr.lower() == 'yes':
                    attributes = {}
                    attr_N = int(input('How many attributes will you like to edit? '))
                    for attr_num in range(attr_N):
                        attr_name = input('Name of attribute ' +str(attr_num + 1)+' to edit(e.g. color, size, etc.)? ')
                        attr_val = input('Enter the value of '+str(attr_name)+' to be change to: ')
                        attributes[attr_name] = attr_val
                        this_bp.set_attribute(attr_name, attr_val)
                
                elif edit_attr == 'NO -a':
                    print('Will not ask to edit attributes again')
                    ask_attr = False


            if bp_type.lower() == 'v' or bp_type.lower() == 'vehicle':
                vehicles_list.append(world.spawn_actor(this_bp, bp_spawn))
                # so we can put at any pose any position and hold it there
                vehicles_list[-1].set_simulate_physics(False)
                print('Actor ' + str(actor_num + 1) + ' has spawned in \n')
            elif bp_type.lower() == 'w' or bp_type.lower() == 'walker':
                walkers_list.append(world.spawn_actor(this_bp, bp_spawn))
                print('Actor ' + str(actor_num + 1) + ' has spawned in \n')
            
            if ask_save:
                save_bp = input('Do you want to save this actor as a preset [y/n] (NO -a to not be asked again)? ')
                if save_bp.lower() == 'y' or save_bp.lower() == 'yes':
                    save = {}
                    save['x'] = coords[0]
                    save['y'] = coords[1]
                    save['z'] = coords[2]
                    save['roll'] = rotations[0]
                    save['pitch'] = rotations[1]
                    save['yaw'] = rotations[2]
                    save['name_tag'] = cur_bp
                    save['bp_type'] = bp_type
                    save['attributes'] = attributes
                    preset_name = input('What do you want to name this preset (ie. uber_crash or tesla_crash)? ')
                    with open('saves.json', 'r+') as fp:
                        saves = json.load(fp)
                        saves[preset_name] = save
                        with open('saves.json', 'w') as fp:
                            json.dump(saves, fp, sort_keys=True, indent=4)
                elif save_bp == 'NO -a':
                    print('Will not ask to save actors again')
                    ask_save = False
#######################################################################################################################################################################


        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor
        
        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')

            # prepare the light state of the cars to spawn
            light_state = vls.NONE
            if args.car_lights_on:
                light_state = vls.Position | vls.LowBeam | vls.LowBeam

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, traffic_manager.get_port()))
                .then(SetVehicleLightState(FutureActor, light_state)))

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 0.0      # how many pedestrians will run
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if not args.sync or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        # example of how to use parameters
        traffic_manager.global_percentage_speed_difference(30.0)

        while True:
            if args.sync and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

    finally:

        if args.sync and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
