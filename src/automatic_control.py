#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of automatic vehicle control from client side."""

from __future__ import print_function

import argparse
import glob
import logging
import os
import random
import sys
import time
import json

import pygame
from pygame.locals import KMOD_CTRL
from pygame.locals import K_ESCAPE
from pygame.locals import K_q

# Find CARLA module
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# Add PythonAPI for release mode
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from carla import VehicleLightState as vls

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.roaming_agent import RoamingAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error

from utils import get_actor_display_name, get_different_spawn_point
from sensors import CollisionSensor, LaneInvasionSensor, GnssSensor, CameraManager
from dynamic_weather import Weather
from lane_utils import LaneExtractor
from spawn_npc import NPCManager
from hud import HUD
import map_info


class World(object):
    """ Class representing the surrounding environment """

    def __init__(self, carla_world, hud, args):
        """Constructor method"""
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self.segmentation_manager = None
        self._weather_speed_factor = 0.05
        self.weather = None
        self._actor_filter = args.filter
        self._gamma = args.gamma
        self.restart(args)
        self.world.on_tick(hud.on_world_tick)

    def restart(self, args):
        """Restart the world"""
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_id = self.camera_manager.transform_index if self.camera_manager is not None else 0
        seg_index = self.segmentation_manager.index if self.segmentation_manager is not None else 5
        seg_pos_id = self.segmentation_manager.transform_index if self.segmentation_manager is not None else 0
        # Set the seed if requested by user
        if args.seed is not None:
            random.seed(args.seed)

        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player.
        print("Spawning the player")
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)

        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            random.shuffle(spawn_points)
            spawn_point = spawn_points[0] if spawn_points else carla.Transform()
            spawn_waypoint = self.map.get_waypoint(spawn_point.location, project_to_road=True)
            idx = 1
            while map_info.is_bad_road_id(self.map.name, spawn_waypoint.road_id):
                print(f"Bad spawn point at road id {spawn_waypoint.road_id}, changing spawn point")
                spawn_point = spawn_points[idx]
                spawn_waypoint = self.map.get_waypoint(spawn_point.location, project_to_road=True)
                idx += 1
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_id
        self.camera_manager.set_sensor(cam_index, notify=False)
        self.segmentation_manager = CameraManager(self.player, self.hud, self._gamma)
        self.segmentation_manager.transform_index = seg_pos_id
        self.segmentation_manager.set_sensor(seg_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)
        # Initialize weather
        self.weather = Weather(self.world.get_weather())
        if args.resume_weather is not None:
            print("Resuming weather state")
            self.weather.resume_state(args.resume_weather)
            self.world.set_weather(self.weather.weather)

    def update_weather(self, clock):
        self.weather.tick(self._weather_speed_factor*clock.get_fps())
        self.world.set_weather(self.weather.weather)
        sys.stdout.write('\r' + str(self.weather) + 12 * ' ')
        sys.stdout.flush()
        # turn on vehicle lights at night
        light_state = vls.NONE
        if self.weather.weather.sun_altitude_angle < 0:
            light_state = vls.Position | vls.LowBeam
        for actor in self.world.get_actors():
            if actor.type_id.startswith("vehicle"):
                actor.set_light_state(carla.VehicleLightState(light_state))

    def tick(self, clock):
        """Method for every tick"""
        self.hud.tick(self, clock)
        self.update_weather(clock)

    def render(self, display):
        """Render world"""
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        """Destroy sensors"""
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None
        self.segmentation_manager.sensor.destroy()
        self.segmentation_manager.sensor = None
        self.segmentation_manager.index = None

    def destroy(self):
        """Destroys all actors"""
        actors = [
            self.camera_manager.sensor,
            self.segmentation_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()


class KeyboardControl(object):
    def __init__(self, world):
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True

    @staticmethod
    def _is_quit_shortcut(key):
        """Shortcut for quitting"""
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


def game_loop(args):
    """ Main loop for agent"""

    pygame.init()
    pygame.font.init()
    tot_target_reached = 0
    num_min_waypoints = 21
    town_idx = 0

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        if args.resume and os.path.exists("resume_progress.json"):
            with open("resume_progress.json", 'r') as file:
                prev_session = json.load(file)
            args = argparse.Namespace(**prev_session["args"])
            completed_towns = prev_session["completed"]
            available_towns = prev_session["towns"]
            town_idx = prev_session["idx"]
            print("Resuming previous session:")
            print(prev_session)
        else:
            args.resume_weather = None
            found_towns = [town_id.split("/")[-1] for town_id in client.get_available_maps() if "Opt" not in town_id]
            available_towns = [town for town in found_towns if town in map_info.available_town_info]
            print("Available towns:", available_towns)
            completed_towns = {}
        
        max_npc_walkers = args.num_npc_walkers
        max_npc_vehicles = args.num_npc_vehicles

        if args.seed is not None:
            random.seed(args.seed)

        while town_idx < len(available_towns):
            print("\nRunning", available_towns[town_idx])

            start_time = time.time()
            if args.seed is not None:
                args.seed = random.randint(0, 2**32-1)
                print("Current simulation seed", args.seed)

            hud = HUD(args.width, args.height, __doc__)
            world = World(client.load_world(available_towns[town_idx]), hud, args)
            # Randomize number of NPCs
            args.num_npc_walkers = random.randint(0, max_npc_walkers)
            args.num_npc_vehicles = random.randint(0, max_npc_vehicles)
            npc_manager = NPCManager(args)
            controller = KeyboardControl(world)
            lane_extractor = LaneExtractor(world, verbose=args.debug)

            npc_manager.spawn_npc()

            if args.agent == "Roaming":
                agent = RoamingAgent(world.player)
            elif args.agent == "Basic":
                agent = BasicAgent(world.player)
                spawn_point = world.map.get_spawn_points()[0]
                agent.set_destination((spawn_point.location.x,
                                       spawn_point.location.y,
                                       spawn_point.location.z))
            else:
                agent = BehaviorAgent(world.player, behavior=args.behavior)
                spawn_points, destination = get_different_spawn_point(world, world.player)
                agent.set_destination(agent.vehicle.get_location(), destination.location, clean=True)

            if available_towns[town_idx] in completed_towns:
                start_time, frame_num = completed_towns[available_towns[town_idx]]
                lane_extractor.camera.frame_count = frame_num
                print(f"Respawned player, resuming from {frame_num} saved frames")

            clock = pygame.time.Clock()
            stopped_count = 0

            while lane_extractor.camera.frame_count < args.images_per_town:
                clock.tick_busy_loop(60)
                if controller.parse_events():
                    return

                # As soon as the server is ready continue!
                if not world.world.wait_for_tick(10.0):
                    continue

                if args.agent == "Roaming" or args.agent == "Basic":
                    if controller.parse_events():
                        return

                    # as soon as the server is ready continue!
                    world.world.wait_for_tick(10.0)

                    world.tick(clock)
                    world.render(display)
                    lane_extractor.update(clock)
                    lane_extractor.visualize_lanes(display)
                    pygame.display.flip()
                    control = agent.run_step()
                    control.manual_gear_shift = False
                    world.player.apply_control(control)
                else:
                    agent.update_information()

                    world.tick(clock)
                    world.render(display)
                    lane_extractor.update(clock)
                    lane_extractor.visualize_lanes(display)
                    pygame.display.flip()

                    # Set new destination when target has been reached
                    if len(agent.get_local_planner().waypoints_queue) < num_min_waypoints:
                        agent.reroute(spawn_points)
                        tot_target_reached += 1
                        world.hud.notification("The target has been reached " +
                                               str(tot_target_reached) + " times.", seconds=4.0)

                    speed_limit = world.player.get_speed_limit()
                    agent.get_local_planner().set_speed(speed_limit)

                    control = agent.run_step()
                    world.player.apply_control(control)

                vehicle_velocity = world.player.get_velocity()
                x_velocity, y_velocity = int(vehicle_velocity.x), int(vehicle_velocity.y)
                if x_velocity == 0 and y_velocity == 0:
                    stopped_count += 1
                else:
                    stopped_count = 0

                if stopped_count >= 15 or lane_extractor.at_bad_road_id:
                    lane_extractor.at_bad_road_id = False
                    completed_towns[available_towns[town_idx]] = (start_time, world.camera_manager.frame_count)
                    break
            
            if world.camera_manager.frame_count == args.images_per_town:
                end_time = time.time()
                hours, rem = divmod(end_time - start_time, 3600)
                minutes, seconds = divmod(rem, 60)
                print("{} done in {:0>2}:{:0>2}:{:05.2f}!".format(available_towns[town_idx], int(hours), int(minutes),
                                                                  seconds), end=" ")
                print(f"{lane_extractor.camera.frame_count} images saved.")
                town_idx += 1
            else:
                args.resume_weather = world.weather.export_state()
            
            npc_manager.destory_npc()
            world.destroy()

    finally:
        with open('resume_progress.json', 'w') as save_file:
            args.num_npc_walkers = max_npc_walkers
            args.num_npc_vehicles = max_npc_vehicles
            session_info = {
                'args': vars(args),
                'towns': available_towns,
                'idx': town_idx,
                'completed': completed_towns,
            }
            json.dump(session_info, save_file)

        pygame.quit()


def main():
    """Main method"""

    # automatic_control args
    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information')
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
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='Window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.tesla.model3',  # use 'vehicle.*' for random vehicle
        help='Actor filter (default: "vehicle.tesla.model3")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of the possible agent behaviors (default: normal) ',
        default='normal')
    argparser.add_argument("-a", "--agent", type=str,
                           choices=["Behavior", "Roaming", "Basic"],
                           help="select which agent to run",
                           default="Roaming")
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None)',
        default=None,
        type=int)
    argparser.add_argument(
        '-i', '--images_per_town',
        help='Set max number of image per town (default: 545)',
        default=545,
        type=int)
    argparser.add_argument(
        '-r', '--resume',
        action='store_true',
        help='Continue from previous session resume_progress.json file')

    # spawn_npc args
    argparser.add_argument(
        '-n', '--num_npc_vehicles',
        metavar='N',
        default=20,
        type=int,
        help='Max number of NPC vehicles (default: 20)')
    argparser.add_argument(
        '-w', '--num_npc_walkers',
        metavar='W',
        default=10,
        type=int,
        help='Max number of NPC walkers (default: 10)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Vehicles filter (default: "vehicle.*")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Pedestrians filter (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--tm_port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Enanble')
    argparser.add_argument(
        '--car_lights_on',
        action='store_true',
        default=False,
        help='Enanble car lights')

    args = argparser.parse_args()

    if args.resume and os.path.exists("resume_progress.json"):
        with open("resume_progress.json", 'r') as file:
            prev_session = json.load(file)
        saved_args = argparse.Namespace()
        saved_args.__dict__.update(prev_session["args"])
        args = argparser.parse_args(namespace=saved_args)

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
