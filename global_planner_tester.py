#!/usr/bin/env python3


import os
import sys
import copy
import argparse
import configparser
import time

sys.path.append(os.path.abspath(sys.path[0] + '/..'))
import carla
from carla.client     import make_carla_client
from carla.settings   import CarlaSettings
from carla.planner.map import CarlaMap
from carla.planner.planner import Planner
from carla.planner.city_track import CityTrack
import global_planner


def make_carla_settings(args):
    """Make a CarlaSettings object with the settings we need.
    """
    settings = CarlaSettings()

    # There is no need for non-agent info requests if there are no pedestrians
    # or vehicles.
    get_non_player_agents_info = False

    # Base level settings
    settings.set(
        SynchronousMode=True,
        SendNonPlayerAgentsInfo=get_non_player_agents_info,
        QualityLevel=args.quality_level)
    return settings

def main(args):
    with make_carla_client(args.host, args.port) as client:
        print('Carla client connected.')

        settings = make_carla_settings(args)

        # Now we load these settings into the server. The server replies
        # with a scene description containing the available start spots for
        # the player. Here we can provide a CarlaSettings object or a
        # CarlaSettings.ini file as string.
        scene = client.load_settings(settings)
        citytrack = CityTrack(scene.map_name)

        # Refer to the player start folder in the WorldOutliner to see the
        # player start information
        player_start = 1

        client.start_episode(player_start)

        time.sleep(3);

        # Notify the server that we want to start the episode at the
        # player_start index. This function blocks until the server is ready
        # to start the episode.
        print('Starting new episode at %r...' % scene.map_name)
        client.start_episode(player_start)
        #############################################
        # Load Configurations
        #############################################

        # Load configuration file (options.cfg) and then parses for the various
        # options. Here we have two main options:
        # live_plotting and live_plotting_period, which controls whether
        # live plotting is enabled or how often the live plotter updates
        # during the simulation run.
        # sampling_resolution = map.get_graph_resolution()
        # grp = get_route_planner(map, sampling_resolution)

        # origin = 317.74, 129.49, 8.333
        # destination = 92.34, 86.73282105965863, 2.5
        node_source = 317.74, 129.49
        source_ori = 0,0
        node_target= 92.34, 86.73282105965863
        target_ori = 0,0
        route = citytrack.compute_route(node_source, source_ori, node_target, target_ori)
        # route_trace = grp.trace_route(origin, destination)
        # print(route_trace)


def get_route_planner(town_map, sampling_resolution):
    GlobalRoutePlanner = global_planner.GlobalRoutePlanner
    grp = GlobalRoutePlanner(town_map, sampling_resolution)
    return grp

if __name__ == '__main__':

    try:
        argparser = argparse.ArgumentParser(description=__doc__)
        argparser.add_argument(
            '-v', '--verbose',
            action='store_true',
            dest='debug',
            help='print debug information')
        argparser.add_argument(
            '--host',
            metavar='H',
            default='localhost',
            help='IP of the host server (default: localhost)')
        argparser.add_argument(
            '-p', '--port',
            metavar='P',
            default=2000,
            type=int,
            help='TCP port to listen to (default: 2000)')
        argparser.add_argument(
            '-a', '--autopilot',
            action='store_true',
            help='enable autopilot')
        argparser.add_argument(
            '-q', '--quality-level',
            choices=['Low', 'Epic'],
            type=lambda s: s.title(),
            default='Low',
            help='graphics quality level.')
        argparser.add_argument(
            '-c', '--carla-settings',
            metavar='PATH',
            dest='settings_filepath',
            default=None,
            help='Path to a "CarlaSettings.ini" file')
        args = argparser.parse_args()
        main(args)
    except KeyboardInterrupt:
        print('\nstop test')