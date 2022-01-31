#!/usr/bin/env python3

import os
import csv
import numpy as np
from basic.load_stopsign import *
from basic.load_parkedcar import *


def load_objects(stopsign_file, stopsign_fencelength, parkedcar_file):
    # load stopsign
    stopsign_data = load_stopsign(stopsign_file)
    stopsign_fences = convert_stopsign_lp(stopsign_data, stopsign_fencelength)
    # load parked car
    parkedcar_data = load_parkedcar(parkedcar_file)
    print(parkedcar_data)
    parkedcar_box_pts = obtain_parkedcar_lp(parkedcar_data)
    return stopsign_fences, parkedcar_box_pts

def load_stopsign_from_file(stopsign_file, stopsign_fencelength):
    # load stopsign
    stopsign_data = load_stopsign(stopsign_file)
    stopsign_fences = convert_stopsign_lp(stopsign_data, stopsign_fencelength)
    return stopsign_fences

def load_waypoints(waypoints_file):
    # Load Waypoints
    # Opens the waypoint file and stores it to "waypoints"
    waypoints_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), waypoints_file)
    waypoints_np = None
    with open(waypoints_filepath) as waypoints_file_handle:
        waypoints = list(
            csv.reader(
                waypoints_file_handle,
                delimiter=",",
                quoting=csv.QUOTE_NONNUMERIC,
            )
        )
        waypoints_np = np.array(waypoints)
    return waypoints, waypoints_np
