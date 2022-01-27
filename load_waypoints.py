import os
import csv
import numpy as np


def load_waypoints(waypoints_file):
    # Load Waypoints
    # Opens the waypoint file and stores it to "waypoints"
    waypoints_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), waypoints_file)
    waypoints_np = None
    with open(waypoints_filepath) as waypoints_file_handle:
        waypoints = list(csv.reader(waypoints_file_handle, delimiter=",", quoting=csv.QUOTE_NONNUMERIC))
        waypoints_np = np.array(waypoints)
    return waypoints, waypoints_np
