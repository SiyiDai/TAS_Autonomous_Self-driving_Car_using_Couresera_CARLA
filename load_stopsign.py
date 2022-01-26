import csv
import numpy as np

def load_stopsign(stopsign_file):
    # Load stop sign parameters
    # Stop sign (X(m), Y(m), Z(m), Yaw(deg))
    stopsign_data = None
    with open(stopsign_file, "r") as stopsign_file:
        next(stopsign_file)  # skip header
        stopsign_reader = csv.reader(stopsign_file, delimiter=",", quoting=csv.QUOTE_NONNUMERIC)
        stopsign_data = list(stopsign_reader)
        # convert to rad
        for i in range(len(stopsign_data)):
            stopsign_data[i][3] = stopsign_data[i][3] * np.pi / 180.0
    return stopsign_data

def convert_stopsign_lp(stopsign_data, stopsign_fencelength):
    # Convert to input params for LP
    stopsign_fences = []  # [x0, y0, x1, y1]
    # obtain stop sign fence points for LP
    for i in range(len(stopsign_data)):
        x = stopsign_data[i][0]
        y = stopsign_data[i][1]
        z = stopsign_data[i][2]
        yaw = stopsign_data[i][3] + np.pi / 2.0  # add 90 degrees for fence
        spos = np.array([[0, 0], [0, stopsign_fencelength]])
        rotyaw = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
        spos_shift = np.array([[x, x], [y, y]])
        spos = np.add(np.matmul(rotyaw, spos), spos_shift)
        stopsign_fences.append([spos[0, 0], spos[1, 0], spos[0, 1], spos[1, 1]])
    return stopsign_fences