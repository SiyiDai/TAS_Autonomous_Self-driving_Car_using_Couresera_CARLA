import csv
import numpy as np

def load_parkedcar(parkedcar_file):
    # Parked car(s) (X(m), Y(m), Z(m), Yaw(deg), RADX(m), RADY(m), RADZ(m))
    parkedcar_data = None
    with open(parkedcar_file, "r") as parkedcar_file:
        next(parkedcar_file)  # skip header
        parkedcar_reader = csv.reader(parkedcar_file, delimiter=",", quoting=csv.QUOTE_NONNUMERIC)
        parkedcar_data = list(parkedcar_reader)
        # convert to rad
        for i in range(len(parkedcar_data)):
            parkedcar_data[i][3] = parkedcar_data[i][3] * np.pi / 180.0
    return parkedcar_data

def obtain_parkedcar_lp(parkedcar_data):
    # obtain parked car(s) box points for local planner
    parkedcar_box_pts = []  # [x,y]
    for i in range(len(parkedcar_data)):
        x = parkedcar_data[i][0]
        y = parkedcar_data[i][1]
        z = parkedcar_data[i][2]
        yaw = parkedcar_data[i][3]
        xrad = parkedcar_data[i][4]
        yrad = parkedcar_data[i][5]
        zrad = parkedcar_data[i][6]
        cpos = np.array(
            [[-xrad, -xrad, -xrad, 0, xrad, xrad, xrad, 0], [-yrad, 0, yrad, yrad, yrad, 0, -yrad, -yrad]]
        )
        rotyaw = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
        cpos_shift = np.array([[x, x, x, x, x, x, x, x], [y, y, y, y, y, y, y, y]])
        cpos = np.add(np.matmul(rotyaw, cpos), cpos_shift)
        for j in range(cpos.shape[1]):
            parkedcar_box_pts.append([cpos[0, j], cpos[1, j]])
    return parkedcar_box_pts