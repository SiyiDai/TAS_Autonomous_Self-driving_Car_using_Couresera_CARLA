from tkinter import HORIZONTAL
import numpy as np
import csv
import skimage
from scipy import ndimage

from object_detection.depth_to_pointcloud import depth_to_pointcloud
from carla import image_converter

HORIZONTAL_LEVEL = 38.10
PEDESTRIAN_SEG_RANGE = [220, 20, 60]
VEHICLE_SEG_RANGE = [0, 0, 255]
CAMERA_MOUNTING_SHIFT_X = 4.2
CAMERA_MOUNTING_SHIFT_Y = 0.14

IMAGE_WIDTH = 180
IMAGE_HEIGHT = 320
YAW_ANGLE = 180

VEHICLE_BOX_X_RADIUS = 2.5
VEHICLE_BOX_Y_RADIUS = 1.0
VEHICLE_BOX_Z_RADIUS = 0.8

PEDESTRIAN_BOX_X_RADIUS = 1
PEDESTRIAN_BOX_Y_RADIUS = 1
PEDESTRIAN_BOX_Z_RADIUS = 2


def object_detection(image_depth, image_segment, ego_x, ego_y):

    point_cloud = depth_to_pointcloud(image_depth)
    segmented_image = image_converter.labels_to_cityscapes_palette(image_segment)

    # if segmented_image == VEHICLE_SEG_RANGE:
    vehicle_3d = segmented_image == VEHICLE_SEG_RANGE
    vehicle_cen_car = object_center_detection(vehicle_3d)
    vehicle_x, vehicle_y = obejct_position_world(vehicle_cen_car, point_cloud, ego_x, ego_y)
    vehicle_pos = [vehicle_x[0], vehicle_y[0], HORIZONTAL_LEVEL]
    print("[INFO] Parked car detected at world map: ", vehicle_pos)
    # print(vehicle_x, vehicle_y)
    data = vehicle_detection_list(vehicle_x, vehicle_y)
    # elif segmented_image == PEDESTRIAN_SEG_RANGE:
    #     pedestrian_3d = segmented_image
    #     pedestrian_cen = object_center_detection(pedestrian_3d)
    #     pedestrian_x, pedestrian_y, pedestrian_z = obejct_position(
    #         pedestrian_cen, point_cloud, x, y
    #     )
    #     write_pedestrian_detection_to_txt(
    #         pedestrian_x, pedestrian_y, pedestrian_z
    #     )
    # print(data)
    return data


def object_center_detection(object_3d):
    object_det = (object_3d[:, :, 0] * object_3d[:, :, 1] * object_3d[:, :, 2]) * 255
    object_det = ndimage.binary_erosion(object_det)
    label_object = skimage.measure.label(object_det, connectivity=object_det.ndim)
    props_object = skimage.measure.regionprops(label_object, intensity_image=None, cache=True)
    object_cen = np.zeros([len(props_object), 2])

    for i in range(len(props_object)):
        object_cen[i, :] = np.round(props_object[i].centroid, 0)

    return object_cen


def obejct_position_world(object_cen, point_cloud, ego_x, ego_y):
    # transformation from ego coordinate to world coordinate
    object_x = []
    object_y = []
    # object_z = []

    for i in object_cen:
        location = i[1] + i[0] * IMAGE_HEIGHT
        object_x_car = -point_cloud[2, int(location)]
        object_y_car = point_cloud[1, int(location)]
        object_x_world = object_x_car + ego_x - CAMERA_MOUNTING_SHIFT_X
        object_y_world = object_y_car + ego_y - CAMERA_MOUNTING_SHIFT_Y
        object_x.append(object_x_world)
        object_y.append(object_y_world)
        # object_z.append(point_cloud[2, int(location)])

    return object_x, object_y


def vehicle_detection_list(objectsx, objectsy):
    for i in range(0, len(objectsx)):
        locations = []
        vehicle_position = [objectsx[i], objectsy[i], HORIZONTAL_LEVEL, np.radians(YAW_ANGLE)]
        vehicle_size = [
            VEHICLE_BOX_X_RADIUS,
            VEHICLE_BOX_Y_RADIUS,
            VEHICLE_BOX_Z_RADIUS,
        ]
        locations = vehicle_position + vehicle_size
    return locations


def write_vehicle_detection_to_txt(objectsx, objectsy):
    header = [
        "X(m)",
        "Y(m)",
        "Z(m)",
        "YAW(deg)",
        "BOX_X_RADIUS(m)",
        "BOX_Y_RADIUS(m)",
        "BOX_Z_RADIUS(m)",
    ]
    with open("parked_vehicle_params.txt", "w", encoding="UTF8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(0, len(objectsx)):
            locations = []
            vehicle_position = [objectsx[i], objectsy[i], HORIZONTAL_LEVEL, np.radians(YAW_ANGLE)]
            vehicle_size = [
                VEHICLE_BOX_X_RADIUS,
                VEHICLE_BOX_Y_RADIUS,
                VEHICLE_BOX_Z_RADIUS,
            ]
            locations = vehicle_position + vehicle_size
            writer.writerow(locations)


def write_pedestrian_detection_to_txt(objectsx, objectsy):
    header = [
        "X(m)",
        "Y(m)",
        "Z(m)",
        "YAW(deg)",
        "BOX_X_RADIUS(m)",
        "BOX_Y_RADIUS(m)",
        "BOX_Z_RADIUS(m)",
    ]
    with open("pedestrian_params.txt", "w", encoding="UTF8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(0, len(objectsx)):
            locations = []
            pedesterian_position = (
                objectsx[i],
                objectsy[i],
                HORIZONTAL_LEVEL,
                YAW_ANGLE,
            )
            pedesterian_size = (
                PEDESTRIAN_BOX_X_RADIUS,
                PEDESTRIAN_BOX_Y_RADIUS,
                PEDESTRIAN_BOX_Z_RADIUS,
            )
            locations = pedesterian_position + pedesterian_size
            writer.writerow(locations)
