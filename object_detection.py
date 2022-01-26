import open3d as o3d
import numpy as np
import numpy
import math
import numpy
from numpy.matlib import repmat
import csv
import open3d as o3d
import numpy as np
import numpy
import math
import numpy
from numpy.matlib import repmat

from skimage.measure import label
import skimage
from carla.depthtopointcloud import depthtopointcloud
from carla import image_converter
from scipy import ndimage

pedesterian = [220, 20, 60]
vehicle = [0, 0, 255]
IMAGE_WIDTH = 180
IMAGE_HEIGHT = 320
YAW_ANGLE= 180


def object_detection(image_depth, image_segment, x, y, z):

    point_cloud = depthtopointcloud(image_depth)
    segmented_image = image_converter.labels_to_cityscapes_palette(image_segment)

    vehicle3d = segmented_image == vehicle
    pedesterian3d = segmented_image == pedesterian

    vehicle_det = (vehicle3d[:, :, 0] * vehicle3d[:, :, 1] * vehicle3d[:, :, 2]) * 255
    pedesterian_det = (pedesterian3d[:, :, 0] * pedesterian3d[:, :, 1] * pedesterian3d[:, :, 2]) * 255

    # Multidimensional binary erosion with a given structuring element. 
    # Binary erosion is a mathematical morphology operation used for image processing.
    vehicle_det = ndimage.binary_erosion(vehicle_det)
    pedesterian_det = ndimage.binary_erosion(pedesterian_det)

    label_vehicle = label(vehicle_det, connectivity=vehicle_det.ndim)
    props_vehicle = skimage.measure.regionprops(label_vehicle, intensity_image=None, cache=True)
    vehicle_cen = np.zeros([len(props_vehicle), 2])

    for i in range(len(props_vehicle)):
        vehicle_cen[i, :] = np.round(props_vehicle[i].centroid, 0)

    label_pedesterian = label(pedesterian_det, connectivity=pedesterian_det.ndim)
    props_pedesterian = skimage.measure.regionprops(label_pedesterian, intensity_image=None, cache=True)
    Pedestrian_cen = np.zeros([len(props_pedesterian), 2])

    for i in range(len(props_pedesterian)):
        Pedestrian_cen[i, :] = np.round(props_pedesterian[i].centroid, 0)

    objectsx = []
    objectsy = []
    objectsz = []

    for i in Pedestrian_cen:
        location = i[1] + i[0] * IMAGE_HEIGHT
        objectsx.append(point_cloud[0, int(location)] + x)
        objectsy.append(point_cloud[1, int(location)] + y)
        objectsz.append(point_cloud[2, int(location)])

    for i in vehicle_cen:
        location = i[1] + i[0] * IMAGE_HEIGHT
        objectsx.append(point_cloud[0, int(location)] + x)
        objectsy.append(point_cloud[1, int(location)] + y)
        objectsz.append(point_cloud[2, int(location)])

    print("x cord:" + str(objectsx))
    print("y cord:" + str(objectsy))

    # f = open("objectparam.txt", "w")
    # f.write('X(m), Y(m), Z(m), YAW(deg), BOX_X_RADIUS(m), BOX_Y_RADIUS(m), BOX_Z_RADIUS(m)\n')
    header = ["X(m)", "Y(m)", "Z(m)", "YAW(deg)", "BOX_X_RADIUS(m)", "BOX_Y_RADIUS(m)", "BOX_Z_RADIUS(m)"]
    data = []
    with open("objectparam.txt", "w", encoding="UTF8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(0, len(objectsx)):
            locations = []
            locations.append(objectsx[i])
            locations.append(objectsy[i])
            locations.append(objectsz[i])
            locations.append(YAW_ANGLE)
            locations.append(2)
            locations.append(2)
            locations.append(2)
            writer.writerow(locations)