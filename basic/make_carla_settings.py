import sys
import os

sys.path.append(os.path.abspath(sys.path[0] + "/.."))
from carla.settings import CarlaSettings
from carla import sensor

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
MINI_WINDOW_WIDTH = 320
MINI_WINDOW_HEIGHT = 180

NUM_PEDESTRIANS = 0  # total number of pedestrians to spawn 0
NUM_VEHICLES = 2  # total number of vehicles to spawn 2
SEED_PEDESTRIANS = 0  # seed for pedestrian spawn randomizer 0
SEED_VEHICLES = 0  # seed for vehicle spawn randomizer 0
CLIENT_WAIT_TIME = 3  # wait time for client before starting episode

WEATHERID = {
    "DEFAULT": 0,
    "CLEARNOON": 1,
    "CLOUDYNOON": 2,
    "WETNOON": 3,
    "WETCLOUDYNOON": 4,
    "MIDRAINYNOON": 5,
    "HARDRAINNOON": 6,
    "SOFTRAINNOON": 7,
    "CLEARSUNSET": 8,
    "CLOUDYSUNSET": 9,
    "WETSUNSET": 10,
    "WETCLOUDYSUNSET": 11,
    "MIDRAINSUNSET": 12,
    "HARDRAINSUNSET": 13,
    "SOFTRAINSUNSET": 14,
}
SIMWEATHER = WEATHERID["DEFAULT"]  # set simulation weather


def make_carla_settings(args):
    """Make a CarlaSettings object with the settings we need."""
    settings = CarlaSettings()

    # There is no need for non-agent info requests if there are no pedestrians
    # or vehicles.
    get_non_player_agents_info = False
    if NUM_PEDESTRIANS > 0 or NUM_VEHICLES > 0:
        get_non_player_agents_info = True

    # set cameras for object detection
    set_depth_camera(sensor, settings)
    set_semseg_camera(sensor, settings)

    # Base level settings
    settings.set(
        SynchronousMode=True,
        SendNonPlayerAgentsInfo=get_non_player_agents_info,
        NumberOfVehicles=NUM_VEHICLES,
        NumberOfPedestrians=NUM_PEDESTRIANS,
        SeedVehicles=SEED_VEHICLES,
        SeedPedestrians=SEED_PEDESTRIANS,
        WeatherId=SIMWEATHER,
        QualityLevel=args.quality_level,
    )
    return settings


def set_camera(sensor, settings):
    RGB_camera = sensor.Camera("CameraRGB")
    RGB_camera.set_image_size(WINDOW_WIDTH, WINDOW_HEIGHT)
    RGB_camera.set_position(2.0, 0.0, 1.4)
    RGB_camera.set_rotation(0.0, 0.0, 0.0)
    settings.add_sensor(RGB_camera)


def set_depth_camera(sensor, settings):
    depth_camera = sensor.Camera("CameraDepth", PostProcessing="Depth")
    depth_camera.set_image_size(MINI_WINDOW_WIDTH, MINI_WINDOW_HEIGHT)
    depth_camera.set_position(2.0, 0.0, 1.4)
    depth_camera.set_rotation(0.0, 0.0, 0.0)
    settings.add_sensor(depth_camera)


def set_semseg_camera(sensor, settings):
    semseg_camera = sensor.Camera("CameraSemSeg", PostProcessing="SemanticSegmentation")
    semseg_camera.set_image_size(MINI_WINDOW_WIDTH, MINI_WINDOW_HEIGHT)
    semseg_camera.set_position(2.0, 0.0, 1.4)
    semseg_camera.set_rotation(0.0, 0.0, 0.0)
    settings.add_sensor(semseg_camera)
