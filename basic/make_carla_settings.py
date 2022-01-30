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

    camera0 = sensor.Camera("CameraRGB")
    camera0.set_image_size(WINDOW_WIDTH, WINDOW_HEIGHT)
    camera0.set_position(2.0, 0.0, 1.4)
    camera0.set_rotation(0.0, 0.0, 0.0)
    settings.add_sensor(camera0)
    camera1 = sensor.Camera("CameraDepth", PostProcessing="Depth")
    camera1.set_image_size(MINI_WINDOW_WIDTH, MINI_WINDOW_HEIGHT)
    camera1.set_position(2.0, 0.0, 1.4)
    camera1.set_rotation(0.0, 0.0, 0.0)
    settings.add_sensor(camera1)
    camera2 = sensor.Camera("CameraSemSeg", PostProcessing="SemanticSegmentation")
    camera2.set_image_size(MINI_WINDOW_WIDTH, MINI_WINDOW_HEIGHT)
    camera2.set_position(2.0, 0.0, 1.4)
    camera2.set_rotation(0.0, 0.0, 0.0)
    settings.add_sensor(camera2)

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
