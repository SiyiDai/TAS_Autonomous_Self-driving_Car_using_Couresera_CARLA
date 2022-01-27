import math


def get_current_pose(measurement):
    """Obtains current x,y,yaw pose from the client measurements

    Obtains the current x,y, and yaw pose from the client measurements.

    Args:
        measurement: The CARLA client measurements (from read_data())

    Returns: (x, y, yaw)
        x: X position in meters
        y: Y position in meters
        yaw: Yaw position in radians
    """
    x = measurement.player_measurements.transform.location.x
    y = measurement.player_measurements.transform.location.y
    yaw = math.radians(measurement.player_measurements.transform.rotation.yaw)

    return (x, y, yaw)


def get_start_pos(scene):
    """Obtains player start x,y, yaw pose from the scene

    Obtains the player x,y, and yaw pose from the scene.

    Args:
        scene: The CARLA scene object

    Returns: (x, y, yaw)
        x: X position in meters
        y: Y position in meters
        yaw: Yaw position in radians
    """
    x = scene.player_start_spots[0].location.x
    y = scene.player_start_spots[0].location.y
    yaw = math.radians(scene.player_start_spots[0].rotation.yaw)

    return (x, y, yaw)
