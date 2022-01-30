# Initialize collision history
prev_collision_vehicles = 0
prev_collision_pedestrians = 0
prev_collision_other = 0


def get_player_collided_flag(measurement):
    """Obtains collision flag from player. Check if any of the three collision
    metrics (vehicles, pedestrians, others) from the player are true, if so the
    player has collided to something.

    Note: From the CARLA documentation:

    "Collisions are not annotated if the vehicle is not moving (<1km/h) to avoid
    annotating undesired collision due to mistakes in the AI of non-player
    agents."
    """
    player_meas = measurement.player_measurements
    current_collision_vehicles = player_meas.collision_vehicles
    current_collision_pedestrians = player_meas.collision_pedestrians
    current_collision_other = player_meas.collision_other

    collided_vehicles = current_collision_vehicles > prev_collision_vehicles
    collided_pedestrians = (
        current_collision_pedestrians > prev_collision_pedestrians
    )
    collided_other = current_collision_other > prev_collision_other

    return collided_vehicles or collided_pedestrians or collided_other
