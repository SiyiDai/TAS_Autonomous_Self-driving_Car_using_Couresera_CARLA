import numpy as np

import os
import sys

from live_plotter_helpers.history_helper import history_init

sys.path.append(os.path.abspath(sys.path[0] + "/.."))
from carla.client import VehicleControl

# controller output directory
CONTROLLER_OUTPUT_FOLDER = (
    os.path.dirname(os.path.realpath(__file__)) + "/controller_output/"
)


def send_control_command(
    client, throttle, steer, brake, hand_brake=False, reverse=False
):
    """Send control command to CARLA client.

    Send control command to CARLA client.

    Args:
        client: The CARLA client object
        throttle: Throttle command for the sim car [0, 1]
        steer: Steer command for the sim car [-1, 1]
        brake: Brake command for the sim car [0, 1]
        hand_brake: Whether the hand brake is engaged
        reverse: Whether the sim car is in the reverse gear
    """
    control = VehicleControl()
    # Clamp all values within their limits
    steer = np.fmax(np.fmin(steer, 1.0), -1.0)
    throttle = np.fmax(np.fmin(throttle, 1.0), 0)
    brake = np.fmax(np.fmin(brake, 1.0), 0)

    control.steer = steer
    control.throttle = throttle
    control.brake = brake
    control.hand_brake = hand_brake
    control.reverse = reverse
    client.send_control(control)


def create_controller_output_dir(output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)


def store_trajectory_plot_all(
    trajectory_fig, forward_speed_fig, throttle_fig, brake_fig, steer_fig
):
    store_trajectory_plot(trajectory_fig.fig, "trajectory.png")
    store_trajectory_plot(forward_speed_fig.fig, "forward_speed.png")
    store_trajectory_plot(throttle_fig.fig, "throttle_output.png")
    store_trajectory_plot(brake_fig.fig, "brake_output.png")
    store_trajectory_plot(steer_fig.fig, "steer_output.png")


def store_trajectory_plot(graph, fname):
    """Store the resulting plot."""
    create_controller_output_dir(CONTROLLER_OUTPUT_FOLDER)

    file_name = os.path.join(CONTROLLER_OUTPUT_FOLDER, fname)
    graph.savefig(file_name)


def write_trajectory_file(history):
    x_list = history[0]
    y_list = history[1]
    v_list = history[3]
    t_list = history[4]
    collided_list = history[5]
    create_controller_output_dir(CONTROLLER_OUTPUT_FOLDER)
    file_name = os.path.join(CONTROLLER_OUTPUT_FOLDER, "trajectory.txt")

    with open(file_name, "w") as trajectory_file:
        for i in range(len(x_list)):
            trajectory_file.write(
                "%3.3f, %3.3f, %2.3f, %6.3f %r\n"
                % (x_list[i], y_list[i], v_list[i], t_list[i], collided_list[i])
            )


def write_collisioncount_file(collided_list):
    create_controller_output_dir(CONTROLLER_OUTPUT_FOLDER)
    file_name = os.path.join(CONTROLLER_OUTPUT_FOLDER, "collision_count.txt")

    with open(file_name, "w") as collision_file:
        collision_file.write(str(sum(collided_list)))
