#!/usr/bin/env python3

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA waypoint follower assessment client script.

A controller assessment to follow a given trajectory, where the trajectory
can be defined using way-points.

STARTING in a moment...
"""
from __future__ import print_function
from __future__ import division
from operator import imod
from tracemalloc import start

import pygame


# System level imports
import sys
import os
import argparse
import logging
import time
import math
import numpy as np
import csv
import matplotlib.pyplot as plt
import controller.controller2d as controller2d
import configparser
import local_planner.local_planner as local_planner
import local_planner.behavioural_planner as behavioural_planner
from global_planner.load_waypoints import *
from object_detection.load_stopsign import *
from object_detection.load_parkedcar import *
from object_detection.load_lead_car import *
from live_plotter_helpers.control_fig_helper import *
from live_plotter_helpers.trajectory_fig_helper import *
from basic.argparser_helper import *
from basic.timer import Timer
from collision_check.get_player_collided_flag import *
from controller.controller_utils import *
from basic.get_pos import *
from basic.config_params import *
from object_detection.object_detection import *

# Script level imports
sys.path.append(os.path.abspath(sys.path[0] + "/.."))
import live_plotter as lv  # Custom live plotting library
from carla import sensor
from carla.client import make_carla_client
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.controller import utils
from carla import image_converter


def make_carla_settings(args):
    """Make a CarlaSettings object with the settings we need."""
    settings = CarlaSettings()

    # There is no need for non-agent info requests if there are no pedestrians
    # or vehicles.
    get_non_player_agents_info = False
    if NUM_PEDESTRIANS > 0 or NUM_VEHICLES > 0:
        get_non_player_agents_info = True

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


def exec_waypoint_nav_demo(args):
    """Executes waypoint navigation demo."""

    with make_carla_client(args.host, args.port) as client:
        print("Carla client connected.")

        settings = make_carla_settings(args)

        # Now we load these settings into the server. The server replies
        # with a scene description containing the available start spots for
        # the player. Here we can provide a CarlaSettings object or a
        # CarlaSettings.ini file as string.
        scene = client.load_settings(settings)

        # Refer to the player start folder in the WorldOutliner to see the
        # player start information
        player_start = PLAYER_START_INDEX

        client.start_episode(player_start)

        time.sleep(CLIENT_WAIT_TIME)

        # Notify the server that we want to start the episode at the
        # player_start index. This function blocks until the server is ready
        # to start the episode.
        print("Starting new episode at %r..." % scene.map_name)
        client.start_episode(player_start)

        #############################################
        # Load Configurations
        #############################################

        # Load configuration file (options.cfg) and then parses for the various
        # options. Here we have two main options:
        # live_plotting and live_plotting_period, which controls whether
        # live plotting is enabled or how often the live plotter updates
        # during the simulation run.
        config = configparser.ConfigParser()
        config.read(os.path.join(os.path.dirname(os.path.realpath(__file__)), "options.cfg"))
        demo_opt = config["Demo Parameters"]

        # Get options
        enable_live_plot = demo_opt.get("live_plotting", "true").capitalize()
        enable_live_plot = enable_live_plot == "True"
        live_plot_period = float(demo_opt.get("live_plotting_period", 0))

        # Set options
        live_plot_timer = Timer(live_plot_period)
        stopsign_data = load_stopsign(stopsign_file=C4_STOP_SIGN_FILE)
        stopsign_fences = convert_stopsign_lp(stopsign_data, stopsign_fencelength=C4_STOP_SIGN_FENCELENGTH)

        parkedcar_data = load_parkedcar(parkedcar_file=C4_PARKED_CAR_FILE)
        parkedcar_box_pts = obtain_parkedcar_lp(parkedcar_data)

        waypoints, waypoints_np = load_waypoints(waypoints_file=WAYPOINTS_FILENAME)

        #############################################
        # Controller 2D Class Declaration
        #############################################
        # This is where we take the controller2d.py class
        # and apply it to the simulator
        controller = controller2d.Controller2D(waypoints)

        #############################################
        # Determine simulation average timestep (and total frames)
        #############################################
        # Ensure at least one frame is used to compute average timestep
        num_iterations = ITER_FOR_SIM_TIMESTEP
        if ITER_FOR_SIM_TIMESTEP < 1:
            num_iterations = 1

        # Gather current data from the CARLA server. This is used to get the
        # simulator starting game time. Note that we also need to
        # send a command back to the CARLA server because synchronous mode
        # is enabled.
        measurement_data, sensor_data = client.read_data()
        sim_start_stamp = measurement_data.game_timestamp / 1000.0
        # Send a control command to proceed to next iteration.
        # This mainly applies for simulations that are in synchronous mode.
        send_control_command(client, throttle=0.0, steer=0, brake=1.0)
        # Computes the average timestep based on several initial iterations
        sim_duration = 0
        for i in range(num_iterations):
            # Gather current data
            measurement_data, sensor_data = client.read_data()
            # Send a control command to proceed to next iteration
            send_control_command(client, throttle=0.0, steer=0, brake=1.0)
            # Last stamp
            if i == num_iterations - 1:
                sim_duration = measurement_data.game_timestamp / 1000.0 - sim_start_stamp

        # Outputs average simulation timestep and computes how many frames
        # will elapse before the simulation should end based on various
        # parameters that we set in the beginning.
        SIMULATION_TIME_STEP = sim_duration / float(num_iterations)
        print("SERVER SIMULATION STEP APPROXIMATION: " + str(SIMULATION_TIME_STEP))
        TOTAL_EPISODE_FRAMES = (
            int((TOTAL_RUN_TIME + WAIT_TIME_BEFORE_START) / SIMULATION_TIME_STEP) + TOTAL_FRAME_BUFFER
        )

        #############################################
        # Frame-by-Frame Iteration and Initialization
        #############################################
        # Store pose history starting from the start position
        measurement_data, sensor_data = client.read_data()
        start_timestamp = measurement_data.game_timestamp / 1000.0
        start_x, start_y, start_yaw = get_current_pose(measurement_data)
        send_control_command(client, throttle=0.0, steer=0, brake=1.0)
        x_history = [start_x]
        y_history = [start_y]
        yaw_history = [start_yaw]
        time_history = [0]
        speed_history = [0]
        collided_flag_history = [False]  # assume player starts off non-collided

        #############################################
        # Vehicle Trajectory Live Plotting Setup
        #############################################
        # Uses the live plotter to generate live feedback during the simulation
        # The two feedback includes the trajectory feedback and
        # the controller feedback (which includes the speed tracking).
        lp_traj = lv.LivePlotter(tk_title="Trajectory Trace")
        lp_1d = lv.LivePlotter(tk_title="Controls Feedback")

        ###
        # Add 2D position / trajectory plot
        ###
        fig_size = (FIGSIZE_X_INCHES, FIGSIZE_Y_INCHES)
        plot_rect = [PLOT_LEFT, PLOT_BOT, PLOT_WIDTH, PLOT_HEIGHT]

        trajectory_fig = trajectory_fig_initialize(lp_traj, fig_size, plot_rect)

        trajectory_fig_set_all(
            trajectory_fig=trajectory_fig,
            waypoints_np=waypoints_np,
            start_x=start_x,
            start_y=start_y,
            stopsign_fences=stopsign_fences,
            parkedcar_box_pts=parkedcar_box_pts,
            num_paths=NUM_PATHS,
            window_size_traj=TOTAL_EPISODE_FRAMES,
            window_size_lookahead=INTERP_MAX_POINTS_PLOT,
        )

        forward_speed_fig = add_forward_speed_fig(lp_1d, window_size=TOTAL_EPISODE_FRAMES)
        throttle_fig = add_throttle_fig(lp_1d, window_size=TOTAL_EPISODE_FRAMES)
        brake_fig = add_brake_fig(lp_1d, window_size=TOTAL_EPISODE_FRAMES)
        steer_fig = add_steer_fig(lp_1d, window_size=TOTAL_EPISODE_FRAMES)

        # live plotter is disabled, hide windows
        if not enable_live_plot:
            lp_traj._root.withdraw()
            lp_1d._root.withdraw()

        #############################################
        # Local Planner Variables
        #############################################
        wp_goal_index = 0
        local_waypoints = None
        path_validity = np.zeros((NUM_PATHS, 1), dtype=bool)
        lp = local_planner.LocalPlanner(
            NUM_PATHS,
            PATH_OFFSET,
            CIRCLE_OFFSETS,
            CIRCLE_RADII,
            PATH_SELECT_WEIGHT,
            TIME_GAP,
            A_MAX,
            SLOW_SPEED,
            STOP_LINE_BUFFER,
        )
        bp = behavioural_planner.BehaviouralPlanner(BP_LOOKAHEAD_BASE, stopsign_fences, LEAD_VEHICLE_LOOKAHEAD)

        #############################################
        # Scenario Execution Loop
        #############################################

        # Iterate the frames until the end of the waypoints is reached or
        # the TOTAL_EPISODE_FRAMES is reached. The controller simulation then
        # ouptuts the results to the controller output directory.
        reached_the_end = False
        skip_first_frame = True

        # Initialize the current timestamp.
        current_timestamp = start_timestamp

        # Initialize collision history
        prev_collision_vehicles = 0
        prev_collision_pedestrians = 0
        prev_collision_other = 0

        for frame in range(TOTAL_EPISODE_FRAMES):
            # Gather current data from the CARLA server
            measurement_data, sensor_data = client.read_data()

            # Update pose and timestamp
            prev_timestamp = current_timestamp
            current_x, current_y, current_yaw = get_current_pose(measurement_data)
            current_speed = measurement_data.player_measurements.forward_speed
            current_timestamp = float(measurement_data.game_timestamp) / 1000.0

            # Wait for some initial time before starting the demo
            if current_timestamp <= WAIT_TIME_BEFORE_START:
                send_control_command(client, throttle=0.0, steer=0, brake=1.0)
                continue
            else:
                current_timestamp = current_timestamp - WAIT_TIME_BEFORE_START

            # Store history
            x_history.append(current_x)
            y_history.append(current_y)
            yaw_history.append(current_yaw)
            speed_history.append(current_speed)
            time_history.append(current_timestamp)

            # Store collision history
            (
                collided_flag,
                prev_collision_vehicles,
                prev_collision_pedestrians,
                prev_collision_other,
            ) = get_player_collided_flag(
                measurement_data, prev_collision_vehicles, prev_collision_pedestrians, prev_collision_other
            )
            collided_flag_history.append(collided_flag)

            # Local Planner Update:
            # This will use the behavioural_planner.py and local_planner.py
            lead_car_pos, lead_car_length, lead_car_speed = load_lead_car(measurement_data)
            update_local_planner = lp.update_local_planner(frame, LP_FREQUENCY_DIVISOR)

            if update_local_planner:
                # Compute open loop speed estimate.
                open_loop_speed = lp._velocity_planner.get_open_loop_speed(current_timestamp - prev_timestamp)

                # Calculate the goal state set in the local frame for the local planner.
                # Current speed should be open loop for the velocity profile generation.
                ego_state = [current_x, current_y, current_yaw, open_loop_speed]

                # Set lookahead based on current speed.
                bp.set_lookahead(BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * open_loop_speed)

                # Perform a state transition in the behavioural planner.
                bp.transition_state(waypoints, ego_state, current_speed)

                # Check to see if we need to follow the lead vehicle.
                bp.check_for_lead_vehicle(ego_state, lead_car_pos[1])

                # Compute the goal state set from the behavioural planner's computed goal state.
                goal_state_set = lp.get_goal_state_set(bp._goal_index, bp._goal_state, waypoints, ego_state)

                # Calculate planned paths in the local frame.
                paths, path_validity = lp.plan_paths(goal_state_set)

                # Transform those paths back to the global frame.
                paths = local_planner.transform_paths(paths, ego_state)

                # Perform collision checking.
                collision_check_array = lp._collision_checker.collision_check(paths, [parkedcar_box_pts])

                # Compute the best local path.
                best_index = lp._collision_checker.select_best_path_index(paths, collision_check_array, bp._goal_state)
                # If no path was feasible, continue to follow the previous best path.
                if best_index == None:
                    best_path = lp._prev_best_path
                else:
                    best_path = paths[best_index]
                    lp._prev_best_path = best_path

                # Compute the velocity profile for the path, and compute the waypoints.
                # Use the lead vehicle to inform the velocity profile's dynamic obstacle handling.
                # In this scenario, the only dynamic obstacle is the lead vehicle at index 1.
                desired_speed = bp._goal_state[2]
                lead_car_state = [lead_car_pos[1][0], lead_car_pos[1][1], lead_car_speed[1]]
                decelerate_to_stop = bp._state == behavioural_planner.DECELERATE_TO_STOP
                local_waypoints = lp._velocity_planner.compute_velocity_profile(
                    best_path,
                    desired_speed,
                    ego_state,
                    current_speed,
                    decelerate_to_stop,
                    lead_car_state,
                    bp._follow_lead_vehicle,
                )
                # --------------------------------------------------------------

                if local_waypoints != None:
                    # Update the controller waypoint path with the best local path.
                    # Linear interpolation computation on the waypoints
                    # is also used to ensure a fine resolution between points.
                    wp_distance = []  # distance array
                    local_waypoints_np = np.array(local_waypoints)
                    for i in range(1, local_waypoints_np.shape[0]):
                        wp_distance.append(
                            np.sqrt(
                                (local_waypoints_np[i, 0] - local_waypoints_np[i - 1, 0]) ** 2
                                + (local_waypoints_np[i, 1] - local_waypoints_np[i - 1, 1]) ** 2
                            )
                        )
                    wp_distance.append(0)  # last distance is 0 because it is the distance
                    # from the last waypoint to the last waypoint

                    # Linearly interpolate between waypoints and store in a list
                    wp_interp = []  # interpolated values
                    # (rows = waypoints, columns = [x, y, v])
                    for i in range(local_waypoints_np.shape[0] - 1):
                        # Add original waypoint to interpolated waypoints list (and append
                        # it to the hash table)
                        wp_interp.append(list(local_waypoints_np[i]))

                        # Interpolate to the next waypoint. First compute the number of
                        # points to interpolate based on the desired resolution and
                        # incrementally add interpolated points until the next waypoint
                        # is about to be reached.
                        num_pts_to_interp = int(np.floor(wp_distance[i] / float(INTERP_DISTANCE_RES)) - 1)
                        wp_vector = local_waypoints_np[i + 1] - local_waypoints_np[i]
                        wp_uvector = wp_vector / np.linalg.norm(wp_vector[0:2])

                        for j in range(num_pts_to_interp):
                            next_wp_vector = INTERP_DISTANCE_RES * float(j + 1) * wp_uvector
                            wp_interp.append(list(local_waypoints_np[i] + next_wp_vector))
                    # add last waypoint at the end
                    wp_interp.append(list(local_waypoints_np[-1]))

                    # Update the other controller values and controls
                    controller.update_waypoints(wp_interp)
                    pass

            ###
            # Controller Update
            ###
            if local_waypoints != None and local_waypoints != []:
                controller.update_values(current_x, current_y, current_yaw, current_speed, current_timestamp, frame)
                controller.update_controls()
                cmd_throttle, cmd_steer, cmd_brake = controller.get_commands()
            else:
                cmd_throttle = 0.0
                cmd_steer = 0.0
                cmd_brake = 0.0

            # Skip the first frame or if there exists no local paths
            if skip_first_frame and frame == 0:
                pass
            elif local_waypoints == None:
                pass
            else:
                trajectory_fig_upadte(trajectory_fig, current_x, current_y, lead_car_pos)
                control_fig_update(
                    forward_speed_fig,
                    throttle_fig,
                    brake_fig,
                    steer_fig,
                    current_timestamp,
                    current_speed,
                    controller,
                    cmd_throttle,
                    cmd_brake,
                    cmd_steer,
                )

                # Local path plotter update
                if update_local_planner:
                    path_counter = 0
                    for i in range(NUM_PATHS):
                        # If a path was invalid in the set, there is no path to plot.
                        if path_validity[i]:
                            # Colour paths according to collision checking.
                            if not collision_check_array[path_counter]:
                                colour = "r"
                            elif i == best_index:
                                colour = "k"
                            else:
                                colour = "b"
                            trajectory_fig.update(
                                "local_path " + str(i), paths[path_counter][0], paths[path_counter][1], colour
                            )
                            path_counter += 1
                        else:
                            trajectory_fig.update("local_path " + str(i), [ego_state[0]], [ego_state[1]], "r")
                # When plotting lookahead path, only plot a number of points
                # (INTERP_MAX_POINTS_PLOT amount of points). This is meant
                # to decrease load when live plotting
                wp_interp_np = np.array(wp_interp)
                path_indices = np.floor(np.linspace(0, wp_interp_np.shape[0] - 1, INTERP_MAX_POINTS_PLOT))
                trajectory_fig.update(
                    "selected_path",
                    wp_interp_np[path_indices.astype(int), 0],
                    wp_interp_np[path_indices.astype(int), 1],
                    new_colour=[1, 0.5, 0.0],
                )

                # Refresh the live plot based on the refresh rate
                # set by the options
                if enable_live_plot and live_plot_timer.has_exceeded_lap_period():
                    lp_traj.refresh()
                    lp_1d.refresh()
                    live_plot_timer.lap()

            # Output controller command to CARLA server
            send_control_command(client, throttle=cmd_throttle, steer=cmd_steer, brake=cmd_brake)

            # Find if reached the end of waypoint. If the car is within
            # DIST_THRESHOLD_TO_LAST_WAYPOINT to the last waypoint,
            # the simulation will end.
            dist_to_last_waypoint = np.linalg.norm(
                np.array([waypoints[-1][0] - current_x, waypoints[-1][1] - current_y])
            )
            if dist_to_last_waypoint < DIST_THRESHOLD_TO_LAST_WAYPOINT:
                reached_the_end = True
            if reached_the_end:
                break

        # End of demo - Stop vehicle and Store outputs to the controller output
        # directory.
        if reached_the_end:
            print("Reached the end of path. Writing to controller_output...")
        else:
            print("Exceeded assessment time. Writing to controller_output...")
        # Stop the car
        send_control_command(client, throttle=0.0, steer=0.0, brake=1.0)
        # Store the various outputs
        store_trajectory_plot(trajectory_fig.fig, "trajectory.png")
        store_trajectory_plot(forward_speed_fig.fig, "forward_speed.png")
        store_trajectory_plot(throttle_fig.fig, "throttle_output.png")
        store_trajectory_plot(brake_fig.fig, "brake_output.png")
        store_trajectory_plot(steer_fig.fig, "steer_output.png")
        write_trajectory_file(x_history, y_history, speed_history, time_history, collided_flag_history)
        write_collisioncount_file(collided_flag_history)


def main():
    # """Main function.
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser_add_argument(argparser)
    args = argparser.parse_args()

    # Logging startup info
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)
    logging.info("listening to server %s:%s", args.host, args.port)

    args.out_filename_format = "_out/episode_{:0>4d}/{:s}/{:0>6d}"

    # Execute when server connection is established
    while True:
        try:
            exec_waypoint_nav_demo(args)
            print("Done.")
            return

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)


if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")
