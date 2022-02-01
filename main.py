#!/usr/bin/env python3

from __future__ import print_function
from __future__ import division

# System level imports
import sys
import os
import argparse
import logging
import time
import numpy as np
import controller.controller2d as controller2d
import local_planner.local_planner as local_planner
import behavioural_planner.behavioural_planner as behavioural_planner

from basic.get_pos import *
from basic.losd_utils.config_params import *
from basic.losd_utils.load_stopsign import *
from basic.losd_utils.load_parkedcar import *
from basic.losd_utils.load_lead_car import *
from basic.losd_utils.load_waypoints import *
from basic.argparser_helper import *
from basic.timer import Timer
from basic.cal_timestep import *
from basic.make_carla_settings import *
from basic.losd_utils.load_config import *

import live_plotter_helpers.live_plotter as lv  # Custom live plotting library
from live_plotter_helpers.control_fig_helper import *
from live_plotter_helpers.trajectory_fig_helper import *
from live_plotter_helpers.history_helper import *

from collision_check.get_player_collided_flag import *
from controller.controller_utils import *
from controller.cal_waypoint_helper import *
from object_detection.object_detection import *
from local_planner.local_planner import update_local_planner


# Script level imports
sys.path.append(os.path.abspath(sys.path[0] + "/.."))
from carla.client import make_carla_client
from carla.tcp import TCPConnectionError


def exec_waypoint_nav_demo(args):
    """Executes waypoint navigation demo."""

    with make_carla_client(args.host, args.port) as client:
        print("Carla client connected.")

        settings = make_carla_settings(args)
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

        # Load Configurations
        demo_opt = load_config(CONFIG_FILE)
        enable_live_plot, live_plot_period = get_options(demo_opt)

        # Set options
        live_plot_timer = Timer(live_plot_period)

        # load objects
        # stopsign_fences, parkedcar_box_pts = load_objects(
        #     stopsign_file=C4_STOP_SIGN_FILE,
        #     stopsign_fencelength=C4_STOP_SIGN_FENCELENGTH,
        #     parkedcar_file=C4_PARKED_CAR_FILE,
        # )
        stopsign_fences = load_stopsign_from_file(
            stopsign_file=C4_STOP_SIGN_FILE,
            stopsign_fencelength=C4_STOP_SIGN_FENCELENGTH,
        )

        # load global planner waypoints
        waypoints, waypoints_np = load_waypoints(waypoints_file=WAYPOINTS_FILENAME)

        # Controller 2D Class Declaration
        controller = controller2d.Controller2D(waypoints)

        num_iterations = regulate_num_iteration(ITER_FOR_SIM_TIMESTEP)

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

        TOTAL_EPISODE_FRAMES = cal_total_episode(sim_duration, num_iterations)

        # Frame-by-Frame Iteration and Initialization
        # Store pose history starting from the start position
        measurement_data, sensor_data = client.read_data()
        start_timestamp = measurement_data.game_timestamp / 1000.0
        start_x, start_y, start_yaw = get_current_pose(measurement_data)

        # get camera images for object detection
        camera_depth_image = sensor_data.get("CameraDepth", None)
        camera_semseg_image = sensor_data.get("CameraSemSeg", None)

        if camera_depth_image is not None:
            if camera_semseg_image is not None:
                parkedcar_data = object_detection(camera_depth_image, camera_semseg_image, start_x, start_y)

        # calculate the parkedcar bbox points base on detection
        parkedcar_box_pts = obtain_parkedcar_lp([parkedcar_data])

        send_control_command(client, throttle=0.0, steer=0, brake=1.0)

        # history = [x_history, y_history, yaw_history, time_history, speed_history, collided_flag_history]
        history = history_init(start_x, start_y, start_yaw)

        # Vehicle Trajectory Live Plotting Setup
        lp_traj = lv.LivePlotter(tk_title="Trajectory Trace")
        lp_1d = lv.LivePlotter(tk_title="Controls Feedback")

        # Add 2D position / trajectory plot
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

        # Local Planner Variables
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

        # Scenario Execution Loop

        # Iterate the frames until the end of the waypoints is reached or
        # the TOTAL_EPISODE_FRAMES is reached. The controller simulation then
        # ouptuts the results to the controller output directory.
        reached_the_end = False
        skip_first_frame = True

        # Initialize the current timestamp.
        current_timestamp = start_timestamp

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

            # Store collision history
            collided_flag = get_player_collided_flag(measurement_data)
            current_state = [
                current_x,
                current_y,
                current_yaw,
                current_speed,
                current_timestamp,
                collided_flag,
            ]
            history_update(history, current_state)

            # Local Planner Update:
            # This will use the behavioural_planner.py and local_planner.py
            lead_car_pos, lead_car_length, lead_car_speed = load_lead_car(measurement_data)
            _update_local_planner = update_local_planner(frame, LP_FREQUENCY_DIVISOR)

            if _update_local_planner:
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
                lead_car_state = [
                    lead_car_pos[1][0],
                    lead_car_pos[1][1],
                    lead_car_speed[1],
                ]
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
                if bp._follow_lead_vehicle:
                    print("[INFO] Following the lead vehicle, lead vehicle speed: {:0.2f} km/h".format(3.6*lead_car_speed[1]))
                # --------------------------------------------------------------

                if local_waypoints != None:
                    local_waypoints_np = np.array(local_waypoints)
                    wp_distance = cal_waypoint_distance(local_waypoints_np)
                    wp_interp = cal_waypoint_interpolate(
                        wp_distance,
                        local_waypoints_np,
                        interp_distance_res=INTERP_DISTANCE_RES,
                    )

                    # Update the other controller values and controls
                    controller.update_waypoints(wp_interp)
                    pass

            # Controller Update
            if local_waypoints != None and local_waypoints != []:
                controller.update_values(
                    current_x,
                    current_y,
                    current_yaw,
                    current_speed,
                    current_timestamp,
                    frame,
                )
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
                if _update_local_planner:
                    trajectory_fig_local_path_update(
                        trajectory_fig,
                        NUM_PATHS,
                        path_validity,
                        collision_check_array,
                        best_index,
                        paths,
                        ego_state,
                    )
                # lookahead path plotter update
                trajectory_fig_lookahead_path_update(trajectory_fig, wp_interp, INTERP_MAX_POINTS_PLOT)

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
        store_trajectory_plot_all(
            trajectory_fig,
            forward_speed_fig,
            throttle_fig,
            brake_fig,
            steer_fig,
        )
        write_trajectory_file(history)
        write_collisioncount_file(collided_list=history[-1])


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
