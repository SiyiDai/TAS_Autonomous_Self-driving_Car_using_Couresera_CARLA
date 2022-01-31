import numpy as np


def trajectory_fig_upadte(trajectory_fig, current_x, current_y, lead_car_pos):
    # Update live plotter with new feedback
    trajectory_fig.roll("trajectory", current_x, current_y)
    trajectory_fig.roll("car", current_x, current_y)
    if lead_car_pos:  # If there exists a lead car, plot it
        trajectory_fig.roll("leadcar", lead_car_pos[1][0], lead_car_pos[1][1])


def trajectory_fig_lookahead_path_update(trajectory_fig, wp_interp, interp_max):
    # When plotting lookahead path, only plot a number of points. This is meant
    # to decrease load when live plotting
    wp_interp_np = np.array(wp_interp)
    path_indices = np.floor(np.linspace(0, wp_interp_np.shape[0] - 1, interp_max))
    trajectory_fig.update(
        "selected_path",
        wp_interp_np[path_indices.astype(int), 0],
        wp_interp_np[path_indices.astype(int), 1],
        new_colour=[1, 0.5, 0.0],
    )


def trajectory_fig_local_path_update(
    trajectory_fig,
    num_paths,
    path_validity,
    collision_check_array,
    best_index,
    paths,
    ego_state,
):
    path_counter = 0
    for i in range(num_paths):
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
                "local_path " + str(i),
                paths[path_counter][0],
                paths[path_counter][1],
                colour,
            )
            path_counter += 1
        else:
            trajectory_fig.update("local_path " + str(i), [ego_state[0]], [ego_state[1]], "r")


def trajectory_fig_set_all(
    trajectory_fig,
    waypoints_np,
    start_x,
    start_y,
    stopsign_fences,
    parkedcar_box_pts,
    num_paths,
    window_size_traj,
    window_size_lookahead,
):
    trajectory_fig_add_waypoints(trajectory_fig, waypoints_np)
    trajectory_fig_add_trajectory_markers(trajectory_fig, start_x, start_y, window_size=window_size_traj)
    trajectory_fig_add_starting_position(trajectory_fig, start_x, start_y)
    trajectory_fig_add_end_position(trajectory_fig, waypoints_np)
    trajectory_fig_add_car_marker(trajectory_fig)
    trajectory_fig_add_lead_car_information(trajectory_fig)
    trajectory_fig_add_stop_sign(trajectory_fig, stopsign_fences)
    trajectory_fig_add_parked_car_points(trajectory_fig, parkedcar_box_pts)
    trajectory_fig_add_lookahead_path(trajectory_fig, start_x, start_y, window_size=window_size_lookahead)
    trajectory_fig_add_local_path_proposals(trajectory_fig, num_paths)


def trajectory_fig_initialize(lp_traj, fig_size, plot_rect):
    trajectory_fig = lp_traj.plot_new_dynamic_2d_figure(
        title="Vehicle Trajectory",
        figsize=fig_size,
        edgecolor="black",
        rect=plot_rect,
    )
    trajectory_fig.set_invert_x_axis()  # Because UE4 uses left-handed
    # coordinate system the X
    # axis in the graph is flipped
    trajectory_fig.set_axis_equal()  # X-Y spacing should be equal in size
    return trajectory_fig


def trajectory_fig_add_car_marker(trajectory_fig):
    # Add car marker
    trajectory_fig.add_graph(
        "car",
        window_size=1,
        marker="s",
        color="b",
        markertext="Car",
        marker_text_offset=1,
    )


def trajectory_fig_add_lead_car_information(trajectory_fig):
    # Add lead car information
    trajectory_fig.add_graph(
        "leadcar",
        window_size=1,
        marker="s",
        color="g",
        markertext="Lead Car",
        marker_text_offset=1,
    )


def trajectory_fig_add_parked_car_points(trajectory_fig, parkedcar_box_pts):
    # Load parked car points
    parkedcar_box_pts_np = np.array(parkedcar_box_pts)
    trajectory_fig.add_graph(
        "parkedcar_pts",
        window_size=parkedcar_box_pts_np.shape[0],
        x0=parkedcar_box_pts_np[:, 0],
        y0=parkedcar_box_pts_np[:, 1],
        linestyle="",
        markertext="Detected Parked Car",
        marker="+",
        color="b",
    )


def trajectory_fig_add_local_path_proposals(trajectory_fig, num_paths):
    # Add local path proposals
    for i in range(num_paths):
        trajectory_fig.add_graph(
            "local_path " + str(i),
            window_size=200,
            x0=None,
            y0=None,
            color=[0.0, 0.0, 1.0],
        )


def trajectory_fig_add_lookahead_path(trajectory_fig, start_x, start_y, window_size):
    # Add lookahead path
    trajectory_fig.add_graph(
        "selected_path",
        window_size=window_size,
        x0=[start_x] * window_size,
        y0=[start_y] * window_size,
        color=[1, 0.5, 0.0],
        linewidth=3,
    )


def trajectory_fig_add_waypoints(trajectory_fig, waypoints_np):
    # Add waypoint markers
    trajectory_fig.add_graph(
        "waypoints",
        window_size=waypoints_np.shape[0],
        x0=waypoints_np[:, 0],
        y0=waypoints_np[:, 1],
        linestyle="-",
        marker="",
        color="g",
    )


def trajectory_fig_add_trajectory_markers(trajectory_fig, start_x, start_y, window_size):
    # Add trajectory markers
    trajectory_fig.add_graph(
        "trajectory",
        window_size=window_size,
        x0=[start_x] * window_size,
        y0=[start_y] * window_size,
        color=[1, 0.5, 0],
    )


def trajectory_fig_add_end_position(trajectory_fig, waypoints_np):
    # Add end position marker
    end_x = waypoints_np[-1, 0]
    end_y = waypoints_np[-1, 1]
    trajectory_fig.add_graph(
        "end_pos",
        window_size=1,
        x0=[end_x],
        y0=[end_y],
        marker="D",
        color="r",
        markertext="End",
        marker_text_offset=1,
    )


def trajectory_fig_add_starting_position(trajectory_fig, start_x, start_y):
    # Add starting position marker
    trajectory_fig.add_graph(
        "start_pos",
        window_size=1,
        x0=[start_x],
        y0=[start_y],
        marker=11,
        color=[1, 0.5, 0],
        markertext="Start",
        marker_text_offset=1,
    )


def trajectory_fig_add_stop_sign(trajectory_fig, stopsign_fences):
    # Add stop sign position
    trajectory_fig.add_graph(
        "stopsign",
        window_size=1,
        x0=[stopsign_fences[0][0]],
        y0=[stopsign_fences[0][1]],
        marker="H",
        color="r",
        markertext="Stop Sign",
        marker_text_offset=1,
    )
    # Add stop sign "stop line"
    trajectory_fig.add_graph(
        "stopsign_fence",
        window_size=1,
        x0=[stopsign_fences[0][0], stopsign_fences[0][2]],
        y0=[stopsign_fences[0][1], stopsign_fences[0][3]],
        color="r",
    )
