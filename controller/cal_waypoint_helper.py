import numpy as np


def cal_waypoint_distance(local_waypoints_np):
    # Update the controller waypoint path with the best local path.
    # Linear interpolation computation on the waypoints
    # is also used to ensure a fine resolution between points.
    wp_distance = []  # distance array
    for i in range(1, local_waypoints_np.shape[0]):
        wp_distance.append(
            np.sqrt(
                (local_waypoints_np[i, 0] - local_waypoints_np[i - 1, 0]) ** 2
                + (local_waypoints_np[i, 1] - local_waypoints_np[i - 1, 1]) ** 2
            )
        )
    # last distance is 0 because it is the distance
    # from the last waypoint to the last waypoint
    wp_distance.append(0)
    return wp_distance


def cal_waypoint_interpolate(
    wp_distance, local_waypoints_np, interp_distance_res
):
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
        num_pts_to_interp = int(
            np.floor(wp_distance[i] / float(interp_distance_res)) - 1
        )
        wp_vector = local_waypoints_np[i + 1] - local_waypoints_np[i]
        wp_uvector = wp_vector / np.linalg.norm(wp_vector[0:2])

        for j in range(num_pts_to_interp):
            next_wp_vector = interp_distance_res * float(j + 1) * wp_uvector
            wp_interp.append(list(local_waypoints_np[i] + next_wp_vector))
    # add last waypoint at the end
    wp_interp.append(list(local_waypoints_np[-1]))
    return wp_interp
