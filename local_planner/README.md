
### Local Planner module

This module consist of three functions: 

1. `get_goal_state_set` derives a set of goal states including positions, yaw values, and velocities at the goal position, for a given goal index in waypoints. 
2. `plan_paths` derives a list of optimized spiral paths which satisfies the given set of goal states, using polynomial spiral optimization.
3. `transform_paths` converts the paths from the local (vehicle) coordinate frame to the global coordinate frame.

*The base code of this module is given from coursera course, we implement the parts tagged with **TODO** sessions.*

#### Logic for get_goal_state_set
    inputs:
    goal_index: Goal index for the vehicle to reach
                i.e. waypoints[goal_index] gives the goal waypoint
    goal_state: Goal state for the vehicle to reach (global frame)
                format: [x_goal, y_goal, v_goal], in units [m, m, m/s]
    waypoints: current waypoints to track. length and speed in m and m/s.
                (includes speed to track at each x,y location.) (global frame)
                format: [[x0, y0, v0],
                            [x1, y1, v1],
                            ...
                            [xn, yn, vn]]
                example:
                    waypoints[2][1]:
                    returns the 3rd waypoint's y position
                    waypoints[5]:
                    returns [x5, y5, v5] (6th waypoint)
    ego_state: ego state vector for the vehicle, in the global frame.
            format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                ego_x and ego_y     : position (m)
                ego_yaw             : top-down orientation [-pi to pi]
                ego_open_loop_speed : open loop speed (m/s)

    output:
    goal_state_set: Set of goal states (offsetted laterally from one
                    another) to be used by the local planner to plan multiple
                    proposal paths. This goal state set is in the vehicle frame.
                    format: [[x0, y0, t0, v0],
                            [x1, y1, t1, v1],
                            ...
                            [xm, ym, tm, vm]]
                    where m is the total number of goal states
                    [x, y, t] are the position and yaw values at each goal
                    v is the goal speed at the goal point.
                    all units are in m, m/s and radians

First of all we compute the **final heading based on the adjacent index**. If the goal index is the last in the set of waypoints, the previous index would be used. Otherwise, we use the next index. 

Next, **the input goal state in global frame is transformed to the ego vehicle's local frame**. The ego state is translated to the origin in the new frame. 

Then we **rotate the frame such that the ego state has zero heading in the new frame**. So that we can compute the goal yaw in the local frame by subtracting off the current ego yaw from the heading variable. 

Velocity is preserved after the transformation. The offset is computed and applied for each path such that all of the paths have the same heading of the goal state. Each offset goal will be used to generate a potential path to be considered by the local planner. 

#### Logic for plan_paths
input:

    goal_state_set: Set of goal states (offsetted laterally from one
                    another) to be used by the local planner to plan multiple
                    proposal paths. These goals are with respect to the vehicle
                    frame.
                    format: [[x0, y0, t0, v0],
                            [x1, y1, t1, v1],
                            ...
                            [xm, ym, tm, vm]]

where `m` is the total number of goal states. `[x, y, t]` are the position and yaw values at each goal.`v` is the goal speed at the goal point. All units are in m, m/s and radians.


output:

    paths: A list of optimized spiral paths which satisfies the set of goal states. A path is a list of points of the following format:
                    [x_points, y_points, t_points]:
                    x_points: List of x values (m) along the spiral
                    y_points: List of y values (m) along the spiral
                    t_points: List of yaw values (rad) along the spiral
                    
Note that this path is in the vehicle frame, since the optimize_spiral function assumes this to be the case.
                    
    path_validity: List of booleans classifying whether a path is valid
                    (true) or not (false) for the local planner to traverse. Each th
                    path_validity corresponds to the ith path in the path list.

#### Logic for transform_paths
    inputs:
    paths:  A list of paths in the local (vehicle) frame.
            A path is a list of points of the following format:
                [x_points, y_points, t_points]:
                x_points: List of x values (m)
                y_points: List of y values (m)
                t_points: List of yaw values (rad)
                Example of accessing the ith path, jth point's t value:
                    paths[i][2][j]
    ego_state: ego state vector for the vehicle, in the global frame.
            format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                ego_x and ego_y     : position (m)
                ego_yaw             : top-down orientation [-pi to pi]
                ego_open_loop_speed : open loop speed (m/s)
    output:
    transformed_paths: A list of transformed paths in the global frame.
                A path is a list of points of the following format:
                    [x_points, y_points, t_points]:
                    x_points: List of x values (m)
                    y_points: List of y values (m)
                    t_points: List of yaw values (rad)
                    Example of accessing the ith transformed path, jth point's y value:
                        paths[i][1][j]