# Behaviour Planner

This module consists of five main functions: `transition_state`, `get_goal_index`, `check_for_stop_signs`, `check_for_lead_vehicle`, and `get_closest_index`. 

The behaviour planner focuses on high level decision making. It is required to follow the traffic rule and make safe decisions in a driving scenario. 

We utilized a **finite state machine** to implement a behaviour planner. 

1. `transition_state` is used to transit and compute the goal states.
2. `get_goal_index` is used to find the earliest waypoint that has accumulated arc length (including closest_len) that is greater than or equal to self._lookahead.
3. `check_for_stop_signs` checks for a stop sign that is intervening the goal path.
4. `check_for_lead_vehicle` checks for lead vehicle within the proximity of the ego car, such that the ego car should begin to follow the lead vehicle.
5. `get_closest_index` gets closest index a given list of waypoints to the vehicle position.
---
## Logic for transition_state
inputs:

    waypoints:  current waypoints to track (global frame).
                length and speed in m and m/s.
                (includes speed to track at each x,y location.)
                format: [[x0, y0, v0],
                        [x1, y1, v1],
                        ...
                        [xn, yn, vn]]

    ego_state:  ego state vector for the vehicle. (global frame)
                format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                ego_x and ego_y     : position (m)
                ego_yaw             : top-down orientation [-pi to pi]
                ego_open_loop_speed : open loop speed (m/s)
                closed_loop_speed: current (closed-loop) speed for vehicle (m/s)

This function realizes a finite state machine. The state machine consists of three states: follow lane, decelerate to stop, and stay stopped, so that it can handle a stop sign.

If the vehicle is in follow lane state and a stop sign is found, the state transits to the decelerate to stop state, and the goal state is set to zero. 

    variables to set:
                self._goal_index: Goal index for the vehicle to reach
                    i.e. waypoints[self._goal_index] gives the goal waypoint
                self._goal_state: Goal state for the vehicle to reach (global frame)
                    format: [x_goal, y_goal, v_goal]
                self._state: The current state of the vehicle.
                    available states:
                        FOLLOW_LANE         : Follow the global waypoints (lane).
                        DECELERATE_TO_STOP  : Decelerate to stop.
                        STAY_STOPPED        : Stay stopped.
                self._stop_count: Counter used to count the number of cycles which the vehicle was in the STAY_STOPPED state so far.

When the vehicle is in the decelerate to stop state, it should be checked if we have reached a complete stop which is determined by a speed threshold, the state transits to stay stop state.

If the vehicle is in stay stopped and has stayed stopped for the required time, and no stop sign is found, the state returns to follow lane state. The required stop time is measured by STOP_COUNTS.

    useful_constants:
                STOP_THRESHOLD: Stop speed threshold (m). The vehicle should fully stop when its speed falls within this threshold.
                STOP_COUNTS: Number of cycles (simulation iterations) before moving from stop sign.
#### Logic for get_goal_index
    waypoints:  current waypoints to track. (global frame)
                length and speed in m and m/s.
                (includes speed to track at each x,y location.)
                format: [[x0, y0, v0],
                        [x1, y1, v1],
                        ...
                        [xn, yn, vn]]
                example:
                    waypoints[2][1]:
                    returns the 3rd waypoint's y position
                    waypoints[5]:
                    returns [x5, y5, v5] (6th waypoint)
    ego_state:  ego state vector for the vehicle. (global frame)
                format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                    ego_x and ego_y     : position (m)
                    ego_yaw             : top-down orientation [-pi to pi]
                    ego_open_loop_speed : open loop speed (m/s)
    closest_len: length (m) to the closest waypoint from the vehicle.
    closest_index: index of the waypoint which is closest to the vehicle.
                i.e. waypoints[closest_index] gives the waypoint closest to the vehicle.
## Logic for check_for_stop_signs
    inputs:
    waypoints:  current waypoints to track. (global frame)
                length and speed in m and m/s.
                (includes speed to track at each x,y location.)
                format: [[x0, y0, v0],
                            [x1, y1, v1],
                            ...
                            [xn, yn, vn]]
                example:
                    waypoints[2][1]:
                    returns the 3rd waypoint's y position
                    waypoints[5]:
                    returns [x5, y5, v5] (6th waypoint)
                closest_index: index of the waypoint which is closest to the vehicle.
                    i.e. waypoints[closest_index] gives the waypoint closest to the vehicle.
                goal_index (current): Current goal index for the vehicle to reach
                    i.e. waypoints[goal_index] gives the goal waypoint
    variables to set:
                [goal_index (updated), stop_sign_found]:
                    goal_index (updated): Updated goal index for the vehicle to reach
                    i.e. waypoints[goal_index] gives the goal waypoint
                stop_sign_found: Boolean flag for whether a stop sign was found or not
## Logic for check_for_lead_vehicle
    ego_state:  ego state vector for the vehicle. (global frame)
                format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                    ego_x and ego_y     : position (m)
                    ego_yaw             : top-down orientation [-pi to pi]
                    ego_open_loop_speed : open loop speed (m/s)
    lead_car_position: The [x, y] position of the lead vehicle.
                Lengths are in meters, and it is in the global frame.
    sets: self._follow_lead_vehicle: Boolean flag on whether the ego vehicle should follow (true) the lead car or not (false).

## Logic for get_closest_index
    inputs:
    waypoints:  current waypoints to track. (global frame)
                length and speed in m and m/s.
                (includes speed to track at each x,y location.)
                format: [[x0, y0, v0],
                        [x1, y1, v1],
                        ...
                        [xn, yn, vn]]

    ego_state:  ego state vector for the vehicle. (global frame)
                format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                    ego_x and ego_y     : position (m)
                    ego_yaw             : top-down orientation [-pi to pi]
                    ego_open_loop_speed : open loop speed (m/s)

    output:
    [closest_len, closest_index]:
                closest_len: length (m) to the closest waypoint from the vehicle.
                closest_index: index of the waypoint which is closest to the vehicle.
                    i.e. waypoints[closest_index] gives the waypoint closest to the vehicle.