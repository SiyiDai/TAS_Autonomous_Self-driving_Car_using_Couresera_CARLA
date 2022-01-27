def history_init(start_x, start_y, start_yaw):
    x_history = [start_x]
    y_history = [start_y]
    yaw_history = [start_yaw]
    time_history = [0]
    speed_history = [0]
    collided_flag_history = [False]  # assume player starts off non-collided
    history = [x_history, y_history, yaw_history, speed_history, time_history, collided_flag_history]
    return history


def history_update(history, current_state):
    # current_state = [current_x, current_y, current_yaw, current_speed, current_timestamp, collided_flag]
    history[0].append(current_state[0])  # x_history
    history[1].append(current_state[1])  # y_history
    history[2].append(current_state[2])  # yaw_history
    history[3].append(current_state[3])  # speed_history
    history[4].append(current_state[4])  # time_history
    history[5].append(current_state[5])  # collided_flag_history
