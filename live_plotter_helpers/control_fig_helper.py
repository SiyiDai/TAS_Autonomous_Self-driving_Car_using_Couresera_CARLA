def control_fig_update(
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
):
    forward_speed_fig.roll("forward_speed", current_timestamp, current_speed)
    forward_speed_fig.roll("reference_signal", current_timestamp, controller._desired_speed)
    throttle_fig.roll("throttle", current_timestamp, cmd_throttle)
    brake_fig.roll("brake", current_timestamp, cmd_brake)
    steer_fig.roll("steer", current_timestamp, cmd_steer)


def add_forward_speed_fig(lp_1d, window_size):
    # Add 1D speed profile updater
    forward_speed_fig = lp_1d.plot_new_dynamic_figure(title="Forward Speed (m/s)")
    forward_speed_fig.add_graph("forward_speed", label="forward_speed", window_size=window_size)
    forward_speed_fig.add_graph("reference_signal", label="reference_Signal", window_size=window_size)
    return forward_speed_fig


def add_throttle_fig(lp_1d, window_size):
    # Add throttle signals graph
    throttle_fig = lp_1d.plot_new_dynamic_figure(title="Throttle")
    throttle_fig.add_graph("throttle", label="throttle", window_size=window_size)
    return throttle_fig


def add_brake_fig(lp_1d, window_size):
    # Add brake signals graph
    brake_fig = lp_1d.plot_new_dynamic_figure(title="Brake")
    brake_fig.add_graph("brake", label="brake", window_size=window_size)
    return brake_fig


def add_steer_fig(lp_1d, window_size):
    # Add steering signals graph
    steer_fig = lp_1d.plot_new_dynamic_figure(title="Steer")
    steer_fig.add_graph("steer", label="steer", window_size=window_size)
    return steer_fig
