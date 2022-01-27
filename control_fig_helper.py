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
    steer_fig.add_grph("steer", label="steer", window_size=window_size)
    return steer_fig
