import os
import configparser


def load_config(config_file):
    config = configparser.ConfigParser()
    config.read(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), config_file)
    )
    demo_opt = config["Demo Parameters"]
    return demo_opt


def get_options(demo_opt):
    # Get options
    enable_live_plot = demo_opt.get("live_plotting", "true").capitalize()
    enable_live_plot = enable_live_plot == "True"
    live_plot_period = float(demo_opt.get("live_plotting_period", 0))
    return enable_live_plot, live_plot_period
