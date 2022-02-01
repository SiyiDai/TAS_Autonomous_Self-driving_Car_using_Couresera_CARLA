

# Basic Module

In this module, the functions relate to basic settings are imlemented. **The scripts tagged with sign: * are originated from Coursera course.**

---
## Load Utils
This folder contains:

`config_params.py*` - storing config pamameters as variables for easy access

`load_config.py` - load config from `config_params.py`

`load_lead_car.py*` - load lead vehicle information from carla's `measurement_data.non_player_agents`

`load_parkedcar.py*` - load parked car from .txt file. Only use for test bevavioral planner, not used in the main code final version. In the project, the parked car is detection by perception module.

`load_stopsign.py*` - load stop sign parameters and convert it to input params for local planner

`load_waypoints.py` - load waypoints for route and stop sign

`params` - the folder contains information include config, route waypoints and stop sign parameters.

## Scripts

`argparser_helper.py*` - add arguements for argparser

`cal_timestep.py` - determine simulation average timestep (and total frames) with necessary constants

`get_pos.py` - obtains current and start x,y,yaw pose from the client measurements and the scene

`make_carla_settings.py*` - make a CarlaSettings object with the settings we need with necessary constants

`timer.py*` - Timer class: the steps are used to calculate FPS, while the lap or seconds since lap is used to compute elapsed time.