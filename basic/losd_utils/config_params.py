"""
Configurable params
"""
ITER_FOR_SIM_TIMESTEP = 10  # no. iterations to compute approx sim timestep

PLAYER_START_INDEX = 1  # spawn index for player (keep to 1)
FIGSIZE_X_INCHES = 8  # x figure size of feedback in inches
FIGSIZE_Y_INCHES = 8  # y figure size of feedback in inches
PLOT_LEFT = 0.1  # in fractions of figure width and height
PLOT_BOT = 0.1
PLOT_WIDTH = 0.8
PLOT_HEIGHT = 0.8

WAYPOINTS_FILENAME = "params/route_waypoints.txt"  # waypoint file to load
DIST_THRESHOLD_TO_LAST_WAYPOINT = 2.0  # some distance from last position before
# simulation ends

# Planning Constants
NUM_PATHS = 7
BP_LOOKAHEAD_BASE = 10.0  # 8m
BP_LOOKAHEAD_TIME = 5.0  # s
PATH_OFFSET = 1.5  # m
CIRCLE_OFFSETS = [-1.0, 1.0, 3.0]  # m
CIRCLE_RADII = [1.5, 1.5, 1.5]  # m
TIME_GAP = 1.0  # s
PATH_SELECT_WEIGHT = 10
A_MAX = 2.5  # m/s^2  1.5
SLOW_SPEED = 2.0  # m/s
STOP_LINE_BUFFER = 3.5  # m
LEAD_VEHICLE_LOOKAHEAD = 20.0  # m
LP_FREQUENCY_DIVISOR = 2  # Frequency divisor to make the
# local planner operate at a lower
# frequency than the controller
# (which operates at the simulation
# frequency). Must be a natural
# number.

# Course 4 specific parameters
C4_STOP_SIGN_FILE = "params/stop_sign_params.txt"
C4_STOP_SIGN_FENCELENGTH = 5  # m
C4_PARKED_CAR_FILE = "params/parked_vehicle_params.txt"

CONFIG_FILE = "params/options.cfg"

# Path interpolation parameters
INTERP_MAX_POINTS_PLOT = 10  # number of points used for displaying
# selected path
INTERP_DISTANCE_RES = 0.01  # distance between interpolated points

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
MINI_WINDOW_WIDTH = 320
MINI_WINDOW_HEIGHT = 180
