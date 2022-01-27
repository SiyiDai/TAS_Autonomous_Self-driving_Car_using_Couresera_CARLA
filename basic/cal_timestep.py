WAIT_TIME_BEFORE_START = 1.00  # game seconds (time before controller start)
TOTAL_RUN_TIME = 100.00  # game seconds (total runtime before sim end)
TOTAL_FRAME_BUFFER = 300  # number of frames to buffer after total runtime


def regulate_num_iteration(num_iterations):
    # Determine simulation average timestep (and total frames)
    # Ensure at least one frame is used to compute average timestep
    num_iterations = num_iterations
    if num_iterations < 1:
        num_iterations = 1
    return num_iterations


def cal_total_episode(sim_duration, num_iterations):
    # Outputs average simulation timestep and computes how many frames
    # will elapse before the simulation should end based on various
    # parameters that we set in the beginning.
    SIMULATION_TIME_STEP = sim_duration / float(num_iterations)
    print("SERVER SIMULATION STEP APPROXIMATION: " + str(SIMULATION_TIME_STEP))
    total_episode_frames = int((TOTAL_RUN_TIME + WAIT_TIME_BEFORE_START) / SIMULATION_TIME_STEP) + TOTAL_FRAME_BUFFER
    return total_episode_frames
