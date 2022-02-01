import os

# controller output directory
CONTROLLER_OUTPUT_FOLDER = os.path.dirname(os.path.realpath(__file__)) + "/controller_output/"

def create_controller_output_dir(output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)


def store_trajectory_plot_all(trajectory_fig, forward_speed_fig, throttle_fig, brake_fig, steer_fig):
    store_trajectory_plot(trajectory_fig.fig, "trajectory.png")
    store_trajectory_plot(forward_speed_fig.fig, "forward_speed.png")
    store_trajectory_plot(throttle_fig.fig, "throttle_output.png")
    store_trajectory_plot(brake_fig.fig, "brake_output.png")
    store_trajectory_plot(steer_fig.fig, "steer_output.png")


def store_trajectory_plot(graph, fname):
    """Store the resulting plot."""
    create_controller_output_dir(CONTROLLER_OUTPUT_FOLDER)

    file_name = os.path.join(CONTROLLER_OUTPUT_FOLDER, fname)
    graph.savefig(file_name)


def write_trajectory_file(history):
    x_list = history[0]
    y_list = history[1]
    v_list = history[3]
    t_list = history[4]
    collided_list = history[5]
    create_controller_output_dir(CONTROLLER_OUTPUT_FOLDER)
    file_name = os.path.join(CONTROLLER_OUTPUT_FOLDER, "trajectory.txt")

    with open(file_name, "w") as trajectory_file:
        for i in range(len(x_list)):
            trajectory_file.write(
                "%3.3f, %3.3f, %2.3f, %6.3f %r\n" % (x_list[i], y_list[i], v_list[i], t_list[i], collided_list[i])
            )


def write_collisioncount_file(collided_list):
    create_controller_output_dir(CONTROLLER_OUTPUT_FOLDER)
    file_name = os.path.join(CONTROLLER_OUTPUT_FOLDER, "collision_count.txt")

    with open(file_name, "w") as collision_file:
        collision_file.write(str(sum(collided_list)))
