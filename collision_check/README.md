
### Collision Check Module

In this module, **three main functions** for checking collision are implemented:

- `collision_check` takes in a set of paths and obstacles, and returns an array of bools that says whether or not each path is collision free.

- `select_best_path_index` selects a path index which is closest to the center line as well as far away from collision paths.

- `get_player_collided_flag` obtains collision flag from player. Check if any of the three collision metrics (vehicles, pedestrians, others) from the player are true, if so the player has collided to something.

*The base code of this module is given from coursera course, we implement the parts tagged with **TODO** sessions.*
#### Logic for collision_check

    paths: A list of paths in the global frame.
        A path is a list of points of the following format:
            [x_points, y_points, t_points]:
                x_points: List of x values (m)
                y_points: List of y values (m)
                t_points: List of yaw values (rad)

In order to **check for any potential collisions along each path with obstacles**, we compute the circle locations along this point in the path, which represents an approximate collision border for the vehicle.

    circle_x = point_x + circle_offset*cos(yaw)
    circle_y = point_y + circle_offset*sin(yaw)

Then we assumes each obstacle is approximated by a collection of points of the form [x, y].

    obstacles: A list of [x, y] points that represent points along the
        border of obstacles, in the global frame.
        Format: [[x0, y0],
                [x1, y1],
                ...,
                [xn, yn]]
        , where n is the number of obstacle points and units are [m, m]

We iterate through the obstacle points, and check **if any of the obstacle points lies within any of our circles**. If so, then the path will collide with an obstacle and the collision_free flag should be set to false for this flag.

*The base code of this function is given from coursera course, we implement the circle calculation part.*



#### Logic for select_best_path_index