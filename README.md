# Group_6

# Method of running our project
Following is the method to run Carla simulator and our algorithm. (NOTE: please run on python version 3.5 or 3.6.)

Run following commands in terminal: 

./CarlaUE4.sh -windowed -carla-server    # (in the root folder of Carla)
python3 main.py     # (in the root folder of scripts)

# Modules Definitions

**main** :   Place all operations in to     def _on_loop(self): part for autonomous driving.

**Global_Planner** : not implemented
  It finds sortestpath using Map of Carla simulation enviroment and outputs waypoints for agent from starting point to end point in the map... (Which should be defined in code not)

**Object_Detection** : not implemented
 It produces waypoints of the current obstacle(cars pedesterians)


**The below models are supposed use information of Global_planner waypoints and Object decetion waypoints.**

**Velocity_Planner** : not  integrated, found some algorithm and code but not integrated.

**local_planner** : not integrated, found some algorithm and code but not integrated.

**controller2** : not integrated , found some algorithm and code but not integrated.

**behevioural_planner** : not integrated , found some algorithm and code but not integrated.

**path_optimiizer** : not integrated , found some algorithm and code but not integrated.

**collision_checker** : not integrated, found some algorithm and code but not integrated.



#  Project Sources:
https://carla.readthedocs.io/en/0.9.2/python_api_tutorial/#vehicles

https://carla.readthedocs.io/en/latest/tuto_M_custom_map_overview/

https://arijitray1993.github.io/CARLA_tutorial/

https://carla.readthedocs.io/en/latest/start_introduction/

https://www.coursera.org/learn/intro-self-driving-cars/home/welcome

# Project Modules:

1. System modelling and controller desing -- Unnecessary for our project--
2. Vehicle localization using sensor fusion(Inertial measurement unit- GPNS/GNSS - Car modelling) / or it might be also possible that finding exact location using  carla... -- Unnecessary for our project--
-- link for locating vehicle and motion model:
https://carla.readthedocs.io/en/0.9.2/python_api_tutorial/#vehicles
-- Unnecessary for our project--

3. Computer vision part (Segmented segmentation- Finding driveable area)
  -- Helpfull link for computer vision --> https://carla.readthedocs.io/en/0.9.2/cameras_and_sensors/#sensorcamerasemantic_segmentation
-- Unnecessary for our project--
4. Global Planning- Local Planning- behavior planning  -Motion Planning
-- Unnecessary for our project-- ### ?

# What we have achieved/ tried to do: 

1. We followed courses "Introduction to Self-Driving Cars" and "Motion Planning for Self-Driving Cars" on Coursera to learn about self-driving cars and complete "TODO" tasks in local_planner/local_planner.py, local_planner/behavioural_planner.py and collision_check/collision_checker.py. 

2. semantic segmentation (used semantic segmentation to distinguish different obstacles --> vehicles, pedestrians, find the centroid of the obstacles as representation of their position)

3. global planner (extracted waypoints and used A* to find the shortest path)

4. 







