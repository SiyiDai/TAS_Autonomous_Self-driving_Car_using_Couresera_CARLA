# Group_6
Running the carla and algorithm.
./CarlaUE4.sh -windowed -carla-server

python3 main.py (please version 3.5- 3.6)


# Modules Definitions

**main ** : 

**Global_Planner** : not implemented
  It finds sortestpath using Map of Carla simulation enviroment and outputs waypoints for agent from starting point to end point in the map... (Which should be defined in code not)

**Object_Detection** : not implemented
 It produces waypoints of the current obstacle(cars pedesterians)

**Velocity_Planner** : not  integrated

**local_planner** : not integrated

**controller2** : not integrated 

**behevioural_planner** : not integrated 

**path_optimiizer** : not integrated 

**collision_checker** : not integrated



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
-- Unnecessary for our project--



