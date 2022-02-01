# Group 6 : Autonomous Self-driving Car using CARLA

## Contribution:
- Batu Kaan Özen - (Perception, Global Planner, whole system Integration) 

- Siyi Dai       - (Local Planner, Behaviour Planner, whole system Integration, clean code and clean project converting)

- Yanni Zhang    - (Global planner, Perception, whole system Integration)

- Yiming Wei      - (Local planner, Behaviour Planner, Controller Part, Car modeling)

## Instruction:

### Building Carla, Running simulator enviroment and our implementation:
Please download the Carla version from our repository because the map used in our algorithm is only included in our version of carla.

--> please correct this part for TA, who should be able to download whole carla version and run .sh with /Game/Maps/Course4  map.


Following is the method to run Carla simulator and our algorithm. (NOTE: please run on python version 3.5 or 3.6.)

Run following commands in terminal: 

**For Ubuntu system:**

./CarlaUE4.sh  /Game/Maps/Course4 -windowed -carla-server    # (in the root folder of Carla)

python3 main.py     # (in the root folder of scripts)

**For Windows system:**

CarlaUE4.exe /Game/Maps/Course4 -windowed -carla-server -benchmark -fps=30

python main.py 

**Note that there are difficulties running Carla with a virtual machine.**

****


## Note from students:

The  logic of carla might seems like that it is easy and offering all posible autonomous driving libarires but there were some problems related to carla.
Firstly, our computers were not able to run last version of carla because of its high gpu memory usage, so we decided to 0.8.4 version of carla but this version libaries are not properly documented. We investigate their implementation to understand some algorithms but there were some algorithms, there were still dissuced from  people in github. We also searched some completed autnomous driving project in Github but the main usage of carla was either Reinforcement learning application or Computer vision application (such as how SegNet performs semenntic segmentation.). There is not sufficient source to learn it logic except Carla source code. We would like to express that we gave a great effort for these implementations and project. 

****
## Description:

Aim of our project is establishg one autonosmous self driving car application using Local Planner, Computer Vision, Global Planner, Behaviour Planner knowledge in Carla enviroment.
**--> Please FILL HERE** 

## Overall Process of Our Project

1. Firstly, we followed courses "Introduction to Self-Driving Cars" and "Motion Planning for Self-Driving Cars" on Coursera to learn about self-driving cars and complete "TODO" tasks in `/local_planner/local_planner.py`, `/behavioural_planner/behavioural_planner.py` and `/collision_check/collision_checker.py`. 
Path_optimizer.py and velocity_planner.py were taken from the course assignment solution file. We solved all the assigments and watched all the course videos 

2. We managed to use semantic segmentation to distinguish different obstacles on the road, such as vehicles and pedestrians, and find the centroid of the obstacles as a representation of their positions. We call it obstacle detection part and tried to apply it during the driving of the vehicle. The obstacle detection part should be able to, as we planned, detect dynamic obstacles and update their positions in every frame. 

3. Because it was hard for us to find a way to directly read the map data from Carla files, we had to make the global plan offline. This part will be explained more detailed in **Global Planner module**. 
We also managed to implement a global planner, which can read map information from Carla server and give an optimal path. 

4. All intergration part is crushed and failed, we completed our project using perception( for parked car detection), local planner, global planner and behavioural planner.





## Contents
- Our planned algorithm:
    - Perception Part
    - Global Planner Part
    - Local Planner Part
    - Behaviour Planner
    - Controller Part
    - Car modeling
    - Localization part
- Completed tasks:
    - Perception Part
    - Global Planner
    - Local Planner Part
    - Behaviour Planner
    - Controller Part
    - Car modeling
- Results:
    - Perception Part (worked)
    - Global Planner (worked)
    - Local Planner Part (worked)
    - Behaviour Planner (worked)
    - Controlller Part (worked)
    - Car Modeling (It is decided that using car modeling uncessarary for our project, we tuned pid experimentally and our research in internet. )
    - Localization part(found uncessarary)
    - Integration of all modules (failed)

#### As the basic structure of our project was taken from the COursera course, we believe we need to point out which scripts we have contributed to: 

**Perception Part Description:** `/object_detection/object_detection.py`, `/object_detection/depth_to_pointcloud.py`. 

**Global Planner Description:** `/global_planner/misc.py`, `/global_planner/shortest_path_test.py`. 

**Local Planner Descriptionn:** `/local_planner/local_planner.py`. 

**Behaviour Planner Description:** `/behavioural_planner/behavioural_planner.py`, `/collision_check/collision_checker.py`. 


## Integration of all modules

This part is failed, so we build one automous driving car using Perception Part, Local Planner Part, Behaviour Planner Part
--> Please FILL HERE, describe whole system connection and you can draw one diagram using one online readme editor ( https://pandao.github.io/editor.md/en.html)









