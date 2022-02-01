# Group 6 : Autonomous self driving Car using CARLA

- Batu Kaan Özen - (Perception, Global Planner, whole system Integration) 

- Siyi Dai       - (Local Planner, Behaviour Planner, whole system Integration, clean code and clean project converting)

- Yanni Zhang    - (Global planner, Perception, whole system Integration)

- Yiming Wei      - (Local planner, Behaviour Planner, Controller Part, Car modeling)

## Note from students:

The  logic of carla might seems like that it is easy and offering all posible autonomous driving libarires but there were some problems related to carla.
Firstly, our computers were not able to run last version of carla because of its high gpu memory usage, so we decided to 0.8.4 version of carla but this version libaries are not properly documented. We investigate their implementation to understand some algorithms but there were some algorithms, there were still dissuced from  people in github. We also searched some completed autnomous driving project in Github but the main usage of carla was either Reinforcement learning application or Computer vision application (such as how SegNet performs semenntic segmentation.). There is not sufficient source to learn it logic except Carla source code. We would like to express that we gave a great effort for these implementations and project. 


****

## Description:

Aim of our project is establishg one autonosmous self driving car application using Local Planner, Computer Vision, Global Planner, Behaviour Planner knowledge in Carla enviroment.
**--> Please FILL HERE** 

## Overall Process of Our Project

1. Firstly, we followed courses "Introduction to Self-Driving Cars" and "Motion Planning for Self-Driving Cars" on Coursera to learn about self-driving cars and complete "TODO" tasks in local_planner/local_planner.py, local_planner/behavioural_planner.py and collision_check/collision_checker.py. 
Path_optimizer.py and velocity_planner.py were taken from the course assignment solution file. We solved all the assigments and watched all the course videos 

2. We managed to use semantic segmentation to distinguish different obstacles on the road, such as vehicles and pedestrians, and find the centroid of the obstacles as a representation of their positions. We call it obstacle detection part and tried to apply it during the driving of the vehicle. The obstacle detection part should be able to, as we planned, detect dynamic obstacles and update their positions in every frame. 

3. Because it was hard for us to find a way to directly read the map data from Carla files, we had to make the global plan offline. This part will be explained more detailed in **Global Planner module**. 
We also managed to implement a global planner, which can read map information from Carla server and give an optimal path. 

4. All intergration part is crushed and failed, we completed our project using perception( for parked car detection), local planner, global planner and behavioural planner.



## Building Carla, Running simulator enviroment and our implementation:
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
    - Car Modeling ( It is decided that using car modeling uncessarary for our project, we tuned pid experimentally and our research in internet. )
    - Localization part(found uncessarary)
    - Integration of all modules (failed)


**Perception Part Description:**

**Global Planner Description:**

**Local Planner Descriptionn:**

**Behaviour Planner Description:**

**Controller Part:**

**Car Modeling Part: **


## Integration of all modules

This part is failed, so we build one automous driving car using Perception Part, Local Planner Part, Behaviour Planner Part
--> Please FILL HERE, describe whole system connection and you can draw one diagram using one online readme editor ( https://pandao.github.io/editor.md/en.html)


### Perception module

For locating object, we used carla depth image and segmented image. After having depth and segmented information, we used threshold of vehicle to find the pixel area of vehicle on segmented image and we employed regiong prob algorithm to find center point of each vehicle.  Afterwards we converted each center point of vehicle using inverse calibration matrix as follow.
![alt text](./calib.png)

![alt text](./project.png)

The main problem related to this algorithm was finding focal lenght because we had just information of FOV angle ( Field of view). After that we used fov angle to compute focal lenght and our camera intristic matrix will be like below:
   **(Intrinsic) K Matrix**
    k = numpy.identity(3)
    k[0, 2] = image.width / 2.0
    k[1, 2] = image.height / 2.0
    k[0, 0] = k[1, 1] = image.width / (2.0 * math.tan(image.fov * math.pi / 360.0))

After that we applied inverse operation to intristic matrix and find our point cloud location. Using knowledge of our car and point cloud distance of object, we calculated the location of object on map. 



### Global Planner module

As the waypoints we got from map do not have a graph-structure, but are only discreted points, we used NearestNeighbors from sklearn package of python to create connection among the nearest points. Every waypoint was set as a node in a Networkx graph and for edges we considered the connections between the nearest neighbours. That is how we created node graph from discreted waypoints distribution. 

After that, we applied A* algorithm on the node graph we extracted from the waypoints and got the shortest path from starting node to goal node. For heuristic function we used the distance from the current node to goal node. For edge weight we considered euclidean distance between two neighbours. 



### Local Planner module

--> Please FILL HERE 

### Behaviour Planner

--> Please FILL HERE 

### Controller Part

For following route, PID controller is employed. 

--> Please FILL HERE 

### Localization module
 Carla offers us exact location and exact speed of car, so it is found that implementiong one EKF algorithm unnecessary.

### Car Modeling
 Carla offers us exact location and exact speed of car, so it is found that employing one car model unnecessary.


