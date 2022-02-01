# Group 6 : Autonomous Self-driving Car using CARLA

## Contribution:
- Batu Kaan Özen - (Perception, Global Planner, whole system integration) 

- Siyi Dai       - (Local Planner, Collision Check, Behaviour Planner, Live Plotter Helpers, whole system integration, clean code and clean project converting, git maintaining)

- Yanni Zhang    - (Global planner, Perception, whole system integration)

- Yiming Wei      - (Local planner, Behaviour Planner, Controller Part, Car modeling)

## Instruction:

#### 1. Install Carla and read the setup guide:


- Linux (Ubuntu 16.04 or later)  --> [CARLA-Setup-Guide-Ubuntu.pdf](Carla_Setup_Guide/CARLA-Setup-Guide-_Ubuntu_.pdf)


- Windows 7 (64-bit) or later -->  [CARLA-Setup-Guide-Windows.pdf](Carla_Setup_Guide/CARLA-Setup-Guide-_Windows-x64_.pdf)

Here you can find the compressed package of Carla Simulator for Ubuntu and Windows: 

https://drive.google.com/drive/folders/14t-zNeNv2UtZi0UqcREMhMZ889Z12xjc?usp=sharing


#### 2. Install Dependencies
(NOTE: please run on **python version 3.5 or 3.6**.)

`python3 -m pip install -r ​ $HOME​ /opt/CarlaSimulator/requirements.txt
--user`


*Trouble shot*: most of matplotlib errors can be solved by downgrading to **matplotlib 3.0.3**

#### 3. Clone the Repository

In the path `~/opt/CarlaSimulator/PythonClient ` run

`git clone git@gitlab.lrz.de:tas202122/tas-project/group_6.git`


#### 4. Run the Project

Run following commands in terminal: 

**For Ubuntu system Linux (Ubuntu 16.04 or later):**

- In the path `~/opt/CarlaSimulator` run

    `./CarlaUE4.sh  /Game/Maps/Course4 -windowed -carla-server`


- In the path `~/opt/CarlaSimulator/PythonClient/group_6` run

    `python3 main.py`

**For Windows system:**

- In the root folder of Carla, run 

    `CarlaUE4.exe /Game/Maps/Course4 -windowed -carla-server -benchmark -fps=30   `       

- In the root folder of repository, run

    `python main.py`



**Note that there are difficulties running Carla with a virtual machine.**

**If errors occur, please refer to Carla setup guide in folder /Carla_Setup_Guide.**

****


## Note from students:

The logic of carla might seems like that it is easy and offering all posible autonomous driving libarires but there were some problems related to carla.

Firstly, our computers were not able to run last version of carla because of its high gpu memory usage, so we decided to 0.8.4 version of carla but this version libaries are not properly documented. We investigate their implementation to understand some algorithms but there were some algorithms, there were still dissuced from people in github. We also searched some completed autnomous driving project in Github but the main usage of carla was either reinforcement learning application or computer vision application (such as how SegNet performs semenntic segmentation.). There is not sufficient source to learn its logic except Carla source code. We would like to express that we gave a great effort for these implementations and project. 

****
## Description:

Aim of our project is establishg one autonosmous self driving car application using Local Planner, Computer Vision, Global Planner, Behaviour Planner knowledge in Carla enviroment.
**--> Please FILL HERE** 

## Overall Process of Our Project

1. Firstly, we followed courses "Introduction to Self-Driving Cars" and "Motion Planning for Self-Driving Cars" on Coursera to learn about self-driving cars and complete "TODO" tasks in `/local_planner/local_planner.py`, `/behavioural_planner/behavioural_planner.py` and `/collision_check/collision_checker.py`. 
Path_optimizer.py and velocity_planner.py were taken from the course assignment solution file. We solved all the assigments and watched all the course videos 

2. We managed to use semantic segmentation to distinguish different obstacles on the road, such as vehicles and pedestrians, and find the centroid of the obstacles as a representation of their positions. We call it obstacle detection part and tried to apply it during the driving of the vehicle. The obstacle detection part should be able to, as we planned, detect dynamic obstacles and update their positions in every frame. 

3. Because it was hard for us to find a way to directly read the map data from Carla files, we had to make the global planning offline. This part will be explained more detailed in **Global Planner module**. 
We also managed to implement a global planner, which can read map information from Carla server and give an optimal path. 

4. Intergration part is crushed and failed, but we completed our project of perception (for parked car detection), local planner, global planner and behavioural planner.





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

#### As the basic structure of our project was taken from the Coursera course, we believe we need to point out which scripts we have contributed to: 

**Base:** `main.py`. 

**Perception Part:** `/object_detection/object_detection.py`, `/object_detection/depth_to_pointcloud.py`. 

**Global Planner:** `/global_planner/misc.py`, `/global_planner/shortest_path_test.py`. 

**Local Planner:** `/local_planner/local_planner.py`. 

**Behaviour Planner:** `/behavioural_planner/behavioural_planner.py`, `/collision_check/collision_checker.py`. 


## Integration of all modules

This part is failed, so we build one automous driving car using Perception Part, Local Planner Part, Behaviour Planner Part
**--> Please FILL HERE, describe whole system connection and you can draw one diagram using one online readme editor ( https://pandao.github.io/editor.md/en.html)**









