# Frontier Exploration

Utility-based frontier exloration.

## Video
https://www.youtube.com/watch?v=YpDb5tQewAs

https://www.youtube.com/watch?v=MQJ5M0dEo20

https://www.youtube.com/watch?v=X4mQs9AEAl8


## Getting Started

* Clone or download to local workspace and build them.
* Make sure the environment is sourced.
* Launch gazebo, then run nav_mapping and demo.py

## Prerequisites

* tested with ros kinetic
* skimage

## Running the tests

```
roslaunch nav_demos/launch/gazebo.launch
```
```
roslaunch nav_demos/launch/nav_mapping.launch
```
```
rviz
```
```
rosrun my_frontier demo.py
```


## What can the code do?

* obtain a map (occupancy grid) and get candidates (frontiers) to be explored
* calculate the best goal to go/explore based on criteria including disance, information gain, and utility
* plan a path from the current pose to the goal
* move the robot and map the environment


## Some limitations

* Gmapping is a 2D SLAM implemetation and only 2D map can be created
* Gmapping will be slow when the map grows large
