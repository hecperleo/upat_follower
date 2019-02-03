# UAV Path Manager

This repository contains all the code for running the uav_path_manager project, which is composed of three nodes mainly.

- [path_generator](https://github.com/hecperleo/uav_path_manager/blob/readme/src/path_generator.cpp) is responsible for receiving an initial path and generating an improved final path. Mainly improves the path using linear or cubic interpolations. 
- [path_follower](https://github.com/hecperleo/uav_path_manager/blob/readme/src/path_follower.cpp) is responsible for receiving the final path and generating a velocity to follow this path.
- [path_manager](https://github.com/hecperleo/uav_path_manager/blob/readme/src/path_manager.cpp) creates the initial path, communicates with path_generator to improve the path, tells path_follower which is the improved path and communicates with UAV to give the speed provided by path_follower.

## Installation Instructions - Ubuntu 16.04 with ROS Kinetic

1. Follow the [instructions](https://github.com/grvcTeam/grvc-ual/wiki/How-to-build-and-install-grvc-ual) for install and build the [GRVC UAV abstraction layer](https://github.com/grvcTeam/grvc-ual).
2. Clone this repository in your workspace.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/hecperleo/uav_path_manager.git
```
3. Install [ecl_geometry](http://wiki.ros.org/ecl_geometry).
```
$ sudo apt-get install ros-kinetic-ecl-geometry
```
4. Build and source your workspace.
```
$ cd ~/catkin_ws
$ catkin build
```
5. Run tests to check everything. For this step, you need two terminals.

    terminal 1: `$ roscore`  
terminal 2: `$ catkin run_tests`

> **Note**: After this step, you can use just one terminal to run tests without roscore using this command.    
>`$ rostest uav_path_manager tests_run.tests`.


## How to use UAV path manager

Running this command in a terminal is enough to launch the simulation.

```
$ roslaunch uav_path_manager mision_ual.launch
```

[mision_ual](https://github.com/hecperleo/uav_path_manager/blob/readme/launch/mision_ual.launch) creates a manager, a generator and a follower per UAV. This launch is an example of the whole project. 
By default, you will see two UAV, if you want to see just one UAV you can turn off the flag `multi`.

```
$ roslaunch uav_path_manager mision_ual.launch multi:=false
```


If you want to use this project, you should replace [path_manager](https://github.com/hecperleo/uav_path_manager/blob/readme/src/path_manager.cpp) to your node. Simply use [path_generator](https://github.com/hecperleo/uav_path_manager/blob/readme/src/path_generator.cpp) and [path_follower](https://github.com/hecperleo/uav_path_manager/blob/readme/src/path_follower.cpp). Those two nodes are servers, so you can communicate with them using [GeneratePath](https://github.com/hecperleo/uav_path_manager/blob/readme/srv/GeneratePath.srv) and [GetGeneratedPath](https://github.com/hecperleo/uav_path_manager/blob/readme/srv/GetGeneratedPath.srv) services. Check [path_manager](https://github.com/hecperleo/uav_path_manager/blob/readme/src/path_manager.cpp) to see how it works.
