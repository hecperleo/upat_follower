# UAV Path And Trajectory Follower

This repository contains all the code for running the **upat_follower** package on ROS. It is composed of two cpp-classes.

- [Generator](https://github.com/hecperleo/upat_follower/blob/dev/src/generator.cpp) is responsible for receiving an initial path and generating an improved final path or trajectory. Mainly improves the path using linear or cubic interpolations. If a trajectory must be generated, this node also needs the percentage of maximum speed at every segment of the initial path.
- [Follower](https://github.com/hecperleo/upat_follower/blob/dev/src/follower.cpp) is responsible for receiving a path or trajectory and generating a velocity to follow it.

## Installation Instructions - Ubuntu 16.04 with ROS Kinetic

1. Follow the [instructions](https://github.com/grvcTeam/grvc-ual/wiki/How-to-build-and-install-grvc-ual) for install and build the [GRVC UAV abstraction layer](https://github.com/grvcTeam/grvc-ual).
2. Clone this repository in your workspace.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/hecperleo/upat_follower.git
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
>`$ rostest upat_follower tests_run.tests`.


## How to use

Running this command in a terminal is enough to launch the simulation.

```
$ roslaunch upat_follower mision_ual.launch
```

[mision_ual](https://github.com/hecperleo/upat_follower/blob/dev/launch/mision_ual.launch) creates a node per UAV that communicates with UAL. This launch is an example of the whole project generating and following a trajectory. 
By default, you will see two UAV, if you want to see just one UAV you can turn off the flag `multi`, you also can turn off the flag `trajectory` to see how this project works with a path instead of a trajectory. If you want to use ROS interface instead of cpp-classes, you can turn off the flag `use_class` and see how this project works using ROS services and topics.

```
$ roslaunch upat_follower mision_ual.launch multi:=false trajectory:=false use_class:=false
```

> **Note**: Check [ual_communication](https://github.com/hecperleo/upat_follower/blob/dev/src/ual_communication.cpp) to see an example.

## C++ class interface

The Follower class is defined in follower.h. You can create one object in your code and use its public methods:

- `updatePose(const geometry_msgs::PoseStamped &_ual_pose)`
- `prepareTrajectory(nav_msgs::Path _init_path, std::vector<double> _max_vel_percentage)`
- `preparePath(nav_msgs::Path _init_path, int _generator_mode, double _look_ahead, double _cruising_speed)`
- `getVelocity()`

The Generator class is defined in generator.h. You can create one object in your code and use its public methods:

- `generateTrajectory(nav_msgs::Path _init_path, std::vector<double> _max_vel_percentage)`
- `generatePath(nav_msgs::Path _init_path, int _generator_mode)`


## ROS interface

You can interact with Follower and Generator classes using a ROS interface. 

- `GeneratePath.srv`
- `PreparePath.srv`
- `PrepareTrajectory.srv`

Create a client of these services with each corresponding requests and you will be able to interact with it and recive exactly the same response as using the cpp class interface.

## Follower and Generator Modes

Generator:

- `Mode 0`: Generate a path using linear interpolations
- `Mode 1`: Generate a path using cubic spline interpolations (Step between mode `0` and mode `2`)
- `Mode 2`: Generate a path using cubic spline interpolations
- `Mode 3`: Generate a trajectory

Follower:

- `Mode 0`: Follow a path
- `Mode 1`: Follow a trajectory