# UAV Path And Trajectory Follower

This repository contains all the code for running this package on ROS. It is composed of two cpp-classes.

- [Generator](https://github.com/hecperleo/upat_follower/blob/icuas/src/generator.cpp) is in charge of generating a trajectory $\Gamma$ based on the ordered lists of 4D waypoints received. The generated trajectory is a much more dense list of 3D waypoints, which can be approximated to the continuous curve $\gamma(\lambda)$. It also generates an interpolated list $\tau(\lambda)$ of the initial times that matches the amount of the more dense list of waypoints. Both interpolated list are necessary for the presented framework to follow a 4D trajectory. 

- [Follower](https://github.com/hecperleo/upat_follower/blob/icuas/src/follower.cpp) receives the desired trajectory $\Gamma$ defined as a list of 4D waypoints. It may receive the look-ahead distance $d$ and the maximum speed $v_{max}$. Setting these parameters properly is key to get a good performance, depending on the desired 4D trajectory. A much more dense list of waypoints is required to apply the trajectory following method efficiently. Hence, it uses the trajectory generator to get a discrete curve $\gamma(\lambda)$ from the ordered list of waypoints $W$. Then, continuously, it receives the UAS pose $\mathbf{p}(t)$ and the actual time $t$ to generate the velocity commands $\mathbf{v}(t)$,

## Installation Instructions - Ubuntu 16.04 with ROS Kinetic

1. Follow the [instructions](https://github.com/grvcTeam/grvc-ual/wiki/How-to-build-and-install-grvc-ual) for install and build the [GRVC UAV abstraction layer](https://github.com/grvcTeam/grvc-ual) on release 3.0.
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
<!-- 5. Run tests to check everything. For this step, you need two terminals.

    terminal 1: `$ roscore`  
terminal 2: `$ catkin run_tests`

> **Note**: After this step, you can use just one terminal to run tests without roscore using this command.    
>`$ rostest upat_follower tests_run.tests`. -->


## How to use

Running this command in a terminal is enough to launch the simulation.

```
$ roslaunch upat_follower sim_empty.launch
```

[sim_empty](https://github.com/hecperleo/upat_follower/blob/icuas20/launch/sim_empty.launch) creates a node per UAV that communicates with UAL. This launch is an example of the whole project generating and following a trajectory. 
By default, you will see one UAV, if you want to see more UAVs you can turn on the flag `multi`, you also can turn off the flag `trajectory` to see how this project works with a path instead of a trajectory. If you want to use ROS interface instead of cpp-classes, you can turn off the flag `use_class` and see how this project works using ROS services and topics.

```
$ roslaunch upat_follower sim_empty.launch multi:=true trajectory:=false use_class:=false
```

> **Note**: Check [ual_communication](https://github.com/hecperleo/upat_follower/blob/icuas20/src/ual_communication.cpp) to see an example.

## C++ class interface

The Follower class is defined in follower.h. You can create one object in your code and use its public methods:

- `updatePose(const geometry_msgs::PoseStamped &_ual_pose)`
- `prepareTrajectory(nav_msgs::Path _init_path, std::vector<double> _times)`
- `preparePath(nav_msgs::Path _init_path, int _generator_mode, double _look_ahead, double _cruising_speed)`
- `updateTrajectory(nav_msgs::Path _new_target_path, nav_msgs::Path _new_target_vel_path)`
- `updatePath(nav_msgs::Path _new_target_path)`
- `getVelocity()`

The Generator class is defined in generator.h. You can create one object in your code and use its public methods:

- `generateTrajectory(nav_msgs::Path _init_path, std::vector<double> _times)`
- `generatePath(nav_msgs::Path _init_path, int _generator_mode)`


## ROS interface

You can interact with Follower and Generator classes using a ROS interface. 

Follower: 

- `PreparePath.srv`
- `PrepareTrajectory.srv`
- `UpdatePath.srv`
- `UpdateTrajectory.srv`

Generator: 

- `GeneratePath.srv`
- `GenerateTrajectory.srv`

Each service will interact with the corresponding cpp method. Create a client of these services with each corresponding requests and you will be able to interact with it and receive exactly the same response as using the cpp class interface.

## Generator and Follower Modes

Generator:

- `Mode 0`: Generate a path using linear interpolations

![Alt text](data/img/robot2019/mode0.png?raw=true)

- `Mode 1`: Generate a path using cubic spline interpolations (Step between mode `0` and mode `2`)

![Alt text](data/img/robot2019/mode1.png?raw=true)

- `Mode 2`: Generate a path using cubic spline interpolations

![Alt text](data/img/robot2019/mode2.png?raw=true)

- `Mode 3`: Generate a trajectory

Follower:

- `Mode 0`: Follow a path
- `Mode 1`: Follow a trajectory