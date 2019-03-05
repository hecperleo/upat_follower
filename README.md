# UAV Path Manager

This repository contains all the code for running the uav_path_manager project, which is composed of three nodes mainly.

- [path_generator](https://github.com/hecperleo/uav_path_manager/blob/master/src/path_generator.cpp) is responsible for receiving an initial path and generating an improved final path or trajectory. Mainly improves the path using linear or cubic interpolations. If a trajectory must be generated, this node also needs the percentage of maximum speed at every segment of the initial path.
- [path_follower](https://github.com/hecperleo/uav_path_manager/blob/master/src/path_follower.cpp) is responsible for receiving the final path or trajectory and generating a velocity to follow it.
- [path_manager](https://github.com/hecperleo/uav_path_manager/blob/master/src/path_manager.cpp) creates the initial path, communicates with path_generator to improve the path or generate a trajectory, tells path_follower which is the improved path or trajectory and communicates with UAV to give the speed provided by path_follower.

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

[mision_ual](https://github.com/hecperleo/uav_path_manager/blob/master/launch/mision_ual.launch) creates a manager, a generator and a follower per UAV. This launch is an example of the whole project generating and following a trajectory. 
By default, you will see two UAV, if you want to see just one UAV you can turn off the flag `multi`, you also can turn off the flag `trajectory` to see how this project works with a path instead of a trajectory.

```
$ roslaunch uav_path_manager mision_ual.launch multi:=false trajectory:=false
```


If you want to use this project, you should replace _path_manager_ to your node. Simply use _path_generator_ and _path_follower_. Those two nodes are servers, so you can communicate with them using GeneratePath and FollowPath services. 

> **Note**: Check [path_manager](https://github.com/hecperleo/uav_path_manager/blob/master/src/path_manager.cpp) to see an example.

### Nodes Modes

You can change the mode of a node using _generator_mode_ or _follower_mode_. To use a path you can use modes 1,2 or 3 of _generator_mode_ and you must use mode 1 of follower_mode. You also need to fill _init_path_ at service GeneratePath. At service FollowPath you need to fill _look_ahead_ and _cruising_speed_. If you want to generate a trajectory, you must use mode 4 of _generator_mode_ and mode 2 of _follower_mode_. 

You need to connect GeneratePath outputs with FollowPath inputs.

### Services Table

|    Service    | Input                         | Type      | Output                        | Type      |
| :-----------: | :---------------------------- | :-------- | :---------------------------- | :-------- |
| Generate Path | init_path                     | Path      | generated_path                | Path      |
|               | generator_mode                | Int8      | generated_path_vel_percentage | Path      |
|               | max_vel_percentage            | Float32[] | max_velocity                  | Float32   |
|               |                               |           | generated_max_vel_percetage[] | Float32[] |
|               |                               |           |                               |           |
|               |                               |           |                               |           |
|  Follow Path  | generated_path                | Path      | ok                            | Bool      |
|               | generated_path_vel_percentage | Path      |                               |           |
|               | follower_mode                 | Int8      |                               |           |
|               | look_ahead                    | Float32   |                               |           |
|               | cruising_speed                | Float32   |                               |           |
|               | max_velocity                  | Float32   |                               |           |
|               | generated_max_vel_percentage  | Float32[] |                               |           |
