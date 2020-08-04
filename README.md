# Deep Grasp Demo
<img src="https://picknik.ai/assets/images/logo.jpg" width="120">

## Overview
Demo showing how to use the Grasp Pose Detection (GPD) library within a grasp pose generator stage of the MoveIt Task Constructor.

This branch interfaces with the GPD library directly. If you want to see the demo using ROS communication to interface with GPD see the `gpd_ros` branch.

## Install
Complete the [Getting Started Tutorial](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html).

This package also requires [GPD](https://github.com/atenpas/gpd). An important note on compiling GPD: remove the -03 compiler optimization on Ubuntu 18.04 (otherwise this optimization causes a seg fault).

### 1) Required Packages

    wstool init
    wstool merge https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/master/.rosinstall
    wstool update

    rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO

    catkin build

### 2) Modify Config
Under `config/gpd_congfig.yaml` navigate to line 32 and update `weights_file` to contain absolute file path to the location of the [lenet params](https://github.com/atenpas/gpd/tree/master/models/lenet/15channels/params) directory in the GPD install.



## Run
### Grasp Pose Detection and MTC Panda demo

```
roslaunch moveit_task_constructor_demo demo.launch
```

```
roslaunch deep_grasp_demo gpd_demo.launch
```

## Results
<p align="center">
  <img src="media/mtc_gpd_panda.gif" width="450" height="450"/>
</p>
