# Deep Grasp Task
<img src="https://picknik.ai/assets/images/logo.jpg" width="120">

## Overview
This package constructs a pick and place task using the MoveIt Task Constructor. It also provides tools for collecting depth sensor data that can be used with the deep grasping libraries.

## Nodes
### deep_grasp_demo
This node is renamed at launch to mtc_tutorial. It constructs the pick and place task and adds objects to the planning scene.
#### Publishes
* planning_scene (moveit_msgs/PlanningScene) - MoveIt planning scene

## Config
* camera.intr </br>
Contains the depth camera intrinsic parameters used by Dex-Net.

* camera.yaml </br>
Contains the depth camera extrinsic parameters used to transform the grasp candidates from the depth camera optical link to the frame of the base link of the robot.  

* panda_object.yaml </br>
Contains Panda configurations and object pick and place configurations.
