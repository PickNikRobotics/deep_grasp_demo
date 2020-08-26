# MoveIt Task Constructor Grasp Pose Detection
<img src="https://picknik.ai/assets/images/logo.jpg" width="120">

## Overview
Demo showing how to use the Grasp Pose Detection (GPD) library within a grasp pose generator stage of the MoveIt Task Constructor.


### 1) Modify Config
Under `config/gpd_congfig.yaml` navigate to line 32 and update `weights_file` to contain absolute file path to the location of the [lenet params](https://github.com/atenpas/gpd/tree/master/models/lenet/15channels/params) directory in the GPD install.



## Run
### Grasp Pose Detection and MTC Panda demo

    roslaunch moveit_task_constructor_demo demo.launch

    roslaunch moveit_task_constructor_gpd gpd_demo.launch

## Results
<p align="center">
  <img src="media/mtc_gpd_panda.gif" width="450" height="450"/>
</p>
