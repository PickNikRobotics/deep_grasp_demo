# Deep Grasp Demo
<img src="https://picknik.ai/assets/images/logo.jpg" width="120">

## Overview
This repository contains several demos for constructing a pick and place pipeline
using deep learning methods for the grasp generation stage within the MoveIt Task Constructor.

The packages where developed and tested on Ubuntu 18.04 running ROS Melodic.

## Packages
* `deep_grasp_task`: constructs a pick and place task using deep learning methods
for the grasp generation stage within the MoveIt Task Constructor

* `moveit_task_constructor_dexnet`: uses [Dex-Net](https://berkeleyautomation.github.io/dex-net/) to sample grasps from a depth image

* `moveit_task_constructor_gpd`: uses [GPD](https://github.com/atenpas/gpd) to sample grasps from 3D point clouds


## Install
First, Complete the [Getting Started Tutorial](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html).

### Dependencies
It is recommended to install the dependencies that are not ROS packages outside of the
catkin workspace.

Before installing the dependencies it is recommended to run:
```
sudo apt update
sudo apt upgrade
```

#### Grasp Pose Detection
1) Requirements
  * PCL >= 1.9: The `pcl_install.sh` script will install PCL 1.11
```
  wget https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/mtc_demos/pcl_install.sh
  chmod +x pcl_install.sh
  sudo ./pcl_install.sh
```

  * OpenCV >= 3.4: The `opencv_install.sh` script will install OpenCV 3.4
```
    wget https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/mtc_demos/opencv_install.sh
    chmod +x opencv_install.sh
    sudo ./opencv_install.sh
```

  * Eigen >= 3.0: If ROS is installed then this requirement is satisfied

2) Clone th GPD library
```
git clone https://github.com/atenpas/gpd
```

3) Modify CMakeLists.txt

First, remove the `-03` compiler optimization. This optimization can cause
a segmentation fault on 18.04.

```
set(CMAKE_CXX_FLAGS "-fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++14")
```

Next, update the `find_package()` functions for the `PCL` and `OpenCV`
versions installed. If you ran the above install scripts `CMakeLists.txt` should read:

```
find_package(PCL 1.11 REQUIRED)
find_package(OpenCV 3.4 REQUIRED)
```


4) Build
```
cd gpd
mkdir build && cd build
cmake ..
make -j
sudo make install
```

#### Dex-Net
Download the required packages and pre-trained models

1) Run the install script </br>
If you have a GPU this option will install tensorflow with GPU support.
```
wget https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/mtc_demos/dexnet_install.sh
wget https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/mtc_demos/dexnet_requirements.txt
chmod +x dexnet_install.sh
./dexnet_install.sh {cpu|gpu}
```

2) Download the pre-trained models
```
./dexnet_deps/gqcnn/scripts/downloads/models/download_models.sh
```

### ROS Packages
#### Deep Grasping Packages
For now it is recommended to create a new workspace to prevent conflicts between packages. This will especially be helpful if you want to use Gazebo with the demos.
```
mkdir -p /grasp_ws/src
cd ~/grasp_ws/src
wstool init
wstool merge https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/master/.rosinstall
wstool update

rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO
```

Note: Here you will need to extend the `grasp_ws` to the `moveit_ws` that was created from the [Getting Started Tutorial](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html).
```
catkin config --extend <path_to_moveit_ws>/devel -DCMAKE_BUILD_TYPE=Release
catkin build
```

#### Panda Gazebo Support (Optional)
You will need the C++ Franka Emika library. This can be installed from [source](https://github.com/frankaemika/libfranka) or by executing:
```
sudo apt install ros-melodic-libfranka
```

You will need two additional packages.
```
git clone https://github.com/tahsinkose/panda_moveit_config.git -b melodic-devel
git clone https://github.com/tahsinkose/franka_ros.git -b simulation
```


### Launching Demos and Further Details
To see how to launch the demos using GPD and Dex-Net see the `moveit_task_constructor_gpd` and `moveit_task_constructor_dexnet` packages.
