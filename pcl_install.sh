#!/bin/bash
# install PCL 1.11.0
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.11.0.zip -O pcl-pcl-1.11.0.zip
sudo apt install unzip
unzip pcl-pcl-1.11.0.zip
cd pcl-pcl-1.11.0
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
sudo make install
