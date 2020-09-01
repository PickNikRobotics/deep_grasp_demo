#!/bin/bash
# install OpenCV 3.4

# dependencies
sudo apt install build-essential
sudo apt install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

# image processing
sudo apt install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

# video processing
sudo apt install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt install libxvidcore-dev libx264-dev

# GUI support
sudo apt install libgtk-3-dev

# Optimization
sudo apt install libatlas-base-dev gfortran pylint

# Download and build OpenCV
wget https://github.com/opencv/opencv/archive/3.4.0.zip -O opencv-3.4.0.zip
wget https://github.com/opencv/opencv_contrib/archive/3.4.0.zip -O opencv_contrib-3.4.0.zip

sudo apt install unzip

unzip opencv-3.4.0.zip
unzip opencv_contrib-3.4.0.zip

cd  opencv-3.4.0
mkdir build
cd build

cmake -DCMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.0/modules -DOPENCV_ENABLE_NONFREE=True ..
make -j4
sudo make install

sudo ldconfig
