#!/bin/bash

# read cmd line inputs
VERSION=$1 # cpu or gpu

# set cpu/gpu conditional libraries
case "${VERSION}"
in
cpu)
	pip install tensorflow==1.15.0
	;;
gpu)
  pip install tensorflow-gpu==1.13.1
	;;
*)
	echo "Usage: $0 {cpu|gpu}"
	exit 1
esac

# install apt deps
sudo apt install cmake libvtk6-dev python-vtk6 python-sip python-qt4 libosmesa6-dev meshlab libhdf5-dev

# install pip deps
python3 -m pip install -r dexnet_requirements.txt

# install deps from source
mkdir dexnet_deps
cd dexnet_deps

# install autolab modules
git clone https://github.com/BerkeleyAutomation/autolab_core.git
git clone https://github.com/BerkeleyAutomation/perception.git
git clone https://github.com/BerkeleyAutomation/gqcnn.git
git clone https://github.com/BerkeleyAutomation/visualization.git


# install all Berkeley AUTOLAB modules
# autolab_core
cd autolab_core
sudo python3 setup.py develop
cd ..

# perception
cd perception
sudo python3 setup.py develop
cd ..

# gqcnn
cd gqcnn
sudo python3 setup.py develop
cd ..

# visualization
cd visualization
sudo python3 setup.py develop
cd ..
