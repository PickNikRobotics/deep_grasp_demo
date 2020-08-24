#!/bin/bash

# read cmd line inputs
VERSION=$1 # cpu or gpu

# set cpu/gpu conditional libraries
case "${VERSION}"
in
cpu)
	pip install tensorflow
	;;
gpu)
  pip install tensorflow-gpu
	;;
*)
	echo "Usage: $0 {cpu|gpu}"
	exit 1
esac

# install apt deps
sudo apt-get install cmake libvtk5-dev python-vtk python-sip python-qt4 libosmesa6-dev meshlab libhdf5-dev

# install pip deps
pip install numpy scipy scikit-learn scikit-image opencv-python pyassimp h5py mayavi matplotlib catkin_pkg multiprocess dill cvxopt ipython pillow pyhull setproctitle trimesh

# install deps from source
mkdir dexnet_deps
cd dexnet_deps

# install SDFGen
git clone https://github.com/jeffmahler/SDFGen.git
cd SDFGen
sudo sh install.sh
cd ..

# install Boost.NumPy
git clone https://github.com/jeffmahler/Boost.NumPy.git
cd Boost.NumPy
sudo sh install.sh
cd ..


# install autolab modules
git clone https://github.com/BerkeleyAutomation/autolab_core.git
git clone https://github.com/BerkeleyAutomation/perception.git
git clone https://github.com/BerkeleyAutomation/gqcnn.git
git clone https://github.com/BerkeleyAutomation/meshpy.git
git clone https://github.com/BerkeleyAutomation/visualization.git


# install all Berkeley AUTOLAB modules
# autolab_core
cd autolab_core
python setup.py develop
cd ..

# perception
cd perception
python setup.py develop
cd ..

# gqcnn
cd gqcnn
python setup.py develop
cd ..

# visualization
cd visualization
python setup.py develop
cd ..
