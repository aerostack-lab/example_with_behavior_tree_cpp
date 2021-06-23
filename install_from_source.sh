#!/bin/bash
BASEDIR=$PWD
#behavior_tree_cpp dependencies and installation
sudo apt-get install libzmq3-dev libboost-dev
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build; cd build
cmake ..
make
sudo make install
#groot dependencies and installation
cd $BASEDIR
sudo apt install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
git clone https://github.com/BehaviorTree/Groot.git
cd Groot
git submodule update --init --recursive
mkdir build; cd build
cmake ..
make
cd $BASEDIR
