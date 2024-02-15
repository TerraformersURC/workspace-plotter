# !/bin/bash

set -e

# Install the KDL library ------------------------------------------------------
cd ~/workspace-plotter && git clone https://github.com/orocos/orocos_kinematics_dynamics.git
mkdir -p orocos_kinematics_dynamics/orocos_kdl/build && cd orocos_kinematics_dynamics/orocos_kdl/build

cmake .. && make
sudo make install

cd ~/workspace-plotter && rm -rf orocos_kinematics_dynamics

# Install yaml-cpp library -----------------------------------------------------
cd ~/workspace-plotter && git clone https://github.com/jbeder/yaml-cpp.git
mkdir -p yaml-cpp/build && cd yaml-cpp/build

cmake [-DYAML_BUILD_SHARED_LIBS=ON] .. && make
sudo make install

cd ~/workspace-plotter && rm -rf yaml-cpp

# Install Matplot++ library ----------------------------------------------------
cd ~/workspace-plotter && git clone https://github.com/alandefreitas/matplotplusplus.git
cd matplotplusplus && cmake --preset=system && cmake --build --preset=system

sudo cmake --install build/system

cd ~/workspace-plotter && rm -rf matplotplusplus
