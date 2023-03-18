# Gazebo DronePort Extensions

Gazebo DronePort Extensions (GDE) is a set of tools that enables the Gazebo simulator to simulate [1] an autonomous battery management system called Droneport [2],[3]. It is proposed and developed by the team from New Technologies for Information Society research centre at the University of West Bohemia. Using GDE, it is possible to simulate all functionalities of the DronePort device in single or multi-agent scenarios, where an agent can be either a drone or the DronePort device. It consists of several parts necessary to run the simulation. In particular, it is the DronePort model for Gazebo, the DronePort plugin for Gazebo, a set of Models used for simulation based on the PX4_SITL_Gazebo repository. Finally, scripts for running the simulation and for testing DronePort capabilities are also part of the GDE.

## Prerequisites

 > sudo apt install gstreamer1.0-plugins-* python3-jinja2

 > sudo apt install build-essential cmake libgtk2.0-dev pkg-config

 > cd ~/

 > git clone https://github.com/PX4/PX4-Autopilot.git --recursive

 > cd ~/PX4-Autopilot/Tools/setup

 > bash ubuntu.sh

 > cd ~/PX4-Autopilot

 > make px4_sitl gazebo

## GDE Installation (Tested on Ubuntu 20.04 with Gazebo 11, python >= 3.8)

 > cd ~/

 * Clone current version of this repository.
 > git clone https://github.com/UWB-C4D/droneport_gazebo_extensions.git ~/gazebo_simulation
  
 > bash install.sh 
 
 * Ensure that OpenCV for python and mavsdk for python is installed (pip3 for default python3 in ubuntu 20.04).

 > pip install opencv-contrib-python

 > pip install mavsdk

## Aruco Landing demo prerequisities (tested for opencv 4.7.0)

 * OpenCV has to be builded from source to have gstreamer support.
 * https://discuss.bluerobotics.com/t/opencv-python-with-gstreamer-backend/8842

 > cd ~/

 > sudo apt install libgtk2.0-dev pkg-config

 > git clone --recursive https://github.com/skvark/opencv-python.git

 > cd ~/opencv-python

 > export CMAKE_ARGS="-DWITH_GSTREAMER=ON"

 > export ENABLE_CONTRIB=1

 > pip3 install --upgrade pip wheel

 > pip3 wheel . --verbose

 > pip install opencv_contrib_python*.whl

## Usage

 To open Gazebo with loaded models open two terminals and in both move to {REPOSITORY_ROOT}/run

 > cd ./run

 In the firts terminal run one of the run files except run_gzclient.sh e.g.:

 > ./run_gazebo_simulation.sh 

 In the second terminal run 

 > ./run_gzclient.bash

Both files call setup_gazebo.sh. Thus, everything should be configured and it should run without problems.

To run precision landing demo, open third terminal and move to scripts/aruco_landing subdirectory and run

> python demo.py

## Possible problems

- Landing algorithm uses GST camera plugin which uses GStreamer libraries. Thus, ensure that they are installed properly.

## References

**[1]** Severa, O., Bouček, Z., Neduchal, P., Bláha, L., Myslivec, T., & Flidr, M. (2022). Droneport: From Concept To Simulation. In System Engineering for constrained embedded systems (pp. 33-38).

**[2]** Bouček, Z., Neduchal, P., & Flídr, M. (2021, September). DronePort: Smart Drone Battery Management System. In International Conference on Interactive Collaborative Robotics (pp. 14-26). Springer, Cham.

**[3]** Bláha, L., Severa, O., Barták, P., Myslivec, T., Jáger, A., & Reitinger, J. (2022, August). Droneport: From Concept To Prototype. In 2022 26th International Conference on Methods and Models in Automation and Robotics (MMAR) (pp. 134-139). IEEE.

