#!/bin/bash
#
# Setup environment to make PX4 visible to Gazebo.
#
# Note, this is not necessary if using a ROS catkin workspace with the px4
# package as the paths are exported.
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

if [ "$#" != 2 ]; then
    echo -e "usage: source setup_gazebo.bash src_dir build_dir\n"
    return 1
fi

SRC_DIR=$1
BUILD_DIR=$2

# setup Gazebo env and update package path
source /usr/share/gazebo/setup.sh; 
source /usr/share/mavlink_sitl_gazebo/setup.sh;
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$SRC_DIR/../gazebo_simulation/build:$BUILD_DIR/build_gazebo:$GAZEBO_PLUGIN_PATH;
export GAZEBO_MODEL_PATH=$SRC_DIR/../gazebo_simulation/models:$SRC_DIR/Tools/sitl_gazebo/models:$SRC_DIR/Tools/simulation/gazebo/sitl_gazebo/models:$GAZEBO_MODEL_PATH;
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$BUILD_DIR/build_gazebo

export PX4_HOME_LAT=49.7252222
export PX4_HOME_LON=13.3675000
export PX4_HOME_ALT=320.0

echo -e "GAZEBO_PLUGIN_PATH $GAZEBO_PLUGIN_PATH"
echo -e "GAZEBO_MODEL_PATH $GAZEBO_MODEL_PATH"
echo -e "LD_LIBRARY_PATH $LD_LIBRARY_PATH"
