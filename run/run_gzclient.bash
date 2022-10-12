#!/bin/bash
#
# Setup environment to make PX4 visible to Gazebo.
#
# Note, this is not necessary if using a ROS catkin workspace with the px4
# package as the paths are exported.
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

#pkill gzclient

echo ${SCRIPT}
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="${SCRIPT_DIR}/../../PX4-Autopilot"
build_path=${src_path}/build/px4_sitl_default


source ./setup_gazebo.bash ${src_path} ${build_path}

gzclient