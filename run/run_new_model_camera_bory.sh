#!/bin/bash
# run multiple instances of the 'px4' binary, with the gazebo SITL simulation
# It assumes px4 is already built, with 'make px4_sitl_default gazebo'

# The simulator is expected to send to TCP port 4560+i for i in [0, N-1]
# For example gazebo can be run like this:
#./Tools/gazebo_sitl_multiple_run.sh -n 10 -m iris

function cleanup() {
	pkill -x px4
	pkill gzclient
	pkill gzserver
}

vehicle_model="iris"
export PX4_SIM_MODEL=${vehicle_model}${LABEL}

echo ${SCRIPT}
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="${SCRIPT_DIR}/../../PX4-Autopilot"
build_path=${src_path}/build/px4_sitl_default


echo "killing running instances"
pkill -x px4 || true

sleep 1

source ./setup_gazebo.bash ${src_path} ${build_path}

echo "Starting gazebo"

gzserver ${SCRIPT_DIR}/../world/borsky_park.world  --verbose &
sleep 5
gz model --spawn-file=${SCRIPT_DIR}/../models/droneport_new/model.sdf --model-name=droneport1 -x 0.0 -y 0.0 -z 0.3 -Y 1.57
sleep 2	
#gz model --spawn-file=${SCRIPT_DIR}/../models/droneport_new_2/model.sdf --model-name=droneport2 -x 7.2 -y 0.0 -z 0.2 -Y 1.57
#sleep 2
${build_path}/bin/px4 -i 0 -d "$build_path/etc" -w "/tmp/sitl_drone_1" -s "$build_path/etc/init.d-posix/rcS" >"/tmp/out_drone1.log" 2>"/tmp/err_drone1.log" &
sleep 2
#sleep 2
gz model --spawn-file=${SCRIPT_DIR}/../models/drone1_cam/model.sdf --model-name=drone1 -x 0.0 -y 0.0 -z 0.5 -Y 1.57
sleep 2
#${build_path}/bin/px4 -i 1 -d "$build_path/etc" -w "/tmp/sitl_drone_2" -s "$build_path/etc/init.d-posix/rcS" >"/tmp/out_drone2.log" 2>"/tmp/err_drone2.log" &
#sleep 2
#gz model --spawn-file=${SCRIPT_DIR}/../models/drone2_cam/model.sdf --model-name=drone2 -x 7.2 -y 0.0 -z 0.5 -Y 1.57
#sleep 2

trap "cleanup" SIGINT SIGTERM EXIT

if [ -z "${GUI}" ]; then
    read -p "Press enter to quit simulation"
else
    echo "Starting gazebo client"
    gzclient
fi
