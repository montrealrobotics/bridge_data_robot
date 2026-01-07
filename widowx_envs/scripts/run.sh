#!/bin/bash

ROBONETV2_ARM=wx250s && $(dirname "$0")/setup.sh || exit 1

source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash
source ~/myenv/bin/activate

# using 'exec' here is very important because ros2 launch needs to do some cleanup after it exits
# so when the container is killed the SIGTERM needs to be passed to ros2 launch
exec ros2 launch widowx_controller widowx_rs.launch.py \
    ${video_stream_provider_string} camera_connector_chart:=/tmp/camera_connector_chart \
    serial_no_camera1:=${REALSENSE_SERIAL} \
    python_node:=false realsense:=${USE_REALSENSE}
