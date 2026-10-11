#!/bin/bash

ROBONETV2_ARM=wx250s && $(dirname "$0")/setup.sh || exit 1

source /opt/ros/jazzy/setup.bash
source ~/interbotix_ws/install/setup.bash
source ~/myenv/bin/activate

adb start-server > /dev/null 2>&1 || true

# make sure the data directory (bind-mounted from the host, see docker-compose.yml) exists and is writable;
# docker creates a missing host directory owned by root
if [ -n "${DATA}" ]; then
    sudo mkdir -p "${DATA}" && sudo chown "$(id -u):$(id -g)" "${DATA}"
fi

# using 'exec' here is very important because ros2 launch needs to do some cleanup after it exits
# so when the container is killed the SIGTERM needs to be passed to ros2 launch
exec ros2 launch widowx_controller widowx_rs.launch.py \
    ${video_stream_provider_string} camera_connector_chart:=/tmp/camera_connector_chart \
    python_node:=false realsense:=${USE_REALSENSE}
