#!/bin/bash
set -e

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/colcon_ws/install/setup.bash"

exec "$@"