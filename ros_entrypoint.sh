#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

# Move to host home directory
cd /home/ioes/host

exec "$@"