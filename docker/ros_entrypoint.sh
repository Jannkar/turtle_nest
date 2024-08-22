#!/bin/bash
set -e

# If the config doesn't exist on the first startup, it will be created with
# root permissions. Need to change the ownership to user for settings to work correctly.
sudo chown -R user:user /home/user/.config/TurtleNest

. "/opt/ros/${ROS_DISTRO}/setup.bash"
. "${ROS2_WS}/install/setup.bash"

exec "$@"