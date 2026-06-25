#!/bin/bash
# Source this in ANY terminal (robot OR VM) to set ROS networking + workspace.
#   source ~/amr_source/set_ros_env.sh
# Auto-detects this machine's IP on the robot's subnet (192.168.1.x), so the
# SAME line works on the robot (-> .124) and the VM (-> .109).
source /opt/ros/melodic/setup.bash
[ -f "$HOME/amr_ws/devel/setup.bash" ]    && source "$HOME/amr_ws/devel/setup.bash"
[ -f "$HOME/catkin_ws/devel/setup.bash" ] && source "$HOME/catkin_ws/devel/setup.bash"
export ROS_MASTER_URI=http://192.168.1.124:11311
MYIP=$(hostname -I | tr ' ' '\n' | grep -E '^192\.168\.1\.[0-9]+$' | head -1)
export ROS_IP=${MYIP:-127.0.0.1}
echo "[ros_env] MASTER=$ROS_MASTER_URI  IP=$ROS_IP"
