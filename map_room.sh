#!/bin/bash
# Build a map of the room with the Livox. Run ON THE ROBOT, after robot_up.sh.
#   bash ~/amr_source/map_room.sh
# KEEP THIS TERMINAL RUNNING the whole time you drive + save.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$DIR/set_ros_env.sh"
echo ""
echo "=================================================================="
echo " MAPPING is starting. >>> LEAVE THIS TERMINAL RUNNING <<<"
echo " 1) Drive the robot SLOWLY around the whole room:"
echo "      - joystick: hold the deadman + drive, OR"
echo "      - new terminal:  source ~/amr_source/set_ros_env.sh"
echo "                       rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
echo " 2) Watch the black map build in RViz on the VM (Fixed Frame = map)."
echo " 3) When it looks like the room, in ANOTHER terminal run:"
echo "      bash ~/amr_source/save_map.sh myroom"
echo "=================================================================="
echo ""
roslaunch amr_bringup amr_mapping.launch
