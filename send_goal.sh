#!/bin/bash
# Send a navigation goal by COORDINATES (no RViz needed).
#   bash ~/amr_source/send_goal.sh 3.82 2.31         # x y  (orientation = 0)
#   bash ~/amr_source/send_goal.sh 3.82 2.31 1.57    # x y yaw(rad)
# Coordinates are in the MAP frame (same as RViz's 2D Nav Goal).
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$DIR/set_ros_env.sh"
X="${1:?usage: send_goal.sh X Y [YAW_rad]}"
Y="${2:?usage: send_goal.sh X Y [YAW_rad]}"
YAW="${3:-0}"
QZ=$(python -c "import math;print(math.sin($YAW/2.0))")
QW=$(python -c "import math;print(math.cos($YAW/2.0))")
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: $X, y: $Y, z: 0.0}, \
    orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}"
echo ">>> goal sent: ($X, $Y) yaw=$YAW rad"
