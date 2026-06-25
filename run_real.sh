#!/bin/bash
# ============================================================
# Run the REAL navigation stack on a chosen map. ON THE ROBOT, after robot_up.sh.
#   bash ~/amr_source/run_real.sh myroom        # map = maps/myroom.yaml, v_max 0.5
#   bash ~/amr_source/run_real.sh myroom 0.8    # + custom v_max
# Uses full paths + the correct workspace, so a stale ~/catkin_ws can't shadow it.
# ============================================================
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP="${1:-myroom}"
VMAX="${2:-0.3}"   # safe default; reactive layer has time to react to obstacles
MAP_YAML="$DIR/amr_bringup/maps/$MAP.yaml"

source "$DIR/set_ros_env.sh"

if [ ! -f "$MAP_YAML" ]; then
  echo "!! map not found: $MAP_YAML"
  echo "!! available maps:"; ls "$DIR/amr_bringup/maps/"/*.yaml 2>/dev/null
  exit 1
fi
echo ">>> launching nav on map: $MAP_YAML   (v_max=$VMAX)"
echo ">>> in RViz: 2D Pose Estimate on the robot's real spot, then 2D Nav Goal."
# args 3+ are forwarded to roslaunch, e.g.:
#   bash run_real.sh myroom 0.5 use_scan_obstacles:=false unknown_is_free:=false
roslaunch "$DIR/amr_bringup/launch/amr_jackal_real.launch" \
    v_max:="$VMAX" map_file:="$MAP_YAML" "${@:3}"
