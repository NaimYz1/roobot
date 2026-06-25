#!/bin/bash
# Save the live gmapping map. Run while map_room.sh is STILL running.
#   bash ~/amr_source/save_map.sh myroom
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="${1:-myroom}"
source "$DIR/set_ros_env.sh"
if ! rostopic list 2>/dev/null | grep -q '^/map$'; then
  echo "!! /map is NOT being published."
  echo "!! Start mapping first (in another terminal):  bash ~/amr_source/map_room.sh"
  echo "!! ...then drive around, THEN run this save command while it's still up."
  exit 1
fi
DEST="$DIR/amr_bringup/maps/$NAME"
echo ">>> saving /map  ->  $DEST.pgm / .yaml"
rosrun map_server map_saver -f "$DEST"
echo ""
echo ">>> SAVED. Now use your new map:"
echo ">>>   bash $DIR/install.sh ~/amr_ws"
echo ">>>   roslaunch amr_bringup amr_jackal_real.launch v_max:=0.5 \\"
echo ">>>       map_file:=\$(rospack find amr_bringup)/maps/$NAME.yaml"
