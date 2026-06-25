#!/bin/bash
# ============================================================
# ONE-TIME setup: link the amr_* packages into the catkin workspace so that a
# plain `git pull` updates the running robot (python / launch / maps) with NO
# re-copy and NO rebuild. Kills the "two folders" confusion.
#   bash ~/amr_source/link_ws.sh ~/amr_ws      # on the ROBOT
#   bash ~/amr_source/link_ws.sh ~/catkin_ws   # on the VM
# ============================================================
set -e
WS="${1:-$HOME/amr_ws}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo ">>> linking packages from $DIR into $WS/src"
mkdir -p "$WS/src"
for pkg in amr_bringup amr_control amr_perception amr_planning; do
  rm -rf "$WS/src/$pkg"
  ln -sfn "$DIR/$pkg" "$WS/src/$pkg"
  echo "    $WS/src/$pkg -> $DIR/$pkg"
done

chmod +x "$DIR"/amr_*/src/*.py 2>/dev/null || true
chmod +x "$DIR"/*.sh           2>/dev/null || true

source /opt/ros/melodic/setup.bash
echo ">>> clean build (one time)"
rm -rf "$WS/build" "$WS/devel"
cd "$WS" && catkin_make

echo ""
echo "=================================================================="
echo " DONE. From now on, updating the robot is just:"
echo "       cd ~/amr_source && git pull"
echo " launch / python / maps are live immediately - no install.sh, no rebuild."
echo " (only re-run this script if you add a brand-new package or C++ code)"
echo "=================================================================="
