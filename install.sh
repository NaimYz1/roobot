#!/bin/bash
# ============================================================
# Copy/refresh these packages into a catkin workspace, make
# scripts executable, build.
#   bash install.sh                  -> uses ~/catkin_ws
#   bash install.sh ~/ros_ws         -> uses ~/ros_ws
# ============================================================
set -e

WS="${1:-$HOME/catkin_ws}"
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
echo ">>> Installing into workspace: $WS"

mkdir -p "$WS/src"
for pkg in amr_bringup amr_control amr_perception amr_planning; do
  rm -rf "$WS/src/$pkg"
  cp -r "$SRC_DIR/$pkg" "$WS/src/$pkg"
done
cp "$SRC_DIR/run.sh" "$WS/run.sh"

# catch corrupted/truncated files from the zip->drive->extract pipeline
echo ">>> sanity-checking python files..."
find "$WS/src/amr_bringup" "$WS/src/amr_control" \
     "$WS/src/amr_perception" "$WS/src/amr_planning" -name "*.py" \
  | while read -r f; do
      python -m py_compile "$f" || { echo "CORRUPT FILE: $f - re-copy the source folder!"; exit 1; }
    done

# stale python bytecode from the old PRM can shadow new code
find "$WS/src" -name "*.pyc" -delete

find "$WS/src/amr_bringup" "$WS/src/amr_control" \
     "$WS/src/amr_perception" "$WS/src/amr_planning" \
     -name "*.py" -exec chmod +x {} \;
chmod +x "$WS/run.sh"

source /opt/ros/melodic/setup.bash

# generate cleaned race world (no baked-in robot, sim_time 0, shadows off)
# prefers the Jackal race world; falls back to the old Dingo one if present
mkdir -p "$WS/src/amr_bringup/worlds"
RACE_WORLD="$(rospack find jackal_gazebo 2>/dev/null || true)/worlds/jackal_race.world"
[ -f "$RACE_WORLD" ] || RACE_WORLD=/opt/ros/melodic/share/dingo_gazebo/worlds/dingo_race.world
if [ -f "$RACE_WORLD" ]; then
  python "$SRC_DIR/make_fast_world.py" \
    "$RACE_WORLD" \
    "$WS/src/amr_bringup/worlds/race_clean.world" \
    || echo "WARN: could not generate cleaned world (will use stock world)"
else
  echo "WARN: no race world found (jackal_gazebo not installed?) - using stock world"
fi

cd "$WS"
catkin_make

echo ""
echo "=== DONE. Launch everything with:  cd ~/catkin_ws && ./run.sh ==="
