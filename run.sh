#!/bin/bash
# ============================================================
# AMR one-command launcher. Your 4 terminals -> this.
#
#   ./run.sh                  manual mode (set goals in RViz: "2D Nav Goal")
#   ./run.sh auto             random obstacles + random goal
#   ./run.sh auto loop        endless random runs (benchmark mode)
#   ./run.sh fast             manual, v_max=2.0 (Jackal hw max, spicy)
#   ./run.sh nogui            no Gazebo window -> big perf boost, use RViz
#   ./run.sh real             REAL Jackal + Livox Mid-360 (no Gazebo)
#
# Any extra roslaunch args pass straight through, e.g.:
#   ./run.sh v_max:=0.8 num_obstacles:=8 seed:=42
# ============================================================
set -e

export JACKAL_LASER=1              # front SICK laser in the description
export JACKAL_LASER_TOPIC=front/scan
export LIBGL_ALWAYS_SOFTWARE=1

# kill zombies from previous crashed/Ctrl-C'd sessions (SIM ONLY:
# on the real Jackal the base runs on the live rosmaster - never kill it!)
# (a stale gzserver causes "Spawn service failed" -> robot never appears)
if [ "$1" != "real" ] && { pgrep -x gzserver >/dev/null 2>&1 \
   || pgrep -x gzclient >/dev/null 2>&1 \
   || pgrep -x rosmaster >/dev/null 2>&1; }; then
  echo ">>> killing stale gazebo/ros processes from a previous session..."
  killall -9 gzserver gzclient rosmaster roscore rosout 2>/dev/null || true
  sleep 2
fi

source /opt/ros/melodic/setup.bash
# find a built workspace: script's dir, then walk up, then common homes
WS_DIR="$(cd "$(dirname "$0")" && pwd)"
for cand in "$WS_DIR" "$WS_DIR/.." "$WS_DIR/../.." "$HOME/ros_ws" "$HOME/catkin_ws"; do
  if [ -f "$cand/devel/setup.bash" ]; then
    source "$cand/devel/setup.bash"
    echo ">>> using workspace: $(cd "$cand" && pwd)"
    WS_FOUND=1
    break
  fi
done
if [ -z "$WS_FOUND" ]; then
  echo "ERROR: no built workspace found (devel/setup.bash missing)."
  echo "       run:  bash install.sh ~/ros_ws   first"
  exit 1
fi

ARGS=""
LAUNCH="amr_full.launch"
for a in "$@"; do
  case "$a" in
    auto)  ARGS="$ARGS auto:=true" ;;
    loop)  ARGS="$ARGS loop:=true" ;;
    fast)  ARGS="$ARGS v_max:=2.0" ;;
    nogui) ARGS="$ARGS gui:=false" ;;
    real)  LAUNCH="amr_jackal_real.launch" ;;
    *)     ARGS="$ARGS $a" ;;
  esac
done

# SIM ONLY world selection (skipped if you pass world_name:= yourself):
#  - default map (dingoMap)    -> dingoMap.world generated from that map
#  - map_file:= overridden     -> leave the launch default / your world
if [ "$LAUNCH" = "amr_full.launch" ]; then
  WORLD_DIR="$(rospack find amr_bringup 2>/dev/null)/worlds"
  case "$ARGS" in
    *world_name:=*) : ;;
    *map_file:=*)
      echo ">>> NOTE: custom map without world_name:= - make sure they match!" ;;
    *)
      if [ -f "$WORLD_DIR/dingoMap1.world" ]; then
        ARGS="$ARGS world_name:=$WORLD_DIR/dingoMap1.world"
      elif [ -f "$WORLD_DIR/dingoMap.world" ]; then
        ARGS="$ARGS world_name:=$WORLD_DIR/dingoMap.world"
      elif [ -f "$WORLD_DIR/race_clean.world" ]; then
        ARGS="$ARGS world_name:=$WORLD_DIR/race_clean.world"
      fi ;;
  esac
fi

echo ">>> roslaunch amr_bringup $LAUNCH $ARGS"
roslaunch amr_bringup $LAUNCH $ARGS
