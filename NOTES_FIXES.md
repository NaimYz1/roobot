# Fixes + Sim/Deploy Workflow (Jackal + Livox Mid-360S)

## What was actually wrong

The 4 problems seen on the real robot (excess spinning, left/right inverted,
collisions, move-spin-move-spin) almost all trace to ONE root cause: the
**Livox Mid-360 is tilted 45 deg but the stack was configured for a flat
mount.** A wrong-tilt point cloud corrupts two things at once:

- **Obstacle perception** -> the floor reads as a wall ~0.4 m ahead, so VFH+
  reports "boxed in" forever -> the robot spins / stutters / creeps into things.
- **Localization** -> pointcloud_to_laserscan feeds AMCL a geometrically wrong
  scan, so the robot localizes mirrored/rotated -> "go right, robot goes left"
  and collisions.

The navigation **algorithm itself is sound**: the headless regression
(`headless_test.py`, real planner+VFH+controller on the real map) reaches every
goal with no collisions.

## Code changes (verified with headless_test.py)

1. `amr_control/src/pp_logic.py` - **drive-while-turning**: the controller now
   eases forward through a turn once it is roughly lined up and the way ahead is
   clear, instead of dead-stop pivoting at every bend. Removes the
   move-spin-move-spin stutter; A-16m turn budget 602 -> 565 deg, all scenarios
   still PASS with safe clearance.

2. `amr_bringup/launch/amr_jackal_real.launch` - **45 deg tilt** wired in:
   - VFH `mount_pitch_deg = 45`, `mount_z (lidar_height) = 0.395`
   - base_link->livox_frame TF pitch = `mount_pitch_rad` (0.7853982)
   - Both are launch args. **If RViz shows the floor as a wall / cloud rotated,
     flip BOTH signs:** `mount_pitch_deg:=-45 mount_pitch_rad:=-0.7853982`

3. **Reverted my own over-cautious tuning** after testing proved it hurt:
   - `v_max 0.6 -> 0.8`  (below ~0.8 the robot drops under the pivot-speed
     threshold and spins MORE - low speed was *causing* the spinning)
   - `inflate_radius 0.50 -> 0.45`, `safety_margin 0.15 -> 0.10`
     (0.50 blocked the map's doorways -> robot could not reach goals)
   - `d_danger 0.65 -> 0.55`  (back to the tuned value)

## Test in SIMULATION first (on the VM)

One-time (if not already installed):
    sudo apt install ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-gazebo

Rebuild from the latest source, then launch EVERYTHING (Gazebo + RViz + stack):
    bash ~/amr_source/install.sh ~/catkin_ws
    cd ~/catkin_ws
    roslaunch amr_bringup amr_full.launch
Send a goal: in RViz click **2D Nav Goal** and drag where you want the robot.
Random stress test instead: `roslaunch amr_bringup amr_full.launch auto:=true`

NOTE: sim uses a clean 2D laser, so it will NOT reproduce the tilt problems -
it validates the algorithm only. The tilt must be verified on the real robot.

## Deploy to the REAL Jackal (after sim looks good)

1. Bring up base + lidar:  `bash ~/robot_up.sh`   (base + Mid-360S in tmux)
2. Copy + build the updated stack on the Jackal:
       scp -r ~/amr_source administrator@192.168.1.124:~/amr_deploy
       # on the Jackal:
       cp -r ~/amr_deploy/amr_* ~/amr_ws/src/ && cd ~/amr_ws && catkin_make
3. Launch the stack:
       source ~/amr_ws/devel/setup.bash
       roslaunch amr_bringup amr_jackal_real.launch
4. **Verify the tilt in RViz** (Fixed Frame base_link; add /livox/lidar +
   /front/scan): floor should be flat, scan should trace real walls. If the
   floor looks like a ramp/wall, relaunch with the negative pitch (above).
5. Set **2D Pose Estimate** to where the robot really is, then **2D Nav Goal**.
