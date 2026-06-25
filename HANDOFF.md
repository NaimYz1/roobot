# Handoff — Jackal + Livox Mid-360 AMR navigation stack

## LATEST UPDATE (most recent work — read this first)
- **Sensor is Livox Mid-360S**, tilted ~38° nose-down, 0.447 m up. Calibrated values
  already in launch arg defaults: `mount_pitch_deg=38.22`, `mount_pitch_rad=0.666988`,
  `lidar_height=0.447`, `z_min=0.15`, `min_range=0.40`.
- **Symptom found**: `/front_distance` FLICKERED between the real obstacle (~0.4–0.9 m)
  and background (1.76 m) when someone stood in front. Root cause = Mid-360
  **non-repetitive scanning**: a single 0.1 s frame is too sparse to reliably
  fill the narrow front corridor (Livox docs: coverage rises with integration time).
- **Fix applied** in `amr_perception/src/vfh_plus_node.py` (rewritten, compiles,
  headless test ALL PASS, simulation-verified):
  1. **temporal accumulation** — buffers last ~0.4 s of obstacle points in the
     ODOM frame (motion-compensated via TF) then runs VFH+ on the dense cloud.
  2. **cluster/density filter** — a cell needs >= `min_cluster`(3) points to be an
     obstacle, rejecting 1–2 stray floor/noise specks.
  Plus the earlier per-frame ground removal + self-box. New params (vfh_node block):
  `accumulate`(true), `accumulate_time`(0.40), `min_cluster`(3), `cluster_ang_deg`(2.0),
  `cluster_rng`(0.15). Backups on robot: `vfh_plus_node.py.bak`/`.bak2`.
- **NOT YET CONFIRMED ON HARDWARE**: user needs to `scp` the new node to the Jackal
  and re-test `rostopic echo /front_distance` standing in front (should be STEADY now).
- **Still open**: AMCL scan path. The recovery (re-enable `pointcloud_to_laserscan`,
  set VFH `publish_scan:=false`, planner `use_scan_obstacles:=false`) was given but
  not confirmed. The cleaner option (VFH publishes a clean `/front/scan`) is built
  (`publish_scan` param) but previously didn't reach AMCL — diagnose with
  `rostopic hz/info /front/scan` if you try it again.
- Battery was flashing yellow (low) — charge it; it can disable motors.

## Goal
Get a Clearpath **Jackal** to drive to an RViz **2D Nav Goal** while avoiding
live obstacles, using a **downward-tilted Livox Mid-360 (3D)** as the only
sensor. The supervisor requires the 3D Livox — **do NOT switch to the 2D Hokuyo**
(the Jackal physically has one, but it's off-limits for this project).

## Hardware facts (measured/confirmed)
- Robot: Jackal, hostname `cpr-j100-0540`, LAN IP **192.168.1.124**, user `administrator`.
- Dev machine: an Ubuntu **VM**, IP **192.168.1.109** (bridged, same subnet). RViz runs here.
- Sensor: **Livox Mid-360**, mounted **tilted ~38° nose-DOWN** (not 45° — that was a wrong assumption from old notes), optical centre **~0.447 m** above floor. It stares at the floor, which is the root of most pain.
- Mid-360 also sees the **ceiling** (parallel plane) and the robot's **own structure** (deck, the 2D Hokuyo in front, a rear antenna/mast).
- Livox is powered by a **separate external battery** (fine, unrelated to issues).
- ROS1 **Melodic**, python2 on the robot.

## Networking / startup (working)
- On Jackal: `bash ~/robot_up.sh` → tmux sessions: roscore + base (`~/jackal_base.launch` = real hardware: accessories+base+hokuyo) + lidar (`livox_ros_driver2 msg_MID360s.launch`, xfer_format=0 → `/livox/lidar` PointCloud2). This is the REAL base, NOT Gazebo.
- Env on every Jackal terminal: `export ROS_MASTER_URI=http://192.168.1.124:11311 ROS_IP=192.168.1.124`
- Env on the VM (for RViz): `ROS_MASTER_URI=http://192.168.1.124:11311`, `ROS_IP=192.168.1.109`. Both machines must ping each other.

## Workspace layout
- Source of truth (edited on Windows/VM): `~/amr_source/` (VM) ⇄ the user's Desktop\src\src folder.
- Built on the robot: `~/amr_ws/` (`catkin_make`, devel space). Deploy with
  `scp ~/amr_source/<pkg>/... administrator@192.168.1.124:~/amr_ws/src/<pkg>/...`
  Python nodes need no rebuild — just relaunch.
- Packages: `amr_bringup` (launch/maps/rviz), `amr_perception` (VFH+), `amr_planning` (A*), `amr_control` (pure pursuit).
- Main launch: `amr_bringup/launch/amr_jackal_real.launch`. Map: `dingoMap1.yaml`.
- The algorithm itself is validated offline: `python3 headless_test.py` → ALL PASS.

## What has been FIXED and verified
1. **Build/deploy hygiene**: don't put the source folder inside `~/amr_ws/src`
   (causes "Multiple packages found"). CMakeLists install lists corrected.
2. **`/cmd_vel` works**: pure_pursuit → twist_mux → `/jackal_velocity_controller/cmd_vel`. No e-stop, drivers active. Direct `rostopic pub /cmd_vel ...` moves the robot.
3. **Sensor calibration** via new tool `amr_perception/src/calibrate_livox.py`
   (RANSAC floor-plane fit; rejects ceiling; prints exact values). Result, now in launch arg defaults:
   - `mount_pitch_deg = 38.22`, `mount_pitch_rad = 0.666988`, `lidar_height = 0.447`, roll ≈ 1° (ok).
   - Also set: `z_min = 0.15`, `min_range = 0.40`.
   With these, on a still robot the floor leaves the obstacle slice: `front` reads
   real metres, `clear=True`. Driving + simple avoidance started working here.
4. **VFH+ node rewritten** (`amr_perception/src/vfh_plus_node.py`, hardened) to add:
   - **per-frame ground-plane removal** (re-fits floor each scan → robust to the
     robot pitching during accel/turn; a static z_min was leaking the floor back
     in as a 0.40 m phantom whenever it moved). Synthetic-tested: floor removed
     even at ±6° pitch, real wall kept, self-box removes own body.
   - **self-box filter** (delete robot's own structure): params
     `self_filter, self_x_min(-0.45), self_x_max(0.45), self_y_abs(0.32)`.
   - **debug logging**: `[VFH pc] raw/kept/nearest` and `[VFH out] front/min/steer/clear/boxed`.
   - optional **clean `/front/scan` publisher** (`publish_scan`, default true) — SEE OPEN ISSUE.
   - Backup of previous node at `amr_perception/src/vfh_plus_node.py.bak`.

## CURRENT BROKEN STATE (what to fix next)
Attempting to feed AMCL a clean scan from the VFH node BROKE localization:
- VFH node now publishes `/front/scan`; the user **commented out** the
  `pointcloud_to_laserscan` (`livox_to_scan`) node in the launch.
- Result: **AMCL receives no `/front/scan`** ("No laser scan received for 221 s"),
  so no `map→odom`/`map→base_link`, so the robot does not appear in RViz, 2D Pose
  Estimate fails, "No transform to fixed frame [map]". Reason VFH's scan isn't
  reaching AMCL was not diagnosed (likely frame/timing or it isn't actually
  publishing — `rostopic hz /front/scan` was never checked).
- Also: Jackal **battery LED flashing yellow** = low battery/fault → can disable
  motors and cause erratic behaviour. Must charge/swap; check `rostopic echo -n1 /status` (`measured_battery`, stop flags).

## IMMEDIATE PLAN (recovery — revert to known-good scan, keep avoidance win)
The last instruction given to the user (apply on the Jackal, then relaunch):
```bash
# 0) CHARGE the Jackal battery first (flashing yellow). Verify /status.
F=~/amr_ws/src/amr_bringup/launch/amr_jackal_real.launch
# re-enable the pointcloud_to_laserscan node (user had wrapped it in <!-- -->):
sed -i 's/<!--  <node pkg="pointcloud_to_laserscan"/  <node pkg="pointcloud_to_laserscan"/' $F
sed -i 's#</node> -->#</node>#' $F
# stop VFH publishing /front/scan (avoid dual-publisher clash):
sed -i '/name="min_range"/a\    <param name="publish_scan"     value="false"/>' $F
# planner uses static map only (floor noise can't cause NO PATH; VFH dodges locally):
sed -i '/name="use_scan_obstacles"/ s/value="true"/value="false"/' $F
grep -nE 'pointcloud_to_laserscan|publish_scan|use_scan_obstacles' $F
roslaunch amr_bringup amr_jackal_real.launch v_max:=0.3
# verify:  rostopic hz /front/scan  (~10 Hz) ;  rosrun tf tf_echo map base_link (after 2D Pose Estimate)
```
Expected after this: AMCL localizes again (scan from converter; tilt is calibrated
so `min_height:0.15` already rejects the static floor), robot shows in RViz, 2D
Pose Estimate + Nav Goal work, VFH+ ground-removal still cleans obstacle avoidance.
**This recovery has NOT yet been confirmed by the user — verify it first.**

## OPEN ISSUES / next steps after recovery
1. **Confirm recovery**: `/front/scan` ~10 Hz, `map→base_link` resolves, robot drives to a goal and dodges a box. Watch `[VFH pc]`/`[VFH out]` WHILE MOVING — floor must stay gone (kept small, no 0.40 m phantom on accel/turn).
2. **AMCL jumpiness during motion**: the converter scan still has dynamic-pitch
   floor leak (no per-frame ground removal). Options: (a) raise AMCL
   `update_min_d/a` already set to 0.05 and `laser_max_beams` 180; (b) properly
   fix the VFH clean-scan publisher and switch AMCL to it (debug why `/front/scan`
   from VFH didn't reach AMCL — check `rostopic hz/info`, frame_id `base_link`,
   timestamp vs `transform_tolerance`).
3. **Persistent return `~0.9 m @ +131°`** (rear-left, rock-steady bearing): decide
   if it's the robot's antenna/mast (then set `self_y_abs:=0.55`) or a real wall
   (leave it; it's behind, doesn't block forward driving).
4. **Map fidelity**: `dingoMap1` may have been built with a different sensor/height
   than the tilted Livox; if AMCL never locks well, re-record the map with this Livox.
5. Tuning already applied: `v_max` 0.3 (slow for testing; knee ~0.8), `a_lat` lower,
   `d_blend` 2.0, `active_range` 1.4, lookahead raised. `w_max` can be added (~1.2) to slow turning.

## Key debug commands
```bash
rostopic hz /livox/lidar /front/scan          # sensors alive?
rostopic echo -n1 /status                      # battery / e-stop
rosrun tf tf_echo odom base_link               # odom good? (it is)
rosrun tf tf_echo map base_link                # AMCL localized?
rostopic echo /front_distance /stop_flag /vfh_direction
# VFH debug prints in the roslaunch terminal: [VFH pc] and [VFH out]
```

## Mental model of the data flow (target architecture)
```
/livox/lidar (PointCloud2, tilted 38° down)
  -> vfh_plus_node: rotate to base, per-frame ground removal + self-box
       -> /vfh_direction /min_distance /front_distance /stop_flag   (avoidance)
       -> (optionally) clean /front/scan                             (BROKEN — reverted)
  -> pointcloud_to_laserscan -> /front/scan -> AMCL + (planner)      (current recovery path)
/map -> planner_node (A*) -> /global_path
/global_path + VFH topics -> pp_controller_node -> /cmd_vel -> twist_mux -> wheels
```
The single biggest recurring trap: this is a **downward-tilted lidar staring at the
floor**, so any miscalibration or dynamic pitch dumps thousands of floor points into
the obstacle slice and the robot spins/boxes-in. Always sanity-check with
`[VFH pc] kept=` (should be small on open floor) and re-run `calibrate_livox.py`
if geometry is ever in doubt.
```
