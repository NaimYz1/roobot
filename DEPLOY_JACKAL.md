# Running the AMR stack on the real Jackal + Livox Mid-360S

Tested before deploy: `headless_test.py` → **ALL PASS** (5 scenarios, 0 collisions,
planner/VFH+/controller run closed-loop on the real `dingoMap1` map). All node
files compile clean. See "Code review notes" at the bottom for what was fixed.

---

## 0. One-time prerequisites (on the Jackal's onboard PC)

```bash
# Livox driver (build from source – works on Melodic)
#   -> follow livox_ros_driver2 README, build in its own catkin/colcon ws
# 2D scan converter used by AMCL + planner:
sudo apt install ros-melodic-pointcloud-to-laserscan
```

Make sure the Livox driver is configured for **PointCloud2 output**:
in the Mid-360S launch file set `xfer_format = 0`. It must publish
`/livox/lidar` (sensor_msgs/PointCloud2) in frame `livox_frame`.

---

## 1. Bring up the base + lidar (terminal 1, ON the Jackal)

```bash
bash ~/robot_up.sh        # base (jackal_bringup) + Mid-360S driver in tmux
```

This must give you:
- `/odom` and TF `odom -> base_link`  (from the Jackal base service)
- `/livox/lidar`  PointCloud2  (from the Livox driver, xfer_format=0)
- a `twist_mux` consuming `/cmd_vel`

Verify before going further:
```bash
rostopic hz /livox/lidar          # ~10 Hz, PointCloud2
rostopic echo -n1 /livox/lidar | head     # frame_id: livox_frame
rosrun tf tf_echo odom base_link  # updates as you push the robot
```

> If your driver only has the non-"s" launch, start it with xfer_format=0:
> `roslaunch livox_ros_driver2 msg_MID360.launch` (edit xfer_format to 0).

---

## 2. Build the stack on the Jackal (terminal 2, ON the Jackal)

From your dev machine, copy the source over, then build:

```bash
# from your laptop:
scp -r <this_src_folder> administrator@192.168.1.124:~/amr_deploy

# on the Jackal:
cp -r ~/amr_deploy/amr_* ~/amr_ws/src/
cd ~/amr_ws && catkin_make
source ~/amr_ws/devel/setup.bash
```

(`install.sh` does the copy + py_compile sanity check + chmod + build if you
prefer: `bash ~/amr_deploy/install.sh ~/amr_ws`.)

---

## 3. Launch the navigation stack (terminal 2)

```bash
source ~/amr_ws/devel/setup.bash
roslaunch amr_bringup amr_jackal_real.launch v_max:=0.8
```

Useful args (defaults already tuned):
- `lidar_height:=0.395` – Mid-360 optical-center height above the floor (set to YOUR bracket)
- `lidar_x:=0.0` – forward offset of the sensor from base_link
- `mount_pitch_deg:=45 mount_pitch_rad:=0.7853982` – the 45° tilt (see step 5)
- `x:= y:= yaw:=` – where the robot starts on the map (also seeds AMCL)
- `map_file:=...` – defaults to `dingoMap1.yaml`

This starts: `pointcloud_to_laserscan` (→ `/front/scan`), `map_server`, `amcl`,
`planner_node`, `vfh_plus_node` (reads the 3D cloud directly), `pp_controller_node`
(→ `/cmd_vel`).

---

## 4. Open RViz on your laptop

```bash
# on the laptop, point it at the robot's master:
export ROS_MASTER_URI=http://192.168.1.124:11311
export ROS_IP=<your_laptop_ip>
rosrun rviz rviz -d $(rospack find amr_bringup)/rviz/amr.rviz
```

---

## 5. VERIFY THE TILT before driving (critical)

Mid-360 is mounted at 45° on this robot. A wrong tilt sign makes the floor
read as a wall ~0.4 m ahead → robot spins / stutters / mislocalizes.

In RViz: Fixed Frame = `base_link`, add `/livox/lidar` (PointCloud2) and
`/front/scan` (LaserScan).
- **Floor should look flat**, the scan should trace the real walls around you.
- **If the floor looks like a ramp/wall** or the cloud is rotated, relaunch with
  the opposite sign on BOTH params:
  ```bash
  roslaunch amr_bringup amr_jackal_real.launch \
      mount_pitch_deg:=-45 mount_pitch_rad:=-0.7853982
  ```

---

## 6. Localize, then send a goal

1. In RViz click **2D Pose Estimate** and click/drag where the robot actually is
   on the map (this initializes AMCL).
2. Click **2D Nav Goal** and drag to the destination.
   (Or from CLI: `rostopic pub /move_base_simple/goal ...`.)

The robot plans (A*), follows with race-tuned Pure Pursuit, and dodges with
VFH+ off the live 3D cloud.

---

## 7. Sanity topics while running

```bash
rostopic echo /cmd_vel        # nonzero when it should move
rostopic echo /stop_flag      # True = VFH says "boxed in"
rostopic echo /global_path -n1
rosrun tf tf_echo map base_link   # AMCL pose tracking
```

---

## Things to confirm for YOUR robot (setup-dependent)

1. **`/cmd_vel` is the right command topic.** The stack publishes `/cmd_vel`,
   assuming your twist_mux listens there. If the Jackal ignores it, your mux input
   is probably `/jackal_velocity_controller/cmd_vel` (or a muxed input name) — add
   a remap on `pp_controller_node` or check `twist_mux` config. Also release the
   wireless **e-stop / hold the deadman** as your setup requires.
2. **No duplicate `livox_frame` TF.** This launch publishes a static
   `base_link -> livox_frame`. If your robot URDF already defines `livox_frame`,
   you'll get a TF conflict — remove one (delete the `livox_tf` node from the
   launch, or rename via `livox_frame:=`).
3. **`lidar_height` / `lidar_x`** must match your actual bracket, or AMCL and the
   obstacle slice will be off.

## Fallback if localization/driving misbehaves

If you also have a 2D laser (`/scan`) and want a known-good baseline that bypasses
the tilted Livox entirely:
```bash
roslaunch amr_bringup amr_jackal_real_2d.launch
```
(Only works if a 2D laser publishes `/scan` — the Mid-360-only robot uses step 3.)

---

## Code review notes (what I changed)

Fixed (low-risk, build-correctness only — runtime behavior unchanged):
- `amr_control/CMakeLists.txt`, `amr_perception/CMakeLists.txt`,
  `amr_planning/CMakeLists.txt`: `catkin_install_python` listed only the OLD
  node scripts (`pure_pursuit.py`, `vfh_node.py`, `prm_node.py`). Added the NEW
  nodes the real launch actually runs (`pp_controller_node.py`,
  `vfh_plus_node.py`, `planner_node.py`) so `catkin_make install` is correct.
  (Devel-space `catkin_make` already worked because the scripts are chmod +x.)
- `amr_bringup/package.xml`: added `pointcloud_to_laserscan` and `tf` as
  `exec_depend` so `rosdep` pulls them.

Verified, no change needed:
- `headless_test.py` → ALL PASS, planning median ~3 ms, 0 collisions.
- All 11 Python files compile.
- Topic wiring is consistent: `vfh_plus_node` publishes `/vfh_direction`,
  `/min_distance`, `/front_distance`, `/stop_flag`; `pp_controller_node`
  subscribes to all of them. Planner/AMCL consume `/front/scan`.
- `dingoMap1.yaml/.pgm` present; real launch defaults to it.
