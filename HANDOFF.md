# HANDOFF — Jackal + Livox Harmonic-Field Navigation (READ THIS FIRST)

Current as of 2026-06-26. **Supersedes all earlier handoff notes.** Written for a
fresh session / different model / post-compaction. Self-contained.

---

## 0. TL;DR — current state: IT WORKS

A Clearpath **Jackal** UGV drives to a commanded goal and **smoothly arcs around
obstacles** using **only a tilted Livox Mid-360 3D LiDAR**, via a custom stack:
**A\*** global planner + a **harmonic / ideal-fluid-flow potential-field local
planner** (the thesis contribution) + **Pure Pursuit** tracking + **AMCL**
localisation. After a long debugging session it reliably avoids a box and reaches
the goal with **no erratic spinning**.

**Run it (on the robot):** `bash ~/amr_source/run_real.sh myroom`
**Status:** working on flat routes; minor transients on ramps. Remaining work is
*capture + write-up*, not debugging.

---

## 1. Project identity (FYP thesis)

- Title: *"Controlling Unmanned Vehicles Beyond Line of Sight."* BVLOS here means
  **perception + onboard autonomy** (the robot's lidar replaces the operator's
  eyes) — **NOT** internet/4G/comms. Don't frame it as a comms project.
- Platform: Clearpath **Jackal**, ROS1 **Melodic**, **Python 2** onboard.
- Sensor: **Livox Mid-360** 3D LiDAR, the **only** sensor. Mounted **~38.22° nose-
  down**, optical centre **0.447 m** up (RANSAC-calibrated; old 45°/0.395 were
  wrong). **SUPERVISOR REQUIRES the 3D Livox — do NOT use the 2D Hokuyo for
  navigation** (the Jackal physically has one; it's off-limits except possibly for
  one-time map-making).
- Map + AMCL: localises on a pre-built 2D map. The original `dingoMap1` was a
  **different room/sensor** and never matched — this session the room was
  **re-mapped with the Livox** (`myroom`). Student does navigation, not SLAM.

---

## 2. THE TWO GOTCHAS THAT COST HOURS — internalise these

1. **TWO separate machines.** The nav stack runs **ON THE ROBOT** (`cpr-j100-0540`,
   `192.168.1.124`). The **VM** (`192.168.1.109`) only runs **RViz** (and Gazebo
   sim). `roslaunch amr_jackal_real.launch` must run **on the robot**.
   - `ROS_MASTER_URI` = **always** `http://192.168.1.124:11311` (roscore is on the
     robot).
   - `ROS_IP` = **the machine you're typing on** (`.124` on the robot, `.109` on
     the VM). Wrong `ROS_IP` → *"Unable to contact my own server."*
   - **`set_ros_env.sh` auto-sets both correctly** — `source ~/amr_source/set_ros_env.sh`.
2. **Source-of-truth = git, deployed via symlink.** `~/amr_ws/src/amr_*` are
   **symlinks** to `~/amr_source/amr_*` (set up once by `link_ws.sh`). So
   **`git pull` is the ONLY update step** — no `install.sh`, no copy, no rebuild
   (Python/launch/maps go live immediately). Do NOT re-introduce per-file `scp`.

---

## 3. Networking / hardware facts

| | Value |
|---|---|
| Robot host / IP / user | `cpr-j100-0540` / `192.168.1.124` / `administrator` (SSH, password) |
| VM IP | `192.168.1.109` (RViz + sim) |
| GitHub remote | `https://github.com/NaimYz1/roobot` (account **NaimYz1**) |
| Key topics | `/livox/lidar` (PointCloud2), `/livox/imu` (Imu), `/odom`, TF `odom->base_link`, `twist_mux` consumes `/cmd_vel` |
| Robot bringup | `bash ~/robot_up.sh` → tmux: roscore + base + Livox driver (xfer_format=0) |

Git-auth gotcha seen once: the machine was logged into GitHub as
`khaifaw650-create` but the repo is `NaimYz1`'s → 403 on push. Fixed by
`printf 'protocol=https\nhost=github.com\n\n' | git credential reject` then re-auth.

---

## 4. How to run the working system

**One-time per machine:**
```bash
cd ~ && git clone https://github.com/NaimYz1/roobot.git amr_source
bash ~/amr_source/link_ws.sh ~/amr_ws        # robot  (or ~/catkin_ws on the VM)
```
**Every update after that:** `cd ~/amr_source && git pull`  (that's it).

**Run (ON THE ROBOT):**
```bash
# Terminal 1: base + lidar
bash ~/robot_up.sh
# Terminal 2: the nav stack (working defaults baked in)
bash ~/amr_source/run_real.sh myroom
#   add a fixed start pose to skip the manual RViz estimate (robot must start there):
#   bash ~/amr_source/run_real.sh myroom 0.3 x:=5.0 y:=5.5 yaw:=1.9
```
**RViz (ON THE VM):**
```bash
source ~/amr_source/set_ros_env.sh
rviz -d $(rospack find amr_bringup)/rviz/amr.rviz
```
**Send a goal:** RViz *2D Nav Goal*, **or** by coordinates:
`bash ~/amr_source/send_goal.sh 3.82 2.31 [yaw_rad]`

**Helper scripts (repo root):** `set_ros_env.sh`, `link_ws.sh`, `map_room.sh`,
`save_map.sh`, `run_real.sh`, `send_goal.sh`. (`install.sh`/`run.sh` are legacy.)

---

## 5. Architecture / data flow

```
/livox/lidar (3D, tilted ~38deg) + /livox/imu
  └─ vfh_plus_node.py:
       IMU dynamic de-tilt -> ground gate -> self-filter -> log-odds MEMORY grid
       -> mask STATIC-MAP walls (avoid LIVE obstacles only)
       -> HARMONIC FIELD (or VFH+)  =>  /vfh_direction /front_distance /stop_flag
/livox/lidar -> pointcloud_to_laserscan -> /front/scan -> AMCL (localise) [+planner]
myroom map -> map_server -> AMCL + A* planner
A* planner (planner_node.py, 1 Hz) -> /global_path
/global_path + /vfh_direction -> pp_controller_node.py (Pure Pursuit, ARC not pivot)
  -> /cmd_vel -> twist_mux -> wheels
```

The harmonic field is the **steering** source; **VFH+ still runs** to provide
`front_distance`/`stop_flag` for the controller's emergency brake (safety net).

---

## 6. The harmonic-field local planner (the contribution)

- File: `amr_perception/src/harmonic_logic.py` (`HarmonicField.solve` = sink form,
  `.solve_flow` = ideal-flow form; both numpy/py2; has a `__main__` self-test).
  Integrated in `vfh_plus_node.py::compute_harmonic`.
- Solves **Laplace ∇²φ = 0** on a cropped, down-sampled local occupancy window each
  cycle via **red-black SOR** (fresh solve, not warm-started — the grid recenters as
  the robot moves). Steers down **−∇φ**. No interior local minima (max principle).
- **`harm_mode` (default `hybrid`):**
  - `flow` — uniform far-field flow toward the goal + obstacles as scatterers
    (ideal-fluid / Green's-function / "Dirac scattering"). Smooth, but **stagnates
    head-on** (velocity→0, back-flow).
  - `sink` — obstacles high, the global-path **carrot** is the low sink. Pulls
    through gaps but steers wide/discrete.
  - `hybrid` — flow primary; switch to sink when the flow deviates `> harm_stag_deg`
    from the goal (stagnation detected). Best of both.
- **Critical:** the field grid has **static-map walls masked out** (`/map` mask via
  the map→odom transform) so it only avoids **LIVE/unexpected** obstacles; A\*
  handles the walls. Without this the field is unstable (deflects around far walls).

---

## 7. Key files

| File | Role |
|---|---|
| `amr_perception/src/vfh_plus_node.py` | Perception + local planner node: IMU de-tilt, ground gate, self-filter, memory grid, static-wall masking, VFH+ **and** harmonic. The brain. |
| `amr_perception/src/harmonic_logic.py` | Harmonic field solver (sink + flow) + self-test |
| `amr_perception/src/local_grid.py` | Rolling log-odds ego-centric occupancy grid (memory) + self-test |
| `amr_perception/src/diag_ground.py` | Standalone ground-z diagnostic (validate de-tilt; run on robot) |
| `amr_perception/src/calibrate_livox.py` | RANSAC floor-fit calibrator (gives mount_pitch/height) |
| `amr_planning/src/planner_node.py` + `planner_logic.py` | A\* + any-angle smoothing, obstacle memory |
| `amr_control/src/pp_controller_node.py` + `pp_logic.py` | Pure Pursuit; **arc-not-pivot** logic |
| `amr_bringup/launch/amr_jackal_real.launch` | The real-robot launch (all params/args) |
| `amr_bringup/launch/amr_mapping.launch` | gmapping on the Livox flattened scan |
| `amr_bringup/maps/myroom.{pgm,yaml}` | Map built with the Livox (use this, not dingoMap1) |
| `amr_bringup/rviz/amr.rviz` | RViz config (PointCloud2 + decay added) |

---

## 8. Tuning params (launch args) — current WORKING values

| Arg | Working value | Meaning / when to change |
|---|---|---|
| `local_planner` | `harmonic` | `vfh` = baseline (for the ablation) |
| `harm_mode` | `hybrid` | `flow`/`sink` force one mechanism |
| `w_max` | **0.5** | turn rate; 0.5 = wide calm arcs (the key working value) |
| `rotate_threshold` | **1.8** | turns below this ARC; above pivot. High = car-like |
| `v_max` | 0.3 | slow so the field keeps up |
| `d_danger` | 0.55 | controller emergency-brake distance |
| `use_imu` | `true` | IMU dynamic de-tilt; `imu_gain:=-1` if floor leaks MORE when moving |
| `z_min` / `z_max` | 0.20 / 1.50 | obstacle height band (0.20 = pitch margin vs floor-leak) |
| `use_scan_obstacles` | `false` | planner ignores live obstacles (local layer dodges them) |
| `unknown_is_free` | `false` | bound A\* to the mapped area (fast) |
| `plan_resolution` | 0.10 | coarser = faster A\* |
| `grid_decay` | 0.92 | memory persistence; higher hoards floor-leak phantoms |
| `harm_radius` | 1.5 | local field window (m) |
| `harm_inflate_cells` | 3 | obstacle inflation; higher = earlier/gentler curve |
| `harm_smooth` | 0.2 | steering low-pass; lower = smoother but laggier (can drive through!) |
| `harm_max_dev_deg` | 80 | clamp: steering never deviates more than this from goal |
| `harm_stag_deg` | 75 | flow deviation that triggers the sink fallback |

---

## 9. The journey — root causes diagnosed & fixed (so you understand WHY)

| Symptom | Root cause | Fix |
|---|---|---|
| Spins, "no memory", lidar blinks in RViz | single-frame VFH+ on sparse Mid-360; RViz Decay 0 | log-odds memory grid; RViz decay |
| Mislocalised (white scan ≠ black walls) | `dingoMap1` = different room/sensor | re-mapped with the Livox (`myroom`) |
| Planning took minutes | A\* explored the whole unknown 992² map | `unknown_is_free=false` + `plan_resolution=0.10` |
| Past obstacle it spun/reversed | planner re-routed around the dynamic box | `use_scan_obstacles=false`; short `scan_decay` |
| Harmonic never fired (`harm_dir=--`) | sink placed behind a wall → no connection | sink = global-path carrot |
| Field swung 80–180° / spin loop | whole-room routing | local window + **mask static walls** |
| `live` cells explode (0→200) only while moving | **dynamic floor-leak** (motion pitch) | **IMU dynamic de-tilt** + `z_min` margin |
| Snappy/jerky stop-rotate motion | skid-steer **pivoted** ("tank") each direction change | controller **arcs forward**; pivot only boxed/reversal |

---

## 10. Key findings (thesis novelty)

1. A field method must see **only live obstacles**, not static walls A\* already
   routes around — masking the static map was the biggest stability win.
2. **Ideal-flow stagnates head-on** → a **hybrid flow+sink** (sink on detected
   stagnation) is the robust form.
3. The platform's **kinematics matter as much as the field** — a skid-steer that
   *pivots* looks erratic; commanding **forward arcs** makes the same field smooth.
4. **Tilted-LiDAR floor-leak is static-clean but dynamic-dirty** — needs IMU
   live-pitch de-tilt under motion.
5. **Localisation is only as good as map–sensor consistency** — rebuild the map
   with the deployment sensor.

---

## 11. Known limitations (state these honestly in the thesis)

- Close-range blind zone (min-range + self-filter): obstacles vanish < ~0.45 m;
  bridged by short-term memory.
- Ramp transitions cause brief IMU de-tilt transients (flat routes most reliable).
- SOR solve adds latency → run slow (0.3 m/s).
- "No local minima" is **continuum-only** (saddle points remain); the field is a
  *local* layer, A\* gives the global route. Never claim global optimality.
- Validated in one mapped environment; single-LiDAR, pre-built-map scope.
- **scikit-fmm is Python-3 only** → FM2/Eikonal is an *offline* comparison, not onboard.

---

## 12. What's next (priority order)

1. **Careful remap** of the room (`map_room.sh` → drive slowly, hug every wall,
   rotate at corners, close the loop → `save_map.sh myroom2`). Cleaner localisation.
2. **Formal ablation** = Chapter 4: same scripted goals (`send_goal.sh`) with
   `local_planner:=vfh` (baseline) vs `:=harmonic`. Record video + the `[HARM]`/`NAV`
   logs. Metrics (offline from a rosbag): success, oscillation/heading-sign-flips,
   time-to-goal, path smoothness/curvature, min clearance.
3. **FM2 offline** (`tools/fm2_offline.py`, py3 + scikit-fmm on a logged map) — the
   HJB/Eikonal comparison, delivered off-robot (not yet built).
4. **Thesis writeup** — material is in `Desktop\src\FYP …\SESSION_SUMMARY_for_thesis.md`.

---

## 13. Debug / diagnostics

```bash
# the [HARM] log line (in the run_real terminal, harmonic mode):
#   harm_dir = field steering angle (-- = field returned None -> VFH fallback)
#   grid_occ = all cells (walls+box) ; live = LIVE cells after wall masking (should be
#              ~0 in open space, a few dozen with a box; explodes => floor-leak)
#   imu = live de-tilt correction (hand-tilt the robot -> should change a few deg)
#   front = collision-corridor clearance ; boxed = VFH "boxed in" flag
rosrun tf tf_echo map base_link        # AMCL localised? (after 2D Pose Estimate)
rostopic echo /global_path -n1 | head  # is a path published?
rostopic hz /front/scan /livox/lidar   # sensors alive (~10 Hz)
python ~/amr_source/amr_perception/src/diag_ground.py   # validate ground de-tilt (robot still)
```

---

## 14. Where things live (outside this repo)

- **Approved plan:** `C:\Users\kfaww\.claude\plans\okay-we-need-to-sunny-duckling.md`
- **Thesis summary (this session):** `Desktop\src\FYP  Controlling unmanned vehicles beyond line of sight\SESSION_SUMMARY_for_thesis.md`
- **Thesis ground-truth briefing + Ch.2:** same FYP folder.
- **Research (citations + fact-checks of harmonic/FM2/flow claims):** background
  workflow output `w7e607v81` (transcript dir) — for the lit review.
- **Auto-memory** (loads each session): `amr-dev-workflow.md`, `amr-nav-diagnosis-and-method.md`.

---

## 15. Conventions for whoever continues

- Edit on Windows → commit (LF enforced) → push → `git pull` on robot/VM. Never scp.
- Test changes on the **robot** (real) or the **VM** (Gazebo sim via `run.sh`).
- `headless_test.py` runs planner+VFH+controller offline on the map (should pass).
- Keep claims honest: **no "globally optimal", no "no local minima" unqualified.**
- Commit messages end with `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`.
