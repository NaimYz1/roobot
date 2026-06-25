# AMR Race Stack — Clearpath Jackal / ROS Melodic / Livox Mid-360

Robot drives to the goal as fast as possible while dodging obstacles.
Global A*/Theta* planner + goal-aware VFH+ avoidance + race-tuned Pure Pursuit.
Ported from the Dingo-O: **the algorithms are unchanged** — only the robot
platform (Jackal), the sensor (Livox Mid-360 3D lidar), and the map changed.

## Quick start — SIMULATION (replaces your 4 terminals)

Needs the Jackal sim packages once:
`sudo apt install ros-melodic-jackal-simulator ros-melodic-jackal-desktop`

```bash
./install.sh          # once: copies packages to ~/catkin_ws, chmods, catkin_make
cd ~/catkin_ws
./run.sh              # manual mode — set goals with "2D Nav Goal" in RViz
./run.sh auto         # random obstacles (invisible to planner) + random goal
./run.sh auto loop    # endless random runs
./run.sh fast         # v_max=2.0 (Jackal hardware max)
```

Extra args pass through: `./run.sh v_max:=0.8 num_obstacles:=8 seed:=42`.
Re-roll a scenario live: `rostopic pub /scenario/regenerate std_msgs/Empty --once`.
Manual goal from CLI still works exactly like before (`/move_base_simple/goal`).

The simulated Jackal carries the front SICK laser (`config:=front_laser`,
topic `/front/scan`) so the stack runs identically to before. **No Livox
is simulated** — in sim the SICK stands in for the flattened Livox scan;
the algorithms see the same topics either way.

### Worlds and maps (they must match!)

- **Default**: map `dingoMap.yaml` + world `dingoMap.world`. The world was
  **generated from the map** by `make_world_from_map.py` (occupied map
  cells extruded into 1 m walls), so Gazebo, AMCL and the planner all
  agree. Default spawn (-2.12, -1.67) is a verified clear spot on this map.
  Regenerate after editing the map:
  `python make_world_from_map.py amr_bringup/maps/dingoMap.yaml amr_bringup/worlds/dingoMap.world`
- **Old setup** still works (needs `dingo_gazebo` installed for the world
  file only — no Dingo robot is spawned):
  `./run.sh map_file:=$(rospack find amr_bringup)/maps/test_map.yaml world_name:=/opt/ros/melodic/share/dingo_gazebo/worlds/dingo_race.world x:=1.49 y:=-5.49`
- `run.sh` picks `dingoMap.world` automatically when you don't override
  anything, and warns if you pass `map_file:=` without `world_name:=`.

## Quick start — REAL JACKAL + Livox Mid-360

1. Install on the Jackal: `livox_ros_driver2` (build from source, works on
   Melodic) and `sudo apt install ros-melodic-pointcloud-to-laserscan`.
2. Start the Livox driver with **PointCloud2 output** (`xfer_format = 0`
   in the launch file) → publishes `/livox/lidar` in frame `livox_frame`.
3. Launch the stack (Jackal base service is already running):

```bash
./run.sh real                          # or:
roslaunch amr_bringup amr_jackal_real.launch v_max:=0.8 \
    lidar_height:=0.30 x:=0 y:=0 yaw:=0
```

4. Send a goal on `/move_base_simple/goal` (RViz "2D Nav Goal" from your
   laptop with `ROS_MASTER_URI` pointed at the Jackal).

**No 2D lidar is needed on the real Jackal.** `/front/scan` there is a
*virtual* scan flattened out of the Livox 3D cloud by
`pointcloud_to_laserscan` — the Mid-360 is the only sensor required.

How the Mid-360 is used:
- **VFH+ obstacle avoidance** consumes the raw 3D cloud directly
  (`vfh_plus_node.py` auto-detects PointCloud2 on `/livox/lidar`). Points
  are filtered to a 0.10–1.50 m slice above the ground so the floor and
  overhead structures are ignored, then flattened to polar bins — same
  VFH+ math as before, now vectorized for the ~200k pts/s the Mid-360 puts out.
- **AMCL + planner** get a 2D `/front/scan` produced from the cloud by
  `pointcloud_to_laserscan` (configured in `amr_jackal_real.launch`).
- Set `lidar_height` to the Mid-360 optical center height above the
  ground for your mount; tilted brackets are supported via the
  `mount_pitch_deg` param of the VFH node.

## What changed and why

| Problem | Root cause | Fix |
|---|---|---|
| Robot circled obstacles | Old "VFH" steered away from closest obstacle with no goal awareness → push-away/pull-back limit cycle | Real **VFH+** (`vfh_logic.py`): finds open gaps, picks the one closest to the path direction, with hysteresis |
| Didn't move toward goal | AMCL initialized at (0,0,0) but Gazebo spawned at (1.49,−5.49) → wrong map→base_link TF until a manual 2D Pose Estimate | `amr_full.launch` feeds the spawn pose to AMCL automatically |
| | Planner skipped replanning whenever an obstacle was within 1.5 m | Replans every 1 s unconditionally (it's cheap now) |
| Planning took 30+ s, jagged paths | PRM: random sampling, O(n²) graph build, O(n²) A* in pure Python | **A\* + any-angle smoothing** (`planner_logic.py`): deterministic, ~3 ms, straight paths. Robust to start/goal near walls (snaps to nearest free cell) |
| Slow robot (0.3 m/s) | Fixed speed + hard 0.10/0.15 caps near obstacles; inflation 0.15 m < robot half-width kept it permanently in "avoid" zone | Pure Pursuit rewrite (`pp_logic.py`): 1.1 m/s cruise, curvature-based cornering, accel limits, proportional obstacle slowdown; inflation 0.35 m |
| `/stop_flag` ignored | subscribed, never used | Used: boxed-in → stop & rotate toward best gap |

Old nodes (`prm_node.py`, `vfh_node.py`, `pure_pursuit.py`, `amr_system.launch`)
are untouched — old workflow still works as fallback.

## Smart-driving fixes (the "constantly spinning" bug)

The robot used to wiggle/spin in place instead of driving. Five root causes,
all fixed and regression-tested by `headless_test.py` (runs the real
planner/VFH+/controller modules closed-loop on the real map, no ROS needed —
every scenario must print GOAL):

1. Two competing rotate behaviors (toward path vs toward VFH gap)
   alternated forever → ONE blended desired heading (`pp_logic.py`).
2. While turning, the steering reference was re-evaluated every scan and
   chased its own tail → rotation target is LATCHED as a world heading,
   the turn completes, then the robot re-decides.
3. "Boxed in" returned no direction and the robot spun blindly forever →
   VFH+ now returns the most open direction + a `boxed` flag, and the
   controller creeps out of the trap after turning.
4. `front_dist` came from the robot-radius-enlarged histogram (and ±30°
   cone), so side walls read as "dead ahead" → phantom stops, crawling.
   Now it's the true collision-corridor distance from raw points.
5. Planner inflation (0.35) was smaller than VFH clearance (0.43), so A*
   planned through gaps VFH+ refused to drive and they argued forever →
   `inflate_radius` 0.45 matches VFH; `active_range` 1.1 so ~2 m doorways
   don't saturate the histogram; `num_bins` 144 so real gaps are visible.

## Obstacle memory (no more old-path / new-path flip-flopping)

The planner remembers lidar obstacles per 5 cm cell and forgets them only
on EVIDENCE: a remembered obstacle is dropped when the lidar looks at that
spot again and measures clearly past it. Previously memory expired on a
fixed 6 s timer, so as soon as the robot turned away from an obstacle the
planner forgot it, snapped back to the short (blocked) path, the robot
turned, saw it again, dodged again - oscillating between two paths.
`scan_decay` (30 s) is now only the maximum *unseen* age, so an obstacle
that genuinely left while unobserved (a person walking away) still fades.

## Headless verification (no Gazebo needed)

Run on the real `test_map.pgm` with a simulated lidar:
clean run 10.35 m in 10 s (~1.0 m/s avg); with 3 obstacles dropped **on** the
planned path and hidden from the planner: goal reached, zero collisions, zero
orbiting. Planning median 3 ms.

## Tuning knobs (in `amr_full.launch` / `amr_jackal_real.launch`)

- `v_max` — cruise speed (default 1.1 sim / 0.8 real, Jackal hw max 2.0)
- `inflate_radius` — wall clearance for global paths (0.35)
- `active_range` — distance at which VFH+ starts treating a gap as blocked (1.6)
- `d_blend` / `d_danger` — where avoidance starts blending / fully takes over (1.2 / 0.45)
- `a_lat` — cornering aggressiveness (1.5; higher = faster corners)

## Architecture

```
/map ─► planner_node (A*/Theta*, 1 Hz replan) ─► /global_path
/front/scan (sim laser, or Livox cloud flattened by pointcloud_to_laserscan)
   ─► AMCL + planner scan-obstacles
/front/scan OR /livox/lidar (Mid-360 PointCloud2, auto-detected)
   ─► vfh_plus_node (goal-aware gaps) ─► /vfh_direction /min_distance /stop_flag
/global_path + VFH topics ─► pp_controller_node (20 Hz) ─► /cmd_vel (Jackal twist_mux)
scenario_node ─► random Gazebo obstacles + goals (auto:=true, sim only)
```
