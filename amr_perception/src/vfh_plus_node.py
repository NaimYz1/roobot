#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
VFH+ Local Avoidance Node - ROS1 Melodic (tilted Livox Mid-360).

Perception for a DOWNWARD-TILTED Livox Mid-360 on a moving base:
  * rotate the raw cloud into base_link using the CALIBRATED mount pitch
    (38.22 deg) so the floor lands at z ~= 0. (Wrong pitch tilts the floor
    into the obstacle slice -> phantom ring -> permanent "boxed in".)
  * per-frame GROUND-PLANE removal -> floor re-fit & subtracted every scan.
  * SELF-BOX filter                -> deletes the robot's own deck/laser/antenna.
  * CLUSTER (density) filter       -> a cell needs >= min_cluster points, so
    1-2 stray points are ignored while a real person/wall is kept.

Temporal accumulation is available (~accumulate) but OFF by default: it relies
on the odom TF and smears any residual floor leak into a phantom ring. A single
Mid-360 frame is dense enough for the front corridor.

Publishes: /vfh_direction /min_distance /front_distance /stop_flag /vfh_scan
Optional:  /front/scan (~publish_scan, default False; AMCL uses livox_to_scan).
"""
from __future__ import division

import math
import os
import sys

import numpy as np
import rospy
import tf
import sensor_msgs.point_cloud2 as pc2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vfh_logic import VFHPlus, ang_diff
from local_grid import LocalGrid
from harmonic_logic import HarmonicField, inflate

from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Float32, Bool
from tf.transformations import euler_from_quaternion


class VFHPlusNode(object):
    def __init__(self):
        rospy.init_node('vfh_node', anonymous=False)

        num_bins = rospy.get_param('~num_bins', 72)
        max_range = rospy.get_param('~max_range', 4.0)
        active_range = rospy.get_param('~active_range', 1.6)
        robot_radius = rospy.get_param('~robot_radius', 0.30)
        safety_margin = rospy.get_param('~safety_margin', 0.10)
        self.lookahead = rospy.get_param('~target_lookahead', 1.2)

        # --- Livox 3D lidar (Mid-360) mount ---
        self.livox_topic = rospy.get_param('~livox_topic', '/livox/lidar')
        self.mount_pitch = math.radians(
            rospy.get_param('~mount_pitch_deg', 0.0))
        self.mount_x = rospy.get_param('~mount_x', 0.0)
        self.mount_z = rospy.get_param('~mount_z', 0.30)
        self.z_min = rospy.get_param('~z_min', 0.10)
        self.z_max = rospy.get_param('~z_max', 1.50)
        self.min_range = rospy.get_param('~min_range', 0.35)
        self.debug = rospy.get_param('~debug', True)

        # --- per-frame ground removal. mode 'gate' = deterministic robust
        # z-band (de-tilt is calibrated; median offset over a TIGHT low band is
        # unbiased by real obstacles, unlike the obstacle-biased plane fit);
        # 'lsq' = legacy least-squares plane fit. ---
        self.ground_filter = rospy.get_param('~ground_filter', True)
        self.ground_mode = rospy.get_param('~ground_mode', 'lsq')
        self.ground_band = rospy.get_param('~ground_band', 0.30)
        self.ground_gate_band = rospy.get_param('~ground_gate_band', 0.10)

        # --- self-box: delete the robot's own body (base frame, metres) ---
        self.self_filter = rospy.get_param('~self_filter', True)
        self.self_x_min = rospy.get_param('~self_x_min', -0.45)
        self.self_x_max = rospy.get_param('~self_x_max', 0.45)
        self.self_y_abs = rospy.get_param('~self_y_abs', 0.32)

        # --- temporal accumulation (OFF by default: it depends on odom TF and
        # smears any floor leak into a phantom ring; a single Mid-360 frame is
        # dense enough, ~3600 pts in the front wedge) ---
        self.accumulate = rospy.get_param('~accumulate', False)
        self.accum_time = rospy.get_param('~accumulate_time', 0.40)  # seconds
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self._accum = []   # list of (t_sec, xo_array, yo_array) in odom frame

        # --- local planner: 'vfh' (gap histogram) or 'harmonic' (Laplace field;
        # smooth, no gap-to-gap oscillation, no interior local minima). The
        # harmonic field solves on the memory grid, so it forces the grid on. ---
        self.local_planner = rospy.get_param('~local_planner', 'vfh')

        # --- short-term local MEMORY: decaying log-odds occupancy grid in the
        # odom frame (cures single-frame flicker/spin; harmonic solves on it).
        # enabled via ~local_grid, or implicitly when local_planner=harmonic. ---
        self.use_grid = (rospy.get_param('~local_grid', False)
                         or self.local_planner == 'harmonic')
        self.grid = None
        if self.use_grid:
            self.grid = LocalGrid(
                size_m=rospy.get_param('~grid_size', 6.0),
                res=rospy.get_param('~grid_res', 0.05),
                decay=rospy.get_param('~grid_decay', 0.92),
                l_hit=rospy.get_param('~grid_l_hit', 0.7),
                occ_thresh=rospy.get_param('~grid_occ_thresh', 1.0),
                min_pts=int(rospy.get_param('~grid_min_pts', 2)))

        # --- harmonic (Laplace) field local planner ---
        self.harm = None
        self._harm_dir = None
        self._harm_filt = None
        self.harm_ds = int(rospy.get_param('~harm_downsample', 3))
        self.harm_lp = rospy.get_param('~harm_smooth', 0.2)   # 0..1 per frame (lower=smoother/slower)
        self.harm_radius = rospy.get_param('~harm_radius', 1.5)   # local window (m)
        self.harm_max_dev = math.radians(
            rospy.get_param('~harm_max_dev_deg', 80.0))   # goal-bias clamp
        # hybrid = FLOW primary + SINK fallback when the flow stagnates (best of
        # both: smooth in the open, pulls around head-on obstacles). flow|sink
        # force one mechanism. harm_stag_deg = flow deviation from goal that
        # counts as stagnation and triggers the sink.
        self.harm_mode = rospy.get_param('~harm_mode', 'hybrid')
        self.harm_stag_ang = math.radians(rospy.get_param('~harm_stag_deg', 75.0))
        if self.local_planner == 'harmonic':
            self.harm = HarmonicField(
                omega=rospy.get_param('~harm_omega', 1.9),
                iters=int(rospy.get_param('~harm_iters', 220)),
                inflate_cells=int(rospy.get_param('~harm_inflate_cells', 2)))

        # --- cluster / density filter (reject isolated specks) ---
        self.front_debug = rospy.get_param('~front_debug', False)
        self.min_cluster = int(rospy.get_param('~min_cluster', 3))
        self.cluster_ang = math.radians(
            rospy.get_param('~cluster_ang_deg', 2.0))
        self.cluster_rng = rospy.get_param('~cluster_rng', 0.15)

        # --- clean LaserScan output for AMCL + planner ---
        self.publish_scan = rospy.get_param('~publish_scan', False)
        self.scan_out_topic = rospy.get_param('~scan_out', '/front/scan')
        self.scan_frame = rospy.get_param('~scan_frame', 'base_link')
        self.scan_bins = int(rospy.get_param('~scan_bins', 720))
        self.scan_max = rospy.get_param('~scan_max', 25.0)

        self.vfh = VFHPlus(num_bins=num_bins, max_range=max_range,
                           active_range=active_range,
                           robot_radius=robot_radius,
                           safety_margin=safety_margin)

        self.path = None
        self.sensor_mode = None
        self.tf_listener = tf.TransformListener()

        self.dir_pub = rospy.Publisher('/vfh_direction', Float32, queue_size=1)
        self.min_pub = rospy.Publisher('/min_distance', Float32, queue_size=1)
        self.front_pub = rospy.Publisher('/front_distance', Float32,
                                         queue_size=1)
        self.stop_pub = rospy.Publisher('/stop_flag', Bool, queue_size=1)
        self.debug_pub = rospy.Publisher('/vfh_scan', LaserScan, queue_size=1)
        self.scan_pub = None
        if self.publish_scan:
            self.scan_pub = rospy.Publisher(self.scan_out_topic, LaserScan,
                                            queue_size=1)

        rospy.Subscriber('/global_path', Path, self.path_callback,
                         queue_size=1)

        # static map -> subtract KNOWN walls from the harmonic field so it only
        # avoids LIVE/unexpected obstacles (the box); A* handles the walls.
        self.static_map = None
        self.map_res = self.map_ox = self.map_oy = 0.0
        self.map_w = self.map_h = 0
        self._harm_occ_n = 0
        if self.local_planner == 'harmonic':
            rospy.Subscriber('/map', OccupancyGrid, self.map_callback,
                             queue_size=1)

        self.detect_sensor()
        rospy.loginfo("=== VFH+ NODE STARTED | Mode: %s | planner=%s | "
                      "pitch=%.2fdeg h=%.3fm ground=%s/%s self=%s grid=%s "
                      "clean_scan=%s ===",
                      self.sensor_mode, self.local_planner,
                      math.degrees(self.mount_pitch), self.mount_z,
                      self.ground_filter, self.ground_mode, self.self_filter,
                      self.use_grid, self.publish_scan)

    # ------------------------------------------------------------------
    def detect_sensor(self):
        try:
            rospy.wait_for_message(self.livox_topic, PointCloud2, timeout=3.0)
            self.sensor_mode = 'livox'
            rospy.Subscriber(self.livox_topic, PointCloud2,
                             self.pc_callback, queue_size=1)
            return
        except rospy.ROSException:
            pass
        try:
            rospy.wait_for_message('/front/scan', LaserScan, timeout=3.0)
            self.sensor_mode = 'scan'
            rospy.Subscriber('/front/scan', LaserScan,
                             self.scan_callback, queue_size=1)
            return
        except rospy.ROSException:
            rospy.logwarn("VFH+: no sensor yet, subscribing to livox...")
            self.sensor_mode = 'waiting'
            rospy.Subscriber(self.livox_topic, PointCloud2,
                             self.pc_callback, queue_size=1)

    def path_callback(self, msg):
        if msg.poses:
            self.path = [(p.pose.position.x, p.pose.position.y)
                         for p in msg.poses]

    def map_callback(self, msg):
        w, h = msg.info.width, msg.info.height
        occ = (np.array(msg.data, dtype=np.int16).reshape(h, w) > 50)
        self.static_map = inflate(occ, 3)        # pad for localisation slack
        self.map_res = msg.info.resolution
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y
        self.map_h, self.map_w = h, w

    # ------------------------------------------------------------------
    def get_pose(self):
        try:
            self.tf_listener.waitForTransform(
                'map', 'base_link', rospy.Time(0), rospy.Duration(0.05))
            (trans, rot) = self.tf_listener.lookupTransform(
                'map', 'base_link', rospy.Time(0))
            _, _, yaw = euler_from_quaternion(rot)
            return trans[0], trans[1], yaw
        except Exception:
            return None

    def get_odom(self):
        """odom->base_link as (x, y, yaw); used to motion-compensate the
        accumulation buffer. Always available from wheel odom (no AMCL needed)."""
        try:
            self.tf_listener.waitForTransform(
                self.odom_frame, self.base_frame, rospy.Time(0),
                rospy.Duration(0.03))
            (trans, rot) = self.tf_listener.lookupTransform(
                self.odom_frame, self.base_frame, rospy.Time(0))
            _, _, yaw = euler_from_quaternion(rot)
            return trans[0], trans[1], yaw
        except Exception:
            return None

    def carrot_base(self):
        """(distance_m, base-frame angle) to the lookahead 'carrot' point on the
        global path; (None, 0.0) if no path/pose."""
        pose = self.get_pose()
        if pose is None or not self.path:
            return None, 0.0
        x, y, yaw = pose
        pts = self.path
        d2 = [(px - x) ** 2 + (py - y) ** 2 for px, py in pts]
        i = int(np.argmin(d2))
        j = i
        for j in range(i, len(pts)):
            if math.hypot(pts[j][0] - x, pts[j][1] - y) >= self.lookahead:
                break
        tx, ty = pts[j]
        return (math.hypot(tx - x, ty - y),
                ang_diff(math.atan2(ty - y, tx - x), yaw))

    def target_angle(self):
        return self.carrot_base()[1]

    def _live_occ(self, od):
        """Local-grid occupancy with STATIC-MAP walls removed -> LIVE/unexpected
        obstacles only (e.g. the box). Stops the harmonic field being deflected
        by room walls that A* already routes around. Needs map+odom poses."""
        occ = self.grid.L > self.grid.occ_thresh
        if self.static_map is None:
            return occ
        pose = self.get_pose()                       # map -> base_link
        if pose is None:
            return occ
        iy, ix = np.nonzero(occ)
        if ix.size == 0:
            return occ
        mx, my, myaw = pose
        ox, oy, oyaw = od                            # odom -> base_link
        dyaw = myaw - oyaw
        cdy, sdy = math.cos(dyaw), math.sin(dyaw)
        half = self.grid.half
        pxo = self.grid.cx + (ix - half) * self.grid.res     # odom coords
        pyo = self.grid.cy + (iy - half) * self.grid.res
        rx, ry = pxo - ox, pyo - oy
        mxp = mx + cdy * rx - sdy * ry               # -> map coords
        myp = my + sdy * rx + cdy * ry
        mcx = ((mxp - self.map_ox) / self.map_res).astype(np.int32)
        mcy = ((myp - self.map_oy) / self.map_res).astype(np.int32)
        valid = (mcx >= 0) & (mcx < self.map_w) & (mcy >= 0) & (mcy < self.map_h)
        is_static = np.zeros(ix.size, dtype=bool)
        is_static[valid] = self.static_map[mcy[valid], mcx[valid]]
        live = occ.copy()
        live[iy[is_static], ix[is_static]] = False   # drop KNOWN walls
        return live

    def _fan_sink(self, occ, goal_odom, half, n, dc):
        """Goal-ward fan search for the farthest free sink cell (carrot ray first,
        then +/-15..60deg). Returns (iy, ix) or None if fully enclosed."""
        for dev in (0.0, 0.26, -0.26, 0.52, -0.52, 0.79, -0.79, 1.05, -1.05):
            if abs(dev) > self.harm_max_dev:
                continue
            ca = math.cos(goal_odom + dev)
            sa = math.sin(goal_odom + dev)
            d = dc
            while d > 1.0:
                ix = min(max(int(round(half + d * ca)), 1), n - 2)
                iy = min(max(int(round(half + d * sa)), 1), n - 2)
                if not occ[iy, ix]:
                    return (iy, ix)
                d -= 1.0
        return None

    def compute_harmonic(self, od):
        """Hybrid harmonic steering on the memory grid: potential FLOW primary
        (smooth, no discrete sink), falling back to a goal-ward SINK when the
        flow stagnates head-on. Returns the base-frame descent angle. None if
        (boxed/saddle) -> the controller rotates out via the VFH boxed path."""
        if self.grid is None or self.harm is None or self.grid.cx is None:
            return None
        dist, tgt = self.carrot_base()
        if dist is None:
            return None                              # no global path yet
        full = self._live_occ(od)          # LIVE obstacles only (walls removed)
        # crop to a LOCAL window: harmonic does LOCAL avoidance, A* does the
        # global route. Keep obstacles within harm_radius.
        ch = self.grid.half
        w = min(int(self.harm_radius / self.grid.res), ch)
        occ = full[ch - w:ch + w, ch - w:ch + w]
        # max-pool by harm_ds for a fast solve
        ds = self.harm_ds
        m = occ.shape[0] - (occ.shape[0] % ds)
        occ = occ[:m, :m].reshape(m // ds, ds, m // ds, ds).max(axis=(1, 3))
        n = occ.shape[0]
        self._harm_occ_n = int(occ.sum())  # LIVE cells the field sees
        half = n // 2
        res_ds = self.grid.res * ds
        _, _, th = od
        goal_odom = th + tgt                         # carrot direction in odom
        dc = min(dist / res_ds, half - 1)
        ang_grid = None
        if self.harm_mode in ('flow', 'hybrid'):
            # FLOW primary: smooth ideal-flow steering (no wide/discrete sink).
            ang_grid, _ = self.harm.solve_flow(occ, goal_odom, (half, half))
            stagnated = (ang_grid is None or
                         abs(ang_diff(ang_grid, goal_odom)) > self.harm_stag_ang)
            if self.harm_mode == 'hybrid' and stagnated:
                # flow stalled (head-on / back-flow) -> SINK pulls it around.
                sink = self._fan_sink(occ, goal_odom, half, n, dc)
                if sink is not None:
                    ang_grid, _ = self.harm.solve(occ, sink, (half, half))
        else:                                        # 'sink' only
            sink = self._fan_sink(occ, goal_odom, half, n, dc)
            if sink is not None:
                ang_grid, _ = self.harm.solve(occ, sink, (half, half))
        if ang_grid is None:
            return None
        new = ang_diff(ang_grid - th, 0.0)          # odom angle -> base frame
        # keep avoidance GOAL-BIASED: never deviate more than harm_max_dev from
        # the carrot direction, so the field cannot reverse the robot -> no spin.
        dev = ang_diff(new, tgt)
        if abs(dev) > self.harm_max_dev:
            new = ang_diff(tgt + math.copysign(self.harm_max_dev, dev), 0.0)
        # low-pass the steering direction to kill frame-to-frame jitter -> no spin
        if self._harm_filt is None:
            self._harm_filt = new
        else:
            self._harm_filt = ang_diff(
                self._harm_filt + self.harm_lp * ang_diff(new, self._harm_filt),
                0.0)
        return self._harm_filt

    # ------------------------------------------------------------------
    def scan_callback(self, msg):
        if self.sensor_mode == 'waiting':
            self.sensor_mode = 'scan'
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges > msg.range_min)
        self.process(msg.header, angles[valid], ranges[valid])

    # ------------------------------------------------------------------
    def cluster_filter(self, ang, r):
        """Keep only points whose (angle,range) cell has >= min_cluster points.
        Removes isolated specks (noise / floor leak); keeps real clusters."""
        if self.min_cluster <= 1 or r.shape[0] == 0:
            return ang, r
        ab = np.floor(ang / self.cluster_ang).astype(np.int64)
        rb = np.floor(r / self.cluster_rng).astype(np.int64)
        key = ab * 100003 + rb
        uniq, inv, cnt = np.unique(key, return_inverse=True,
                                   return_counts=True)
        m = cnt[inv] >= self.min_cluster
        return ang[m], r[m]

    def pc_callback(self, msg):
        if self.sensor_mode == 'waiting':
            self.sensor_mode = 'livox'
        pts = np.array(list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32)
        if pts.size == 0:
            return
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        ca = math.cos(self.mount_pitch)
        sa = math.sin(self.mount_pitch)
        xb = x * ca + z * sa + self.mount_x
        yb = y
        zb = -x * sa + z * ca + self.mount_z
        r = np.hypot(xb, yb)

        keep = (r > self.min_range) & (r < self.scan_max)
        keep_rng = keep.copy()                  # after min_range only
        if self.self_filter:
            inbox = ((xb > self.self_x_min) & (xb < self.self_x_max) &
                     (np.abs(yb) < self.self_y_abs))
            keep &= ~inbox
        keep_self = keep.copy()                 # after min_range + self-box
        height = zb
        if self.ground_filter and self.ground_mode == 'gate':
            # deterministic: robust floor offset = median over a TIGHT low band
            # (real obstacles sit above it, so they don't bias the offset).
            low = keep & (np.abs(zb) < self.ground_gate_band)
            off = (float(np.median(zb[low]))
                   if int(np.count_nonzero(low)) > 50 else 0.0)
            height = zb - off
            keep &= (height > self.z_min) & (height < self.z_max)
        elif self.ground_filter:
            low = keep & (np.abs(zb) < self.ground_band)
            ground = np.zeros_like(zb)
            nlow = int(np.count_nonzero(low))
            if nlow > 200:
                A = np.column_stack((xb[low], yb[low],
                                     np.ones(nlow, dtype=np.float32)))
                bvec = zb[low]
                try:
                    coef = np.linalg.solve(A.T.dot(A), A.T.dot(bvec))
                    ground = coef[0] * xb + coef[1] * yb + coef[2]
                except np.linalg.LinAlgError:
                    ground = np.zeros_like(zb)
            height = zb - ground
            keep &= (height > self.z_min) & (height < self.z_max)
        else:
            keep &= (zb > self.z_min) & (zb < self.z_max)

        # ---- front-corridor staged diagnostics (only when ~front_debug) ----
        if self.front_debug:
            fc = (xb > 0.2) & (np.abs(yb) < 0.6) & (r < 3.0)
            n_fc = int(np.count_nonzero(fc))
            n_rng = int(np.count_nonzero(fc & keep_rng))
            n_self = int(np.count_nonzero(fc & keep_self))
            n_grnd = int(np.count_nonzero(fc & keep))
            fk = fc & keep
            if np.any(fk):
                idx = np.nonzero(fk)[0][int(np.argmin(r[fk]))]
                near_s = ("near=%.2fm y=%+.2f z=%.2f h=%.2f"
                          % (float(r[idx]), float(yb[idx]),
                             float(zb[idx]), float(height[idx])))
            elif n_fc:
                idx = np.nonzero(fc)[0][int(np.argmin(r[fc]))]
                near_s = ("CUT nearest_raw=%.2fm y=%+.2f z=%.2f h=%.2f"
                          % (float(r[idx]), float(yb[idx]),
                             float(zb[idx]), float(height[idx])))
            else:
                near_s = "no_raw_points_in_corridor"
            rospy.loginfo_throttle(
                0.5, "[FRONT diag] raw=%d ->minrng=%d ->selfbox=%d "
                "->ground=%d | %s" % (n_fc, n_rng, n_self, n_grnd, near_s))

        xbk = xb[keep]
        ybk = yb[keep]

        # ---- short-term local memory: rolling log-odds grid (preferred) or the
        # legacy temporal point buffer (~accumulate); both in the odom frame ----
        now = msg.header.stamp.to_sec() if msg.header.stamp else rospy.get_time()
        od = self.get_odom() if (self.use_grid or self.accumulate) else None
        if self.use_grid and od is not None:
            tx, ty, th = od
            c = math.cos(th)
            s = math.sin(th)
            xo = tx + xbk * c - ybk * s
            yo = ty + xbk * s + ybk * c
            self.grid.update(tx, ty, xo, yo)
            xb2, yb2 = self.grid.occupied_base(tx, ty, th)
        elif self.accumulate and od is not None:
            tx, ty, th = od
            c = math.cos(th)
            s = math.sin(th)
            xo = tx + xbk * c - ybk * s
            yo = ty + xbk * s + ybk * c
            self._accum.append((now, xo, yo))
            self._accum = [e for e in self._accum
                           if now - e[0] <= self.accum_time]
            XO = np.concatenate([e[1] for e in self._accum])
            YO = np.concatenate([e[2] for e in self._accum])
            dx = XO - tx
            dy = YO - ty
            xb2 = dx * c + dy * s
            yb2 = -dx * s + dy * c
        else:
            self._accum = []          # no TF -> single frame, no smear
            xb2, yb2 = xbk, ybk

        if self.local_planner == 'harmonic':
            self._harm_dir = (self.compute_harmonic(od)
                              if od is not None else None)

        a_keep = np.arctan2(yb2, xb2)
        r_keep = np.hypot(xb2, yb2)

        # ---- cluster / density filter ----
        a_keep, r_keep = self.cluster_filter(a_keep, r_keep)

        if self.front_debug:
            n_raw = int(pts.shape[0])
            n_keep = int(r_keep.shape[0])
            if n_keep:
                j = int(np.argmin(r_keep))
                rospy.loginfo_throttle(
                    1.0, "[VFH pc] raw=%d kept=%d | nearest=%.2fm @ %+.0fdeg"
                    % (n_raw, n_keep, float(r_keep[j]),
                       math.degrees(float(a_keep[j]))))
            else:
                rospy.loginfo_throttle(1.0, "[VFH pc] raw=%d kept=0 (clear)"
                                       % n_raw)

        if self.scan_pub is not None:
            self.publish_clean_scan(msg.header.stamp, a_keep, r_keep)

        near = r_keep < self.vfh.max_range
        self.process(msg.header, a_keep[near], r_keep[near])

    # ------------------------------------------------------------------
    def publish_clean_scan(self, stamp, angles, ranges):
        n = self.scan_bins
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.scan_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / n
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = float(self.scan_max)
        out = np.full(n, float('inf'), dtype=np.float32)
        if ranges.shape[0]:
            bins = ((angles + math.pi) / scan.angle_increment).astype(np.int32)
            np.clip(bins, 0, n - 1, out=bins)
            order = np.argsort(ranges)[::-1]
            out[bins[order]] = ranges[order]
        scan.ranges = out.tolist()
        self.scan_pub.publish(scan)

    # ------------------------------------------------------------------
    def process(self, header, angles, ranges):
        if rospy.is_shutdown():
            return
        tgt = self.target_angle()
        res = self.vfh.steer(angles, ranges, tgt)

        direction = res['direction']
        boxed_in = res.get('boxed', direction is None)
        # harmonic mode: VFH still gives front/min distance + boxed (safety),
        # but the SMOOTH harmonic descent gives the steering direction.
        if self.local_planner == 'harmonic' and not boxed_in:
            if self._harm_dir is not None:
                direction = self._harm_dir
            elif self._harm_filt is not None:
                direction = self._harm_filt   # hold last good (don't VFH-whiplash)

        # harmonic visibility: is the box IN the grid? is the field steering?
        if self.local_planner == 'harmonic' and self.grid is not None:
            nocc = int(np.count_nonzero(self.grid.L > self.grid.occ_thresh))
            hd = ('--' if self._harm_dir is None
                  else '%+.0f' % math.degrees(self._harm_dir))
            rospy.loginfo_throttle(
                1.0, "[HARM] harm_dir=%s deg | grid_occ=%d live=%d | "
                "front=%.2fm boxed=%s" % (hd, nocc, self._harm_occ_n,
                                          res['front_dist'], boxed_in))

        self.dir_pub.publish(Float32(
            data=0.0 if direction is None else float(direction)))
        self.min_pub.publish(Float32(data=res['min_dist']))
        self.front_pub.publish(Float32(data=res['front_dist']))
        self.stop_pub.publish(Bool(data=boxed_in))

        if self.debug:
            state = ('BLOCKED' if boxed_in
                     else ('DRIVE' if res['clear'] else 'AVOID'))
            steer_s = ('   -- ' if direction is None
                       else '%+5.0f' % math.degrees(direction))
            rospy.loginfo_throttle(
                1.0, "NAV %-7s | front %4.2fm  nearest %4.2fm  "
                "steer %s deg  goal %+5.0f deg"
                % (state, res['front_dist'], res['min_dist'],
                   steer_s, math.degrees(tgt)))

        self.publish_debug(header, res['dist'])

    def publish_debug(self, header, dist):
        scan = LaserScan()
        scan.header = header
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        n = self.vfh.num_bins
        scan.angle_increment = 2 * math.pi / n
        scan.range_min = 0.05
        scan.range_max = self.vfh.max_range
        scan.ranges = [float(d) for d in dist]
        self.debug_pub.publish(scan)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        VFHPlusNode().run()
    except rospy.ROSInterruptException:
        pass
