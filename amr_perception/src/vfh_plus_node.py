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

from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Path
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

        # --- per-frame ground-plane removal ---
        self.ground_filter = rospy.get_param('~ground_filter', True)
        self.ground_band = rospy.get_param('~ground_band', 0.30)

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

        self.detect_sensor()
        rospy.loginfo("=== VFH+ NODE STARTED | Mode: %s | pitch=%.2fdeg "
                      "h=%.3fm ground=%s self=%s accum=%s clean_scan=%s ===",
                      self.sensor_mode, math.degrees(self.mount_pitch),
                      self.mount_z, self.ground_filter, self.self_filter,
                      self.accumulate, self.publish_scan)

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

    def target_angle(self):
        pose = self.get_pose()
        if pose is None or not self.path:
            return 0.0
        x, y, yaw = pose
        pts = self.path
        d2 = [(px - x) ** 2 + (py - y) ** 2 for px, py in pts]
        i = int(np.argmin(d2))
        for j in range(i, len(pts)):
            if math.hypot(pts[j][0] - x, pts[j][1] - y) >= self.lookahead:
                break
        tx, ty = pts[j]
        return ang_diff(math.atan2(ty - y, tx - x), yaw)

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
        height = zb - 0.0
        if self.ground_filter:
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

        # ---- temporal accumulation in the odom frame (optional) ----
        now = msg.header.stamp.to_sec() if msg.header.stamp else rospy.get_time()
        od = self.get_odom() if self.accumulate else None
        if od is not None:
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
        tgt = self.target_angle()
        res = self.vfh.steer(angles, ranges, tgt)

        direction = res['direction']
        boxed_in = res.get('boxed', direction is None)

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
