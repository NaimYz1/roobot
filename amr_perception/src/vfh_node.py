#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
VFH Local Planner Node - ROS1 (Melodic) Port
Auto-detects sensor:
  - Real Jackal  : Livox PointCloud2 (~livox_topic, default /livox/lidar)
  - Simulation   : /front/scan       (LaserScan)
"""
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Float32, Bool


class VFHNode:
    def __init__(self):
        rospy.init_node('vfh_node', anonymous=False)

        self.stop_counter        = 0
        self.obstacle_confidence = 0.0
        self.sensor_mode         = None

        self.NUM_BINS       = rospy.get_param('~num_bins',       360)
        self.MAX_RANGE      = rospy.get_param('~max_range',      5.0)
        self.STOP_DIST      = rospy.get_param('~stop_dist',      0.80)
        self.WAIT_LIMIT     = rospy.get_param('~wait_limit',     5)
        self.CONFIRM_FRAMES = rospy.get_param('~confirm_frames', 10.0)
        self.Z_MIN          = rospy.get_param('~z_min',          0.05)
        self.Z_MAX          = rospy.get_param('~z_max',          2.0)
        self.EXPANSION_W    = rospy.get_param('~expansion_w',    6)

        self.dir_pub   = rospy.Publisher('/vfh_direction', Float32,   queue_size=10)
        self.min_pub   = rospy.Publisher('/min_distance',  Float32,   queue_size=10)
        self.stop_pub  = rospy.Publisher('/stop_flag',     Bool,      queue_size=10)
        self.debug_pub = rospy.Publisher('/vfh_scan',      LaserScan, queue_size=10)

        self.livox_topic = rospy.get_param('~livox_topic', '/livox/lidar')
        rospy.loginfo("VFH: Checking for Livox LiDAR (%s)...", self.livox_topic)
        try:
            rospy.wait_for_message(self.livox_topic, PointCloud2, timeout=3.0)
            rospy.loginfo("VFH: Livox LiDAR detected! Using 3D PointCloud mode.")
            self.sensor_mode = 'livox'
            rospy.Subscriber(self.livox_topic, PointCloud2, self.pc_callback, queue_size=1)
        except rospy.ROSException:
            rospy.logwarn("VFH: No Livox data. Trying /front/scan...")
            try:
                rospy.wait_for_message('/front/scan', LaserScan, timeout=3.0)
                rospy.loginfo("VFH: 2D LaserScan detected! Using /front/scan mode.")
                self.sensor_mode = 'scan'
                rospy.Subscriber('/front/scan', LaserScan, self.scan_callback, queue_size=1)
            except rospy.ROSException:
                rospy.logerr("VFH: No sensor found! Waiting...")
                self.sensor_mode = 'waiting'
                rospy.Subscriber(self.livox_topic, PointCloud2, self.pc_callback, queue_size=1)
                rospy.Subscriber('/front/scan', LaserScan, self.scan_callback, queue_size=1)

        rospy.loginfo("=== VFH NODE STARTED | Mode: {} ===".format(self.sensor_mode))

    def scan_callback(self, msg):
        if self.sensor_mode == 'waiting':
            self.sensor_mode = 'scan'

        ranges_s  = np.full(self.NUM_BINS, self.MAX_RANGE, dtype=np.float32)
        angle     = msg.angle_min
        increment = msg.angle_increment

        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < min(msg.range_max, self.MAX_RANGE):
                idx = int((angle + np.pi) / (2 * np.pi) * self.NUM_BINS)
                idx = np.clip(idx, 0, self.NUM_BINS - 1)
                start = max(0, idx - self.EXPANSION_W)
                end   = min(self.NUM_BINS - 1, idx + self.EXPANSION_W)
                for k in range(start, end + 1):
                    if r < ranges_s[k]:
                        ranges_s[k] = r
            angle += increment

        # Wide front detection: 120 degrees (bins 120-240)
        wide_front = ranges_s[120:241]
        dist_to_obs = float(np.min(wide_front))
        self._process_ranges(msg.header, ranges_s, dist_to_obs)

    def pc_callback(self, msg):
        if self.sensor_mode == 'waiting':
            self.sensor_mode = 'livox'

        angle_rad = np.radians(45)
        cos_a     = np.cos(angle_rad)
        sin_a     = np.sin(angle_rad)
        points    = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ranges_s  = np.full(self.NUM_BINS, self.MAX_RANGE, dtype=np.float32)

        for x, y, z in points:
            x_corrected = x * cos_a + z * sin_a + 0.1
            z_corrected = -x * sin_a + z * cos_a
            y_corrected = y
            if self.Z_MIN < z_corrected < self.Z_MAX:
                r = np.hypot(x_corrected, y_corrected)
                if 0.1 < r < self.MAX_RANGE:
                    a   = np.arctan2(y_corrected, x_corrected)
                    idx = int((a + np.pi) / (2 * np.pi) * self.NUM_BINS)
                    idx = np.clip(idx, 0, self.NUM_BINS - 1)
                    start = max(0, idx - self.EXPANSION_W)
                    end   = min(self.NUM_BINS - 1, idx + self.EXPANSION_W)
                    for k in range(start, end + 1):
                        if r < ranges_s[k]:
                            ranges_s[k] = r

        wide_front = ranges_s[120:241]
        dist_to_obs = float(np.min(wide_front))
        self._process_ranges(msg.header, ranges_s, dist_to_obs)

    def _process_ranges(self, header, ranges_s, dist_to_obs):
        stop_flag  = False
        best_angle = 0.0

        if dist_to_obs <= self.STOP_DIST:
            self.obstacle_confidence = min(self.obstacle_confidence + 1.0, 20.0)
        else:
            self.obstacle_confidence = max(self.obstacle_confidence - 2.0, 0.0)

        if self.obstacle_confidence >= self.CONFIRM_FRAMES:
            if self.stop_counter < self.WAIT_LIMIT:
                stop_flag = True
                self.stop_counter += 1
                best_angle = self.calculate_vfh_angle(ranges_s)
            else:
                stop_flag = False
                self.stop_counter = 0
                best_angle = self.calculate_vfh_angle(ranges_s)
                rospy.logwarn("VFH AVOIDANCE: steering angle={:.2f}".format(best_angle))
        else:
            stop_flag = False
            self.stop_counter = 0
            best_angle = self.calculate_vfh_angle(ranges_s)
            if self.obstacle_confidence == 0:
                rospy.loginfo_throttle(5, "Path Clear. Dist: {:.2f}m".format(dist_to_obs))

        self.dir_pub.publish(Float32(data=float(best_angle)))
        self.min_pub.publish(Float32(data=float(dist_to_obs)))
        self.stop_pub.publish(Bool(data=stop_flag))
        self.publish_debug_scan(header, ranges_s)

    def calculate_vfh_angle(self, ranges):
        center_idx = self.NUM_BINS // 2  # bin 180 = forward

        # Strategy: find where obstacle IS and steer AWAY from it
        # Check wide front area (120 degrees each side)
        front_start = 120
        front_end   = 241
        front_ranges = ranges[front_start:front_end]

        # Find the closest obstacle bin in the front area
        min_range = np.min(front_ranges)

        if min_range >= self.MAX_RANGE - 0.05:
            # No obstacle nearby — go straight
            return 0.0

        # Find which bin has the closest obstacle
        obs_bin_local = np.argmin(front_ranges)
        obs_bin_global = obs_bin_local + front_start

        # Calculate obstacle angle relative to forward
        # bin 180 = forward = 0 angle
        # bin < 180 = obstacle on right = steer LEFT (positive)
        # bin > 180 = obstacle on left  = steer RIGHT (negative)
        obs_offset = obs_bin_global - center_idx

        # Steer away: opposite direction of obstacle
        # Stronger steering when obstacle is more to the front
        # Weaker when obstacle is far to the side
        frontness = 1.0 - abs(obs_offset) / 60.0  # 1.0 = dead ahead, 0.0 = far side
        frontness = max(0.2, frontness)

        # Base avoidance angle: steer opposite to obstacle
        if obs_offset <= 0:
            # Obstacle on right side — steer LEFT
            avoid_angle = 0.8 * frontness
        else:
            # Obstacle on left side — steer RIGHT
            avoid_angle = -0.8 * frontness

        # Scale by how close the obstacle is
        closeness = 1.0 - (min_range / self.MAX_RANGE)
        avoid_angle *= (1.0 + closeness)

        return np.clip(avoid_angle, -1.5, 1.5)

    def publish_debug_scan(self, header, ranges):
        scan                 = LaserScan()
        scan.header          = header
        scan.header.frame_id = "base_link"
        scan.angle_min       = -np.pi
        scan.angle_max       =  np.pi
        scan.angle_increment = (2 * np.pi) / self.NUM_BINS
        scan.range_min       = 0.1
        scan.range_max       = self.MAX_RANGE
        scan.ranges          = ranges.tolist()
        self.debug_pub.publish(scan)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = VFHNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
