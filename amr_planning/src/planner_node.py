#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Global Planner Node (A* + any-angle smoothing) - ROS1 Melodic
Replaces prm_node.py. Deterministic, plans in milliseconds.

Also ingests /front/scan: obstacles the laser sees (e.g. spawned cylinders
that are not in the static map) are added to the planning grid, so the
global path routes AROUND them instead of leaving VFH to fight the path.

Subscribes:  /map, /move_base_simple/goal, /front/scan
Publishes:   /global_path (nav_msgs/Path)
"""
from __future__ import division

import math
import os
import sys
import threading

import numpy as np
import rospy
import tf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from planner_logic import GridPlanner, downsample_max, inflate

from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False


class PlannerNode(object):
    def __init__(self):
        rospy.init_node('planner_node', anonymous=False)

        self.inflate_radius = rospy.get_param('~inflate_radius', 0.35)
        self.plan_resolution = rospy.get_param('~plan_resolution', 0.05)
        self.replan_period = rospy.get_param('~replan_period', 1.0)
        self.heuristic_weight = rospy.get_param('~heuristic_weight', 1.2)
        self.goal_done_radius = rospy.get_param('~goal_done_radius', 0.25)
        self.unknown_is_free = rospy.get_param('~unknown_is_free', True)
        self.use_scan = rospy.get_param('~use_scan_obstacles', True)
        self.scan_decay = rospy.get_param('~scan_decay', 6.0)
        self.scan_max_range = rospy.get_param('~scan_max_range', 4.5)

        self.map_info = None
        self.planner = None
        self.ds_factor = 1
        self.target = None
        self.planning = False
        self.lock = threading.Lock()
        # remembered scan obstacles: {(cell_x, cell_y): last_seen_stamp},
        # map frame, 5 cm cells (dict = dedup + O(1) refresh)
        self.scan_cells = {}
        self.scan_cell_size = 0.05

        self.tf_listener = tf.TransformListener()

        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped,
                         self.goal_callback, queue_size=1)
        if self.use_scan:
            rospy.Subscriber('/front/scan', LaserScan,
                             self.scan_callback, queue_size=1)

        rospy.Timer(rospy.Duration(self.replan_period), self.replan_timer)

        rospy.loginfo("=== GLOBAL PLANNER (A*/Theta*) STARTED ===")

    # ------------------------------------------------------------------
    def map_callback(self, msg):
        t0 = rospy.get_time()
        self.map_info = msg.info
        grid = np.asarray(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))

        occupied = grid > 50
        if not self.unknown_is_free:
            occupied |= (grid == -1)

        res = msg.info.resolution
        r_cells = int(round(self.inflate_radius / res))
        if HAVE_CV2:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            inflated = cv2.dilate(occupied.astype(np.uint8), kernel) > 0
        else:
            inflated = inflate(occupied, r_cells)

        self.ds_factor = max(1, int(round(self.plan_resolution / res)))
        blocked = downsample_max(inflated, self.ds_factor)

        with self.lock:
            self.planner = GridPlanner(blocked, weight=self.heuristic_weight)

        rospy.loginfo(
            "Planner map ready: %dx%d -> %dx%d (res %.3f m, inflated %.2f m) "
            "in %.2fs", msg.info.width, msg.info.height,
            blocked.shape[1], blocked.shape[0],
            res * self.ds_factor, self.inflate_radius, rospy.get_time() - t0)

    # ------------------------------------------------------------------
    def scan_callback(self, msg):
        """Project laser hits into the map frame and REMEMBER them so
        replanning routes around obstacles missing from the map.
        Forgetting is evidence-based - see comments below."""
        if self.map_info is None:
            return
        try:
            self.tf_listener.waitForTransform(
                'map', msg.header.frame_id, rospy.Time(0),
                rospy.Duration(0.05))
            (t, q) = self.tf_listener.lookupTransform(
                'map', msg.header.frame_id, rospy.Time(0))
            _, _, yaw = euler_from_quaternion(q)
        except Exception:
            return
        now = rospy.get_time()
        rmax = min(msg.range_max, self.scan_max_range)
        n = len(msg.ranges)
        cs = self.scan_cell_size

        # current measured ranges (nan/inf -> "sees far")
        meas = [r if (r == r and msg.range_min < r < msg.range_max)
                else msg.range_max for r in msg.ranges]

        # ---- EVIDENCE-BASED forgetting ----
        # The old fixed 6 s decay forgot an obstacle as soon as it left
        # the field of view; the planner then snapped back to the short
        # (blocked) path, the robot turned, saw the obstacle again,
        # dodged again - flip-flopping between two paths forever.
        # Now a remembered obstacle is dropped only when the lidar looks
        # at that spot and measures clearly PAST it (proof it is gone).
        # scan_decay is the maximum UNSEEN age (memory cap, e.g. a person
        # who walked away sideways eventually fades).
        kept = {}
        for cell, stamp in self.scan_cells.items():
            px = (cell[0] + 0.5) * cs
            py = (cell[1] + 0.5) * cs
            dx = px - t[0]
            dy = py - t[1]
            d = math.hypot(dx, dy)
            b = math.atan2(dy, dx) - yaw
            b = math.atan2(math.sin(b), math.cos(b))
            idx = int(round((b - msg.angle_min) / msg.angle_increment))
            if 0 <= idx < n and d < rmax:
                lo = max(0, idx - 2)
                r_meas = min(meas[lo:min(n, idx + 3)])
                if r_meas > d + 0.25:
                    continue              # lidar sees past it: it is gone
                if abs(r_meas - d) <= 0.25:
                    stamp = now           # re-confirmed: refresh age
            if now - stamp <= self.scan_decay:
                kept[cell] = stamp

        # ---- add current hits ----
        a = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if i % 2 == 0 and msg.range_min < r < rmax:
                px = t[0] + r * math.cos(yaw + a)
                py = t[1] + r * math.sin(yaw + a)
                kept[(int(math.floor(px / cs)),
                      int(math.floor(py / cs)))] = now
            a += msg.angle_increment

        if len(kept) > 8000:   # hard memory cap, drop oldest
            kept = dict(sorted(kept.items(),
                               key=lambda kv: kv[1])[-8000:])
        self.scan_cells = kept

    def blocked_with_scan(self):
        """Static inflated grid OR recent laser obstacles (inflated)."""
        with self.lock:
            planner = self.planner
        if planner is None:
            return None, None
        base = planner.blocked
        cells = self.scan_cells
        if not self.use_scan or not cells:
            return planner, base
        cs = self.scan_cell_size
        extra = np.zeros(base.shape, dtype=np.uint8)
        res = self.map_info.resolution * self.ds_factor
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = base.shape
        for (cx, cy) in list(cells.keys()):
            gx = int(((cx + 0.5) * cs - ox) / res)
            gy = int(((cy + 0.5) * cs - oy) / res)
            if 0 <= gx < w and 0 <= gy < h:
                extra[gy, gx] = 1
        r_cells = max(1, int(round(self.inflate_radius / res)))
        if HAVE_CV2:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            extra = cv2.dilate(extra, kernel)
        else:
            extra = inflate(extra > 0, r_cells).astype(np.uint8)
        combined = base | (extra > 0)
        return planner, combined

    # ------------------------------------------------------------------
    def goal_callback(self, msg):
        self.target = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo("New goal: (%.2f, %.2f)", self.target[0], self.target[1])
        self.trigger_plan()

    def replan_timer(self, event=None):
        if self.target is None:
            return
        pose = self.get_robot_pose()
        if pose is not None:
            d = np.hypot(self.target[0] - pose[0], self.target[1] - pose[1])
            if d < self.goal_done_radius:
                rospy.loginfo("Goal reached - planner idle.")
                self.target = None
                return
        self.trigger_plan()

    def trigger_plan(self):
        if self.planning or self.planner is None or self.target is None:
            return
        t = threading.Thread(target=self.plan_once)
        t.daemon = True
        t.start()

    # ------------------------------------------------------------------
    def get_robot_pose(self):
        try:
            self.tf_listener.waitForTransform(
                'map', 'base_link', rospy.Time(0), rospy.Duration(0.2))
            (trans, _) = self.tf_listener.lookupTransform(
                'map', 'base_link', rospy.Time(0))
            return trans[0], trans[1]
        except Exception:
            # any TF error must never kill the replan timer thread
            return None

    def world_to_plan(self, x, y):
        res = self.map_info.resolution * self.ds_factor
        gx = (x - self.map_info.origin.position.x) / res
        gy = (y - self.map_info.origin.position.y) / res
        return int(gx), int(gy)

    def plan_to_world(self, gx, gy):
        res = self.map_info.resolution * self.ds_factor
        wx = (gx + 0.5) * res + self.map_info.origin.position.x
        wy = (gy + 0.5) * res + self.map_info.origin.position.y
        return wx, wy

    # ------------------------------------------------------------------
    def plan_once(self):
        self.planning = True
        try:
            pose = self.get_robot_pose()
            if pose is None:
                rospy.logwarn_throttle(5, "Planner: no map->base_link TF yet")
                return
            target = self.target
            if target is None:
                return

            t0 = rospy.get_time()
            start = self.world_to_plan(pose[0], pose[1])
            goal = self.world_to_plan(target[0], target[1])

            planner, combined = self.blocked_with_scan()
            if planner is None:
                return
            path = None
            if combined is not planner.blocked:
                # plan around live laser obstacles first
                live = GridPlanner(combined, weight=self.heuristic_weight)
                path = live.plan(start, goal)
                if path is None:
                    rospy.logwarn_throttle(
                        5, "no path around live obstacles - using static map")
            if path is None:
                path = planner.plan(start, goal)

            if path is None:
                rospy.logerr_throttle(
                    5, "NO PATH from (%.2f,%.2f) to (%.2f,%.2f)"
                    % (pose[0], pose[1], target[0], target[1]))
                return

            msg = Path()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            for gx, gy in path:
                ps = PoseStamped()
                ps.header = msg.header
                wx, wy = self.plan_to_world(gx, gy)
                ps.pose.position.x = wx
                ps.pose.position.y = wy
                ps.pose.orientation.w = 1.0
                msg.poses.append(ps)
            # make the final point the exact clicked goal
            if msg.poses:
                msg.poses[-1].pose.position.x = target[0]
                msg.poses[-1].pose.position.y = target[1]
            self.path_pub.publish(msg)
            rospy.loginfo_throttle(
                5, "Planned %d points in %.0f ms"
                % (len(msg.poses), 1000.0 * (rospy.get_time() - t0)))
        finally:
            self.planning = False

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        PlannerNode().run()
    except rospy.ROSInterruptException:
        pass
