#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Race-tuned Pure Pursuit Controller Node - ROS1 Melodic
(replaces pure_pursuit.py)

Subscribes:  /global_path, /vfh_direction, /min_distance, /stop_flag
Publishes:   /cmd_vel
"""
from __future__ import division

import os
import sys

import rospy
import tf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pp_logic import PurePursuitController

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Bool
from tf.transformations import euler_from_quaternion


class PPControllerNode(object):
    def __init__(self):
        rospy.init_node('pure_pursuit_node', anonymous=False)

        self.ctrl = PurePursuitController(
            v_max=rospy.get_param('~v_max', 1.1),
            v_min=rospy.get_param('~v_min', 0.12),
            w_max=rospy.get_param('~w_max', 2.5),
            a_max=rospy.get_param('~a_max', 1.2),
            a_lat=rospy.get_param('~a_lat', 1.5),
            lookahead_gain=rospy.get_param('~lookahead_gain', 0.9),
            lookahead_min=rospy.get_param('~lookahead_min', 0.45),
            lookahead_max=rospy.get_param('~lookahead_max', 1.8),
            goal_radius=rospy.get_param('~goal_radius', 0.15),
            d_danger=rospy.get_param('~d_danger', 0.45),
            d_blend=rospy.get_param('~d_blend', 1.2),
            vfh_gain=rospy.get_param('~vfh_gain', 1.8),
            # turn (rad) above which it pivots in place instead of arcing forward.
            # high (1.8 ~= 100deg) => car-like arcs for avoidance, pivot only for
            # near-reversals/boxed-in.
            rotate_threshold=rospy.get_param('~rotate_threshold', 1.8),
        )
        self.rate_hz = rospy.get_param('~rate', 20.0)

        self.path = None
        self.vfh_dir = 0.0
        self.min_dist = 10.0
        self.front_dist = None
        self.stop_flag = False
        self.done_announced = False

        self.tf_listener = tf.TransformListener()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.Subscriber('/global_path', Path, self.path_cb, queue_size=1)
        rospy.Subscriber('/vfh_direction', Float32, self.vfh_cb, queue_size=1)
        rospy.Subscriber('/min_distance', Float32, self.min_cb, queue_size=1)
        rospy.Subscriber('/front_distance', Float32, self.front_cb, queue_size=1)
        rospy.Subscriber('/stop_flag', Bool, self.stop_cb, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.loop)
        rospy.loginfo("=== PURE PURSUIT (race-tuned) STARTED | v_max=%.2f ===",
                      self.ctrl.v_max)

    # ------------------------------------------------------------------
    def path_cb(self, msg):
        new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not new_path:
            return
        old_goal = self.path[-1] if self.path else None
        self.path = new_path
        self.ctrl.last_index = 0
        if old_goal is None or \
           (old_goal[0] - new_path[-1][0]) ** 2 + \
           (old_goal[1] - new_path[-1][1]) ** 2 > 0.04:
            self.done_announced = False
            rospy.loginfo("Path: %d pts -> goal (%.2f, %.2f)",
                          len(new_path), new_path[-1][0], new_path[-1][1])

    def vfh_cb(self, msg):
        self.vfh_dir = msg.data

    def min_cb(self, msg):
        self.min_dist = msg.data

    def front_cb(self, msg):
        self.front_dist = msg.data

    def stop_cb(self, msg):
        self.stop_flag = msg.data

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
            # any TF error (frame not published yet, extrapolation, ...)
            # must NEVER kill the 20 Hz control thread
            return None

    def loop(self, event=None):
        try:
            self._loop()
        except Exception as e:
            rospy.logwarn_throttle(5, "control loop error: %s" % e)

    def _loop(self):
        pose = self.get_pose()
        cmd = Twist()
        if pose is None or self.path is None:
            self.cmd_pub.publish(cmd)
            return

        v, w, done = self.ctrl.step(
            pose, self.path, self.vfh_dir,
            self.min_dist, self.stop_flag, 1.0 / self.rate_hz,
            front_dist=self.front_dist)

        if done:
            if not self.done_announced:
                rospy.loginfo("GOAL REACHED! Mission complete.")
                self.done_announced = True
            self.path = None
            self.ctrl.reset()
        else:
            cmd.linear.x = v
            cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        PPControllerNode().run()
    except rospy.ROSInterruptException:
        pass
