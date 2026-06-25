#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Scenario Node - auto-generated obstacles + goals (ROS1 Melodic)

Modes (params):
  auto_obstacles: spawn N random cylinders into Gazebo. They are NOT added
                  to the planner map -> they truly test live VFH+ avoidance.
  auto_goal:      pick a random reachable free-space goal and publish it.
  loop:           when the robot reaches the goal, publish a new random one.

Manual workflow is untouched: RViz "2D Nav Goal" still works any time, and
you can still place obstacles by hand in Gazebo.

Extra: publish std_msgs/Empty on /scenario/regenerate to re-roll everything.
"""
from __future__ import division

import math
import random

import numpy as np
import rospy
import tf

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

CYLINDER_SDF = """<?xml version="1.0"?>
<sdf version="1.4">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><cylinder><radius>{radius}</radius><length>0.6</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{radius}</radius><length>0.6</length></cylinder></geometry>
        <material><ambient>1 0.3 0.1 1</ambient><diffuse>1 0.3 0.1 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


class ScenarioNode(object):
    def __init__(self):
        rospy.init_node('scenario_node', anonymous=False)

        self.auto_obstacles = rospy.get_param('~auto_obstacles', False)
        self.auto_goal = rospy.get_param('~auto_goal', False)
        self.loop = rospy.get_param('~loop', False)
        self.num_obstacles = rospy.get_param('~num_obstacles', 5)
        self.obstacle_radius = rospy.get_param('~obstacle_radius', 0.20)
        self.min_goal_dist = rospy.get_param('~min_goal_dist', 4.0)
        self.clearance = rospy.get_param('~clearance', 0.6)
        self.goal_reach_radius = rospy.get_param('~goal_reach_radius', 0.35)
        seed = rospy.get_param('~seed', -1)
        if seed >= 0:
            random.seed(seed)

        self.free_pts = None      # Nx2 world coords of safely-free cells
        self.goal = None
        self.spawned = []
        self.tf_listener = tf.TransformListener()

        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)

        rospy.Subscriber('/map', OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber('/scenario/regenerate', Empty,
                         self.regen_cb, queue_size=1)
        if self.loop:
            rospy.Timer(rospy.Duration(0.5), self.loop_check)

        rospy.loginfo("=== SCENARIO NODE | obstacles=%s goal=%s loop=%s ===",
                      self.auto_obstacles, self.auto_goal, self.loop)

    # ------------------------------------------------------------------
    def map_cb(self, msg):
        grid = np.asarray(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        res = msg.info.resolution
        free = (grid == 0)
        # erode free space so samples keep `clearance` from walls
        r = int(self.clearance / res)
        if r > 0:
            try:
                import cv2
                kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
                free = cv2.erode(free.astype(np.uint8), kernel) > 0
            except ImportError:
                pass
        ys, xs = np.nonzero(free)
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        self.free_pts = np.column_stack(
            (xs * res + ox, ys * res + oy)).astype(np.float32)
        rospy.loginfo("Scenario: %d candidate free cells", len(self.free_pts))

        if self.free_pts is not None and len(self.free_pts):
            rospy.Timer(rospy.Duration(2.0), self.start_once, oneshot=True)

    def start_once(self, event=None):
        if self.auto_obstacles:
            self.spawn_obstacles()
        if self.auto_goal:
            self.publish_random_goal()

    def regen_cb(self, msg):
        rospy.loginfo("Scenario: regenerating...")
        self.delete_obstacles()
        self.start_once()

    # ------------------------------------------------------------------
    def robot_xy(self):
        try:
            self.tf_listener.waitForTransform(
                'map', 'base_link', rospy.Time(0), rospy.Duration(0.5))
            (t, _) = self.tf_listener.lookupTransform(
                'map', 'base_link', rospy.Time(0))
            return t[0], t[1]
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            return None

    def sample_free(self, away_from, min_d):
        """Random free point at least min_d from every point in away_from."""
        for _ in range(500):
            x, y = self.free_pts[random.randrange(len(self.free_pts))]
            ok = True
            for (ax, ay) in away_from:
                if math.hypot(x - ax, y - ay) < min_d:
                    ok = False
                    break
            if ok:
                return float(x), float(y)
        return None

    # ------------------------------------------------------------------
    def spawn_obstacles(self):
        rospy.loginfo("Scenario: waiting for /gazebo/spawn_sdf_model...")
        try:
            rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=10.0)
        except rospy.ROSException:
            rospy.logerr("Scenario: Gazebo spawn service unavailable")
            return
        spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        keep_out = []
        robot = self.robot_xy()
        if robot:
            keep_out.append(robot)
        if self.goal:
            keep_out.append(self.goal)

        placed = []
        for i in range(self.num_obstacles):
            pt = self.sample_free(keep_out + placed, 1.2)
            if pt is None:
                rospy.logwarn("Scenario: no room for obstacle %d", i)
                continue
            name = 'amr_obstacle_%d' % i
            pose = Pose()
            pose.position.x = pt[0]
            pose.position.y = pt[1]
            pose.position.z = 0.3
            sdf = CYLINDER_SDF.format(name=name, radius=self.obstacle_radius)
            try:
                spawn(name, sdf, '', pose, 'world')
                self.spawned.append(name)
                placed.append(pt)
            except rospy.ServiceException as e:
                rospy.logerr("Scenario: spawn failed: %s", e)
        rospy.loginfo("Scenario: spawned %d obstacles (unknown to planner)",
                      len(placed))

    def delete_obstacles(self):
        if not self.spawned:
            return
        try:
            rospy.wait_for_service('/gazebo/delete_model', timeout=5.0)
            delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            for name in self.spawned:
                try:
                    delete(name)
                except rospy.ServiceException:
                    pass
        except rospy.ROSException:
            pass
        self.spawned = []

    # ------------------------------------------------------------------
    def publish_random_goal(self):
        robot = self.robot_xy()
        keep_out = [robot] if robot else []
        pt = None
        # try far goals first, relax if the map is small
        for min_d in (self.min_goal_dist, self.min_goal_dist / 2.0, 1.0):
            pt = self.sample_free(keep_out, min_d) if robot else None
            if pt is None and not robot:
                pt = self.sample_free([], 0.0)
            if pt:
                break
        if pt is None:
            rospy.logerr("Scenario: could not sample a goal")
            return
        self.goal = pt
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = pt[0]
        msg.pose.position.y = pt[1]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        rospy.loginfo("Scenario: new goal (%.2f, %.2f)", pt[0], pt[1])

    def loop_check(self, event=None):
        if self.goal is None:
            return
        robot = self.robot_xy()
        if robot is None:
            return
        if math.hypot(robot[0] - self.goal[0],
                      robot[1] - self.goal[1]) < self.goal_reach_radius:
            rospy.loginfo("Scenario: goal reached -> rolling a new one")
            self.publish_random_goal()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        ScenarioNode().run()
    except rospy.ROSInterruptException:
        pass
