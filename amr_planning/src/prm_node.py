#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
PRM Global Planner Node - ROS1 (Melodic) Port
Only replans when path is clear — lets VFH handle obstacles
"""
import rospy
import numpy as np
import cv2
import os
import sys
import tf
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from prm_logic import PRM

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


class PRMNode:
    def __init__(self):
        rospy.init_node('prm_node', anonymous=False)
        rospy.loginfo("=== PRM NODE STARTED (ROS1) ===")

        self.num_nodes         = rospy.get_param('~num_nodes',          1500)
        self.connection_radius = rospy.get_param('~connection_radius',  20)
        self.inflate_radius    = rospy.get_param('~inflate_radius',     0.4)

        self.map_data    = None
        self.map_info    = None
        self.target_pose = None
        self.planning    = False
        self.min_dist    = 5.0  # Current obstacle distance
        self.has_path    = False

        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        rospy.Subscriber('/min_distance', Float32, self.dist_callback, queue_size=10)

        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=10)

        # Check if we need to replan every 2 seconds
        rospy.Timer(rospy.Duration(2.0), self.check_replan)

        rospy.loginfo("PRM Node ready. Smart replanning enabled.")

    def dist_callback(self, msg):
        self.min_dist = msg.data

    def get_robot_pose_in_map(self):
        try:
            self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            return [trans[0], trans[1]]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def map_callback(self, msg):
        rospy.loginfo("Map received")
        self.map_info = msg.info

        grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        rospy.loginfo("Map size: {}x{}, res={}".format(
            msg.info.width, msg.info.height, msg.info.resolution))

        grid[grid == -1] = 0
        grid[grid == 205] = 0

        resolution   = self.map_info.resolution
        radius_cells = int(self.inflate_radius / resolution)
        rospy.loginfo("Inflating map with radius {}m ({} cells)".format(
            self.inflate_radius, radius_cells))

        kernel       = np.ones((radius_cells * 2 + 1, radius_cells * 2 + 1), np.uint8)
        obstacle_map = (grid > 50).astype(np.uint8)
        inflated     = cv2.dilate(obstacle_map, kernel)

        self.map_data = inflated * 100

    def goal_callback(self, msg):
        self.target_pose = [
            msg.pose.position.x,
            msg.pose.position.y
        ]
        rospy.loginfo("New goal received: {}".format(self.target_pose))
        self.has_path = False
        self.trigger_plan()

    def check_replan(self, event=None):
        """Only replan when obstacle is clear — don't fight VFH"""
        if self.target_pose is None:
            return

        # Don't replan while actively avoiding obstacle
        if self.min_dist < 1.5:
            rospy.loginfo_throttle(5, "Near obstacle ({:.2f}m) — VFH in control, skipping replan".format(
                self.min_dist))
            return

        # Replan if we don't have a path, or periodically when clear
        self.trigger_plan()

    def trigger_plan(self):
        if self.map_data is None or self.target_pose is None:
            return
        if self.planning:
            return

        t = threading.Thread(target=self.plan_path_background)
        t.daemon = True
        t.start()

    def plan_path_background(self):
        self.planning = True
        try:
            robot_pose = self.get_robot_pose_in_map()
            if robot_pose is None:
                rospy.logwarn("No TF for planning")
                return

            dist_to_goal = np.hypot(
                self.target_pose[0] - robot_pose[0],
                self.target_pose[1] - robot_pose[1])
            if dist_to_goal < 0.3:
                return

            rospy.loginfo("=== REPLANNING from ({:.2f},{:.2f}) | obs={:.2f}m ===".format(
                robot_pose[0], robot_pose[1], self.min_dist))

            start = self.world_to_grid(*robot_pose)
            goal  = self.world_to_grid(*self.target_pose)

            try:
                if self.map_data[start[1]][start[0]] > 50:
                    rospy.logerr("Start inside obstacle!")
                    return
                if self.map_data[goal[1]][goal[0]] > 50:
                    rospy.logerr("Goal inside obstacle!")
                    return
            except Exception as e:
                rospy.logerr("Index error: {}".format(e))
                return

            prm = PRM(
                start, goal,
                self.map_data,
                num_nodes=self.num_nodes,
                connection_radius=self.connection_radius,
                logger=rospy.loginfo
            )

            path = prm.find_path()

            if not path:
                rospy.logerr("PATH NOT FOUND")
                return

            rospy.loginfo("PATH FOUND — {} points".format(len(path)))

            path_msg = Path()
            path_msg.header.stamp    = rospy.Time.now()
            path_msg.header.frame_id = "map"

            for (gx, gy) in path:
                wx, wy = self.grid_to_world(gx, gy)
                pose = PoseStamped()
                pose.header.stamp    = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(wx)
                pose.pose.position.y = float(wy)
                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            self.has_path = True

        finally:
            self.planning = False

    def world_to_grid(self, x, y):
        gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = gx * self.map_info.resolution + self.map_info.origin.position.x
        wy = gy * self.map_info.resolution + self.map_info.origin.position.y
        return wx, wy

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PRMNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
