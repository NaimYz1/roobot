#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Pure Pursuit + VFH Control Node - ROS1 (Melodic) Port
Near obstacle: VFH takes over. Clear path: Pure Pursuit leads.
"""
import rospy
import numpy as np
import time
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Bool
from tf.transformations import euler_from_quaternion


class PurePursuitNode:
    def __init__(self):
        rospy.init_node('pure_pursuit_node', anonymous=False)

        self.L_fw            = rospy.get_param('~lookahead_distance', 0.4)
        self.v_des           = rospy.get_param('~desired_linear_vel', 0.3)
        self.goal_radius     = rospy.get_param('~goal_radius',        0.05)
        self.angle_threshold = 0.3

        self.current_path      = None
        self.robot_pose        = None
        self.last_target_index = 0
        self.min_dist          = 5.0
        self.vfh_angle         = 0.0
        self.stop_flag         = False
        self.last_log_time     = time.time()

        self.smooth_min_dist  = 5.0
        self.smooth_vfh_angle = 0.0

        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('/global_path',   Path,    self.path_callback, queue_size=10)
        rospy.Subscriber('/min_distance',  Float32, self.min_callback,  queue_size=10)
        rospy.Subscriber('/vfh_direction', Float32, self.vfh_callback,  queue_size=10)
        rospy.Subscriber('/stop_flag',     Bool,    self.stop_callback, queue_size=10)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Timer(rospy.Duration(0.05), self.control_loop)

        rospy.loginfo("=== PURE PURSUIT + VFH NODE STARTED (ROS1) ===")

    def path_callback(self, msg):
        if len(msg.poses) == 0:
            return
        self.current_path      = msg
        self.last_target_index = 0
        rospy.loginfo("New Path Received: {} points".format(len(msg.poses)))

    def min_callback(self, msg):
        self.min_dist = msg.data
        self.smooth_min_dist = 0.8 * self.smooth_min_dist + 0.2 * msg.data

    def vfh_callback(self, msg):
        self.vfh_angle = msg.data
        self.smooth_vfh_angle = 0.8 * self.smooth_vfh_angle + 0.2 * msg.data

    def stop_callback(self, msg):
        self.stop_flag = msg.data

    def get_robot_pose_in_map(self):
        try:
            self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(0.1))
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            _, _, yaw = euler_from_quaternion(rot)
            return [trans[0], trans[1], yaw]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def control_loop(self, event=None):
        pose = self.get_robot_pose_in_map()
        if pose is not None:
            self.robot_pose = pose

        now = time.time()
        if now - self.last_log_time > 1.0 and self.robot_pose is not None:
            rospy.loginfo("POSE: ({:.2f}, {:.2f}) | OBS: {:.2f}m | VFH: {:.2f}".format(
                self.robot_pose[0], self.robot_pose[1],
                self.smooth_min_dist, self.smooth_vfh_angle))
            self.last_log_time = now

        if self.current_path is not None and self.robot_pose is not None:
            self.calculate_control()

    def calculate_control(self):
        cmd = Twist()

        goal_pos     = self.current_path.poses[-1].pose.position
        dist_to_goal = np.hypot(goal_pos.x - self.robot_pose[0],
                                goal_pos.y - self.robot_pose[1])

        if dist_to_goal < self.goal_radius:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            rospy.loginfo("GOAL REACHED! Mission complete.")
            self.current_path = None
            self.cmd_pub.publish(cmd)
            return

        # Pure pursuit steering calculation
        current_Lfw = max(0.2, min(self.L_fw, dist_to_goal))
        target_x, target_y = self.get_lookahead_point(current_Lfw)

        dx = target_x - self.robot_pose[0]
        dy = target_y - self.robot_pose[1]
        target_angle = np.arctan2(dy, dx)
        alpha_pp = np.arctan2(
            np.sin(target_angle - self.robot_pose[2]),
            np.cos(target_angle - self.robot_pose[2]))

        steer_pp = (2.0 * self.v_des * np.sin(alpha_pp)) / current_Lfw

        obs = self.smooth_min_dist

        # 3 zones with smooth transitions
        DANGER_DIST = 0.5
        AVOID_DIST  = 1.0
        CLEAR_DIST  = 1.5

        if obs < DANGER_DIST:
            # DANGER: VFH takes full control, slow crawl
            speed = 0.10
            steer = np.clip(self.smooth_vfh_angle * 2.0, -1.5, 1.5)
            rospy.logwarn_throttle(2, "DANGER: obs={:.2f}m | VFH controls".format(obs))

        elif obs < AVOID_DIST:
            # AVOID: VFH dominant with slight PP influence
            speed = 0.15
            blend = (AVOID_DIST - obs) / (AVOID_DIST - DANGER_DIST)
            steer = blend * self.smooth_vfh_angle * 1.5 + (1.0 - blend) * steer_pp
            steer = np.clip(steer, -1.5, 1.5)
            rospy.logwarn_throttle(2, "AVOID: obs={:.2f}m | blend={:.2f}".format(obs, blend))

        elif obs < CLEAR_DIST:
            # CAUTION: PP dominant with slight VFH nudge
            slow = obs / CLEAR_DIST
            speed = self.v_des * max(0.5, slow)
            nudge = self.smooth_vfh_angle * 0.3 * (1.0 - slow)
            steer = steer_pp + nudge
            steer = np.clip(steer, -1.2, 1.2)

        else:
            # CLEAR: pure pursuit only
            if dist_to_goal < 0.5:
                speed = max(0.05, self.v_des * (dist_to_goal / 0.5))
            else:
                speed = self.v_des
            steer = np.clip(steer_pp, -1.2, 1.2)

        # Large angle: rotate in place (only when clear)
        if abs(alpha_pp) > self.angle_threshold and obs >= CLEAR_DIST:
            cmd.linear.x  = 0.02
            cmd.angular.z = 0.7 * np.sign(alpha_pp)
        else:
            cmd.linear.x  = speed
            cmd.angular.z = steer

        self.cmd_pub.publish(cmd)

    def get_lookahead_point(self, l_fw):
        for i in range(self.last_target_index, len(self.current_path.poses)):
            p    = self.current_path.poses[i].pose.position
            dist = np.hypot(p.x - self.robot_pose[0],
                            p.y - self.robot_pose[1])
            if dist >= l_fw:
                self.last_target_index = i
                return p.x, p.y

        last_p = self.current_path.poses[-1].pose.position
        return last_p.x, last_p.y

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PurePursuitNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
