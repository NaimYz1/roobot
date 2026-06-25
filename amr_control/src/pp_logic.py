#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Race-tuned Pure Pursuit + VFH+ blending controller logic.
Pure Python - no ROS imports. Python 2.7 and 3 compatible.
"""
from __future__ import division, print_function

import math


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def ang_diff(a, b):
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class PurePursuitController(object):
    def __init__(self,
                 v_max=1.1,
                 v_min=0.12,
                 w_max=2.5,
                 a_max=1.2,
                 a_lat=1.5,
                 a_dec_goal=0.8,
                 lookahead_gain=0.9,
                 lookahead_min=0.45,
                 lookahead_max=1.8,
                 goal_radius=0.15,
                 rotate_threshold=1.0,
                 d_danger=0.45,
                 d_blend=1.2,
                 vfh_gain=1.8):
        self.v_max = v_max
        self.v_min = v_min
        self.w_max = w_max
        self.a_max = a_max
        self.a_lat = a_lat
        self.a_dec_goal = a_dec_goal
        self.k_L = lookahead_gain
        self.L_min = lookahead_min
        self.L_max = lookahead_max
        self.goal_radius = goal_radius
        self.rotate_threshold = rotate_threshold
        self.d_danger = d_danger
        self.d_blend = d_blend
        self.vfh_gain = vfh_gain

        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.last_index = 0
        self._spinning = False
        self._spin_target = None
        self._beta_f = 0.0
        self.last_branch = 'init'

    def reset(self):
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.last_index = 0
        self._spinning = False
        self._spin_target = None
        self._beta_f = 0.0

    def lookahead_point(self, x, y, path):
        L = clamp(self.k_L * abs(self.v_cmd), self.L_min, self.L_max)
        n = len(path)
        best_i = self.last_index
        best_d = 1e18
        for i in range(self.last_index, min(self.last_index + 80, n)):
            d = (path[i][0] - x) ** 2 + (path[i][1] - y) ** 2
            if d < best_d:
                best_d = d
                best_i = i
        self.last_index = best_i
        for j in range(best_i, n):
            if math.hypot(path[j][0] - x, path[j][1] - y) >= L:
                return path[j], L
        return path[-1], L

    def step(self, pose, path, vfh_dir, min_dist, stop_flag, dt,
             front_dist=None):
        self.last_branch = 'no_path'
        if not path:
            return self._ramp(0.0, 0.0, dt) + (False,)

        x, y, yaw = pose
        gx, gy = path[-1]
        dist_goal = math.hypot(gx - x, gy - y)

        if dist_goal < self.goal_radius:
            self.last_branch = 'goal'
            self.v_cmd = 0.0
            self.w_cmd = 0.0
            return 0.0, 0.0, True

        if stop_flag:
            self.last_branch = 'stop_flag'
            x_, y_, yaw_ = pose
            if not self._spinning:
                self._spinning = True
                self._spin_target = yaw_ + (vfh_dir if abs(vfh_dir) > 0.05
                                            else 0.8)
            err = ang_diff(self._spin_target, yaw_)
            if abs(err) > 0.25:
                v, w = self._ramp(0.0, clamp(2.0 * err,
                                             -self.w_max, self.w_max), dt)
                return v, w, False
            self._spinning = False
            self._spin_target = None
            if (front_dist if front_dist is not None else 0.0) > \
                    self.d_danger:
                v, w = self._ramp(0.25, 0.0, dt)
                return v, w, False
            v, w = self._ramp(0.0, 0.0, dt)
            return v, w, False

        (tx, ty), L = self.lookahead_point(x, y, path)
        alpha = ang_diff(math.atan2(ty - y, tx - x), yaw)

        fd = front_dist if front_dist is not None else min_dist
        if fd >= self.d_blend:
            beta = 0.0
        elif fd <= self.d_danger:
            beta = 1.0
        else:
            beta = (self.d_blend - fd) / (self.d_blend - self.d_danger)

        goal_close = dist_goal < 1.0
        if goal_close:
            beta *= clamp(dist_goal - self.goal_radius, 0.0, 1.0)

        self._beta_f += 0.25 * (beta - self._beta_f)
        beta = self._beta_f

        dva = ang_diff(vfh_dir, alpha)
        if beta > 0.2 and abs(dva) > 1.2:
            desired = vfh_dir
        else:
            desired = ang_diff(alpha + beta * dva, 0.0)

        d_stop = min(self.d_danger, 0.30) if goal_close else self.d_danger
        if self._spinning or fd <= d_stop or \
                (abs(desired) > self.rotate_threshold
                 and abs(self.v_cmd) < 0.3):
            if not self._spinning and (abs(desired) > 0.35 or fd <= d_stop):
                self._spinning = True
                self._spin_target = yaw + desired
            if self._spinning:
                err = ang_diff(self._spin_target, yaw)
                if abs(err) > 0.25:
                    self.last_branch = 'rotate_in_place'
                    w_spin = clamp(2.0 * err, -self.w_max, self.w_max)
                    # drive-while-turning: once roughly lined up AND the way
                    # ahead is clear, ease forward instead of pivoting dead
                    # still. cos(err) -> ~0 for big turns (true pivot), grows
                    # as we align (smooth arc). Removes the dead-stop between
                    # every turn (the move-spin-move-spin stutter) without
                    # driving forward while aimed at a wall (gated fd>d_danger).
                    v_creep = 0.0
                    if fd > self.d_danger and abs(err) < 0.9:
                        v_creep = clamp(0.4 * math.cos(err), 0.0, 0.4)
                    v, w = self._ramp(v_creep, w_spin, dt)
                    return v, w, False
                self._spinning = False
                self._spin_target = None

        kappa = 2.0 * math.sin(alpha) / max(L, 0.05)
        v_curve = math.sqrt(self.a_lat / max(abs(kappa), 1e-3))
        v_goal = math.sqrt(2.0 * self.a_dec_goal * max(dist_goal - 0.05, 0.0))
        if fd >= self.d_blend:
            obs_scale = 1.0
        else:
            obs_scale = clamp(
                (fd - self.d_danger) / (self.d_blend - self.d_danger),
                0.0, 1.0)
        v_target = min(self.v_max, v_curve, v_goal) * obs_scale
        if obs_scale >= 1.0:
            v_target = max(self.v_min, v_target)

        w_pp = kappa * max(v_target, 0.2)
        w_vfh = self.vfh_gain * vfh_dir
        w_target = (1.0 - beta) * w_pp + beta * w_vfh

        self.last_branch = 'drive'
        v, w = self._ramp(v_target, w_target, dt)
        return v, w, False

    def _ramp(self, v_target, w_target, dt):
        dv = self.a_max * dt
        self.v_cmd = clamp(v_target, self.v_cmd - dv, self.v_cmd + dv)
        dw = 6.0 * dt
        self.w_cmd = clamp(w_target, self.w_cmd - dw, self.w_cmd + dw)
        self.v_cmd = clamp(self.v_cmd, 0.0, self.v_max)
        self.w_cmd = clamp(self.w_cmd, -self.w_max, self.w_max)
        return self.v_cmd, self.w_cmd
