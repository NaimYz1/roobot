#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
VFH+ (Vector Field Histogram Plus) steering logic.
Pure Python/numpy - no ROS imports. Python 2.7 and 3 compatible.

Key difference vs the old node: the old code steered AWAY from the closest
obstacle with no idea where the goal was -> orbit/limit-cycle around
obstacles. VFH+ instead finds all open gaps (valleys) and picks the
candidate direction CLOSEST to the target direction, with hysteresis.
"""
from __future__ import division, print_function

import math
import numpy as np


def ang_diff(a, b):
    """Smallest signed difference a-b, wrapped to [-pi, pi]."""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class VFHPlus(object):
    def __init__(self,
                 num_bins=72,          # 5 deg per bin
                 max_range=4.0,        # ignore obstacles beyond this
                 active_range=1.6,     # bins blocked if obstacle closer
                 robot_radius=0.30,    # half-width + a bit
                 safety_margin=0.10,
                 valley_min_width=3,   # bins; narrower gaps are unsafe
                 mu_target=5.0,        # cost weight: deviation from target
                 mu_heading=2.0,       # cost weight: deviation from straight
                 mu_prev=3.0):         # cost weight: deviation from last cmd
                                       # (3.0 damps left/right flip-flopping
                                       #  that made the robot spin in clutter)
        self.num_bins = num_bins
        self.max_range = max_range
        self.active_range = active_range
        self.enlarge = robot_radius + safety_margin
        self.valley_min_width = valley_min_width
        self.mu_t = mu_target
        self.mu_h = mu_heading
        self.mu_p = mu_prev
        self.prev_dir = 0.0
        self.bin_w = 2.0 * math.pi / num_bins

    # ------------------------------------------------------------------
    def bin_angle(self, i):
        """Center angle of bin i, robot frame; bin num_bins//2 = forward."""
        return (i - self.num_bins // 2) * self.bin_w

    def angle_bin(self, a):
        i = int(round(a / self.bin_w)) + self.num_bins // 2
        return i % self.num_bins

    # ------------------------------------------------------------------
    def build_histogram(self, angles, ranges):
        """angles/ranges: obstacle points in robot frame (rad, m).
        Returns (blocked bool array, per-bin min distance array)."""
        n = self.num_bins
        dist = np.full(n, self.max_range, dtype=np.float32)
        blocked = np.zeros(n, dtype=bool)

        for a, r in zip(angles, ranges):
            if not (0.05 < r < self.max_range):
                continue
            # robot-size enlargement: each point blocks an angular window
            gamma = math.asin(min(1.0, self.enlarge / max(r, self.enlarge)))
            b0 = self.angle_bin(a - gamma)
            b1 = self.angle_bin(a + gamma)
            i = b0
            while True:
                if r < dist[i]:
                    dist[i] = r
                if r < self.active_range:
                    blocked[i] = True
                if i == b1:
                    break
                i = (i + 1) % n
        return blocked, dist

    # ------------------------------------------------------------------
    def find_valleys(self, blocked):
        """Return list of (start_bin, width) of free runs (wrap-aware)."""
        n = self.num_bins
        if not blocked.any():
            return [(0, n)]
        if blocked.all():
            return []
        valleys = []
        # rotate so index 0 is blocked -> runs don't wrap
        first_blocked = int(np.argmax(blocked))
        rot = np.roll(blocked, -first_blocked)
        i = 0
        while i < n:
            if not rot[i]:
                j = i
                while j < n and not rot[j]:
                    j += 1
                valleys.append(((i + first_blocked) % n, j - i))
                i = j
            else:
                i += 1
        return valleys

    # ------------------------------------------------------------------
    def steer(self, angles, ranges, target_angle):
        """Main entry. target_angle: desired direction in robot frame (rad).
        Returns dict: direction (rad or None), clear (bool),
        front_dist (m, min over +-30 deg), min_dist (m, min over +-90 deg),
        dist (per-bin distance array for debug).
        direction None => boxed in, caller should stop/rotate."""
        blocked, dist = self.build_histogram(angles, ranges)
        n = self.num_bins
        c = n // 2

        # front/min distance from the RAW points, not the enlarged
        # histogram: the robot-radius enlargement smears side walls into
        # the front cone, which made the controller think a wall beside
        # the robot was dead ahead -> permanent crawl / phantom stops.
        ang = np.asarray(angles, dtype=np.float32)
        rng = np.asarray(ranges, dtype=np.float32)
        valid = (rng > 0.05) & (rng < self.max_range)
        # front_dist = COLLISION-CORRIDOR distance: nearest point inside
        # the robot's swept width straight ahead. A +-30 deg cone vastly
        # overstated danger at close range - an obstacle being passed
        # 0.5 m beside the robot read as "dead ahead", causing phantom
        # emergency stops while overtaking it.
        ahead = valid & (np.cos(ang) > 0.0) & \
            (np.abs(rng * np.sin(ang)) < self.enlarge)
        in90 = valid & (np.abs(ang) < math.radians(90))
        # WEDGE = wider forward cone (+-40 deg) used to DECIDE whether to
        # avoid. The razor collision-corridor (front_dist) alone missed walls
        # sitting just off the centre-line, so the robot reported "clear" and
        # drove straight into them. The wedge catches anything broadly ahead.
        wedge = valid & (np.cos(ang) > 0.0) & (np.abs(ang) < math.radians(40))
        front_dist = float(rng[ahead].min()) if ahead.any() else self.max_range
        min_dist = float(rng[in90].min()) if in90.any() else self.max_range
        front_wedge = float(rng[wedge].min()) if wedge.any() else self.max_range

        # only cruise straight if the goal bin is open AND the whole forward
        # wedge is clear out to active_range. Otherwise fall through to the
        # valley search and steer around the obstacle.
        target_bin_free = not blocked[self.angle_bin(target_angle)]
        if target_bin_free and front_wedge >= self.active_range:
            self.prev_dir = target_angle
            return {'direction': target_angle, 'clear': True, 'boxed': False,
                    'front_dist': front_dist, 'min_dist': min_dist,
                    'dist': dist}

        valleys = self.find_valleys(blocked)
        candidates = []
        wide = max(self.valley_min_width * 2, 6)
        for start, width in valleys:
            if width < self.valley_min_width:
                continue
            if width >= n:  # fully open
                candidates.append(target_angle)
                continue
            # borders pulled inward by half the min safe width
            inset = (self.valley_min_width / 2.0) * self.bin_w
            a_lo = self.bin_angle(start) + inset
            a_hi = self.bin_angle(start + width - 1) - inset
            if width >= wide:
                candidates.append(a_lo)
                candidates.append(a_hi)
                # target direction itself, if it lies inside this valley
                if a_lo <= a_hi:  # valley does not wrap behind the robot
                    if a_lo <= ang_diff(target_angle, 0.0) <= a_hi:
                        candidates.append(target_angle)
            else:
                candidates.append((a_lo + a_hi) / 2.0)

        if not candidates:
            # Histogram saturated (everything within active_range). The old
            # behavior returned None -> controller spun in place FOREVER,
            # because rotating never unblocks a saturated histogram.
            # Instead point at the most open direction so the caller can
            # rotate there, see daylight, and drive out.
            best_bin = int(np.argmax(dist))
            fallback = ang_diff(self.bin_angle(best_bin), 0.0)
            self.prev_dir = fallback
            return {'direction': fallback, 'clear': False, 'boxed': True,
                    'front_dist': front_dist, 'min_dist': min_dist,
                    'dist': dist}

        # CRITICAL: wrap every candidate to [-pi, pi]. Valleys that span the
        # rear produce raw bin angles like 6.5 rad; publishing those made the
        # controller spin at max rate into obstacles.
        candidates = [ang_diff(cand, 0.0) for cand in candidates]

        def cost(cand):
            return (self.mu_t * abs(ang_diff(cand, target_angle)) +
                    self.mu_h * abs(ang_diff(cand, 0.0)) +
                    self.mu_p * abs(ang_diff(cand, self.prev_dir)))

        best = min(candidates, key=cost)
        self.prev_dir = best
        return {'direction': best, 'clear': False, 'boxed': False,
                'front_dist': front_dist, 'min_dist': min_dist,
                'dist': dist}
