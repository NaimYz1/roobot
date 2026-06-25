#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Livox tilt calibrator (ROS1 Melodic, py2/py3).

A tilted Mid-360 stares at the floor, so the floor must be rejected very
accurately or it leaks into the obstacle slice as thousands of phantom
points. This tool fits the actual FLOOR plane from the live /livox/lidar
cloud and prints the EXACT values for amr_jackal_real.launch:
    mount_pitch_deg , mount_pitch_rad , lidar_height (mount_z)

The Mid-360 also sees the ceiling (a plane parallel to the floor), so we
only accept a plane that lies BELOW the sensor at a plausible floor height
(FLOOR_MIN..FLOOR_MAX). Run with the robot still on flat, open floor:
    python ~/amr_ws/src/amr_perception/src/calibrate_livox.py
"""
from __future__ import division, print_function

import math
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

N_CLOUDS = 5
RANSAC_ITERS = 400
RANSAC_TOL = 0.03
FLOOR_MIN = 0.10      # m: floor must be at least this far below the sensor
FLOOR_MAX = 1.20      # m: ... and no more (rejects ceiling / far walls)


class Calib(object):
    def __init__(self):
        self.topic = rospy.get_param('~livox_topic', '/livox/lidar')
        self.pts = []
        self.got = 0
        rospy.Subscriber(self.topic, PointCloud2, self.cb, queue_size=1)
        rospy.loginfo("Calibrator listening on %s ... keep the robot still "
                      "on clear, flat floor", self.topic)

    def cb(self, msg):
        if self.got >= N_CLOUDS:
            return
        arr = np.array(list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float64)
        if arr.size:
            self.pts.append(arr)
            self.got += 1
            rospy.loginfo("  collected cloud %d/%d (%d pts)",
                          self.got, N_CLOUDS, arr.shape[0])

    def ready(self):
        return self.got >= N_CLOUDS

    def solve(self):
        pts = np.vstack(self.pts)
        r = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[(r > 0.2) & (r < 4.0)]
        n = pts.shape[0]
        rospy.loginfo("Fitting floor plane over %d points ...", n)

        np.random.seed(0)
        best_cnt, best = -1, None
        for _ in range(RANSAC_ITERS):
            idx = np.random.choice(n, 3, replace=False)
            p0, p1, p2 = pts[idx[0]], pts[idx[1]], pts[idx[2]]
            nrm = np.cross(p1 - p0, p2 - p0)
            ln = np.linalg.norm(nrm)
            if ln < 1e-9:
                continue
            nrm = nrm / ln
            d = -float(nrm.dot(p0))
            # orient normal up so the floor (below the sensor) has d > 0
            if nrm[2] < 0:
                nrm = -nrm
                d = -d
            # accept ONLY a plane below the sensor at floor height
            # (this rejects the ceiling, which is parallel but far away)
            if not (FLOOR_MIN < d < FLOOR_MAX):
                continue
            dist = np.abs(pts.dot(nrm) + d)
            cnt = int(np.count_nonzero(dist < RANSAC_TOL))
            if cnt > best_cnt:
                best_cnt, best = cnt, (nrm.copy(), d)

        if best is None:
            rospy.logerr("No floor-like plane found %.2f-%.2f m below the "
                         "sensor. Is the robot on the ground on OPEN floor?",
                         FLOOR_MIN, FLOOR_MAX)
            return

        nrm, d = best
        # refit on inliers via 3x3 covariance eigvec (instant; never svd Nx3)
        dist = np.abs(pts.dot(nrm) + d)
        inl = pts[dist < RANSAC_TOL]
        c = inl.mean(axis=0)
        cov = np.cov((inl - c).T)
        evals, evecs = np.linalg.eigh(cov)
        nrm = evecs[:, 0]
        d = -float(nrm.dot(c))
        if nrm[2] < 0:
            nrm = -nrm
            d = -d

        nx, ny, nz = float(nrm[0]), float(nrm[1]), float(nrm[2])
        pitch = math.atan2(-nx, nz)
        roll = math.atan2(ny, nz)
        height = abs(d)
        inl_frac = 100.0 * inl.shape[0] / n

        print("\n" + "=" * 56)
        print(" LIVOX FLOOR-PLANE CALIBRATION RESULT")
        print("=" * 56)
        print(" floor inliers      : %.0f%% of points" % inl_frac)
        print(" plane normal (sxyz): [% .3f % .3f % .3f]" % (nx, ny, nz))
        print(" --- put these in amr_jackal_real.launch defaults ---")
        print("   mount_pitch_deg  := %.2f" % math.degrees(pitch))
        print("   mount_pitch_rad  := %.6f" % pitch)
        print("   lidar_height     := %.3f" % height)
        print(" ----------------------------------------------------")
        if abs(math.degrees(roll)) > 3.0:
            print(" WARNING: roll = %.1f deg (side tilt). Pitch-only model"
                  % math.degrees(roll))
            print("          cannot fully flatten this - straighten bracket.")
        else:
            print(" roll = %.1f deg (ok, level side-to-side)"
                  % math.degrees(roll))
        if inl_frac < 15.0:
            print(" NOTE: few floor inliers - calibrate on open flat floor.")
        print("=" * 56 + "\n")


def main():
    rospy.init_node('calibrate_livox', anonymous=True)
    c = Calib()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not c.ready():
        rate.sleep()
    if c.ready():
        c.solve()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
