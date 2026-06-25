#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Ground-Z diagnostic for the tilted Livox Mid-360 (ROS1 Melodic, py2/py3).

Read-only. Subscribes to /livox/lidar, de-tilts the cloud with the CONFIGURED
STATIC extrinsic (mount_pitch_deg, mount_z) exactly like vfh_plus_node.py, and
prints a terminal histogram of the de-tilted height `zb` for near points.

What to look for (robot STILL on open, flat floor):
  * The floor should collapse to ONE TIGHT SPIKE at zb ~= 0.00 m.
  * That spike must sit well BELOW the obstacle gate z_min (default 0.15 m),
    with margin (a few sigma), or the floor leaks in as phantom obstacles.
  * "leak into gate" should be ~0% on empty floor. A big number means the
    de-tilt is wrong or the gate is too low -> phantom walls -> boxed-in/spin.

It also re-runs the de-tilt with the NEGATED pitch and reports which sign gives
the tighter floor spike, to catch a wrong-sign extrinsic.

    python ~/amr_ws/src/amr_perception/src/diag_ground.py
    python diag_ground.py _mount_pitch_deg:=38.22 _mount_z:=0.447 _near_range:=3.0
"""
from __future__ import division, print_function

import math

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


def detilt(x, y, z, pitch, mount_x, mount_z):
    """Same transform as vfh_plus_node.pc_callback: livox_frame -> base_link.
    Returns xb (forward), yb (left), zb (height above ground)."""
    ca, sa = math.cos(pitch), math.sin(pitch)
    xb = x * ca + z * sa + mount_x
    yb = y
    zb = -x * sa + z * ca + mount_z
    return xb, yb, zb


def floor_stats(zb, near):
    """Peak (mode) of the near-point height distribution in the plausible
    floor band, and the 1-sigma spread of points around that peak."""
    znear = zb[near]
    if znear.size < 50:
        return None
    band = znear[(znear > -0.5) & (znear < 0.5)]   # plausible floor band
    if band.size < 50:
        return None
    hist, edges = np.histogram(band, bins=100, range=(-0.5, 0.5))
    pk = 0.5 * (edges[int(np.argmax(hist))] + edges[int(np.argmax(hist)) + 1])
    tight = band[np.abs(band - pk) < 0.10]
    sigma = float(np.std(tight)) if tight.size else float('nan')
    return pk, sigma, int(tight.size), int(znear.size)


def ascii_hist(zb, near, z_min, z_max, lo=-0.5, hi=2.0, nb=50, width=46):
    znear = zb[near]
    znear = znear[(znear >= lo) & (znear <= hi)]
    if znear.size == 0:
        print("   (no near points in window)")
        return
    hist, edges = np.histogram(znear, bins=nb, range=(lo, hi))
    mx = max(1, int(hist.max()))
    for i in range(nb):
        c = 0.5 * (edges[i] + edges[i + 1])
        bar = "#" * int(round(width * hist[i] / mx))
        tag = ""
        if edges[i] <= 0.0 < edges[i + 1]:
            tag = "  <- floor (zb=0)"
        elif edges[i] <= z_min < edges[i + 1]:
            tag = "  <- z_min (gate)"
        elif edges[i] <= z_max < edges[i + 1]:
            tag = "  <- z_max (gate)"
        print("  %+5.2f | %-46s %5d%s" % (c, bar, int(hist[i]), tag))


class GroundDiag(object):
    def __init__(self):
        self.topic = rospy.get_param('~livox_topic', '/livox/lidar')
        self.pitch = math.radians(rospy.get_param('~mount_pitch_deg', 38.22))
        self.mount_x = rospy.get_param('~mount_x', 0.0)
        self.mount_z = rospy.get_param('~mount_z', 0.447)
        self.z_min = rospy.get_param('~z_min', 0.15)
        self.z_max = rospy.get_param('~z_max', 1.50)
        self.min_range = rospy.get_param('~min_range', 0.40)
        self.near_range = rospy.get_param('~near_range', 3.0)
        self.n_clouds = int(rospy.get_param('~n_clouds', 10))
        # self-box (match vfh_plus_node defaults) so the robot's own body is out
        self.sx0 = rospy.get_param('~self_x_min', -0.45)
        self.sx1 = rospy.get_param('~self_x_max', 0.45)
        self.sya = rospy.get_param('~self_y_abs', 0.32)
        self.pts, self.got = [], 0
        rospy.Subscriber(self.topic, PointCloud2, self.cb, queue_size=1)
        rospy.loginfo("ground-diag: listening on %s; keep robot STILL on open "
                      "flat floor (collecting %d clouds)...",
                      self.topic, self.n_clouds)

    def cb(self, msg):
        if self.got >= self.n_clouds:
            return
        arr = np.array(list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float64)
        if arr.size:
            self.pts.append(arr)
            self.got += 1

    def ready(self):
        return self.got >= self.n_clouds

    def report(self):
        pts = np.vstack(self.pts)
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        xb, yb, zb = detilt(x, y, z, self.pitch, self.mount_x, self.mount_z)
        r = np.hypot(xb, yb)
        inbox = (xb > self.sx0) & (xb < self.sx1) & (np.abs(yb) < self.sya)
        near = (r > self.min_range) & (r < self.near_range) & (~inbox)

        print("\n" + "=" * 62)
        print(" GROUND-Z DIAGNOSTIC  (configured static de-tilt)")
        print("=" * 62)
        print(" pitch=%.2f deg  mount_z=%.3f m  near<=%.1f m  gate z=[%.2f, %.2f]"
              % (math.degrees(self.pitch), self.mount_z, self.near_range,
                 self.z_min, self.z_max))
        print(" near points analysed: %d of %d" % (int(np.count_nonzero(near)),
                                                    pts.shape[0]))

        fs = floor_stats(zb, near)
        if fs is None:
            print(" !! too few near floor points - run on open flat floor.")
        else:
            pk, sig, ntight, ntot = fs
            leak = zb[near]
            ngate = int(np.count_nonzero((leak > self.z_min) &
                                         (leak < self.z_max)))
            pct = 100.0 * ngate / max(1, leak.size)
            print(" floor spike  : zb = %+.3f m   (ideal ~ 0.000)" % pk)
            print(" floor spread : sigma = %.1f cm  (%d pts in +/-10 cm)"
                  % (100.0 * sig, ntight))
            print(" leak into gate [%.2f, %.2f]: %d pts = %.1f%% of near"
                  % (self.z_min, self.z_max, ngate, pct))
            suggest = pk + 4.0 * sig
            print(" suggested deterministic ground gate z_min >= %.3f m"
                  " (peak + 4 sigma)" % suggest)
            if abs(pk) > 0.08 or sig > 0.05:
                print(" VERDICT: floor spike is OFF/ SMEARED -> de-tilt suspect"
                      " (see sign check below).")
            elif pct > 5.0:
                print(" VERDICT: de-tilt ok but gate too low -> raise z_min to"
                      " the suggested value.")
            else:
                print(" VERDICT: de-tilt looks GOOD; floor well clear of gate.")

        # --- sign sanity check: which pitch sign gives the tighter floor? ---
        print("-" * 62)
        for lbl, p in (("configured", self.pitch), ("NEGATED", -self.pitch)):
            xb2, yb2, zb2 = detilt(x, y, z, p, self.mount_x, self.mount_z)
            r2 = np.hypot(xb2, yb2)
            near2 = (r2 > self.min_range) & (r2 < self.near_range)
            f = floor_stats(zb2, near2)
            if f:
                print("  pitch %-10s (%+.2f deg): floor zb=%+.3f m sigma=%.1f cm"
                      % (lbl, math.degrees(p), f[0], 100.0 * f[1]))
        print("  -> the sign with floor zb nearest 0 and smallest sigma is"
              " correct.")

        # --- terminal histogram of de-tilted height for near points ---
        print("-" * 62)
        print(" de-tilted height histogram (near points):")
        ascii_hist(zb, near, self.z_min, self.z_max)
        print("=" * 62 + "\n")


def main():
    rospy.init_node('diag_ground', anonymous=True)
    d = GroundDiag()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not d.ready():
        rate.sleep()
    if d.ready():
        d.report()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
