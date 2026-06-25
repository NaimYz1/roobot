#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Rolling ego-centric log-odds occupancy grid (ROS1 Melodic, py2/py3, numpy only).

Short-term LOCAL MEMORY for the reactive layer. The Mid-360 gives a sparse,
non-repetitive frame every 0.1 s, so single-frame VFH+ flickers and the robot
re-discovers (and re-dodges) obstacles it just turned away from -> spinning.
This grid fuses frames over time in the ODOM frame with Bayesian-style log-odds
and a forgetting decay, restoring the persistent 2-D histogram grid the ORIGINAL
VFH (Borenstein & Koren 1991) was defined on. The same grid is what the harmonic
field will later solve on.

Model:
  * each frame: recenter on the robot, multiply all cells by `decay` (forgetting),
    then add `l_hit` to every cell that received >= `min_pts` points this frame.
  * a cell is OCCUPIED when its log-odds exceeds `occ_thresh`, which requires a
    couple of consistent frames -> rejects single-frame specks but reacts fast.
  * values are clamped to [l_min, l_max] so a persistently-seen wall stays put
    and a transient (a passer-by) fades in ~ log(thresh/l_hit)/log(decay) frames.

decay=0.92 @ 10 Hz forgets an unseen obstacle in ~2-3 s; raise to forget slower.
"""
from __future__ import division

import math

import numpy as np


class LocalGrid(object):
    def __init__(self, size_m=6.0, res=0.05, decay=0.92, l_hit=0.7,
                 l_min=-2.0, l_max=3.0, occ_thresh=1.0, min_pts=2):
        self.res = float(res)
        self.n = int(round(size_m / res))          # cells per side
        self.half = self.n // 2
        self.decay = float(decay)
        self.l_hit = float(l_hit)
        self.l_min = float(l_min)
        self.l_max = float(l_max)
        self.occ_thresh = float(occ_thresh)
        self.min_pts = int(min_pts)
        self.L = np.zeros((self.n, self.n), dtype=np.float32)   # [iy, ix]
        self.cx = None                             # grid centre in odom (m)
        self.cy = None

    def _recenter(self, rx, ry):
        """Keep the grid centred on the robot; shift stored cells by the integer
        cell delta and zero the freshly-exposed edges (never-seen = unknown)."""
        ncx = round(rx / self.res) * self.res
        ncy = round(ry / self.res) * self.res
        if self.cx is None:
            self.cx, self.cy = ncx, ncy
            return
        dx = int(round((ncx - self.cx) / self.res))
        dy = int(round((ncy - self.cy) / self.res))
        if dx == 0 and dy == 0:
            return
        self.L = np.roll(self.L, (-dy, -dx), axis=(0, 1))
        if dx > 0:
            self.L[:, -dx:] = 0.0
        elif dx < 0:
            self.L[:, :-dx] = 0.0
        if dy > 0:
            self.L[-dy:, :] = 0.0
        elif dy < 0:
            self.L[:-dy, :] = 0.0
        self.cx, self.cy = ncx, ncy

    def update(self, rx, ry, xo, yo):
        """Advance one frame. rx,ry = robot in odom; xo,yo = obstacle points in
        odom (already ground-removed + self-filtered)."""
        self._recenter(rx, ry)
        self.L *= self.decay
        if xo is not None and xo.size:
            ix = np.round((xo - self.cx) / self.res).astype(np.int32) + self.half
            iy = np.round((yo - self.cy) / self.res).astype(np.int32) + self.half
            m = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
            ix, iy = ix[m], iy[m]
            if ix.size:
                key = iy.astype(np.int64) * self.n + ix
                uniq, cnt = np.unique(key, return_counts=True)
                hit = uniq[cnt >= self.min_pts]
                if hit.size:
                    hy = (hit // self.n).astype(np.int32)
                    hx = (hit % self.n).astype(np.int32)
                    self.L[hy, hx] += self.l_hit
        np.clip(self.L, self.l_min, self.l_max, out=self.L)

    def occupied_base(self, rx, ry, yaw):
        """Occupied cell centres expressed in base_link (xb forward, yb left),
        for feeding VFH+. Matches vfh_plus_node's world->base convention."""
        iy, ix = np.nonzero(self.L > self.occ_thresh)
        if ix.size == 0:
            return np.empty(0, np.float32), np.empty(0, np.float32)
        wx = self.cx + (ix - self.half) * self.res
        wy = self.cy + (iy - self.half) * self.res
        dx = wx - rx
        dy = wy - ry
        c, s = math.cos(yaw), math.sin(yaw)
        xb = dx * c + dy * s
        yb = -dx * s + dy * c
        return xb.astype(np.float32), yb.astype(np.float32)

    def occupancy(self):
        """Probability grid (0..1) for the harmonic solver / debug. p = sigma(L)."""
        return 1.0 / (1.0 + np.exp(-self.L))


# --- offline self-test (no ROS): mark a wall, drive past it, watch it persist
#     then fade. Run: python local_grid.py
if __name__ == '__main__':
    g = LocalGrid(size_m=4.0, res=0.05, decay=0.9, l_hit=0.7,
                  occ_thresh=1.0, min_pts=1)
    # robot at origin, a point obstacle 1 m ahead (odom)
    ox = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    oy = np.array([0.0, 0.02, -0.02], dtype=np.float32)
    for k in range(3):
        g.update(0.0, 0.0, ox, oy)
    xb, yb = g.occupied_base(0.0, 0.0, 0.0)
    print("after 3 hits: %d occupied cells, nearest fwd=%.2f m"
          % (xb.size, float(np.min(np.hypot(xb, yb))) if xb.size else -1))
    assert xb.size > 0, "obstacle should be remembered"
    # now stop seeing it (robot turned away): decay only
    faded = None
    for k in range(40):
        g.update(0.0, 0.0, np.empty(0, np.float32), np.empty(0, np.float32))
        xb, yb = g.occupied_base(0.0, 0.0, 0.0)
        if xb.size == 0:
            faded = k + 1
            break
    print("forgotten after %s unseen frames (~%.1f s @10Hz)"
          % (faded, (faded or 0) / 10.0))
    assert faded is not None, "obstacle should eventually be forgotten"
    print("local_grid self-test PASS")
