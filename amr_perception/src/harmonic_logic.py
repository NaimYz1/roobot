#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Harmonic (Laplace) potential-field local planner. ROS1 Melodic, py2/py3, numpy.

Solves Laplace's equation  del^2 phi = 0  on the local occupancy grid with
obstacles as HIGH-potential Dirichlet boundaries and a goal-direction SINK as
the LOW boundary. Gradient descent of phi is a smooth steering direction that
flows AROUND obstacles. By the strong maximum principle, a harmonic function
has NO interior local minimum other than the sink - the rigorous cure for the
gap-to-gap oscillation of VFH+ and the local-minima traps of classical APF.
This is the Green's-function / Dirac-source ("obstacle scattering") field made
concrete on a grid.

Honest caveats (handled here): saddle points can still occur -> we detect a
near-flat gradient and report 'boxed' so the controller rotates out; the solver
is run toward convergence each cycle, WARM-STARTED from the previous solution
(the field changes slowly); obstacles are inflated by ~the robot radius for
clearance. Pure Pursuit still does the kinematic/curvature tracking.

The grid is odom-aligned (ix=+x_odom, iy=+y_odom), robot at the centre, so the
returned angle is in the ODOM frame; the node subtracts yaw to get base_link.
"""
from __future__ import division

import math

import numpy as np


def inflate(occ, cells):
    """Binary-dilate the obstacle grid by `cells` (Chebyshev) for clearance."""
    if cells <= 0:
        return occ
    out = occ.copy()
    for _ in range(int(cells)):
        g = out.copy()
        g[1:, :] |= out[:-1, :]
        g[:-1, :] |= out[1:, :]
        g[:, 1:] |= out[:, :-1]
        g[:, :-1] |= out[:, 1:]
        out = g
    return out


class HarmonicField(object):
    def __init__(self, omega=1.9, iters=600, inflate_cells=3, flat_eps=1e-10):
        self.omega = float(omega)
        self.iters = int(iters)
        self.inflate_cells = int(inflate_cells)
        self.flat_eps = float(flat_eps)
        self.phi = None
        self._red = None
        self._black = None
        self._shape = None

    def _colors(self, fixed):
        H, W = fixed.shape
        if self._shape != (H, W):
            ii, jj = np.indices((H, W))
            self._parity = (ii + jj) % 2
            self._shape = (H, W)
        red = (self._parity == 0) & (~fixed)
        black = (self._parity == 1) & (~fixed)
        return red, black

    def solve(self, occ, sink_ij, robot_ij, iters=None):
        """occ: bool grid (True=obstacle). sink_ij/robot_ij: (iy, ix).
        Returns (angle_rad_in_grid_axes or None, grad_magnitude).
        None => field is flat at the robot (boxed / saddle) -> rotate out."""
        H, W = occ.shape
        occ = inflate(occ, self.inflate_cells)
        # fresh field each solve: the memory grid recenters as the robot moves,
        # so a persisted phi would be spatially MISALIGNED with the shifted occ
        # grid -> noisy gradient -> steering jitter. Re-solve from scratch.
        phi = np.full((H, W), 0.5, dtype=np.float64)

        fixed = occ.copy()
        phi[occ] = 1.0
        fixed[0, :] = fixed[-1, :] = fixed[:, 0] = fixed[:, -1] = True
        phi[0, :] = phi[-1, :] = phi[:, 0] = phi[:, -1] = 1.0

        sy = min(max(int(sink_ij[0]), 0), H - 1)
        sx = min(max(int(sink_ij[1]), 0), W - 1)
        phi[sy, sx] = 0.0
        fixed[sy, sx] = True
        # also pin a small neighbourhood of the sink low, so a single border
        # cell next to a wall still pulls the field
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            ny, nx = sy + dy, sx + dx
            if 0 <= ny < H and 0 <= nx < W and not occ[ny, nx]:
                phi[ny, nx] = 0.0
                fixed[ny, nx] = True

        red, black = self._colors(fixed)
        n = self.iters if iters is None else int(iters)
        nb = np.zeros_like(phi)
        for _ in range(n):
            for mask in (red, black):
                nb[1:-1, 1:-1] = 0.25 * (phi[:-2, 1:-1] + phi[2:, 1:-1] +
                                         phi[1:-1, :-2] + phi[1:-1, 2:])
                phi[mask] = (1.0 - self.omega) * phi[mask] + self.omega * nb[mask]
        self.phi = phi

        ry = min(max(int(robot_ij[0]), 1), H - 2)
        rx = min(max(int(robot_ij[1]), 1), W - 2)
        gx = 0.5 * (phi[ry, rx + 1] - phi[ry, rx - 1])   # d/d(ix) = d/dx_odom
        gy = 0.5 * (phi[ry + 1, rx] - phi[ry - 1, rx])   # d/d(iy) = d/dy_odom
        mag = math.hypot(gx, gy)
        if mag < self.flat_eps:
            return None, 0.0
        # descend the potential: move along -grad
        return math.atan2(-gy, -gx), mag

    def solve_flow(self, occ, flow_dir, robot_ij, iters=None):
        """POTENTIAL-FLOW form (smooth, no discrete sink). The domain boundary is
        a LINEAR RAMP - low toward flow_dir, high opposite - so the field drifts
        uniformly toward the goal in open space; obstacles (high Dirichlet) make
        it flow AROUND them like a fluid stream past a body. -grad is the smooth
        steering direction. This is the ideal-flow / obstacle-scattering form of
        the harmonic idea and removes the wide/jumpy steering of a point sink.
        flow_dir is in GRID axes (atan2(iy, ix))."""
        H, W = occ.shape
        occ = inflate(occ, self.inflate_cells)
        phi = np.full((H, W), 0.5, dtype=np.float64)
        fixed = occ.copy()
        cdir, sdir = math.cos(flow_dir), math.sin(flow_dir)
        ii, jj = np.indices((H, W))
        # project position onto -flow_dir: HIGH opposite the goal, LOW toward it
        proj = -((jj - (W - 1) / 2.0) * cdir + (ii - (H - 1) / 2.0) * sdir)
        rng = float(proj.max() - proj.min()) + 1e-9
        ramp = (0.1 + 0.8 * (proj - proj.min()) / rng).astype(np.float64)
        fixed[0, :] = fixed[-1, :] = fixed[:, 0] = fixed[:, -1] = True
        phi[0, :] = ramp[0, :]
        phi[-1, :] = ramp[-1, :]
        phi[:, 0] = ramp[:, 0]
        phi[:, -1] = ramp[:, -1]
        phi[occ] = 1.0                    # obstacles above the ramp -> repulsive
        red, black = self._colors(fixed)
        n = self.iters if iters is None else int(iters)
        nb = np.zeros_like(phi)
        for _ in range(n):
            for mask in (red, black):
                nb[1:-1, 1:-1] = 0.25 * (phi[:-2, 1:-1] + phi[2:, 1:-1] +
                                         phi[1:-1, :-2] + phi[1:-1, 2:])
                phi[mask] = (1.0 - self.omega) * phi[mask] + self.omega * nb[mask]
        self.phi = phi
        ry = min(max(int(robot_ij[0]), 1), H - 2)
        rx = min(max(int(robot_ij[1]), 1), W - 2)
        gx = 0.5 * (phi[ry, rx + 1] - phi[ry, rx - 1])
        gy = 0.5 * (phi[ry + 1, rx] - phi[ry - 1, rx])
        mag = math.hypot(gx, gy)
        if mag < self.flat_eps:
            return None, 0.0
        return math.atan2(-gy, -gx), mag


# --- offline self-test (no ROS): wall with a gap; the descent must steer toward
#     the gap, not straight into the wall. Run: python harmonic_logic.py
if __name__ == '__main__':
    n = 41
    occ = np.zeros((n, n), dtype=bool)
    occ[20, 0:31] = True              # wall across, GAP on the right (ix>=31)
    hf = HarmonicField(omega=1.9, iters=1200, inflate_cells=1)
    # robot bottom-centre (iy=35, ix=15); sink top-centre (iy=2, ix=15)
    ang, mag = hf.solve(occ, sink_ij=(2, 15), robot_ij=(35, 15))
    deg = math.degrees(ang)
    cx, cy = math.cos(ang), math.sin(ang)   # cx = ix-comp, cy = iy-comp
    print("grad mag=%.4g  angle(grid)=%.1f deg  ix-comp=%+.2f iy-comp=%+.2f"
          % (mag, deg, cx, cy))
    # must head UP (toward sink => -iy => iy-comp < 0) AND toward the gap on the
    # RIGHT (+ix => ix-comp > 0). Straight-into-wall would be ix-comp ~ 0.
    assert cy < 0, "should head up toward the sink, not down"
    assert cx > 0.15, "should veer RIGHT toward the gap, not straight into wall"
    print("harmonic_logic self-test PASS (sink field steers around the wall)")

    # potential-FLOW form. Smooth in the open, but facing a wall HEAD-ON it
    # STAGNATES (velocity -> 0, back-flow) - the known ideal-flow limitation. The
    # node runs 'hybrid' mode: flow primary, and when the flow points far off the
    # goal (stagnation, detected here), it falls back to the SINK that pulls
    # around the obstacle.
    hf2 = HarmonicField(omega=1.9, iters=1500, inflate_cells=1)
    a2, m2 = hf2.solve_flow(occ, -math.pi / 2.0, (35, 15))   # flow straight INTO wall
    dev = abs((math.degrees(a2) - (-90.0) + 180.0) % 360.0 - 180.0)
    print("FLOW head-on: angle(grid)=%.0f deg, dev-from-goal=%.0f deg (>75 => stagnation)"
          % (math.degrees(a2), dev))
    assert a2 is not None and dev > 75.0, "head-on flow must stagnate (detectable)"
    print("solve_flow self-test PASS (smooth field; head-on stagnation is detectable)")
