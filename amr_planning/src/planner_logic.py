#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Grid A* + any-angle smoothing (Theta*-style) global planner logic.
Pure Python/numpy - no ROS imports. Python 2.7 and 3 compatible.

Replaces PRM: deterministic, always finds a path if one exists,
plans in milliseconds instead of tens of seconds.
"""
from __future__ import division, print_function

import heapq
import math
import numpy as np

SQRT2 = math.sqrt(2.0)

# 8-connected neighborhood: (dx, dy, step_cost)
NEIGHBORS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2),
]


def octile(ax, ay, bx, by):
    dx = abs(ax - bx)
    dy = abs(ay - by)
    return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)


def nearest_free(blocked, x, y, max_radius=40):
    """If (x, y) is blocked/off-grid, spiral outward to the nearest free cell.
    Makes planning robust to AMCL drift and goals clicked near walls."""
    h, w = blocked.shape
    x = int(min(max(x, 0), w - 1))
    y = int(min(max(y, 0), h - 1))
    if not blocked[y, x]:
        return x, y
    for r in range(1, max_radius + 1):
        x0, x1 = max(0, x - r), min(w - 1, x + r)
        y0, y1 = max(0, y - r), min(h - 1, y + r)
        # ring cells only
        best = None
        best_d = 1e9
        for yy in range(y0, y1 + 1):
            for xx in (x0, x1):
                if not blocked[yy, xx]:
                    d = (xx - x) ** 2 + (yy - y) ** 2
                    if d < best_d:
                        best, best_d = (xx, yy), d
        for xx in range(x0, x1 + 1):
            for yy in (y0, y1):
                if not blocked[yy, xx]:
                    d = (xx - x) ** 2 + (yy - y) ** 2
                    if d < best_d:
                        best, best_d = (xx, yy), d
        if best is not None:
            return best
    return None


def line_of_sight(blocked, p1, p2):
    """Supercover Bresenham: True if straight segment p1->p2 crosses only
    free cells (no diagonal corner-cutting)."""
    x0, y0 = int(p1[0]), int(p1[1])
    x1, y1 = int(p2[0]), int(p2[1])
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        if blocked[y, x]:
            return False
        if x == x1 and y == y1:
            return True
        e2 = 2 * err
        moved_x = moved_y = False
        if e2 > -dy:
            err -= dy
            x += sx
            moved_x = True
        if e2 < dx:
            err += dx
            y += sy
            moved_y = True
        # supercover: when stepping diagonally also check the two adjacent
        # cells so the path cannot slip between two diagonal obstacles
        if moved_x and moved_y:
            if blocked[y - sy, x] or blocked[y, x - sx]:
                return False


def a_star(blocked, start, goal, weight=1.2):
    """Weighted A* on a boolean grid (True = blocked).
    start/goal: (x, y). Returns list of (x, y) or None."""
    h, w = blocked.shape
    sx, sy = start
    gx, gy = goal

    g_score = np.full((h, w), np.inf, dtype=np.float32)
    g_score[sy, sx] = 0.0
    came = np.full((h, w), -1, dtype=np.int32)  # packed parent index
    closed = np.zeros((h, w), dtype=bool)

    open_heap = [(weight * octile(sx, sy, gx, gy), sx, sy)]

    while open_heap:
        _, x, y = heapq.heappop(open_heap)
        if closed[y, x]:
            continue
        closed[y, x] = True
        if x == gx and y == gy:
            # reconstruct
            path = []
            cx, cy = x, y
            while True:
                path.append((cx, cy))
                p = came[cy, cx]
                if p < 0:
                    break
                cx, cy = p % w, p // w
            path.reverse()
            return path
        gc = g_score[y, x]
        for dx, dy, cost in NEIGHBORS:
            nx, ny = x + dx, y + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            if blocked[ny, nx] or closed[ny, nx]:
                continue
            # no corner cutting on diagonals
            if dx != 0 and dy != 0:
                if blocked[y, x + dx] or blocked[y + dy, x]:
                    continue
            ng = gc + cost
            if ng < g_score[ny, nx]:
                g_score[ny, nx] = ng
                came[ny, nx] = y * w + x
                heapq.heappush(
                    open_heap, (ng + weight * octile(nx, ny, gx, gy), nx, ny))
    return None


def smooth_path(blocked, path):
    """Greedy string-pulling: replace the grid path with the fewest
    straight segments that keep line of sight. Theta*-quality output."""
    if path is None or len(path) < 3:
        return path
    out = [path[0]]
    i = 0
    n = len(path)
    while i < n - 1:
        j = n - 1
        while j > i + 1:
            if line_of_sight(blocked, path[i], path[j]):
                break
            # bisect-ish backoff: step down faster on long paths
            j -= max(1, (j - i) // 8)
        out.append(path[j])
        i = j
    return out


def densify(path, max_seg):
    """Insert intermediate points so no segment is longer than max_seg
    (in grid cells). Pure pursuit likes dense paths."""
    if path is None or len(path) < 2:
        return path
    out = [path[0]]
    for k in range(1, len(path)):
        x0, y0 = out[-1]
        x1, y1 = path[k]
        d = math.hypot(x1 - x0, y1 - y0)
        steps = int(d // max_seg)
        for s in range(1, steps + 1):
            t = s / (steps + 1.0)
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        out.append(path[k])
    return out


class GridPlanner(object):
    """Plans on a (possibly downsampled) boolean occupancy grid."""

    def __init__(self, blocked, weight=1.2):
        self.blocked = blocked
        self.weight = weight

    def plan(self, start, goal, densify_step=4.0):
        """start/goal: (x, y) grid coords. Returns list of float (x, y)."""
        s = nearest_free(self.blocked, start[0], start[1])
        g = nearest_free(self.blocked, goal[0], goal[1])
        if s is None or g is None:
            return None
        raw = a_star(self.blocked, s, g, self.weight)
        if raw is None:
            return None
        short = smooth_path(self.blocked, raw)
        return densify(short, densify_step)


def downsample_max(grid, factor):
    """Max-pool a 2D array by integer factor (occupied wins).
    Pads bottom/right edges as blocked."""
    if factor <= 1:
        return grid.astype(bool)
    h, w = grid.shape
    ph = int(math.ceil(h / float(factor))) * factor
    pw = int(math.ceil(w / float(factor))) * factor
    padded = np.ones((ph, pw), dtype=bool)
    padded[:h, :w] = grid.astype(bool)
    return padded.reshape(ph // factor, factor,
                          pw // factor, factor).max(axis=(1, 3))


def inflate(occupied, radius_cells):
    """Binary disk inflation, numpy-only (no cv2 needed)."""
    if radius_cells <= 0:
        return occupied.astype(bool)
    r = int(radius_cells)
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    disk = (xx * xx + yy * yy) <= r * r
    occ = occupied.astype(bool)
    out = np.zeros_like(occ)
    ys, xs = np.nonzero(disk)
    h, w = occ.shape
    for dy, dx in zip(ys - r, xs - r):
        y0s, y1s = max(0, dy), min(h, h + dy)
        y0d, y1d = max(0, -dy), min(h, h - dy)
        x0s, x1s = max(0, dx), min(w, w + dx)
        x0d, x1d = max(0, -dx), min(w, w - dx)
        out[y0d:y1d, x0d:x1d] |= occ[y0s:y1s, x0s:x1s]
    return out
