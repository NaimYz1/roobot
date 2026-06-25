#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Generate a Gazebo .world from a ROS map (yaml + pgm).

Occupied cells are merged into rectangles and extruded into 1 m high
static walls, so the Gazebo world physically matches the map AMCL
localizes against. Unknown (gray) cells are ignored.

Usage:
    python make_world_from_map.py <map.yaml> <out.world> [wall_height]

Works with python2 (Melodic) and python3. No dependencies.
"""
from __future__ import print_function
import os
import sys


def read_yaml(path):
    """Tiny parser for the few flat keys map_server yaml files use."""
    d = {}
    with open(path) as f:
        for line in f:
            line = line.split('#')[0].strip()
            if ':' not in line:
                continue
            k, v = line.split(':', 1)
            d[k.strip()] = v.strip()
    res = float(d['resolution'])
    origin = [float(t) for t in
              d['origin'].strip('[]').split(',')[:2]]
    img = d['image']
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(os.path.abspath(path)), img)
    occ = float(d.get('occupied_thresh', 0.65))
    neg = int(d.get('negate', 0))
    return img, res, origin, occ, neg


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    # tokenize header (magic, width, height, maxval), skipping comments
    toks, i = [], 0
    while len(toks) < 4:
        while i < len(data) and data[i:i+1].isspace():
            i += 1
        if data[i:i+1] == b'#':
            while i < len(data) and data[i:i+1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j+1].isspace():
            j += 1
        toks.append(data[i:j])
        i = j
    if toks[0] != b'P5':
        raise ValueError('only binary PGM (P5) supported, got %r' % toks[0])
    w, h, maxval = int(toks[1]), int(toks[2]), int(toks[3])
    i += 1  # single whitespace after maxval
    px = data[i:i + w * h]
    if len(px) < w * h:
        raise ValueError('truncated PGM')
    rows = [bytearray(px[r * w:(r + 1) * w]) for r in range(h)]
    return rows, w, h, maxval


def occupied_grid(rows, w, h, maxval, occ_thresh, negate):
    """True where the map cell is a wall/obstacle (occupancy > thresh)."""
    grid = [[False] * w for _ in range(h)]
    for r in range(h):
        row = rows[r]
        g = grid[r]
        for c in range(w):
            v = row[c]
            p = v / float(maxval) if negate else (maxval - v) / float(maxval)
            g[c] = p > occ_thresh
    return grid


def merge_rects(grid, w, h):
    """Greedy merge of occupied cells into rectangles (row, col, nrows, ncols)."""
    used = [[False] * w for _ in range(h)]
    rects = []
    for r in range(h):
        c = 0
        while c < w:
            if grid[r][c] and not used[r][c]:
                # grow right
                c2 = c
                while c2 + 1 < w and grid[r][c2 + 1] and not used[r][c2 + 1]:
                    c2 += 1
                # grow down while the whole span stays occupied & unused
                r2 = r
                while r2 + 1 < h and all(
                        grid[r2 + 1][k] and not used[r2 + 1][k]
                        for k in range(c, c2 + 1)):
                    r2 += 1
                for rr in range(r, r2 + 1):
                    for cc in range(c, c2 + 1):
                        used[rr][cc] = True
                rects.append((r, c, r2 - r + 1, c2 - c + 1))
                c = c2 + 1
            else:
                c += 1
    return rects


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    yaml_path, out_path = sys.argv[1], sys.argv[2]
    wall_h = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0

    img, res, (ox, oy), occ, neg = read_yaml(yaml_path)
    rows, w, h, maxval = read_pgm(img)
    grid = occupied_grid(rows, w, h, maxval, occ, neg)
    rects = merge_rects(grid, w, h)
    print('map %dx%d cells @ %.3f m -> %d wall boxes' % (w, h, res, len(rects)))

    blocks = []
    for n, (r, c, nr, nc) in enumerate(rects):
        sx = nc * res
        sy = nr * res
        cx = ox + (c + nc / 2.0) * res
        # pgm row 0 is the TOP of the map (max y)
        cy = oy + (h - r - nr / 2.0) * res
        blocks.append(
            '      <collision name="c%d"><pose>%.3f %.3f %.3f 0 0 0</pose>'
            '<geometry><box><size>%.3f %.3f %.3f</size></box></geometry>'
            '</collision>\n'
            '      <visual name="v%d"><pose>%.3f %.3f %.3f 0 0 0</pose>'
            '<geometry><box><size>%.3f %.3f %.3f</size></box></geometry>'
            '<material><script><uri>file://media/materials/scripts/gazebo.material</uri>'
            '<name>Gazebo/Grey</name></script></material></visual>'
            % (n, cx, cy, wall_h / 2.0, sx, sy, wall_h,
               n, cx, cy, wall_h / 2.0, sx, sy, wall_h))

    world = (
        '<?xml version="1.0"?>\n'
        '<sdf version="1.6">\n'
        '  <world name="default">\n'
        '    <include><uri>model://ground_plane</uri></include>\n'
        '    <include><uri>model://sun</uri></include>\n'
        '    <scene><shadows>false</shadows></scene>\n'
        '    <model name="map_walls">\n'
        '      <static>true</static>\n'
        '      <link name="walls">\n'
        + '\n'.join(blocks) + '\n'
        '      </link>\n'
        '    </model>\n'
        '  </world>\n'
        '</sdf>\n')
    with open(out_path, 'w') as f:
        f.write(world)
    print('wrote', out_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
