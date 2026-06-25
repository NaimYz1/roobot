#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Create a CLEANED copy of the race world:

  1. removes any robot model saved inside the world (dingo or jackal -
     it makes urdf_spawner fail with "model name ... already exists",
     breaks the spawn position, and desyncs AMCL localization)
  2. resets the saved sim_time (1517s) back to 0
  3. turns shadows off (performance)

install.sh runs this automatically; run.sh uses the cleaned world if present.
Manual usage:  python make_fast_world.py [src.world] [dst.world]
"""
from __future__ import print_function
import os
import re
import sys
import xml.etree.ElementTree as ET

SRC = '/opt/ros/melodic/share/jackal_gazebo/worlds/jackal_race.world'
DST = os.path.expanduser('~/race_fast.world')


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else SRC
    dst = sys.argv[2] if len(sys.argv) > 2 else DST
    if not os.path.exists(src):
        print('ERROR: world not found:', src)
        return 1

    tree = ET.parse(src)
    root = tree.getroot()
    world = root.find('world')
    if world is None:
        print('ERROR: no <world> in', src)
        return 1

    removed = 0
    # remove robot model definition(s) saved into the world
    for parent in [world] + world.findall('state'):
        for m in list(parent.findall('model')):
            if m.get('name', '').lower().startswith(('dingo', 'jackal')):
                parent.remove(m)
                removed += 1
    # reset saved simulation clock
    for st in world.findall('state'):
        t = st.find('sim_time')
        if t is not None:
            t.text = '0 0'
        for tag in ('wall_time', 'real_time'):
            e = st.find(tag)
            if e is not None:
                e.text = '0 0'
        it = st.find('iterations')
        if it is not None:
            it.text = '0'
    # shadows off
    scene = world.find('scene')
    if scene is None:
        scene = ET.SubElement(world, 'scene')
    sh = scene.find('shadows')
    if sh is None:
        sh = ET.SubElement(scene, 'shadows')
    sh.text = 'false'

    tree.write(dst)
    print('Wrote %s (removed %d saved robot model(s), sim_time reset, '
          'shadows off)' % (dst, removed))
    return 0


if __name__ == '__main__':
    sys.exit(main())
