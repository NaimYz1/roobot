#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Headless closed-loop regression test: REAL planner_logic + vfh_logic +
pp_logic on the real map, simulated lidar, no ROS/Gazebo needed.
Run after any tuning change:  python3 headless_test.py
Every scenario must print GOAL (no COLLIDED, no TIMEOUT)."""
# -*- coding: utf-8 -*-
"""Headless sim v2: REAL planner (A*) + REAL VFH+ + REAL controller."""
import math, sys
import os
B=os.path.dirname(os.path.abspath(__file__))
for sub in ['amr_perception/src','amr_control/src','amr_planning/src','']:
    sys.path.insert(0, B+'/'+sub)
import numpy as np
from vfh_logic import VFHPlus, ang_diff
from pp_logic import PurePursuitController
from planner_logic import GridPlanner, inflate, downsample_max
from make_world_from_map import read_yaml, read_pgm

img,res,(ox,oy),occ,neg = read_yaml(B+'/amr_bringup/maps/dingoMap1.yaml')
rows,w,h,maxval = read_pgm(img)
# bottom-up grid like map_server: row 0 = origin
wall = np.zeros((h,w),bool); unk = np.zeros((h,w),bool)
for r in range(h):
    for c in range(w):
        v=rows[r][c]
        if (maxval-v)/maxval>occ: wall[h-1-r][c]=True
        elif v<=250: unk[h-1-r][c]=True
occupied_map = wall | unk                       # planner: unknown blocked
blocked = downsample_max(inflate(occupied_map, int(round(0.45/res))), 1)
planner = GridPlanner(blocked, weight=1.2)

def occ_at(x,y):                                # ground truth: walls only
    c=int((x-ox)/res); r=int((y-oy)/res)
    if not(0<=c<w and 0<=r<h): return True
    return bool(wall[r][c])

def raycast(x,y,yaw,fov=math.radians(270),nrays=180,rmax=4.5):
    angs=np.linspace(-fov/2,fov/2,nrays); rngs=np.empty(nrays)
    for i,a in enumerate(angs):
        th=yaw+a; ca,sa=math.cos(th),math.sin(th); r=0.05
        while r<rmax and not occ_at(x+r*ca,y+r*sa): r+=res/2
        rngs[i]=r
    return angs,rngs

def plan_path(x,y,gx,gy):
    p=planner.plan((int((x-ox)/res),int((y-oy)/res)),
                   (int((gx-ox)/res),int((gy-oy)/res)))
    if p is None: return None
    return [((px+0.5)*res+ox,(py+0.5)*res+oy) for px,py in p]

def target_angle(pose,path,lookahead=1.2):
    x,y,yaw=pose
    d2=[(px-x)**2+(py-y)**2 for px,py in path]
    i=int(np.argmin(d2)); j=i
    for j in range(i,len(path)):
        if math.hypot(path[j][0]-x,path[j][1]-y)>=lookahead: break
    return ang_diff(math.atan2(path[j][1]-y,path[j][0]-x),yaw)

def run(start,goal,label,seconds=60):
    vfh=VFHPlus(num_bins=72,max_range=4.0,active_range=1.4,
                robot_radius=0.33,safety_margin=0.10)
    c=PurePursuitController(v_max=0.9,a_lat=0.9,lookahead_gain=0.9,
                            goal_radius=0.15,d_danger=0.55,d_blend=1.5)
    x,y,yaw=start; dt=0.05
    path=plan_path(x,y,goal[0],goal[1])
    if not path: print('[%s] NO INITIAL PATH'%label); return
    vfh_dir=0.0; mind=10.0; fd=10.0; stop=False
    spin=0.0; dist=0.0; br={}; collided=False; minclear=9e9
    for k in range(int(seconds/dt)):
        if k%20==0 and k>0:                       # 1 Hz replan
            np2=plan_path(x,y,goal[0],goal[1])
            if np2: path=np2; c.last_index=0
        if k%2==0:                                # 10 Hz sensing
            angs,rngs=raycast(x,y,yaw)
            resd=vfh.steer(angs,rngs,target_angle((x,y,yaw),path))
            vfh_dir=0.0 if resd['direction'] is None else resd['direction']
            stop=resd.get('boxed',False) or resd['direction'] is None
            mind=resd['min_dist']; fd=resd['front_dist']
            minclear=min(minclear,float(np.min(rngs)))
        v,wc,done=c.step((x,y,yaw),path,vfh_dir,mind,stop,dt,front_dist=fd)
        br[c.last_branch]=br.get(c.last_branch,0)+1
        x+=v*math.cos(yaw)*dt; y+=v*math.sin(yaw)*dt
        yaw=ang_diff(yaw+wc*dt,0)
        spin+=abs(wc)*dt; dist+=v*dt
        if occ_at(x,y): collided=True; break
        if done:
            print("[%s] GOAL %.1fs | %.1fm | turn %d deg | clear %.2f | %s"%(
                label,k*dt,dist,math.degrees(spin),minclear,br)); return
    print("[%s] %s at (%.2f,%.2f) | %.1fm | turn %d deg | clear %.2f | %s"%(
        label,'COLLIDED' if collided else 'TIMEOUT',x,y,dist,
        math.degrees(spin),minclear,br))


import math, numpy as np
def run3(start,goal,label,seconds=120,nb=144):
    vfh=VFHPlus(num_bins=nb,max_range=4.0,active_range=1.1,robot_radius=0.33,safety_margin=0.10)
    c=PurePursuitController(v_max=0.9,a_lat=0.9,lookahead_gain=0.9,goal_radius=0.15,d_danger=0.55,d_blend=1.5)
    x,y,yaw=start; dt=0.05
    path=plan_path(x,y,goal[0],goal[1])
    if not path: print('[%s] NO PATH'%label); return False
    vfh_dir=0.0; mind=10.0; fd=10.0; stop=False; spin=0.0; dist=0.0; br={}; mincl=9e9
    for k in range(int(seconds/dt)):
        if k%60==0 and k>0:
            np2=plan_path(x,y,goal[0],goal[1])
            if np2: path=np2; c.last_index=0
        if k%2==0:
            angs,rngs=raycast(x,y,yaw)
            resd=vfh.steer(angs,rngs,target_angle((x,y,yaw),path))
            vfh_dir=0.0 if resd['direction'] is None else resd['direction']
            stop=resd.get('boxed',False) or resd['direction'] is None
            mind=resd['min_dist']; fd=resd['front_dist']; mincl=min(mincl,float(np.min(rngs)))
        v,wc,done=c.step((x,y,yaw),path,vfh_dir,mind,stop,dt,front_dist=fd)
        br[c.last_branch]=br.get(c.last_branch,0)+1
        x+=v*math.cos(yaw)*dt; y+=v*math.sin(yaw)*dt
        yaw=ang_diff(yaw+wc*dt,0)
        spin+=abs(wc)*dt; dist+=v*dt
        if occ_at(x,y): print('[%s] COLLIDED (%.2f,%.2f)'%(label,x,y)); return False
        if done:
            print('[%s] GOAL %.1fs | %.1fm | turn %d deg | min clearance %.2f m | %s'%(
                label,k*dt,dist,math.degrees(spin),mincl,br)); return True
    print('[%s] TIMEOUT (%.2f,%.2f) %.1fm | %s'%(label,x,y,dist,br)); return False

if __name__=='__main__':
    ok=all([
        run3((-2.12,-1.67,0.0),(1.5,-1.67),'open-straight'),
        run3((-2.12,-1.67,0.0),(-2.0,3.0),'cross-map'),
        run3((-6.47,-8.28,0.0),(1.33,-8.88),'corridor-8m'),
        run3((1.33,-8.88,3.14),(-8.28,4.33),'A-16m'),
        run3((-5.27,-0.47,1.57),(1.33,-8.88),'B-11m'),
    ])
    print('ALL PASS' if ok else 'FAILURES - do not deploy')
