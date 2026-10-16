"""Accelerated simulation, using the same laser geometry as the live ROS simulator."""
import json,math
from pathlib import Path
from turtlebot_patrol.controller import Controller
from turtlebot_patrol.world import scan,advance,clearance

c=Controller();x,y,yaw=1.,1.,0.;hits=0;min_clear=99;states={};dt=.1
for i in range(12000):
    now=i*dt;c.odom(x,y,yaw,now);c.scan(scan(x,y,yaw),0.,math.radians(1),.02,3.5,now)
    v,w=c.step(now);x,y,yaw,hit=advance(x,y,yaw,v,w,dt);hits+=int(hit)
    min_clear=min(min_clear,clearance(x,y));states[c.state]=states.get(c.state,0)+1
    if c.laps>=2:break
result={'seconds_simulated':round(now,1),'laps':c.laps,'collision_guard_hits':hits,'minimum_footprint_clearance_m':min_clear,'states':states,'final_pose':[x,y,yaw]}
Path('evidence').mkdir(exist_ok=True);Path('evidence/closed_loop.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2));assert hits==0;assert c.laps>=2,'Patrol failed to complete two loops'
