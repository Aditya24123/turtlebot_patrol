"""Deterministic controller. Distances are measured from the laser origin."""
from dataclasses import dataclass
import math

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

@dataclass
class Settings:
    linear_speed: float = 0.20
    angular_speed: float = 0.8
    stop_distance: float = 0.55
    clear_distance: float = 0.80
    hard_distance: float = 0.24
    sensor_timeout: float = 0.6
    waypoint_tolerance: float = 0.25

class Controller:
    def __init__(self, settings=None, waypoints=None):
        self.cfg = settings or Settings()
        c=self.cfg
        if not (0<c.linear_speed<=0.22 and 0<c.angular_speed<=2.84 and
                0<c.hard_distance<c.stop_distance<c.clear_distance and
                c.sensor_timeout>0 and c.waypoint_tolerance>0):
            raise ValueError('Invalid TurtleBot speed, distance or timeout parameters')
        self.waypoints=waypoints or [(6.3,1.0),(6.3,4.8),(1.0,4.8),(1.0,1.0)]
        if not self.waypoints or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in self.waypoints):
            raise ValueError('Waypoints must be finite x,y pairs')
        self.pose=None; self.pose_at=-math.inf; self.scan_at=-math.inf
        self.front=self.left=self.right=0.0; self.scan_ok=False
        self.estop=False; self.avoiding=False; self.turn=1; self.target=0
        self.detour_start=None;self.detour_heading=0.
        self.state='WAIT_SENSORS'; self.laps=0

    def odom(self,x,y,yaw,now):
        if all(math.isfinite(v) for v in (x,y,yaw)):
            self.pose=(x,y,yaw); self.pose_at=now
        else:
            self.pose=None

    def scan(self,ranges,angle_min,angle_increment,range_min,range_max,now):
        sectors={'front':[],'left':[],'right':[]}; expected={k:0 for k in sectors}
        if not ranges or not all(math.isfinite(v) for v in (angle_min,angle_increment,range_min,range_max)) or angle_increment==0 or not 0<=range_min<range_max:
            self.scan_ok=False; return
        for i,r in enumerate(ranges):
            a=math.degrees(wrap(angle_min+i*angle_increment))
            key='front' if abs(a)<=35 else 'left' if 35<a<=100 else 'right' if -100<=a<-35 else None
            if key is None: continue
            expected[key]+=1
            if r==math.inf: sectors[key].append(range_max)
            elif math.isfinite(r):
                # A finite below-minimum return is unsafe, not free space.
                sectors[key].append(r if range_min<=r<=range_max else 0.0)
        self.scan_ok=all(expected[k]>0 and len(sectors[k])/expected[k]>=0.8 for k in sectors)
        if self.scan_ok:
            self.front,self.left,self.right=(min(sectors[k]) for k in ('front','left','right'))
            self.scan_at=now

    def healthy(self,now):
        return self.pose is not None and self.scan_ok and 0<=now-self.scan_at<=self.cfg.sensor_timeout and 0<=now-self.pose_at<=self.cfg.sensor_timeout

    def reset(self,now):
        if not self.healthy(now): return False,'Reset refused: scan/odometry missing, invalid or stale'
        if min(self.front,self.left,self.right)<=self.cfg.hard_distance:
            return False,'Reset refused: obstacle inside safety clearance'
        self.estop=False; self.avoiding=False;self.detour_start=None
        return True,'Emergency stop reset; autonomous patrol enabled'

    def step(self,now):
        c=self.cfg
        if self.estop: self.state='ESTOP'; return 0.,0.
        if not self.healthy(now): self.state='SENSOR_FAULT'; return 0.,0.
        if min(self.front,self.left,self.right)<=c.hard_distance:
            self.state='BLOCKED'; return 0.,0.
        if self.front<c.stop_distance and not self.avoiding:
            self.avoiding=True; self.turn=1 if self.left>=self.right else -1;self.detour_start=None
        if self.avoiding:
            if self.front<c.clear_distance:
                self.state='AVOID'; return 0.,self.turn*c.angular_speed
            self.avoiding=False
            self.detour_start=self.pose[:2];self.detour_heading=self.pose[2]
        x,y,yaw=self.pose; tx,ty=self.waypoints[self.target]
        if self.detour_start is not None:
            if math.hypot(x-self.detour_start[0],y-self.detour_start[1])<0.85:
                self.state='BYPASS'
                return min(c.linear_speed,.15),max(-c.angular_speed,min(c.angular_speed,1.5*wrap(self.detour_heading-yaw)))
            self.detour_start=None
        if math.hypot(tx-x,ty-y)<c.waypoint_tolerance:
            self.target=(self.target+1)%len(self.waypoints)
            if self.target==0:self.laps+=1
            tx,ty=self.waypoints[self.target]
        error=wrap(math.atan2(ty-y,tx-x)-yaw)
        angular=max(-c.angular_speed,min(c.angular_speed,1.5*error))
        # A turn-in-place threshold prevents a sharp corner cutting into an obstacle.
        linear=0. if abs(error)>0.65 else c.linear_speed*max(0.,math.cos(error))
        # Avoid immediately steering back into the object after a reactive turn.
        if self.front<1.1 and abs(error)>0.25:
            linear=min(linear,0.10)
        self.state='PATROL'
        return linear,angular
