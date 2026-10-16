import math
import pytest
from turtlebot_patrol.controller import Controller,Settings

def feed(c,now=1.,front=2.,left=2.,right=2.):
    ranges=[2.]*360
    for i in range(360):
        a=math.degrees(math.atan2(math.sin(math.radians(i)),math.cos(math.radians(i))))
        if abs(a)<=35:ranges[i]=front
        elif 35<a<=100:ranges[i]=left
        elif -100<=a<-35:ranges[i]=right
    c.scan(ranges,0.,math.radians(1),.02,3.5,now);c.odom(1.,1.,0.,now)

def test_01_clear_patrol():
    c=Controller();feed(c);v,w=c.step(1.);assert v>0 and abs(w)<1e-5 and c.state=='PATROL'

def test_02_front_obstacle_left_turn():
    c=Controller();feed(c,front=.4,left=2.,right=.7);assert c.step(1.)==(0.,.8)

def test_03_right_escape():
    c=Controller();feed(c,front=.4,left=.7,right=2.);assert c.step(1.)==(0.,-.8)

def test_04_blocked_clearance():
    c=Controller();feed(c,front=.15);assert c.step(1.)==(0.,0.) and c.state=='BLOCKED'

def test_05_missing_and_stale_sensors():
    c=Controller();assert c.step(1.)==(0.,0.);feed(c);assert c.step(2.)==(0.,0.)

def test_06_invalid_laser_and_partial_fov():
    c=Controller();feed(c);c.scan([math.nan]*360,0.,math.radians(1),.02,3.5,1.);assert c.step(1.)==(0.,0.)
    c.scan([math.inf]*360,0.,math.radians(1),.02,3.5,1.);assert c.step(1.)[0]>0
    c.scan([2.]*5,0.,.01,.02,3.5,1.);assert c.step(1.)==(0.,0.)

def test_07_hysteresis_and_bypass():
    c=Controller();feed(c,front=.4);c.step(1.)
    feed(c,front=.65);assert c.step(1.)[0]==0 and c.state=='AVOID'
    feed(c,front=1.2);assert c.step(1.)[0]>0 and c.state=='BYPASS'

def test_08_stop_latch_and_guarded_reset():
    c=Controller();feed(c);c.estop=True;assert c.step(1.)==(0.,0.)
    assert not c.reset(2.)[0] and c.estop
    feed(c,front=.1);assert not c.reset(1.)[0] and c.estop
    feed(c);assert c.reset(1.)[0] and c.step(1.)[0]>0

def test_09_waypoint_cycle():
    c=Controller(waypoints=[(1.,1.),(2.,1.)]);feed(c);c.step(1.);assert c.target==1
    c.odom(2.,1.,0.,1.);c.step(1.);assert c.target==0 and c.laps==1

def test_10_parameter_validation():
    with pytest.raises(ValueError):Controller(Settings(stop_distance=.9,clear_distance=.8))
    with pytest.raises(ValueError):Controller(Settings(linear_speed=2.))
    with pytest.raises(ValueError):Controller(waypoints=[(math.nan,0.)])
    with pytest.raises(ValueError):Controller(waypoints=[])
