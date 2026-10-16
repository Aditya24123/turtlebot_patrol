"""Collect a real DDS velocity trace while calling the portal's named Trigger services."""
import json,time,tempfile
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from test_ros_integration import Rig

rclpy.init()
with tempfile.TemporaryDirectory() as tmp:
    rig=Rig(Path(tmp)/'stop.json');start=time.monotonic();trace=[];events=[]
    rig.peer.create_subscription(Twist,'cmd_vel',lambda m:trace.append({'time':time.monotonic()-start,'linear':m.linear.x,'angular':m.angular.z}),10)
    def call(name):
        client=rig.peer.create_client(Trigger,name);assert client.wait_for_service(timeout_sec=2)
        before=time.monotonic()-start;future=client.call_async(Trigger.Request());end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:rig.pump(.02)
        assert future.done() and future.result().success
        events.append({'service':name,'requested_at':before,'response_at':time.monotonic()-start,'success':future.result().success,'message':future.result().message})
    rig.pump(1.);call('trigger_estop');rig.pump(1.5);call('reset_estop');rig.pump(1.)
    result={'trace':trace,'events':events,'transport':'Real ROS 2 DDS, synthetic clear LaserScan/Odometry input'}
    Path('evidence/stop_trace.json').write_text(json.dumps(result,indent=2))
    low=events[0]['response_at']+.1;high=events[1]['requested_at']
    samples=[s for s in trace if low<s['time']<high]
    assert samples and all(s['linear']==0 and s['angular']==0 for s in samples)
    assert trace[-1]['linear']>0
    print(json.dumps(events,indent=2));print('PASS: stopped interval zero, reset interval moving')
    rig.close()
rclpy.shutdown()
