"""Ten real-DDS scenarios: no mock ROS interfaces or direct controller mutation."""
import json,math,time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from turtlebot_patrol.node import PatrolNode

@pytest.fixture(scope='module',autouse=True)
def context():
    rclpy.init();yield;rclpy.shutdown()

class Rig:
    def __init__(self,path):
        self.path=str(path);self.executor=SingleThreadedExecutor()
        self.node=PatrolNode(namespace='/qa',parameter_overrides=[Parameter('latch_file',value=self.path)])
        self.peer=Node('test_peer',namespace='/qa');self.executor.add_node(self.node);self.executor.add_node(self.peer)
        self.scan_pub=self.peer.create_publisher(LaserScan,'scan',qos_profile_sensor_data)
        self.odom_pub=self.peer.create_publisher(Odometry,'odom',qos_profile_sensor_data)
        self.status={};self.commands=[]
        self.peer.create_subscription(String,'patrol/status',lambda m:setattr(self,'status',json.loads(m.data)),10)
        self.peer.create_subscription(Twist,'cmd_vel',lambda m:self.commands.append((m.linear.x,m.angular.z)),10)
        self.client=self.peer.create_client(SetBool,'emergency_stop')
        self.pump(.4)

    def pump(self,seconds=.35,front=2.,left=2.,right=2.,scan=True,odom=True):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if scan:
                m=LaserScan();m.angle_min=0.;m.angle_increment=math.radians(1);m.range_min=.02;m.range_max=3.5
                ranges=[]
                for i in range(360):
                    a=math.degrees(math.atan2(math.sin(math.radians(i)),math.cos(math.radians(i))))
                    ranges.append(float(front if abs(a)<=35 else left if 35<a<=100 else right if -100<=a<-35 else 2.))
                m.ranges=ranges;self.scan_pub.publish(m)
            if odom:
                m=Odometry();m.pose.pose.position.x=1.;m.pose.pose.position.y=1.;m.pose.pose.orientation.w=1.;self.odom_pub.publish(m)
            self.executor.spin_once(timeout_sec=.01)

    def call(self,value,publish=True):
        assert self.client.wait_for_service(timeout_sec=2)
        req=SetBool.Request();req.data=value;future=self.client.call_async(req)
        end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:self.pump(.03,scan=publish,odom=publish)
        assert future.done(),'Service response timed out'
        return future.result()

    def close(self):
        self.executor.remove_node(self.node);self.executor.remove_node(self.peer)
        self.node.destroy_node();self.peer.destroy_node();self.executor.shutdown()

@pytest.fixture
def rig(tmp_path):
    r=Rig(tmp_path/'latch.json');yield r;r.close()

def test_ros_01_patrol_topics(rig):
    rig.pump();assert rig.commands[-1][0]>0;assert rig.status['state']=='PATROL'

def test_ros_02_obstacle_and_right_escape(rig):
    rig.pump(front=.4,left=.7,right=2.);assert rig.commands[-1]==(0.,-.8);assert rig.status['state']=='AVOID'

def test_ros_03_hard_blockage(rig):
    rig.pump(front=.15);assert rig.commands[-1]==(0.,0.);assert rig.status['state']=='BLOCKED'

def test_ros_04_scan_dropout(rig):
    rig.pump();rig.pump(.9,scan=False);assert rig.commands[-1]==(0.,0.);assert rig.status['state']=='SENSOR_FAULT'

def test_ros_05_odometry_dropout(rig):
    rig.pump();rig.pump(.9,odom=False);assert rig.commands[-1]==(0.,0.)

def test_ros_06_bad_scan(rig):
    rig.pump(front=math.nan,left=math.nan,right=math.nan);assert rig.commands[-1]==(0.,0.)
    rig.pump();assert rig.commands[-1][0]>0

def test_ros_07_stop_service_latches(rig):
    rig.pump();response=rig.call(True);assert response.success
    rig.commands.clear();rig.pump(.8);assert rig.commands and all(v==0 and w==0 for v,w in rig.commands)
    assert rig.status['latched'];assert json.loads(open(rig.path).read())['latched']

def test_ros_08_guarded_reset(rig):
    assert rig.call(True).success
    rig.pump(.9,scan=False,odom=False)
    assert not rig.call(False,publish=False).success
    rig.pump();assert rig.call(False).success;rig.pump();assert rig.commands[-1][0]>0

def test_ros_09_stop_survives_restart(rig):
    assert rig.call(True).success
    rig.executor.remove_node(rig.node);rig.node.destroy_node()
    rig.node=PatrolNode(namespace='/qa',parameter_overrides=[Parameter('latch_file',value=rig.path)])
    rig.executor.add_node(rig.node);rig.pump(.6)
    assert rig.status['latched'] and rig.commands[-1]==(0.,0.)
    assert rig.call(False).success

def test_ros_10_repeated_stop_reset(rig):
    for _ in range(5):
        assert rig.call(True).success;rig.pump(.1);assert rig.commands[-1]==(0.,0.)
        assert rig.call(False).success;rig.pump(.1);assert rig.commands[-1][0]>0
