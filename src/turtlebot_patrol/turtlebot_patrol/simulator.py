"""Actual ROS node: 2D lidar/odom and command watchdog for a TurtleBot test world."""
import json, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from .world import advance,scan

class Simulator(Node):
    def __init__(self):
        super().__init__('turtlebot_simulator')
        self.x,self.y,self.yaw=1.,1.,0.;self.v=self.w=0.
        self.command_at=-math.inf;self.previous=time.monotonic();self.ticks=0;self.collisions=0;self.distance=0.
        self.scan_pub=self.create_publisher(LaserScan,'scan',qos_profile_sensor_data)
        self.odom_pub=self.create_publisher(Odometry,'odom',qos_profile_sensor_data)
        self.telemetry=self.create_publisher(String,'sim/telemetry',10)
        self.create_subscription(Twist,'cmd_vel',self.command,10)
        self.create_timer(.02,self.tick)

    def command(self,m):
        if not all(math.isfinite(v) for v in (m.linear.x,m.angular.z)):return
        self.v=max(-.22,min(.22,m.linear.x));self.w=max(-2.84,min(2.84,m.angular.z));self.command_at=time.monotonic()

    def tick(self):
        now=time.monotonic();dt=min(.1,now-self.previous);self.previous=now
        if now-self.command_at>.4:self.v=self.w=0.
        old=(self.x,self.y)
        self.x,self.y,self.yaw,hit=advance(self.x,self.y,self.yaw,self.v,self.w,dt)
        self.collisions+=int(hit);self.distance+=math.hypot(self.x-old[0],self.y-old[1]);self.ticks+=1
        if self.ticks%2==0:
            m=Odometry();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='odom';m.child_frame_id='base_link'
            m.pose.pose.position.x=self.x;m.pose.pose.position.y=self.y
            m.pose.pose.orientation.z=math.sin(self.yaw/2);m.pose.pose.orientation.w=math.cos(self.yaw/2)
            m.twist.twist.linear.x=self.v;m.twist.twist.angular.z=self.w;self.odom_pub.publish(m)
        if self.ticks%5==0:
            m=LaserScan();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='base_scan'
            m.angle_min=0.;m.angle_max=math.radians(359);m.angle_increment=math.radians(1)
            m.range_min=.02;m.range_max=3.5;m.scan_time=.1;m.ranges=scan(self.x,self.y,self.yaw);self.scan_pub.publish(m)
            s=String();s.data=json.dumps({'x':self.x,'y':self.y,'yaw':self.yaw,'distance':round(self.distance,3),'collision_guard_hits':self.collisions})
            self.telemetry.publish(s)

def main(args=None):
    rclpy.init(args=args);n=Simulator()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
