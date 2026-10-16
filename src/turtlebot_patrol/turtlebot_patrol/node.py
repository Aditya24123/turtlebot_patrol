"""ROS interface and persistent, fail-latched stop service."""
import json, math, os, tempfile, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from .controller import Controller,Settings

class PatrolNode(Node):
    def __init__(self,**kwargs):
        super().__init__('patrol',**kwargs)
        cfg=Settings(**{k:self.declare_parameter(k,v).value for k,v in vars(Settings()).items()})
        flat=self.declare_parameter('waypoints',[6.3,1.,6.3,4.8,1.,4.8,1.,1.]).value
        if len(flat)<2 or len(flat)%2:raise ValueError('waypoints must contain x,y pairs')
        self.ctrl=Controller(cfg,list(zip(flat[::2],flat[1::2])))
        default=str(Path.home()/'.ros'/'turtlebot_patrol_estop.json')
        self.latch_path=Path(self.declare_parameter('latch_file',default).value)
        try:
            if self.latch_path.exists():
                data=json.loads(self.latch_path.read_text())
                self.ctrl.estop=data['latched'] is not False
        except (OSError,ValueError,KeyError):
            self.ctrl.estop=True
            self.get_logger().error('Unreadable stop latch: remaining stopped until explicit successful reset')
        self.cmd=self.create_publisher(Twist,'cmd_vel',10)
        self.status=self.create_publisher(String,'patrol/status',10)
        self.create_subscription(LaserScan,'scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'odom',self.on_odom,qos_profile_sensor_data)
        self.create_service(SetBool,'emergency_stop',self.stop_service)
        self.create_timer(0.05,self.tick)

    def on_scan(self,m):
        self.ctrl.scan(m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max,time.monotonic())

    def on_odom(self,m):
        p=m.pose.pose; q=p.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.ctrl.odom(p.position.x,p.position.y,yaw,time.monotonic())

    def persist(self,latched):
        self.latch_path.parent.mkdir(parents=True,exist_ok=True)
        fd,tmp=tempfile.mkstemp(prefix='stop-',suffix='.tmp',dir=str(self.latch_path.parent))
        try:
            with os.fdopen(fd,'w') as f:
                json.dump({'latched':latched},f);f.flush();os.fsync(f.fileno())
            os.replace(tmp,self.latch_path)
        finally:
            if os.path.exists(tmp):os.unlink(tmp)

    def stop_service(self,req,res):
        if req.data:
            self.ctrl.estop=True;self.cmd.publish(Twist())
            try:self.persist(True);res.success=True;res.message='Emergency stop latched; zero velocity commanded'
            except OSError as e:res.success=False;res.message=f'STOPPED, but persistence failed: {e}'
        else:
            ok,message=self.ctrl.reset(time.monotonic())
            if ok:
                try:self.persist(False)
                except OSError as e:
                    self.ctrl.estop=True;ok=False;message=f'Reset refused: latch write failed: {e}'
            res.success=ok;res.message=message
        self.tick()
        self.get_logger().info(res.message)
        return res

    def tick(self):
        v,w=self.ctrl.step(time.monotonic());m=Twist();m.linear.x=v;m.angular.z=w;self.cmd.publish(m)
        s=String();s.data=json.dumps({'state':self.ctrl.state,'latched':self.ctrl.estop,'linear':v,'angular':w,'target':self.ctrl.target,'laps':self.ctrl.laps,'front':round(self.ctrl.front,3),'left':round(self.ctrl.left,3),'right':round(self.ctrl.right,3)})
        self.status.publish(s)

def main(args=None):
    rclpy.init(args=args);node=PatrolNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.cmd.publish(Twist());node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
