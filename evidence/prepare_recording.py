import rclpy,time
from std_srvs.srv import SetBool
rclpy.init();n=rclpy.create_node('demo_prepare_stop');c=n.create_client(SetBool,'/emergency_stop')
assert c.wait_for_service(timeout_sec=8),'Stop service unavailable'
r=SetBool.Request();r.data=True;f=c.call_async(r);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
assert f.done() and f.result().success
print(f.result().message)
n.destroy_node();rclpy.shutdown()
