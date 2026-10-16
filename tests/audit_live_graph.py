import rclpy,time,json
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('graph_audit');data={}
n.create_subscription(String,'patrol/status',lambda m:data.update(status=json.loads(m.data)),10)
n.create_subscription(String,'sim/telemetry',lambda m:data.update(pose=json.loads(m.data)),10)
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
data['nodes']=n.get_node_names();data['services']=[s for s,t in n.get_service_names_and_types() if 'emergency' in s]
print(json.dumps(data,indent=2));open('evidence/live_graph.json','w').write(json.dumps(data,indent=2))
n.destroy_node();rclpy.shutdown()
