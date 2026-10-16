"""Local-only browser dashboard, reflecting actual ROS topics and service replies."""
import json,threading,time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .world import OBSTACLES

PAGE=r'''<!doctype html><html><head><meta charset="utf-8"><title>TurtleBot Patrol | Live ROS 2</title>
<style>*{box-sizing:border-box}body{margin:0;background:#0b1421;color:#edf4ff;font:15px system-ui}main{width:1160px;margin:auto;padding:20px}h1{font-size:28px;margin:0 0 4px}header p{color:#a7bbd2;margin:0 0 16px}.layout{display:grid;grid-template-columns:760px 1fr;gap:24px}canvas{background:#14243a;border:2px solid #52657f;border-radius:10px}aside{padding:8px}h2{font-size:14px;color:#a7bbd2;letter-spacing:2px}#state{font-size:34px;color:#50e1cd;font-weight:700}.metric{display:flex;justify-content:space-between;line-height:2.2;border-bottom:1px solid #21334a}.value{font-weight:700}button{font:700 14px system-ui;padding:14px 20px;border:0;border-radius:7px;cursor:pointer}#stop{background:#e86572;color:#fff}#reset{background:#50d4b7;color:#102323}footer{display:flex;align-items:center;gap:12px;margin-top:14px}small{color:#a7bbd2}#events{font:12px monospace;color:#c5daee;white-space:pre-wrap;line-height:1.5}.tag{display:inline-block;padding:4px 8px;background:#21334a;border-radius:6px;font-size:12px}#warn{color:#ffd16b;font-size:13px}</style></head>
<body><main><header><h1>TURTLEBOT / AUTONOMOUS PATROL</h1><p>ROS 2 Jazzy · 2D differential-drive simulation · live DDS topics + service</p></header>
<div class="layout"><canvas id="world" width="760" height="570"></canvas><aside><h2>LIVE CONTROLLER <span class="tag" id="online">Connecting</span></h2><div id="state">WAITING</div><div id="metrics"></div><h2>SERVICE RESPONSES</h2><div id="events">No calls yet.</div><p id="warn"></p></aside></div>
<footer><button id="stop" onclick="service(true)">TRIGGER EMERGENCY STOP</button><button id="reset" onclick="service(false)">RESET / RESUME</button><small>ROS Trigger: /trigger_estop · /reset_estop</small></footer>
<p><small>/scan → patrol → /cmd_vel &nbsp; /odom → waypoint tracking &nbsp; /trigger_estop → persistent latch</small></p></main>
<script>
let trail=[],lastPose=null; const c=document.getElementById('world'),ctx=c.getContext('2d'),start=performance.now();
const pos=(x,y)=>[20+x*90,555-y*90];
async function service(stop){const r=await fetch('/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({stop})});const d=await r.json();document.getElementById('warn').textContent=d.message||'';}
function circle(x,y,r,fill,stroke){const p=pos(x,y);ctx.beginPath();ctx.arc(...p,r*90,0,Math.PI*2);ctx.fillStyle=fill;ctx.fill();ctx.strokeStyle=stroke;ctx.lineWidth=2;ctx.stroke();}
function draw(d){let s=d.status,p=d.pose;ctx.clearRect(0,0,760,570);ctx.strokeStyle='#273a53';ctx.lineWidth=1;
for(let x=0;x<=8;x++){ctx.beginPath();ctx.moveTo(...pos(x,0));ctx.lineTo(...pos(x,6));ctx.stroke()}
for(let y=0;y<=6;y++){ctx.beginPath();ctx.moveTo(...pos(0,y));ctx.lineTo(...pos(8,y));ctx.stroke()}
for(let o of d.obstacles)circle(o[0],o[1],o[2],'#a87343','#efb46e');
let goals=[[6.3,1],[6.3,4.8],[1,4.8],[1,1]];goals.forEach((g,i)=>{let color=i===s.target?'#50e1cd':'#8295ae';circle(...g,.09,'#14243a',color);let q=pos(...g);ctx.fillStyle=color;ctx.font='bold 13px system-ui';ctx.fillText('P'+(i+1),q[0]+12,q[1]-12)});
if(p.x!==undefined){if(!lastPose||Math.hypot(p.x-lastPose.x,p.y-lastPose.y)>.02){trail.push([p.x,p.y]);lastPose=p;if(trail.length>4000)trail.shift()}
ctx.strokeStyle='#35b6a5';ctx.lineWidth=2;ctx.beginPath();trail.forEach((q,i)=>{let z=pos(...q);if(i)ctx.lineTo(...z);else ctx.moveTo(...z)});ctx.stroke();
circle(p.x,p.y,.12,'#50e1cd','#f3fffe');let q=pos(p.x,p.y);ctx.strokeStyle='#fff';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(...q);ctx.lineTo(q[0]+25*Math.cos(p.yaw),q[1]-25*Math.sin(p.yaw));ctx.stroke();
ctx.setLineDash([5,4]);ctx.strokeStyle='#ffcf68';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(...q);let r=Math.min(2,s.front||0)*90;ctx.lineTo(q[0]+r*Math.cos(p.yaw),q[1]-r*Math.sin(p.yaw));ctx.stroke();ctx.setLineDash([]);}
let state=s.state||'WAITING';document.getElementById('state').textContent=state;document.getElementById('state').style.color=state==='ESTOP'?'#fa7d87':['AVOID','BLOCKED','SENSOR_FAULT'].includes(state)?'#ffd26d':'#50e1cd';
let rows=[['Emergency latch',s.latched?'ON':'OFF'],['Forward',(s.linear||0).toFixed(2)+' m/s'],['Yaw',(s.angular||0).toFixed(2)+' rad/s'],['Front clearance',(s.front||0).toFixed(2)+' m'],['Waypoint','P'+((s.target||0)+1)+' / 4'],['Patrol laps',s.laps||0],['Distance',(p.distance||0).toFixed(2)+' m'],['Collision guard hits',p.collision_guard_hits||0],['Elapsed',((performance.now()-start)/1000).toFixed(1)+' s']];
document.getElementById('metrics').innerHTML=rows.map(r=>'<div class="metric"><span>'+r[0]+'</span><span class="value">'+r[1]+'</span></div>').join('');
document.getElementById('events').textContent=d.events.slice(-3).join('\n\n')||'No calls yet.';document.getElementById('online').textContent=d.status_age<1?'ROS LIVE':'STALE';}
setInterval(async()=>{try{draw(await(await fetch('/state',{cache:'no-store'})).json())}catch(e){document.getElementById('online').textContent='DISCONNECTED'}},100);
</script></body></html>'''

class WebDashboard(Node):
    def __init__(self):
        super().__init__('patrol_dashboard');self.status={};self.pose={};self.events=[];self.received=0
        self.create_subscription(String,'patrol/status',self.on_status,10)
        self.create_subscription(String,'sim/telemetry',lambda m:setattr(self,'pose',json.loads(m.data)),10)
        self.stop_client=self.create_client(Trigger,'trigger_estop')
        self.reset_client=self.create_client(Trigger,'reset_estop')
    def on_status(self,m):self.status=json.loads(m.data);self.received=time.monotonic()

def main(args=None):
    rclpy.init(args=args);n=WebDashboard();thread=threading.Thread(target=rclpy.spin,args=(n,),daemon=True);thread.start()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,code,data,typ='application/json'):
            payload=data.encode() if isinstance(data,str) else json.dumps(data).encode()
            self.send_response(code);self.send_header('Content-Type',typ);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(payload)));self.end_headers();self.wfile.write(payload)
        def do_GET(self):
            if self.path=='/':self.reply(200,PAGE,'text/html; charset=utf-8')
            elif self.path=='/state':self.reply(200,{'status':n.status,'pose':n.pose,'events':n.events[-10:],'obstacles':OBSTACLES,'status_age':time.monotonic()-n.received})
            else:self.reply(404,{'error':'Not found'})
        def do_POST(self):
            # Only same-origin browser calls may trigger the local ROS service.
            origin=self.headers.get('Origin','')
            if origin and origin not in ('http://127.0.0.1:8840','http://localhost:8840'):
                self.reply(403,{'message':'Origin refused'});return
            if self.path!='/stop':self.reply(404,{'message':'Not found'});return
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<256:raise ValueError('Invalid request size')
                data=json.loads(self.rfile.read(size));stop=data['stop']
                if not isinstance(stop,bool):raise ValueError('stop must be boolean')
                client=n.stop_client if stop else n.reset_client
                if not client.wait_for_service(timeout_sec=1):raise ValueError('ROS service unavailable')
                f=client.call_async(Trigger.Request());done=threading.Event();f.add_done_callback(lambda _:done.set())
                if not done.wait(2):raise ValueError('ROS service timed out')
                result=f.result();message=('STOP' if stop else 'RESET')+f' success={result.success}: '+result.message
                n.events.append(message);self.reply(200,{'success':result.success,'message':message})
            except (ValueError,KeyError) as e:self.reply(400,{'success':False,'message':str(e)})
    server=ThreadingHTTPServer(('127.0.0.1',8840),Handler)
    n.get_logger().info('Live ROS dashboard: http://127.0.0.1:8840')
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close();n.destroy_node();rclpy.shutdown();thread.join(timeout=1)

if __name__=='__main__':main()
