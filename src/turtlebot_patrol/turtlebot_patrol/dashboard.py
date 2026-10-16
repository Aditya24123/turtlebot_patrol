"""Live ROS dashboard: all displayed movement and stop state arrive over DDS."""
import json,math,threading,time,tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from .world import WIDTH,HEIGHT,RADIUS,OBSTACLES

class Dashboard:
    def __init__(self,node):
        self.node=node;self.status={};self.pose={};self.events=[];self.trail=[];self.started=time.monotonic()
        self.root=tk.Tk();self.root.title('TurtleBot Patrol - Live ROS 2');self.root.geometry('1160x780+40+40');self.root.resizable(False,False)
        self.root.configure(bg='#0b1421')
        self.canvas=tk.Canvas(self.root,width=1160,height=695,bg='#0b1421',highlightthickness=0);self.canvas.pack()
        buttons=tk.Frame(self.root,bg='#0b1421');buttons.pack(fill='x',padx=24)
        tk.Button(buttons,text='TRIGGER EMERGENCY STOP',command=lambda:self.service(True),bg='#d95361',fg='white',font=('Segoe UI',12,'bold'),padx=16,pady=8).pack(side='left')
        tk.Button(buttons,text='RESET / RESUME',command=lambda:self.service(False),bg='#33ba9b',fg='#08151c',font=('Segoe UI',12,'bold'),padx=20,pady=8).pack(side='left',padx=12)
        tk.Label(buttons,text='Buttons call /emergency_stop (std_srvs/SetBool)',bg='#0b1421',fg='#a7bbd2',font=('Segoe UI',10)).pack(side='left')
        self.client=node.create_client(SetBool,'emergency_stop')
        node.create_subscription(String,'patrol/status',self.on_status,10)
        node.create_subscription(String,'sim/telemetry',self.on_pose,10)
        self.root.protocol('WM_DELETE_WINDOW',self.close)
        self.root.after(50,self.draw)

    def on_status(self,m):self.status=json.loads(m.data)
    def on_pose(self,m):
        p=json.loads(m.data);self.pose=p
        if not self.trail or math.hypot(p['x']-self.trail[-1][0],p['y']-self.trail[-1][1])>.025:
            self.trail.append((p['x'],p['y']))
            self.trail=self.trail[-2500:]

    def service(self,stop):
        if not self.client.service_is_ready():self.events.append('Service unavailable');return
        req=SetBool.Request();req.data=stop
        future=self.client.call_async(req)
        def complete(f):
            try:
                result=f.result();self.events.append(('STOP' if stop else 'RESET')+f' success={result.success}: '+result.message)
            except Exception as e:self.events.append(str(e))
        future.add_done_callback(complete)

    def close(self):
        self.service(True);self.root.after(150,self.root.destroy)

    def draw(self):
        c=self.canvas;c.delete('all');s=self.status;p=self.pose
        c.create_text(24,23,text='TURTLEBOT / AUTONOMOUS PATROL',anchor='nw',fill='#ecf4ff',font=('Segoe UI',22,'bold'))
        c.create_text(26,63,text='ROS 2 Jazzy  |  2D differential-drive simulation  |  live DDS topics + service',anchor='nw',fill='#9eb3cc',font=('Segoe UI',11))
        scale=90;ox=28;oy=655
        def point(x,y):return ox+x*scale,oy-y*scale
        c.create_rectangle(*point(0,HEIGHT),*point(WIDTH,0),fill='#14243a',outline='#50627a',width=3)
        for x in range(9):c.create_line(*point(x,0),*point(x,HEIGHT),fill='#22344d')
        for y in range(7):c.create_line(*point(0,y),*point(WIDTH,y),fill='#22344d')
        goals=[(6.3,1),(6.3,4.8),(1,4.8),(1,1)]
        for i,(x,y) in enumerate(goals):
            px,py=point(x,y);color='#48d4bd' if i==s.get('target') else '#6a7e9a'
            c.create_oval(px-9,py-9,px+9,py+9,outline=color,width=2);c.create_text(px+14,py-14,text=f'P{i+1}',fill=color,font=('Segoe UI',10,'bold'))
        for x,y,r in OBSTACLES:
            px,py=point(x,y);rr=r*scale;c.create_oval(px-rr,py-rr,px+rr,py+rr,fill='#b67b46',outline='#efb46e',width=2)
        if len(self.trail)>1:c.create_line(*[v for pt in self.trail for v in point(*pt)],fill='#35b6a5',width=2)
        if p:
            x,y=p['x'],p['y'];a=p['yaw'];px,py=point(x,y);rr=RADIUS*scale
            c.create_oval(px-rr,py-rr,px+rr,py+rr,fill='#50e1cd',outline='white',width=2)
            c.create_line(px,py,px+math.cos(a)*25,py-math.sin(a)*25,fill='white',width=3)
            r=min(2,s.get('front',0))*scale;c.create_line(px,py,px+math.cos(a)*r,py-math.sin(a)*r,fill='#f6d76d',dash=(4,4))
        x=785
        c.create_text(x,114,text='LIVE CONTROLLER',anchor='nw',fill='#a4b8d1',font=('Segoe UI',12,'bold'))
        state=s.get('state','WAITING');color='#fa7d87' if state=='ESTOP' else '#ffd26d' if state in ('AVOID','BLOCKED','SENSOR_FAULT') else '#50e1cd'
        c.create_text(x,147,text=state,anchor='nw',fill=color,font=('Segoe UI',28,'bold'))
        lines=[f"Emergency latch: {'ON' if s.get('latched') else 'OFF'}",f"Forward: {s.get('linear',0):.2f} m/s",f"Yaw: {s.get('angular',0):.2f} rad/s",f"Front clearance: {s.get('front',0):.2f} m",f"Waypoint: P{s.get('target',0)+1} / 4",f"Distance: {p.get('distance',0):.2f} m",f"Collision guard hits: {p.get('collision_guard_hits',0)}",f"Elapsed: {time.monotonic()-self.started:.1f} s"]
        for i,line in enumerate(lines):c.create_text(x,211+i*31,text=line,anchor='nw',fill='#e0eaf5',font=('Segoe UI',13))
        c.create_text(x,490,text='SERVICE RESPONSES',anchor='nw',fill='#a4b8d1',font=('Segoe UI',11,'bold'))
        for i,event in enumerate(self.events[-3:]):c.create_text(x,519+i*48,text=event,anchor='nw',fill='#d4e3f6',font=('Segoe UI',10),width=345)
        c.create_text(28,675,text='/scan → patrol → /cmd_vel     /odom → waypoint tracking     /emergency_stop → persistent latch',anchor='nw',fill='#9eb3cc',font=('Segoe UI',10))
        self.root.after(50,self.draw)

def main(args=None):
    rclpy.init(args=args);n=Node('patrol_dashboard');d=Dashboard(n)
    t=threading.Thread(target=rclpy.spin,args=(n,),daemon=True);t.start()
    try:d.root.mainloop()
    finally:n.destroy_node();rclpy.shutdown();t.join(timeout=1)

if __name__=='__main__':main()
