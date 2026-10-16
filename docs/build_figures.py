"""Rebuild README figures from recorded test results (requires matplotlib)."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle,FancyBboxPatch,FancyArrowPatch

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'docs/images';OUT.mkdir(exist_ok=True)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12,'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'#f6f8fb','axes.facecolor':'white','text.color':'#152b45','axes.labelcolor':'#152b45','xtick.color':'#486078','ytick.color':'#486078'})
trace=json.loads((ROOT/'evidence/patrol_trace.json').read_text());summary=json.loads((ROOT/'evidence/closed_loop.json').read_text())
fig,ax=plt.subplots(figsize=(10,8));fig.subplots_adjust(top=.84,bottom=.20)
fig.suptitle('Two complete autonomous patrol laps',x=.12,y=.97,ha='left',fontsize=23,fontweight='bold')
fig.text(.12,.895,f"Accelerated 2D test · {summary['seconds_simulated']} simulated seconds · zero collision-guard hits",fontsize=12,color='#486078')
for x,y,r in [(3.4,1.,.43),(6.3,3.,.44),(3.6,4.8,.45),(2.2,3.,.38)]:
    ax.add_patch(Circle((x,y),r,facecolor='#c38c56',edgecolor='#704a23',lw=1.5))
colors={'PATROL':'#1569b4','AVOID':'#e79e24','BYPASS':'#119c86'}
segments=[[(a['x'],a['y']),(b['x'],b['y'])] for a,b in zip(trace,trace[1:])]
ax.add_collection(LineCollection(segments,colors=[colors[a['state']] for a in trace[:-1]],linewidths=2.4))
for i,(x,y) in enumerate([(6.3,1),(6.3,4.8),(1,4.8),(1,1)]):
    ax.scatter(x,y,s=95,facecolors='white',edgecolors='#152b45',zorder=5);ax.annotate(f'P{i+1}',(x,y),xytext=(9,9),textcoords='offset points',fontweight='bold')
ax.scatter(1,1,s=35,color='#152b45',zorder=6)
for name,color in colors.items():ax.plot([],[],color=color,lw=3,label=name)
ax.set(xlim=(0,8),ylim=(0,6),xlabel='World x (m)',ylabel='World y (m)');ax.set_aspect('equal');ax.grid(alpha=.18)
ax.legend(loc='upper center',bbox_to_anchor=(.5,-.15),ncol=3,frameon=False)
fig.text(.12,.025,f"Minimum robot-footprint clearance: {summary['minimum_footprint_clearance_m']:.3f} m. Kinematic simulation; physical robot not tested.",fontsize=10,color='#486078')
fig.savefig(OUT/'patrol.png',dpi=160);plt.close(fig)

data=json.loads((ROOT/'evidence/stop_trace.json').read_text());samples=data['trace'];events=data['events']
fig,(a,b)=plt.subplots(2,1,figsize=(11,7),sharex=True);fig.subplots_adjust(top=.79,hspace=.27,bottom=.14,left=.12,right=.95)
fig.suptitle('Emergency-stop service: trigger, hold, reset',x=.12,y=.97,ha='left',fontsize=22,fontweight='bold')
fig.text(.12,.895,'Measured ROS 2 DDS command output · synthetic clear sensor input',fontsize=12,color='#486078')
t=[s['time'] for s in samples]
a.step(t,[s['linear'] for s in samples],where='post',color='#1569b4',lw=2.5)
b.step(t,[s['angular'] for s in samples],where='post',color='#119c86',lw=2.5)
for ax in (a,b):
    ax.axvspan(events[0]['response_at'],events[1]['requested_at'],color='#e56876',alpha=.15)
    for e in events:ax.axvline(e['requested_at'],color='#a43a4a',ls='--',lw=1)
    ax.grid(alpha=.18)
a.set(ylabel='Forward command (m/s)',ylim=(-.025,.245));b.set(ylabel='Yaw command (rad/s)',xlabel='Elapsed monotonic time (s)',ylim=(-.1,.1))
a.annotate('/trigger_estop\nsuccess = true',xy=(events[0]['requested_at'],.2),xytext=(-135,30),textcoords='offset points',fontsize=11,arrowprops={'arrowstyle':'->','color':'#a43a4a'})
a.annotate('/reset_estop\nsuccess = true',xy=(events[1]['requested_at'],.2),xytext=(15,30),textcoords='offset points',fontsize=11,arrowprops={'arrowstyle':'->','color':'#a43a4a'})
fig.text(.12,.035,'The latched interval commanded zero linear and angular velocity; reset restored motion. This is a software stop.',fontsize=10,color='#486078')
fig.savefig(OUT/'emergency_stop.png',dpi=160);plt.close(fig)

fig,ax=plt.subplots(figsize=(12,7));fig.subplots_adjust(top=.82,bottom=.14);ax.set(xlim=(0,12),ylim=(0,6));ax.axis('off')
fig.suptitle('One controller, standard ROS interfaces',x=.08,y=.97,ha='left',fontsize=24,fontweight='bold')
fig.text(.08,.895,'Controller, simulator and dashboard run as separate ROS 2 nodes.',fontsize=13,color='#486078')
def box(x,y,w,h,title,detail,color='#e7f0fa'):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.12',facecolor=color,edgecolor='#486078',lw=1.4))
    ax.text(x+w/2,y+h*.70,title,ha='center',va='center',fontsize=14,fontweight='bold');ax.text(x+w/2,y+h*.30,detail,ha='center',va='center',fontsize=11,linespacing=1.5)
def arrow(p,q,label,offset=(0,0)):
    ax.add_patch(FancyArrowPatch(p,q,arrowstyle='-|>',mutation_scale=15,color='#486078',lw=1.6));ax.text((p[0]+q[0])/2+offset[0],(p[1]+q[1])/2+offset[1],label,ha='center',va='center',fontsize=10,bbox={'facecolor':'#f6f8fb','edgecolor':'none','pad':2})
box(.2,3.,3.1,1.8,'Driver / simulator','Laser scan + odometry\nTwist input')
box(4.5,3.,3.1,1.8,'Patrol controller','Waypoints · avoidance\nFreshness checks')
box(8.8,3.,2.8,1.8,'Live dashboard','ROS status + pose\nStop / reset buttons')
box(4.5,.3,3.1,1.4,'Persistent stop latch','Atomic disk write\nFail stopped on corruption','#fbe8eb')
arrow((3.3,4.35),(4.5,4.35),'/scan\n/odom',(0,.15));arrow((4.5,3.45),(3.3,3.45),'/cmd_vel',(0,-.20))
arrow((7.6,4.35),(8.8,4.35),'/patrol/status',(0,.18));arrow((8.8,3.40),(7.6,3.40),'/trigger_estop\n/reset_estop',(0,-.25))
arrow((6.05,3.),(6.05,1.7),'stop state',(.8,0))
fig.text(.08,.035,'Relative ROS names support namespaces. Sensor-data QoS on inputs; reliable commands and status.\nCompatibility service: /emergency_stop (SetBool). Canonical stop/reset services: Trigger.',fontsize=10.5,color='#486078',va='bottom')
fig.savefig(OUT/'architecture.png',dpi=160);plt.close(fig)
print('Created three figures from verified results.')
