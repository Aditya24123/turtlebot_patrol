"""Small deterministic differential-drive TurtleBot test world (metres)."""
import math

WIDTH,HEIGHT=8.,6.
RADIUS=0.12 # conservative circular footprint around the TurtleBot3 Burger
OBSTACLES=[(3.4,1.,0.43),(6.3,3.,0.44),(3.6,4.8,0.45),(2.2,3.0,0.38)]

def clearance(x,y):
    return min(x,WIDTH-x,y,HEIGHT-y,*(math.hypot(x-ox,y-oy)-r for ox,oy,r in OBSTACLES))-RADIUS

def ray(x,y,a):
    dx,dy=math.cos(a),math.sin(a); hits=[3.5]
    for edge,origin,direction in [(0,x,dx),(WIDTH,x,dx),(0,y,dy),(HEIGHT,y,dy)]:
        if abs(direction)>1e-10:
            distance=(edge-origin)/direction
            if distance>=0:hits.append(distance)
    for ox,oy,r in OBSTACLES:
        px,py=x-ox,y-oy; b=px*dx+py*dy; disc=b*b-(px*px+py*py-r*r)
        if disc>=0:
            t=-b-math.sqrt(disc)
            if t>=0:hits.append(t)
    return min(hits)

def scan(x,y,yaw):
    return [ray(x,y,yaw+math.radians(i)) for i in range(360)]

def advance(x,y,yaw,v,w,dt):
    # Midpoint integration; refusing colliding motion is a simulator guard, reported separately.
    nx=x+v*math.cos(yaw+w*dt/2)*dt;ny=y+v*math.sin(yaw+w*dt/2)*dt
    collision=clearance(nx,ny)<0
    return (x if collision else nx,y if collision else ny,math.atan2(math.sin(yaw+w*dt),math.cos(yaw+w*dt)),collision)
