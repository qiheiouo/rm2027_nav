"""Environment only. Neither observation-only provider imports this module."""
import math

SCENES = ("empty", "crossing", "head_on", "narrow", "sudden_stop", "wait_release")

def parameters(scene, seed):
    # Fixed small phase variations declared before the first formal run.
    phase = {11: 0.0, 13: -.20, 29: 0.0, 47: .20}[seed]
    return {"scene": scene, "phase": phase, "wall": .70 if scene in ("narrow", "wait_release") else 3.5,
            "radius": .36, "goal": (4., 0.)}

def truth(p, t):
    name = p["scene"]
    if name == "empty":
        return None
    t = max(0., t + p["phase"])
    if name == "crossing":
        return (2., -1.8 + .6 * max(0., t - 1.))
    if name == "head_on":
        return (3.5 - .35 * t, 0.)
    if name == "narrow":
        return (2., -1.4 + .30 * max(0., t - 1.))
    if name == "sudden_stop":
        return (2., -1.5 + .6 * min(max(0., t - 1.), 2.5))
    if name == "wait_release":
        return (2., -1.2 + .6 * min(max(0., t - 1.), 2.) + .6 * max(0., t - 7.))
    raise ValueError(name)

def clearance(state, point, radius, padding=0.):
    if point is None:
        return None
    x,y,yaw = state[:3]
    dx,dy=point[0]-x,point[1]-y
    c,s=math.cos(yaw),math.sin(yaw)
    qx,qy=abs(c*dx+s*dy)-(.325+padding),abs(-s*dx+c*dy)-(.300+padding)
    # Independent signed rectangle/disk distance, including penetration.
    return math.hypot(max(qx,0.),max(qy,0.))+min(max(qx,qy),0.)-radius

def wall_clearance(state, wall, padding=0.):
    yaw=state[2]
    extent=abs(math.sin(yaw))*(.325+padding)+abs(math.cos(yaw))*(.3+padding)
    return wall-abs(state[1])-extent
