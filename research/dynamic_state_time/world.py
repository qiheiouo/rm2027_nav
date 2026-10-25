"""Small deterministic synthetic world. No hidden future access in consumers."""
import heapq
import math
import pathlib
import sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/"research_snapshots/prediction_value"))
from scenes import clearance as original_clearance

DT=.01

def cap(v,limit):
    n=math.hypot(*v)
    return [x*min(1,limit/max(n,1e-15)) for x in v]

def advance(s,u,dt=DT):
    target=cap(u,.5)
    dv=cap([target[0]-s[2],target[1]-s[3]],.8*dt)
    return [s[0]+(s[2]+dv[0]/2)*dt,s[1]+(s[3]+dv[1]/2)*dt,s[2]+dv[0],s[3]+dv[1]]

def scene(name,seed):
    phase={13:-.2,29:0.,47:.2}[seed]
    static=[[2.,phase*.5,0.,0.,.30]] if name=="static" else []
    dynamic={
        "static":[],
        "crossing":[[2.,-2.5+.6*phase,0.,.6,.30]],
        "head_on":[[5.-.45*phase,0.,-.45,0.,.30]],
        "multiple":[[1.5,-2.2+.6*phase,0.,.6,.30],[2.7,2.8-.6*phase,0.,-.6,.30]]}[name]
    return {"name":name,"seed":seed,"phase":phase,"wall":2.,"static":static,"dynamic":dynamic,"goal":[4.,0.]}

def actors_at(cfg,t):
    return [[x+vx*t,y+vy*t,vx,vy,r] for x,y,vx,vy,r in cfg["static"]+cfg["dynamic"]]

def observed(cfg,t):
    # Full known source state is deliberately ideal in this first gate.
    source=actors_at(cfg,t-.1)
    predicted=[[x+vx*.1,y+vy*.1,vx,vy,r] for x,y,vx,vy,r in source]
    return [[a[0],a[1],a[4]] for a in source],predicted

def clearance(s,actors,wall=2.,padding=0.):
    values=[wall-abs(s[1])-(.3+padding)]
    values.extend(original_clearance([s[0],s[1],0.],a[:2],a[4],padding) for a in actors)
    return min(values)

def static_grid(cfg):
    # Conservative orientation-independent disk inflation ONLY for common static heuristic/path.
    x0,y0,res,w,h=-1.,-2.,.1,61,41
    inflated=math.hypot(.355,.33)+.025
    allowed=[]
    for iy in range(h):
        for ix in range(w):
            x,y=x0+ix*res,y0+iy*res
            allowed.append(abs(y)<cfg["wall"]-inflated and all(math.hypot(x-a[0],y-a[1])>a[4]+inflated for a in cfg["static"]))
    goal=(round((4-x0)/res),round(-y0/res));values=[100.]*len(allowed)
    gi=goal[1]*w+goal[0];values[gi]=0.;q=[(0.,gi)]
    adjacent=[(dx,dy) for dx in (-1,0,1) for dy in (-1,0,1) if dx or dy]
    while q:
        d,k=heapq.heappop(q)
        if d>values[k]+1e-10:continue
        x,y=k%w,k//w
        for dx,dy in adjacent:
            xx,yy=x+dx,y+dy
            if not(0<=xx<w and 0<=yy<h):continue
            j=yy*w+xx
            if not allowed[j]:continue
            if dx and dy and (not allowed[y*w+xx] or not allowed[yy*w+x]):continue
            nd=d+res*math.hypot(dx,dy)
            if nd<values[j]-1e-10:values[j]=nd;heapq.heappush(q,(nd,j))
    x,y=round(-x0/res),round(-y0/res);path=[[0.,0.]]
    for _ in range(w*h):
        if (x,y)==goal:break
        choices=[]
        for dx,dy in adjacent:
            xx,yy=x+dx,y+dy
            if 0<=xx<w and 0<=yy<h and allowed[yy*w+xx]:
                if dx and dy and (not allowed[y*w+xx] or not allowed[yy*w+x]):continue
                choices.append((values[yy*w+xx]+res*math.hypot(dx,dy),values[yy*w+xx],xx,yy))
        _,d,xx,yy=min(choices)
        if d>=values[y*w+x]:raise RuntimeError("static reference has no descending neighbor")
        end=[x0+xx*res,y0+yy*res];start=path[-1]
        count=math.ceil(math.dist(start,end)/.05)
        path.extend([[start[j]+(end[j]-start[j])*k/count for j in (0,1)] for k in range(1,count+1)])
        x,y=xx,yy
    return {"x0":x0,"y0":y0,"resolution":res,"width":w,"height":h,"values":values},path

def follow(s,path):
    nearest=min(range(len(path)),key=lambda k:math.dist(s[:2],path[k]))
    k=nearest
    while k+1<len(path) and math.dist(s[:2],path[k])<.3:k+=1
    return cap([(path[k][0]-s[0])*2.,(path[k][1]-s[1])*2.],.5)

def witness(cfg,path,t,s):
    # Evaluation-only hand schedules; never supplied as dynamic references.
    if cfg["name"] in ("crossing","multiple") and t<8:return [0.,0.]
    if cfg["name"]=="head_on":
        if t<14:return cap([-2*s[0],2*(1.1-s[1])],.5)
        # Dense polyline: the common lookahead follower expects adjacent waypoints.
        # Three sparse corners made nearest-point tracking turn back toward the start.
        reference=[[k*.1,1.1] for k in range(41)]+[[4.,1.1-k*.1] for k in range(1,12)]
        return follow(s,reference)
    return follow(s,path)

def stats(xs):
    if not xs:return {k:None for k in ("n","mean","min","p05","p50","p95","p99","max")}
    xs=sorted(xs)
    def q(p):
        i=(len(xs)-1)*p;l=int(i);h=min(l+1,len(xs)-1)
        return xs[l]+(xs[h]-xs[l])*(i-l)
    return dict(n=len(xs),mean=sum(xs)/len(xs),min=xs[0],p05=q(.05),p50=q(.5),p95=q(.95),p99=q(.99),max=xs[-1])
