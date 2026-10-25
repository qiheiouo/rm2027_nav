// Research only: no ROS, hardware, or velocity publisher. JSON stdin/stdout.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <vector>
#include <nlohmann/json.hpp>
using J=nlohmann::json;
using V=std::array<double,2>;
struct State {double x,y,vx,vy;};
struct Actor {double x,y,vx,vy,r;};
constexpr double DT=.01, VMAX=.5, AMAX=.8, HX=.355, HY=.330;
V cap(V a,double limit){double n=std::hypot(a[0],a[1]);if(n>limit){a[0]*=limit/n;a[1]*=limit/n;}return a;}
State advance(State s,V u,double dt=DT){
  u=cap(u,VMAX);V d=cap({u[0]-s.vx,u[1]-s.vy},AMAX*dt);
  s.x+=(s.vx+d[0]/2)*dt;s.y+=(s.vy+d[1]/2)*dt;s.vx+=d[0];s.vy+=d[1];return s;
}
double signed_box_disk(State s,const Actor&a,double t){
  double qx=std::abs(a.x+a.vx*t-s.x)-HX,qy=std::abs(a.y+a.vy*t-s.y)-HY;
  return std::hypot(std::max(qx,0.),std::max(qy,0.))+std::min(std::max(qx,qy),0.)-a.r;
}
// Minimum signed clearance from a stationary box to an observed CV ray, tau >= 0.
// Squared distance to the box is piecewise quadratic; inspect slab breakpoints
// and each interval's stationary point. No sampled future / ground truth input.
double ray_clearance(State s,Actor a,double epoch){
  a.x+=a.vx*epoch;a.y+=a.vy*epoch;
  V p{a.x-s.x,a.y-s.y},v{a.vx,a.vy},h{HX,HY};
  std::vector<double> cuts{0};
  for(int j=0;j<2;j++)if(std::abs(v[j])>1e-14)
    for(int sign:{-1,1}){double t=(sign*h[j]-p[j])/v[j];if(t>0)cuts.push_back(t);}
  std::sort(cuts.begin(),cuts.end());
  double best=std::numeric_limits<double>::infinity();
  for(size_t k=0;k<cuts.size();k++){
    double lo=cuts[k],hi=k+1<cuts.size()?cuts[k+1]:std::numeric_limits<double>::infinity();
    best=std::min(best,signed_box_disk(s,a,lo));
    double mid=std::isfinite(hi)?(lo+hi)/2:lo+1,numer=0,denom=0;
    for(int j=0;j<2;j++){
      double q=p[j]+v[j]*mid;
      if(std::abs(q)>h[j]){double b=p[j]-std::copysign(h[j],q);numer+=v[j]*b;denom+=v[j]*v[j];}
    }
    if(denom>0){double t=std::max(lo,std::min(hi,-numer/denom));best=std::min(best,signed_box_disk(s,a,t));}
  }
  // Within the box the signed minimum need not be exact; it is already negative,
  // which suffices to reject. Positive minima (the certificate) are exact.
  return best;
}
struct Node {State s; double t=0,cost=0,effort=0;std::vector<V> controls;J continuation=J::array();};
struct Planner {
  std::vector<Actor> actors;double wall=2.;J grid;std::vector<V> path;
  int continuation_checks=0,continuation_rejections=0;
  bool safe(State s,double t)const{
    if(wall-std::abs(s.y)-HY<.02+VMAX*DT/2)return false;
    for(auto&a:actors)if(signed_box_disk(s,a,t)<.02+(VMAX+std::hypot(a.vx,a.vy))*DT/2)return false;
    return true;
  }
  bool segment(Node &n,V u,int steps)const{
    if(!safe(n.s,n.t))return false;
    for(int k=0;k<steps;k++){n.s=advance(n.s,u);n.t+=DT;if(!safe(n.s,n.t))return false;}return true;
  }
  double distance(State s)const{
    double gx=(s.x-grid["x0"].get<double>())/grid["resolution"].get<double>();
    double gy=(s.y-grid["y0"].get<double>())/grid["resolution"].get<double>();
    int x=std::lround(gx),y=std::lround(gy),w=grid["width"],h=grid["height"];
    if(x<0||y<0||x>=w||y>=h)return 100+std::hypot(4-s.x,s.y);
    double base=grid["values"][y*w+x];
    // Subcell projection toward the next descending neighbor avoids flat score plateaus.
    double best=base;
    for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++){
      int xx=x+dx,yy=y+dy;if(xx<0||yy<0||xx>=w||yy>=h)continue;
      double v=grid["values"][yy*w+xx];if(v>=base)continue;
      best=std::min(best,v+std::hypot(gx-xx,gy-yy)*grid["resolution"].get<double>());
    }
    return best;
  }
  bool tail(Node n)const{
    int steps=std::ceil(std::hypot(n.s.vx,n.s.vy)/AMAX/DT)+30;
    return segment(n,{0,0},steps);
  }
  bool refuge(const Node&n)const{
    if(std::hypot(n.s.vx,n.s.vy)>1e-9||!safe(n.s,n.t))return false;
    for(auto&a:actors)if(ray_clearance(n.s,a,n.t)<.02+(VMAX+std::hypot(a.vx,a.vy))*DT/2)return false;
    return true;
  }
  bool backup(Node n,J&schedule){
    continuation_checks++;
    // Brake first; then all eight fixed directions. 1.4 s is two speed-ramp
    // times rounded up to the unchanged .2 s action grid (see preregistration).
    for(int index=-1;index<8;index++){
      Node b=n;J events=J::array();
      if(index>=0){
        const V dirs[8]={{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
        V u=cap(dirs[index],VMAX);events.push_back({b.t,u[0],u[1]});
        if(!segment(b,u,140))continue;
      }
      events.push_back({b.t,0,0});
      int steps=std::ceil(std::hypot(b.s.vx,b.s.vy)/AMAX/DT)+30;
      if(segment(b,{0,0},steps)&&refuge(b)){schedule=events;return true;}
    }
    continuation_rejections++;return false;
  }
  J solve(const J& in){
    auto wall_start=std::chrono::steady_clock::now();
    continuation_checks=continuation_rejections=0;
    actors.clear();for(auto&a:in["actors"])actors.push_back({a[0],a[1],a[2],a[3],a[4]});
    wall=in["wall"];
    auto s=in["state"];Node start{{s[0],s[1],s[2],s[3]}};
    if(in.contains("recourse_probe")){
      start.t=in.value("epoch",0.);J continuation;
      bool stationary=refuge(start),exists=backup(start,continuation);
      return {{"stationary_refuge",stationary},{"backup_exists",exists},{"schedule",continuation}};
    }
    grid=in["grid"];bool recourse=in["mode"]=="recourse";
    V target=in["target"].get<V>();bool prefix_safe=true;
    // Commit horizon = reserved compute slot + transport; previously queued commands remain.
    for(int k=0;k<10;k++){
      double mid=(k+.5)*DT;
      for(auto&e:in["queue"])if(e[0].get<double>()<=mid)target={e[1],e[2]};
      start.s=advance(start.s,target);start.t+=DT;prefix_safe&=safe(start.s,start.t);
    }
    int expanded=0;std::vector<Node> beam{start};bool simple=in["mode"]=="simple";
    for(int depth=0;depth<10;depth++){
      std::vector<Node> next;
      for(auto &p:beam)for(int ax=-1;ax<=1;ax++)for(int ay=-1;ay<=1;ay++){
        if(expanded++>=6000)break;
        V dv=cap({ax*AMAX*.2,ay*AMAX*.2},AMAX*.2);
        V u=cap({p.s.vx+dv[0],p.s.vy+dv[1]},VMAX);
        // Simple uses one reachable target held through the whole same horizon.
        Node n=p;V previous=n.controls.empty()?target:n.controls.back();
        n.effort+=std::pow(u[0]-previous[0],2)+std::pow(u[1]-previous[1],2);
        if(!segment(n,u,simple?200:20))continue;
        n.controls.push_back(u);
        n.cost=distance(n.s)+.1*(n.s.vx*n.s.vx+n.s.vy*n.s.vy)+.01*n.effort;
        next.push_back(std::move(n));
      }
      std::stable_sort(next.begin(),next.end(),[](const Node&a,const Node&b){return a.cost<b.cost;});
      beam.clear();std::set<std::array<int,4>> used;
      for(auto&n:next){
        // Preserve continuous state, coalesce only the search key.
        std::array<int,4> key{int(std::lround(n.s.x/.05)),int(std::lround(n.s.y/.05)),int(std::lround(n.s.vx/.08)),int(std::lround(n.s.vy/.08))};
        if(recourse&&(used.count(key)||!backup(n,n.continuation)))continue;
        if(used.insert(key).second)beam.push_back(std::move(n));
        if(beam.size()==64)break;
      }
      if(simple||beam.empty())break;
    }
    Node chosen=start;bool verified=false;int viable=0;
    for(auto&n:beam)if(recourse?!n.continuation.empty():tail(n)){viable++;if(!verified){chosen=n;verified=true;}}
    verified&=prefix_safe;
    V cmd{0,0};std::string reason="no_verified_action";J fallback=J::array();
    if(verified){cmd=chosen.controls.front();reason="selected";}
    else {
      // Revalidate the unexecuted previous sequence at its original absolute stage epochs.
      Node old=start;std::vector<V> reused;bool ok=prefix_safe;
      if(in.contains("previous")&&!in["previous"].empty()){
        V last{0,0};
        for(auto&e:in["previous"])if(e[0].get<double>()<=.1+1e-8)last={e[1],e[2]};
        cmd=last;
        for(int k=0;k<200&&ok;k++){
          double mid=old.t+DT/2;
          for(auto&e:in["previous"])if(e[0].get<double>()<=mid)last={e[1],e[2]};
          ok=segment(old,last,1);
        }
        if(ok&&(recourse?backup(old,fallback):tail(old))){verified=true;reason="reused";}
      }
      if(!verified){
        cmd={0,0};
        if(recourse){
          if(prefix_safe&&backup(start,fallback)){verified=true;reason="backup";cmd={fallback[0][1],fallback[0][2]};}
        }else if(prefix_safe&&tail(start)){verified=true;reason="brake";}
      }
    }
    J schedule=J::array();
    if(reason=="selected"){
      for(size_t k=0;k<chosen.controls.size();k++)schedule.push_back({.1+.2*k,chosen.controls[k][0],chosen.controls[k][1]});
      if(recourse){for(auto&e:chosen.continuation)schedule.push_back(e);}
      else schedule.push_back({2.1,0,0});
    }else if(reason=="reused"){
      if(recourse){for(auto&e:in["previous"])if(e[0].get<double>()<2.1-1e-8)schedule.push_back(e);for(auto&e:fallback)schedule.push_back(e);}
      else schedule=in["previous"];
    }else if(reason=="backup")schedule=fallback;
    else schedule.push_back({.1,0,0});
    double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-wall_start).count();
    return {{"command",cmd},{"verified",verified},{"reason",reason},{"prefix_safe",prefix_safe},{"viable",viable},{"expanded",expanded},{"compute_ms",ms},{"schedule",schedule},{"continuation_checks",continuation_checks},{"continuation_rejections",continuation_rejections}};
  }
};
int main(){Planner p;std::string line;while(std::getline(std::cin,line)){try{
  J q=J::parse(line);if(q.contains("probe")){
    auto s=q["state"];auto a=q["actor"];State st{s[0],s[1],s[2],s[3]};Actor ob{a[0],a[1],a[2],a[3],a[4]};
    State next=advance(st,q["command"].get<V>());
    std::cout<<J({{"distance",signed_box_disk(st,ob,0)},{"next",{next.x,next.y,next.vx,next.vy}}}).dump()<<std::endl;
  }else std::cout<<p.solve(q).dump()<<std::endl;
}catch(const std::exception&e){std::cout<<J({{"error",e.what()}}).dump()<<std::endl;}}}
