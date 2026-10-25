// Research only: no ROS, hardware, or velocity publisher. JSON stdin/stdout.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
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
struct Node {State s; double t=0,cost=0,effort=0;std::vector<V> controls;};
struct Planner {
  std::vector<Actor> actors;double wall=2.;J grid;std::vector<V> path;
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
  J solve(const J& in){
    auto wall_start=std::chrono::steady_clock::now();
    actors.clear();for(auto&a:in["actors"])actors.push_back({a[0],a[1],a[2],a[3],a[4]});
    wall=in["wall"];grid=in["grid"];
    auto s=in["state"];Node start{{s[0],s[1],s[2],s[3]}};
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
        if(used.insert(key).second)beam.push_back(std::move(n));
        if(beam.size()==64)break;
      }
      if(simple||beam.empty())break;
    }
    Node chosen=start;bool verified=false;int viable=0;
    for(auto&n:beam)if(tail(n)){viable++;if(!verified){chosen=n;verified=true;}}
    verified&=prefix_safe;
    V cmd{0,0};std::string reason="no_verified_action";
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
        if(ok&&tail(old)){verified=true;reason="reused";}
      }
      if(!verified){cmd={0,0};if(prefix_safe&&tail(start)){verified=true;reason="brake";}}
    }
    J schedule=J::array();
    if(reason=="selected"){
      for(size_t k=0;k<chosen.controls.size();k++)schedule.push_back({.1+.2*k,chosen.controls[k][0],chosen.controls[k][1]});
      schedule.push_back({2.1,0,0});
    }else if(reason=="reused")schedule=in["previous"];
    else schedule.push_back({.1,0,0});
    double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-wall_start).count();
    return {{"command",cmd},{"verified",verified},{"reason",reason},{"prefix_safe",prefix_safe},{"viable",viable},{"expanded",expanded},{"compute_ms",ms},{"schedule",schedule}};
  }
};
int main(){Planner p;std::string line;while(std::getline(std::cin,line)){try{
  J q=J::parse(line);if(q.contains("probe")){
    auto s=q["state"];auto a=q["actor"];State st{s[0],s[1],s[2],s[3]};Actor ob{a[0],a[1],a[2],a[3],a[4]};
    State next=advance(st,q["command"].get<V>());
    std::cout<<J({{"distance",signed_box_disk(st,ob,0)},{"next",{next.x,next.y,next.vx,next.vy}}}).dump()<<std::endl;
  }else std::cout<<p.solve(q).dump()<<std::endl;
}catch(const std::exception&e){std::cout<<J({{"error",e.what()}}).dump()<<std::endl;}}}
