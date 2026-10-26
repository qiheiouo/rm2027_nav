// Research certificate service. Reuse frozen plant/checker/library, not search edits.
#define main archived_search_main
#include "search_consumer.cpp"
#undef main

J state_json(State s){return J::array({s.x,s.y,s.vx,s.vy});}
State state_from(const J&s){return {s[0],s[1],s[2],s[3]};}
V control_at(const J&events,double t,V initial){
  for(const auto&e:events)if(e[0].get<double>()<=t+1e-8)initial={e[1],e[2]};
  return initial;
}
J ordered(J events){std::stable_sort(events.begin(),events.end(),[](const J&a,const J&b){return a[0].get<double>()<b[0].get<double>();});return events;}
struct ReserveChecker:Planner {
  J certify(const J&q,J events,double switch_time,const J&backup_controls){
    const double epoch=q["epoch"];Node n{state_from(q["state"])};
    V u=q["target"].get<V>();events=ordered(events);
    double end=std::max(epoch,events.back()[0].get<double>())+VMAX/AMAX+.31;
    J samples=J::array({state_json(n.s)});
    for(int k=0;k<int(std::ceil((end-epoch)/DT));k++){
      u=control_at(events,epoch+n.t+DT/2,u);
      if(!segment(n,u,1))return nullptr;
      samples.push_back(state_json(n.s));
    }
    if(!refuge(n))return nullptr;
    double rest_clear=wall-std::abs(n.s.y)-HY;
    for(const auto&a:actors)rest_clear=std::min(rest_clear,ray_clearance(n.s,a,n.t));
    return {{"epoch",epoch},{"start_state",q["state"]},{"initial_target",q["target"]},
      {"immutable_queue_version",q["queue_version"]},{"immutable_queue",q["queue"]},
      {"controls",events},{"backup_controls",backup_controls},{"switch_time",switch_time},
      {"execution_cursor",0},{"reference_states",samples},
      {"prediction",{{"epoch",epoch},{"source_epoch",q["source_epoch"]},{"version",q["source_version"]},{"actors",q["actors"]}}},
      {"assumptions",{{"model","exact CV + frozen plant"},{"numerical_tolerance",1e-7},{"maximum_source_age_s",.15},{"wall",wall}}},
      {"safe_rest",{{"epoch",epoch+n.t},{"state",state_json(n.s)},{"cv_ray_clearance_m",rest_clear},{"certified",true}}}};
  }
  std::string valid(const J&q,const J&b){
    if(b.is_null())return "unavailable";
    double now=q["epoch"],age=now-q["source_epoch"].get<double>();
    if(age<0||age>.15+1e-9)return "stale_observation";
    double offset=now-b["epoch"].get<double>();
    if(offset< -1e-8||std::abs(offset/DT-std::round(offset/DT))>1e-6)return "clock_mismatch";
    if(q["wall"]!=b["assumptions"]["wall"])return "geometry_changed";
    const auto&samples=b["reference_states"];
    auto expected=samples[std::min(size_t(std::llround(offset/DT)),samples.size()-1)];
    for(int j=0;j<4;j++)if(std::abs(q["state"][j].get<double>()-expected[j].get<double>())>1e-7)return "state_deviation";
    auto old=b["prediction"]["actors"];
    if(old.size()!=q["actors"].size())return "obstacle_set_changed";
    for(size_t i=0;i<old.size();i++)for(int j=0;j<5;j++){
      double v=old[i][j];if(j<2)v+=old[i][j+2].get<double>()*offset;
      if(std::abs(v-q["actors"][i][j].get<double>())>1e-7)return "prediction_changed";
    }
    V expected_u=control_at(b["controls"],now-DT/2,b["initial_target"].get<V>());
    V actual=q["target"].get<V>();
    if(std::hypot(expected_u[0]-actual[0],expected_u[1]-actual[1])>1e-7)return "executed_command_mismatch";
    for(const auto&e:q["queue"]){
      V v=control_at(b["controls"],e[0],b["initial_target"].get<V>());
      if(std::hypot(v[0]-e[1].get<double>(),v[1]-e[2].get<double>())>1e-7)return "immutable_queue_mismatch";
    }
    return "valid";
  }
  J check(const J&q){
    auto begin=std::chrono::steady_clock::now();actors.clear();wall=q["wall"];
    for(const auto&a:q["actors"])actors.push_back({a[0],a[1],a[2],a[3],a[4]});
    std::string op=q["op"],reason="unavailable";J cert=nullptr;
    if(op=="validate")return {{"reason",valid(q,q["backup"])}};
    Node n{state_from(q["state"])};V u=q["target"].get<V>();double epoch=q["epoch"];
    J events=ordered(q["queue"]);bool ok=true;
    for(int k=0;k<10&&ok;k++){u=control_at(events,epoch+n.t+DT/2,u);ok=segment(n,u,1);}
    if(op=="admit"&&ok){u=q["command"].get<V>();events.push_back({epoch+.1,u[0],u[1]});ok=segment(n,u,5);}
    double branch=epoch+n.t;
    if(!ok)reason="unsafe_prefix";
    else {
      const J old=q.value("backup",J(nullptr));
      if(op=="admit"&&!old.is_null()){
        // Splice at the actual branch state. Remaining old event times stay absolute.
        J suffix=J::array();V v=control_at(old["backup_controls"],branch,{0,0});
        suffix.push_back({branch,v[0],v[1]});
        for(const auto&e:old["backup_controls"])if(e[0].get<double>()>branch+1e-8)suffix.push_back(e);
        J all=events;for(const auto&e:suffix)all.push_back(e);
        cert=certify(q,all,branch,suffix);if(!cert.is_null())reason="splice_old";
      }
      if(cert.is_null()){
        J suffix=J::array();
        if(backup(n,suffix)){
          for(auto&e:suffix)e[0]=epoch+e[0].get<double>();
          J all=events;for(const auto&e:suffix)all.push_back(e);
          cert=certify(q,all,branch,suffix);if(!cert.is_null())reason="new_backup";
        }
      }
    }
    return {{"reason",reason},{"certificate",cert},{"available",!cert.is_null()},
      {"compute_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()}};
  }
};
int main(){ReserveChecker c;std::string line;while(std::getline(std::cin,line)){try{
  J q=J::parse(line);if(q.contains("probe")){std::cout<<"{}"<<std::endl;continue;}
  std::cout<<c.check(q).dump()<<std::endl;
}catch(const std::exception&e){std::cout<<J({{"error",e.what()}}).dump()<<std::endl;}}}
