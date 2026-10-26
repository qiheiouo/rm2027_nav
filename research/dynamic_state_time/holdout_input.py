"""Causal synthetic measurement source. Future truth is evaluator-only.

The planner receives only the result of sample(t), never the event schedule.
Every measurement comes from a source epoch <= the current request epoch.
"""
import math


def truth(case, t):
    actors=[]
    for index,a in enumerate(case['scene']['static']+case['scene']['dynamic']):
        x,y,vx,vy,r=a;last=0.
        for change in case['motion_events']:
            if change['actor']!=index or change['t']>t:continue
            x+=vx*(change['t']-last);y+=vy*(change['t']-last)
            vx,vy=change['velocity'];last=change['t']
        actors.append([x+vx*(t-last),y+vy*(t-last),vx,vy,r])
    return actors


class CausalInput:
    def __init__(self, case):
        self.case=case

    def truth(self,t):
        return truth(self.case,t)

    def sample(self,t):
        c=self.case;d=c['input'];source=t-d['latency_s']
        if d['kind']=='stale' and d['start_s']<=t<d['start_s']+d['duration_s']:
            source=d['start_s']-d['latency_s']
        assert source<=t+1e-10
        # Read only present/past source state. No future change is disclosed.
        actors=truth(c,source);indices=list(range(len(actors)));ids=indices.copy()
        if d['kind']=='occlusion' and d['start_s']<=source<d['start_s']+d['duration_s']:
            actors=actors[1:];indices=indices[1:];ids=ids[1:]
        changed=source>=d['start_s']
        if changed and d['kind']=='velocity_bias':
            for a in actors:
                a[2]+=d['error_mps'];a[3]+=d.get('error_y_mps',0.)
        if changed and d['kind']=='position_bias':
            for a in actors:a[0]+=d['error_m']
        if changed and d['kind']=='id_relabel':ids=[v+100 for v in ids]
        if changed and d['kind']=='id_misassociate' and source<d['start_s']+d['duration_s']:
            velocities=[a[2:4] for a in actors][::-1]
            for a,v in zip(actors,velocities):a[2:4]=v
        current=[[a[0],a[1],a[4]] for a in actors]
        predicted=[[x+vx*(t-source),y+vy*(t-source),vx,vy,r] for x,y,vx,vy,r in actors]
        metadata=dict(source_epoch=source,source_version=round((source+10)*1000000),
                      age_s=t-source,track_ids=ids,evaluation_actor_indices=indices,
                      source_states=actors,kind=d['kind'])
        return current,predicted,metadata


def check_inputs(cases):
    """Boundary checks independent of controller outcomes, no algorithm tuning."""
    import copy
    checked=0
    for case in cases:
        source=CausalInput(case)
        for t in (0.,1.,1.95,2.,2.1,2.2,2.5,3.,4.,8.):
            _,actors,meta=source.sample(t)
            assert meta['source_epoch']<=t and meta['age_s']>=0
            assert len(actors)==len(meta['track_ids'])
            assert all(all(math.isfinite(v) for v in a) for a in actors)
            # Changing every future event cannot change the current observation.
            alternate=copy.deepcopy(case)
            for event in alternate['motion_events']:
                if event['t']>meta['source_epoch']+1e-9:event['velocity']=[7.,-9.]
            assert source.sample(t)==CausalInput(alternate).sample(t)
            checked+=1
    return dict(cases=len(cases),causality_checks=checked,passed=True)
