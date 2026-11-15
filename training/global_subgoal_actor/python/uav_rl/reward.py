"""Full-flight SMDP credit: each physics interval belongs to one decision."""
import numpy as np
from scipy.io import loadmat
from .maps import geodesic_field,field_values
from .replay import Episode
from .global_obs import encode_map
from .reward_v2 import revisit_flags,near_safety_densities


def motion_costs(trajectory,dt):
    """True motion prevents sensor jitter from creating path or revisits."""
    delta=np.diff(trajectory[:3],axis=2)
    leader=np.linalg.norm(delta[:,0],axis=0)
    fleet=np.linalg.norm(delta,axis=0).sum(0)
    points=trajectory[:3,0].T;n=len(leader)
    stagnant=np.zeros(n,bool);revisit=np.zeros(n,bool)
    window=max(1,round(2./dt));separation=max(1,round(8./dt))
    cells=np.floor(points/3.).astype(np.int32);last={}
    for k in range(n):
        if k>=window:
            stagnant[k]=np.max(np.linalg.norm(points[k-window:k+2]-points[k+1],axis=1))<.25
        cell=tuple(cells[k+1]);previous=last.get(cell)
        if previous is not None and k-previous>=separation:revisit[k]=True
        last[cell]=k
    revisit=revisit_flags(points,dt)
    return leader,fleet,stagnant,revisit


def read_transition(path,cfg,case,decision_path=None):
    data=loadmat(path,simplify_cells=True);row=data['row'];record=data['record'];r=data['result']
    assert bool(row['audit_passed']) and int(row['teacher_queries'])==0
    assert row['task_protocol']=='global_subgoal_v1' and row['leader_route_policy']=='actor_global'
    actions=np.asarray(record['actions'],np.float32).reshape(3,-1).T
    assert len(actions)>0,'Full-mission candidate must decide at t=0.'
    observations=np.asarray(record['observations'],np.float32).reshape(128,-1).T
    positions=np.asarray(record['positions'],np.float32).reshape(3,-1).T
    global_features=np.asarray(record['global_features'],np.float32).reshape(-1)
    assert global_features.shape==(1289,) and np.max(np.abs(global_features-encode_map(case)))<2e-7
    action_index=np.asarray(record['action_index']).astype(np.int64).reshape(-1)
    times=np.asarray(record['times'],float).reshape(-1);obs_times=np.asarray(record['observation_times'],float).reshape(-1)
    h=r['history'];dt=float(r['params']['dt']);states=np.asarray(h['state']).reshape(6,5,-1);n=states.shape[2]
    trajectory=np.concatenate([np.asarray(r['initialFleetState'])[:,:,None],states],axis=2)
    next_index=np.r_[action_index[1:],len(observations)-1]
    end_times=np.r_[times[1:],float(row['flight_seconds'])]
    steps=np.rint(times/dt).astype(int);ends=np.rint(end_times/dt).astype(int)
    elapsed=end_times-times
    assert abs(times[0])<1e-9 and steps[0]==0 and ends[-1]==n
    assert np.all(elapsed>0) and np.array_equal(ends[:-1],steps[1:])
    assert np.max(np.abs(obs_times[action_index]-times))<1e-8
    assert np.max(np.abs(obs_times[next_index]-end_times))<1e-8
    assert np.all(elapsed[:-1]>=cfg['environment']['minimum_decision_seconds']-1e-8)
    discount=np.power(cfg['learner']['gamma'],elapsed/cfg['environment']['decision_seconds']).astype(np.float32)
    done=np.zeros(len(actions),np.float32);done[-1]=1
    field=geodesic_field(case);scale=cfg['reward']['potential_scale_meters']
    phi=-field_values(field,trajectory[:3,0,steps].T)/scale
    phi_next=-field_values(field,trajectory[:3,0,ends].T)/scale;phi_next[-1]=0
    w=cfg['reward'];components={'potential':discount*phi_next-phi,'time':-w['time_per_second']*elapsed}
    for key in ['effort','safety','stagnation','leader_path','fleet_path','revisit','smoothness','projection','repeated','near_obstacle','near_pair','near_workspace','outcome']:
        components[key]=np.zeros(len(actions))
    command=np.asarray(h['command']).reshape(3,5,-1);safety=np.asarray(h['safetyTriggered']).reshape(-1)
    leader,fleet,stagnant,revisit=motion_costs(trajectory,dt)
    near=near_safety_densities(trajectory,h,r['params'],w)
    covered=np.zeros(n,np.int16)
    projection=np.asarray(record['projection_distance'],float).reshape(-1)
    repeated=np.asarray(record['repeated_subgoal'],bool).reshape(-1)
    for i,(a,b) in enumerate(zip(steps,ends)):
        covered[a:b]+=1
        for key,values in near.items():components[key][i]=-w['near_safety_per_second']*dt*float(values[a:b].sum())
        components['effort'][i]=-w['effort_per_unit']*dt*float(np.sum(command[:,:,a:b]**2))
        components['safety'][i]=-w['safety_per_second']*dt*int(np.count_nonzero(safety[a:b]))
        components['stagnation'][i]=-w['stagnation_per_second']*dt*int(np.count_nonzero(stagnant[a:b]))
        components['leader_path'][i]=-w['leader_path_per_meter']*float(leader[a:b].sum())
        components['fleet_path'][i]=-w['fleet_path_per_meter']*float(fleet[a:b].sum())
        components['revisit'][i]=-w['revisit_per_second']*dt*int(np.count_nonzero(revisit[a:b]))
        components['projection'][i]=-w['projection_per_meter']*projection[i]
        components['repeated'][i]=-w['repeated_subgoal']*repeated[i]
        if i:components['smoothness'][i]=-w['smoothness']*float(np.sum((actions[i]-actions[i-1])**2))
    assert np.all(covered==1),'A physics interval was omitted or double credited.'
    safe_success=bool(row['success']) and bool(row['safe'])
    outcome=w['full_success'] if safe_success else w['collision'] if bool(row['critical']) or not bool(row['safe']) else w['failure']
    components['outcome'][-1]=outcome
    rewards=np.sum(list(components.values()),axis=0).astype(np.float32)
    ep=Episode(str(row['id']),str(row['group']),observations,action_index,next_index,actions,rewards,done,discount,global_features,positions);ep.validate()
    ep.outcome=1 if safe_success else 3 if not row['safe'] or row['critical'] else 2;ep.map_key=case['geometry_sha256']
    accounting={'transitions':len(actions),'total_reward':float(rewards.sum()),'outcome_credit':float(outcome),
        'reward_component_sums':{k:float(v.sum()) for k,v in components.items()},'global_map_visible_to_actor':True,
        'actor_schema':6,'reward_schema':2,'progress_interpolation':'trilinear','loop_penalty_multiplier':cfg['reward']['revisit_multiplier'],'global_observation_schema':1,'map_geometry_feature_count':1289,'teacher_actions_or_labels':False,
        'terminal_boundaries':int(done.sum()),'physics_steps_covered':int(covered.sum()),'physics_coverage_min':int(covered.min()),
        'physics_coverage_max':int(covered.max()),'accounted_seconds':float(elapsed.sum()),
        'mean_decision_hold_seconds':float(elapsed.mean()),'last_decision_to_terminal_seconds':float(elapsed[-1]),
        'projection_count':int(np.count_nonzero(projection>1e-7)),
        'actual_new_physics_steps':max(0,int(row['steps'])-(int(row['resume_step'])-1))}
    if decision_path is not None:
        from pathlib import Path
        from .io import sha256
        destination=Path(decision_path);destination.parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(destination,raw_actions=actions,requested_subgoals=np.asarray(record['requested_subgoals']).reshape(3,-1).T,
            executed_subgoals=np.asarray(record['executed_subgoals']).reshape(3,-1).T,projection_distance=projection,
            repeated_subgoal=repeated,reselection_reasons=np.asarray(record['reselection_reasons']).astype(str),
            projection_methods=np.asarray(record['projection_methods']).astype(str),decision_times=times,end_times=end_times,
            physics_start=steps,physics_end=ends,rewards=rewards,discount=discount,done=done,
            **{'reward_'+k:v for k,v in components.items()})
        accounting.update(decision_record=str(destination),decision_record_sha256=sha256(destination))
    return ep,dict(row),accounting
