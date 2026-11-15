"""Reuse deterministic TRAIN trajectories only for an identical mean network."""
from pathlib import Path
import torch
import numpy as np
from .io import read_json,sha256,stamp
from .replay import load_episode
from .global_obs import state_at

def verify(parent_run,actor,replay_ids,target):
    parent_run=Path(parent_run);manifest=read_json(parent_run/'MIGRATION_COMPLETE.json')
    original=manifest.get('permanent_checkpoint',manifest['checkpoint'])
    assert sha256(original['path'])==original['sha256']
    saved=torch.load(original['path'],map_location='cpu',weights_only=True)
    current=actor.state_dict();old=saved['learner']['actor']
    identical=all(torch.equal(v.cpu(),old[k]) for k,v in current.items() if k.startswith('body.') or k.startswith('mu.'))
    report={'timestamp':stamp(),'identical_mean_network_tensors':bool(identical),'reusable':False,'episodes':[],'native_trials_claimed_new':0}
    if not identical:return report
    rows=[read_json_line(line) for line in (parent_run/'flights.jsonl').read_text(encoding='utf-8-sig').splitlines() if line.strip()]
    rows=[r for r in rows if r.get('mean_calibration',False)][:target]
    if len(rows)!=target:return report
    available=set(replay_ids);maximum=0.
    for row in rows:
        if row['id'] not in available:return report
        assert not row['stochastic'] and row['audit_passed'] and row['teacher_queries']==0 and row['leader_rule_planner_calls']==0
        path=parent_run/'episodes'/f"{row['id']}.npz";ep=load_episode(path);ep.validate()
        with torch.no_grad():
            for begin in range(0,len(ep.actions),128):
                ix=range(begin,min(begin+128,len(ep.actions)))
                states=torch.as_tensor(np.stack([state_at(ep,ep.action_index[i]) for i in ix]),device=next(actor.parameters()).device)
                error=float(np.max(np.abs(actor(states).cpu().numpy()-ep.actions[begin:begin+len(states)])))
                maximum=max(maximum,error)
        report['episodes'].append({'id':row['id'],'sha256':sha256(path),'group':row['group'],'success':bool(row['success'] and row['safe']),
            'unsafe':not bool(row['safe']),'stage':0,'behavior_exploration':False,'stochastic':False})
    report.update(maximum_recorded_action_error=maximum,reusable=maximum<5e-5,reused_physical_flights=target)
    return report

def read_json_line(line):
    import json
    return json.loads(line)
