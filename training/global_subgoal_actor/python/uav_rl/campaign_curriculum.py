"""Training-only repeated maps; validation/test never enter this pool."""
import time
from collections import Counter
from pathlib import Path
import numpy as np
from .io import ROOT,read_json,atomic_json,append_json,stamp,digest
from .queue import make_job

def next_job(trainer):
    cfg=trainer.cfg;state=trainer.state
    if not hasattr(trainer,'campaign_pool'):
        pool_path=Path(cfg['campaign']['pool_path']);pool_path=pool_path if pool_path.is_absolute() else ROOT/pool_path
        trainer.campaign_pool=read_json(pool_path)
        assert digest(trainer.campaign_pool)==cfg['campaign']['pool_digest'],'Curriculum pool changed; fork a new audited run.'
    pool=trainer.campaign_pool;stage=state.setdefault('campaign_stage',0)
    origin=state.setdefault('campaign_stage_origin',state['reward_stage_start_flights'])
    actor_origin=state.setdefault('campaign_stage_actor_origin',state['total_actor_steps'])
    accepted=state['total_actor_steps']-actor_origin
    recent=state.get('campaign_recent',[]);rules=cfg['campaign']['curriculum'][stage]
    elapsed=state['flights']-origin
    relevant=[r for r in recent if r['stage']==stage and not r.get('behavior_exploration',False) and not r.get('stochastic',True)]
    if stage<3 and elapsed>=rules['minimum_flights']:
        groups={g:[r for r in relevant if r['group']==g] for g in rules['mastery_families']}
        mastered=all(len(v)>=8 and sum(r['success'] for r in v)/len(v)>=rules['minimum_rate'] for v in groups.values())
        mastered=mastered and not any(r['unsafe'] for r in relevant[-64:])
        if mastered:
            state['campaign_stage']=stage+1;state['campaign_stage_origin']=state['flights']
            state['campaign_stage_actor_origin']=state['total_actor_steps'];state['campaign_rotation_mode']=False
            append_json(trainer.run/'events.jsonl',{'event':'curriculum_promotion','timestamp':stamp(),'from':stage,'to':stage+1,'flights':state['flights']})
            stage+=1;rules=cfg['campaign']['curriculum'][stage]
        elif elapsed>=rules['review_after_flights'] and accepted>=cfg['campaign']['minimum_accepted_actor_updates_for_review']:
            assert cfg['campaign']['curriculum_failed_review_action']=='record_failure_rotate_priority'
            # The review still fails. Record its actual rates, then change
            # sampling priority; a weak U-trap score does not erase gains in
            # the other fifteen families or certify U-trap mastery.
            report={'event':'curriculum_review_failed_priority_rotated','timestamp':stamp(),
                'from':stage,'to':(stage+1)%3,'flights':state['flights'],'elapsed_flights':elapsed,
                'accepted_actor_updates':accepted,'mastered':False,
                'family_rates':{g:{'tasks':len(v),'safe_success':sum(r['success'] for r in v),
                    'unsafe':sum(r['unsafe'] for r in v)} for g,v in groups.items()}}
            append_json(trainer.run/'events.jsonl',report)
            stage=(stage+1)%3;state['campaign_stage']=stage;rules=cfg['campaign']['curriculum'][stage]
            state.update(campaign_stage_origin=state['flights'],campaign_stage_actor_origin=state['total_actor_steps'],
                campaign_review_rotations=state.get('campaign_review_rotations',0)+1,
                campaign_rotation_mode=True,campaign_last_failed_review=report)
            trainer.save_checkpoint('curriculum_failed_review_rotated')
    state['campaign_stage_name']=('review rotation: ' if state.get('campaign_rotation_mode',False) else '')+rules['name']
    scheduled=state.setdefault('mean_calibration_scheduled',0)
    calibration=scheduled<cfg['campaign']['mean_calibration_flights']
    draw=trainer.rng.random() if not calibration else 1.
    fresh=not calibration and draw<cfg['campaign']['fresh_probability']
    rehearsal=calibration or (not fresh and draw<cfg['campaign']['fresh_probability']+cfg['campaign']['rehearsal_probability'])
    if fresh:
        serial=state.setdefault('campaign_fresh_scheduled',0)
        family=cfg['dataset']['families'][serial%len(cfg['dataset']['families'])]
        state['campaign_fresh_scheduled']=serial+1
        job=trainer.random_job(family_override=family,deterministic=trainer.rng.random()<cfg['campaign']['fresh_mean_probability'])
        job['campaign_fresh']=True
        return job
    probes=False if calibration else trainer.rng.random()<.15
    cases=pool['probe_cases'] if probes else pool['train_cases']
    families=cfg['dataset']['families'] if rehearsal else rules['families']
    family=families[scheduled%len(families)] if calibration else families[int(trainer.rng.integers(len(families)))]
    choices=cases[family]
    # Initial learnability uses two known-success and two challenging clean maps.
    if stage==0 and not rehearsal:choices=[c for c in choices if c['noise_profile']=='clean'] or choices
    case=choices[2*((scheduled//len(families))%4)] if calibration else choices[int(trainer.rng.integers(len(choices)))]
    if calibration:state['mean_calibration_scheduled']=scheduled+1
    assert case['split']=='train'
    assert case['geometry_sha256'] not in trainer.heldout_hashes
    number=state['next_job'];state['next_job']+=1;identifier=f'f{number:09d}'
    path=trainer.run/'maps'/f'{case["id"]}.json'
    if path.exists():assert read_json(path)==case
    else:atomic_json(path,case)
    deterministic=rehearsal or trainer.rng.random()<cfg['campaign']['targeted_mean_probability']
    job=make_job(case,'candidate',identifier,trainer.policy,not deterministic,cfg['training']['seed']+number+900001,environment=cfg['environment'])
    job['campaign_rehearsal']=rehearsal
    job['mean_calibration']=calibration
    job['noise_hold_seconds']=cfg['training']['rl_noise_hold_seconds' if state['critic_ready'] else 'warmup_noise_hold_seconds']
    return job

def note_outcome(trainer,row):
    recent=trainer.state.setdefault('campaign_recent',[])
    recent.append({'stage':trainer.state.get('campaign_stage',0),'group':row['group'],
                   'success':bool(row['success'] and row['safe']),'unsafe':not bool(row['safe']),'behavior_exploration':bool(row.get('behavior_exploration',False)),
                   'stochastic':bool(row['stochastic'])})
    trainer.state['campaign_recent']=recent[-1024:]
