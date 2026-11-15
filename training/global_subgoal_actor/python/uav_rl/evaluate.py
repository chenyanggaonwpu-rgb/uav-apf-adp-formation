from pathlib import Path
import collections,time
import numpy as np
from .io import ROOT,read_json,atomic_json,sha256,stamp
from .queue import Queue,make_job


def score(rows):
    families=collections.defaultdict(list)
    for r in rows:families[r['group']].append(r)
    return {'tasks':len(rows),'safe_success':sum(r['safe'] and r['success'] for r in rows),'unsafe':sum(not r['safe'] for r in rows),
        'critical':sum(r['critical'] for r in rows),'triggered':sum(r['activations']>0 for r in rows),
        'mean_leader_path_meters':float(np.mean([r.get('leader_path_meters',0) for r in rows])) if rows else None,
        'mean_fleet_path_meters':float(np.mean([r.get('fleet_path_meters',0) for r in rows])) if rows else None,
        'mean_stagnant_seconds':float(np.mean([r.get('stagnant_seconds',0) for r in rows])) if rows else None,
        'mean_revisit_seconds':float(np.mean([r.get('revisit_seconds',0) for r in rows])) if rows else None,
        'failure_counted_120_mean':float(np.mean([r['flight_seconds'] if r['success'] and r['safe'] else 120 for r in rows])) if rows else None,
        'by_family':{g:{'tasks':len(v),'safe_success':sum(r['safe'] and r['success'] for r in v),'unsafe':sum(not r['safe'] for r in v)} for g,v in families.items()}}


def evaluate(cases,models,destination,workroot,cfg,on_progress=None):
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    q=Queue(destination/'queue',workroot,cfg);q.recover();jobs=[]
    for i,c in enumerate(cases):
        for arm,model in models.items():
            j=make_job(c,arm,f'e{i:05d}_{arm}',model,False,c['stochastic_seed']+11,environment=cfg['environment']);jobs.append(j);q.submit(j)
    if not all(q.ready(j['id']) for j in jobs):
        q.start();started=time.monotonic()
        try:
            while True:
                q.check(jobs);done=sum(q.ready(j['id']) for j in jobs)
                if on_progress:on_progress({'phase':'evaluation','evaluation':destination.name,'completed':done,'total':len(jobs),'elapsed_seconds':round(time.monotonic()-started,1)})
                if done==len(jobs):break
                time.sleep(1)
        finally:q.stop(force=not all(q.ready(j['id']) for j in jobs))
    rows=[read_json(str(q.result(j['id']))+'.json') for j in jobs]
    assert all(r['audit_passed'] and r['teacher_queries']==0 and not r['stochastic'] for r in rows)
    for j,r in zip(jobs,rows):
        if j.get('model_sha256'):assert j['model_sha256']==r['model_sha256']
    summary={a:score([r for r in rows if r['arm']==a]) for a in models}
    atomic_json(destination/'results.json',{'timestamp':stamp(),'maps':len(cases),'summary':summary,'rows':rows,
        'model_hashes':{a:sha256(p) if p else None for a,p in models.items()},'teacher_queries':0})
    return rows,summary


def baselines():return {'global_rule':None,'apf':None,'rule':None,'v2':ROOT/'assets/v2_numeric.mat','v4_anchor':ROOT/'assets/v4_initial.mat'}


def admissible(candidate,anchor,cfg,full=True):
    a={r['case_id']:r for r in candidate};b={r['case_id']:r for r in anchor};assert set(a)==set(b)
    new_critical=[k for k in a if a[k]['critical'] and not b[k]['critical']]
    current,old=score(candidate),score(anchor);problems=[]
    if new_critical:problems.append('new_critical_collisions')
    if current['unsafe']>old['unsafe']:problems.append('more_unsafe_tasks')
    tolerance=0 if full else 2
    if current['safe_success']<old['safe_success']-tolerance:problems.append('fewer_safe_full_missions')
    if full:
        for g,x in current['by_family'].items():
            y=old['by_family'][g]
            if (y['safe_success']-x['safe_success'])/x['tasks']>cfg['validation']['maximum_family_regression']+1e-12:problems.append('family_regression_'+g)
    improved=not problems and (current['safe_success']>old['safe_success'] or
        (current['safe_success']==old['safe_success'] and current['safe_success']>0 and current['failure_counted_120_mean']<old['failure_counted_120_mean']-.01))
    return {'admissible':not problems,'improved':improved,'problems':problems,'new_critical_cases':new_critical,'candidate':current,'reference':old}
