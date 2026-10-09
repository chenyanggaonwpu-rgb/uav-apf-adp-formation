"""Frozen-map clearance experiment orchestration; Python standard library only.

Commands: screen, refine, nominal, robust, select, holdout, boundary, geometry,
ablation, timing, verify. Each stage freezes its exact job CSV before launching MATLAB.
"""
from __future__ import annotations
import argparse, csv, hashlib, itertools, json, math, os, subprocess, time
from collections import Counter, defaultdict
from pathlib import Path

MATLAB = Path('C:/Program Files/MATLAB/R2024a/bin/matlab.exe')
FIELDS = ['jobId','stage','configId','seed','enabled','pathXY','refXY','planZ','refZ',
          'noiseStd','disturbanceMax','disturbanceHoldSteps','repeat','stochasticSeed','layerPolicy','mapFile']
ROOT = Path(__file__).resolve().parents[1]

def read_csv(path):
    with Path(path).open(encoding='utf-8-sig',newline='') as f: return list(csv.DictReader(f))
def write_csv(path,rows,fields):
    with Path(path).open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(rows)
def write_json(path,data):
    Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''): h.update(block)
    return h.hexdigest()
def yes(row): return row['success'] in ('1','true','True')
def finite_values(rows,field):
    result=[]
    for r in rows:
        try: x=float(r[field])
        except (KeyError,ValueError): continue
        if math.isfinite(x): result.append(x)
    return result
def wilson(good,n):
    if not n: return [None,None]
    z=1.959963984540054; p=good/n; scale=1+z*z/n
    center=(p+z*z/(2*n))/scale
    radius=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/scale
    return [max(0,center-radius),min(1,center+radius)]

def baseline():
    return dict(configId='baseline',enabled=0,pathXY=4.2,refXY=4.0,planZ=4.2,refZ=4.0)
def config(name,h,d,z):
    return dict(configId=name,enabled=1,pathXY=round(h,6),refXY=round(d,6),planZ=round(z,6),refZ=round(z,6))
def configs_from_rows(rows):
    found={}
    for r in rows:
        found[r['configId']]={k:(r[k] if k=='configId' else int(r[k]) if k=='enabled' else float(r[k]))
                              for k in ('configId','enabled','pathXY','refXY','planZ','refZ')}
    return found
def metrics(rows):
    good=sum(yes(r) for r in rows); outcomes=dict(Counter(r['reason'] for r in rows))
    families={}
    for family in sorted({r['family'] for r in rows}):
        sub=[r for r in rows if r['family']==family]; g=sum(yes(r) for r in sub)
        families[family]=dict(n=len(sub),success=g,rate=g/len(sub),wilson95=wilson(g,len(sub)))
    result=dict(n=len(rows),success=good,rate=good/len(rows),wilson95=wilson(good,len(rows)),
                outcomes=outcomes,families=families,worstFamily=min(x['rate'] for x in families.values()))
    successful=[r for r in rows if yes(r)]
    for field in ('minObstacleClearance','minPairDistance','followerHeightMax','formationRms',
                  'path','verticalTravel','wallSeconds','cycleP99ms','initialPlanSeconds'):
        v=finite_values(successful,field)
        result[field]=dict(mean=sum(v)/len(v),min=min(v),max=max(v)) if v else None
    return result

def collect(study,stage):
    jobs=read_csv(study/f'{stage}.csv'); rows=[]
    for path in sorted((study/'stages'/stage).glob('worker_*/trials.csv')): rows+=read_csv(path)
    lookup={r['jobId']:r for r in rows}
    if len(lookup)!=len(rows): raise AssertionError('Duplicate job result')
    assert set(lookup)=={j['jobId'] for j in jobs}, (stage,len(lookup),len(jobs))
    rows=[lookup[j['jobId']] for j in jobs]
    maps={r['mapFile'].replace('\\','/'):r for r in read_csv(study/'map_manifest.csv')}
    for job,row in zip(jobs,rows):
        assert row['seed']==job['seed'] and row['configId']==job['configId']
        assert row['mapHash']==maps[job['mapFile'].replace('\\','/')]['mapHash']
        for k in ('pathXY','refXY','planZ','refZ','noiseStd','disturbanceMax','stochasticSeed'):
            assert float(row[k])==float(job[k]), (row['jobId'],k)
    write_csv(study/f'{stage}_results.csv',rows,list(rows[0]))
    groups=defaultdict(list)
    for row in rows: groups[row['configId']].append(row)
    summary={c:metrics(rs) for c,rs in groups.items()}
    write_json(study/f'{stage}_summary.json',summary)
    print(f'STAGE_COLLECTED {stage} jobs={len(rows)} success={sum(yes(r) for r in rows)}',flush=True)
    return rows,summary

def rank_configs(rows):
    grouped=defaultdict(list)
    for r in rows:
        if r['configId']!='baseline': grouped[r['configId']].append(r)
    candidates=[]
    for cid,rs in grouped.items():
        m=metrics(rs)
        if any(m['outcomes'].get(x,0) for x in ('execution_error','collision','invalid_state','invalid_scene')):
            continue
        clearance=m['minObstacleClearance']['min'] if m['minObstacleClearance'] else 0
        candidates.append((m['rate'],m['worstFamily'],clearance,cid))
    return [c[-1] for c in sorted(candidates,reverse=True)]
def separated(ids,configs,count):
    selected=[]
    for cid in ids:
        q=configs[cid]
        if all(math.sqrt(sum(((q[k]-configs[old][k])/scale)**2 for k,scale in
                             [('pathXY',5),('refXY',3.4),('planZ',3.6)]))>=0.28 for old in selected):
            selected.append(cid)
        if len(selected)==count: break
    return selected

def make_jobs(study,stage,configs,seeds,conditions=None,policies=('fixed5',)):
    conditions=conditions or [(0,0,1,0)]
    maps={int(r['seed']):r for r in read_csv(study/'map_manifest.csv')}
    jobs=[]
    # Group equal planning margins so the bounded worker graph cache can be
    # reused across reference settings without retaining a large map library.
    configs=sorted(configs,key=lambda c:(c['enabled'],c['pathXY'],c['planZ'],c['refXY']))
    for c,(noise,dist,hold,rep),policy,seed in itertools.product(configs,conditions,policies,seeds):
        job=dict(c,stage=stage,seed=seed,noiseStd=noise,disturbanceMax=dist,
                 disturbanceHoldSteps=hold,repeat=rep,stochasticSeed=seed+100000*rep,
                 layerPolicy=policy,mapFile=maps[seed]['mapFile'])
        job['jobId']=f'{stage}_{c["configId"]}_s{seed:06d}_n{noise:g}_a{dist:g}_r{rep}_{policy}'
        jobs.append(job)
    path=study/f'{stage}.csv'
    if path.exists(): assert read_csv(path)==[{k:str(j[k]) for k in FIELDS} for j in jobs], 'Existing job protocol differs'
    else: write_csv(path,jobs,FIELDS)
    protocol_path=study/f'{stage}_protocol.json'
    if not protocol_path.exists(): write_json(protocol_path,dict(stage=stage,scheduled=len(jobs),
                jobCsvSha256=sha256(path),createdLocal=time.strftime('%Y-%m-%d %H:%M:%S'),configs=configs,
                seeds=list(seeds),conditions=conditions,policies=policies))
    return path

def run_workers(study,jobs_path,n_workers):
    stage=jobs_path.stem; procs=[]; handles=[]
    for index in range(1,n_workers+1):
        runtime=ROOT/'.runtime'/'margin_study_20261006'/stage/f'worker_{index:02d}'
        runtime.mkdir(parents=True,exist_ok=True)
        env=os.environ.copy(); env['MATLAB_PREFDIR']=str(runtime/'prefs')
        Path(env['MATLAB_PREFDIR']).mkdir(exist_ok=True)
        code=(f"addpath('{(study/'source_snapshot'/'experiments3d').as_posix()}'); "
              f"run_clearance_jobs3d('{study.as_posix()}','{jobs_path.as_posix()}',{index},{n_workers});")
        out=(runtime/'stdout.log').open('wb'); handles.append(out)
        info=subprocess.STARTUPINFO(); info.dwFlags|=subprocess.STARTF_USESHOWWINDOW; info.wShowWindow=0
        procs.append(subprocess.Popen([str(MATLAB),'-singleCompThread','-batch',code,'-logfile',str(runtime/'matlab.log')],
            cwd=str(ROOT),env=env,stdout=out,stderr=subprocess.STDOUT,startupinfo=info,
            creationflags=subprocess.CREATE_NO_WINDOW))
    started=time.monotonic(); last=0
    try:
        while any(p.poll() is None for p in procs):
            time.sleep(2)
            elapsed=time.monotonic()-started
            if elapsed-last>=30:
                statuses=[]
                for path in (study/'stages'/stage).glob('worker_*/status.json'):
                    try: statuses.append(json.loads(path.read_text(encoding='utf-8-sig')))
                    except (OSError,json.JSONDecodeError): pass
                print(f'RUNNING {stage} elapsed={elapsed:.0f}s completed={sum(s["completed"] for s in statuses)} '
                      f'success={sum(s["success"] for s in statuses)} active={sum(p.poll() is None for p in procs)}',flush=True)
                last=elapsed
        codes=[p.wait() for p in procs]
        if any(codes): raise RuntimeError(f'MATLAB workers exited {codes}; see runtime logs')
    finally:
        for h in handles: h.close()
    return collect(study,stage)

def create_stage(study,stage):
    protocol=json.loads((study/'protocol.json').read_text(encoding='utf-8-sig'))
    if stage=='screen':
        cs=[baseline()]
        for i,(h,z,d) in enumerate(itertools.product(protocol['pathXYLevels'],protocol['zLevels'],protocol['refXYLevels']),1):
            cs.append(config(f'c{i:03d}',h,d,z))
        return make_jobs(study,stage,cs,protocol['screenSeeds'])
    if stage=='refine':
        rows,_=collect(study,'screen'); configs=configs_from_rows(rows)
        centers=separated(rank_configs(rows),configs,3); assert len(centers)>=2
        cs=[baseline()]; seen=set(); index=0
        for cid in centers:
            q=configs[cid]; h,d,z=q['pathXY'],q['refXY'],q['planZ']
            values=[(h,d,z),(max(.61,h-.5),d,z),(min(6,h+.5),d,z),
                    (h,max(.61,d-.4),z),(h,min(4.5,d+.4),z),
                    (h,d,max(.61,z-.25)),(h,d,min(4.2,z+.25))]
            for triple in values:
                if triple in seen: continue
                seen.add(triple); index+=1; cs.append(config(f'f{index:03d}',*triple))
        write_json(study/'refinement_decision.json',dict(screenCenters=centers,reason='Three separated coarse regions, with axial neighbours; no holdout inspection'))
        return make_jobs(study,stage,cs,protocol['refinementSeeds'])
    if stage=='nominal':
        rows,_=collect(study,'refine'); configs=configs_from_rows(rows)
        ids=separated(rank_configs(rows),configs,2); assert len(ids)==2
        write_json(study/'nominal_decision.json',dict(configs=[configs[i] for i in ids],reason='Two separated refined candidates; existing 1000 maps only'))
        return make_jobs(study,stage,[configs[i] for i in ids],protocol['nominalSeeds'])
    if stage=='robust':
        chosen=json.loads((study/'nominal_decision.json').read_text())['configs']
        conditions=[(0,0,1,0),(.02,0,1,1),(.05,0,1,1),(.1,0,1,1),
                    (0,.1,20,1),(0,.2,20,1),(.05,.2,20,1)]
        return make_jobs(study,stage,[baseline()]+chosen,protocol['robustnessSeeds'],conditions)
    if stage=='select':
        nominal,_=collect(study,'nominal'); robust,_=collect(study,'robust')
        configs=configs_from_rows(nominal); scores=[]
        for cid,q in configs.items():
            nr=[r for r in nominal if r['configId']==cid]; rr=[r for r in robust if r['configId']==cid]
            nm=metrics(nr); rm=metrics(rr); unsafe=sum(rm['outcomes'].get(x,0) for x in ('collision','invalid_state','execution_error'))
            scores.append((unsafe, -rm['rate'],-nm['rate'],-nm['worstFamily'],-q['planZ'],cid))
        winner=min(scores)[-1]
        write_json(study/'selection.json',dict(config=configs[winner],scores=scores,
                   rule='Fewest observed collisions/invalid/error cases, then robustness rate, nominal rate, worst family, larger Z margin',
                   selectedLocal=time.strftime('%Y-%m-%d %H:%M:%S'),holdoutInspected=False))
        print('FINAL_CANDIDATE '+json.dumps(configs[winner]),flush=True); return None
    final=json.loads((study/'selection.json').read_text(encoding='utf-8'))['config']
    if stage=='holdout': return make_jobs(study,stage,[baseline(),final],protocol['holdoutSeeds'])
    if stage=='ablation':
        z=final['planZ']
        plan=config('plan_vertical_only',4.2,4,z); plan['refZ']=4.0
        reference=config('reference_vertical_only',4.2,4,z); reference['planZ']=4.2
        both=config('both_vertical',4.2,4,z)
        horizontal=config('horizontal_only',final['pathXY'],final['refXY'],4.2); horizontal['refZ']=4.0
        return make_jobs(study,stage,[baseline(),plan,reference,both,horizontal,final],list(range(1,41)))
    if stage=='boundary':
        seeds=list(range(81,89)); cs=[baseline(),final]; conditions=[]
        for rep in (1,2):
            conditions += [(n,0,1,rep) for n in protocol['noiseBoundary']]
            conditions += [(0,a,20,rep) for a in protocol['disturbanceBoundary'] if a!=0]
        return make_jobs(study,stage,cs,seeds,conditions)
    if stage=='geometry':
        seeds=[int(r['seed']) for r in read_csv(study/'map_manifest.csv') if int(r['seed'])>900000]
        cs=[baseline(),final,config('floor_margin',.61,.61,.61)]
        return make_jobs(study,stage,cs,seeds,policies=('fixed5','add_window_center'))
    if stage=='timing': return make_jobs(study,stage,[baseline(),final],list(range(121,133)))
    raise ValueError(stage)

def verify(study):
    for row in read_csv(study/'source_sha256.csv'):
        assert sha256(study/'source_snapshot'/row['relativePath'].replace('\\','/'))==row['sha256']
    records=[]
    for path in sorted((study/'stages').glob('*/worker_*/trials/*.mat')):
        records.append(dict(relativePath=path.relative_to(study).as_posix(),bytes=path.stat().st_size,sha256=sha256(path)))
    write_csv(study/'raw_mat_sha256.csv',records,['relativePath','bytes','sha256'])
    write_json(study/'file_integrity.json',dict(rawMatCount=len(records),sourceHashesMatch=True,checkedLocal=time.strftime('%Y-%m-%d %H:%M:%S')))
    print(f'FILE_INTEGRITY_PASSED rawMatCount={len(records)}',flush=True)

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('stage'); parser.add_argument('study',type=Path)
    parser.add_argument('--workers',type=int,default=4); args=parser.parse_args()
    study=args.study.resolve()
    if args.stage=='verify': verify(study); return
    path=create_stage(study,args.stage)
    if path: run_workers(study,path,args.workers)
if __name__=='__main__': main()
