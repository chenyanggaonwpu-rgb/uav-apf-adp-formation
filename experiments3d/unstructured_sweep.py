"""Unconditioned random 3-D maps: frozen inputs, complete flights and raw audits."""
from __future__ import annotations
import argparse,csv,hashlib,json,math,os,statistics,subprocess,time
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MATLAB=Path('C:/Program Files/MATLAB/R2024a/bin/matlab.exe')
RUNTIME=ROOT/'.runtime'/'unstructured_random_20261008'

def read_csv(path):
    with Path(path).open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def write_csv(path,rows,fields=None):
    fields=fields or list(rows[0])
    with Path(path).open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
def write_json(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1048576),b''):h.update(chunk)
    return h.hexdigest()
def yes(value):return str(value).lower() in ('1','true')
def wilson(g,n):
    if not n:return [None,None]
    z=1.959963984540054;p=g/n;d=1+z*z/n;c=(p+z*z/(2*n))/d
    r=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/d;return [max(0,c-r),min(1,c+r)]
def launch(name,code):
    directory=RUNTIME/name;directory.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env['MATLAB_PREFDIR']=str(directory/'prefs');Path(env['MATLAB_PREFDIR']).mkdir(exist_ok=True)
    log=(directory/'stdout.log').open('wb')
    info=subprocess.STARTUPINFO();info.dwFlags|=subprocess.STARTF_USESHOWWINDOW;info.wShowWindow=0
    process=subprocess.Popen([str(MATLAB),'-singleCompThread','-batch',code,'-logfile',str(directory/'matlab.log')],
        cwd=str(ROOT),env=env,stdout=log,stderr=subprocess.STDOUT,startupinfo=info,creationflags=subprocess.CREATE_NO_WINDOW)
    return process,log,directory

def run_single(name,code):
    process,log,directory=launch(name,code);start=time.monotonic();last=0
    try:
        while process.poll() is None:
            time.sleep(2);elapsed=time.monotonic()-start
            if elapsed-last>=25:print(f'MATLAB {name} elapsed={elapsed:.0f}s',flush=True);last=elapsed
        assert process.returncode==0,f'{name} exited {process.returncode}; inspect {directory}'
    finally:log.close()
    print(f'MATLAB_COMPLETE {name}',flush=True)

def verify_sources(study):
    rows=read_csv(study/'source_sha256.csv')
    for row in rows:assert sha256(study/'source_snapshot'/row['file'])==row['sha256'],row['file']
    return len(rows)
def freeze_inputs(study):
    names=['protocol.mat','protocol.json','map_manifest.csv','obstacle_catalog.csv','source_sha256.csv']
    names += [r['mapFile'] for r in read_csv(study/'map_manifest.csv')]
    write_csv(study/'input_sha256.csv',[dict(file=name,sha256=sha256(study/name)) for name in names])
    write_json(study/'pre_run_freeze.json',dict(status='all_maps_frozen_before_trials',maps=200,seeds=[2001,2200],
        sourceFiles=verify_sources(study),inputFiles=len(names),freezeManifestSha256=sha256(study/'input_sha256.csv'),
        noTrialsYet=not any((study/'workers').glob('worker_*/trials/*.mat'))))
    assert json.loads((study/'pre_run_freeze.json').read_text())['noTrialsYet']
def verify_inputs(study):
    for row in read_csv(study/'input_sha256.csv'):assert sha256(study/row['file'])==row['sha256'],row['file']
    return verify_sources(study)
def collect(study):
    rows=[]
    protocol=json.loads((study/'protocol.json').read_text(encoding='utf-8-sig'))
    for worker,(first,last) in enumerate(protocol['ranges'],1):
        directory=study/'workers'/f'worker_{worker:02d}'
        status=json.loads((directory/'status.json').read_text(encoding='utf-8-sig'))
        assert status['status']=='complete' and status['completed']==last-first+1
        trial=read_csv(directory/'trials.csv');assert len(trial)==last-first+1;rows+=trial
    rows.sort(key=lambda r:int(r['seed']))
    assert [int(r['seed']) for r in rows]==list(range(2001,2201))
    write_csv(study/'all_results.csv',rows);return rows

def run(study):
    verify_inputs(study);processes=[];logs=[];start=time.monotonic();last=0
    for worker in range(1,5):
        code=f"restoredefaultpath;addpath('{(study/'source_snapshot'/'experiments3d').as_posix()}');run_unstructured_sweep3d('{study.as_posix()}',{worker});"
        process,log,_=launch(f'worker_{worker:02d}',code);processes.append(process);logs.append(log)
    try:
        while any(p.poll() is None for p in processes):
            time.sleep(2);elapsed=time.monotonic()-start
            if elapsed-last>=25:
                statuses=[]
                for path in (study/'workers').glob('worker_*/status.json'):
                    try:statuses.append(json.loads(path.read_text(encoding='utf-8-sig')))
                    except (OSError,json.JSONDecodeError):pass
                print(f'UNSTRUCTURED_RUNNING elapsed={elapsed:.0f}s completed={sum(s["completed"] for s in statuses)}/200 '
                    f'success={sum(s["success"] for s in statuses)} errors={sum(s["executionErrors"] for s in statuses)} '
                    f'active={sum(p.poll() is None for p in processes)}',flush=True);last=elapsed
        exits=[p.wait() for p in processes];assert not any(exits),f'Worker exit codes {exits}'
    finally:
        for log in logs:log.close()
    rows=collect(study);verify_inputs(study)
    print(f'ALL_200_UNSTRUCTURED_TRIALS_COMPLETE success={sum(yes(r["success"]) for r in rows)}',flush=True)

def cohort(name,sample):
    n=len(sample);g=sum(yes(r['success']) for r in sample)
    return dict(cohort=name,n=n,success=g,percent=100*g/n if n else None,
        wilson95=wilson(g,n),initialPlans=sum(yes(r['initialPlanFound']) for r in sample),outcomes=dict(Counter(r['reason'] for r in sample)))
def finite(rows,column):
    values=[]
    for row in rows:
        try:value=float(row[column])
        except (ValueError,KeyError):continue
        if math.isfinite(value):values.append(value)
    return values

def analyze(study):
    verify_inputs(study);rows=collect(study);audit=json.loads((study/'raw_mat_audit.json').read_text(encoding='utf-8-sig'))
    assert audit['passed'] and audit['records']==200 and audit['success']==sum(yes(r['success']) for r in rows)
    reasons=Counter(r['reason'] for r in rows);assert reasons['execution_error']==0,'Execution errors invalidate a completed rate.'
    initially_safe=[r for r in rows if yes(r['initialHardSafe'])]
    safe_endpoints=[r for r in initially_safe if yes(r['leaderEndpointsPlanClear'])]
    flights=[r for r in rows if yes(r['initialPlanFound'])]
    samples=[cohort('all_200',rows),cohort('initial_formation_hard_safe',initially_safe),
        cohort('initial_safe_and_leader_endpoints_4p2m',safe_endpoints),cohort('flight_started',flights)]
    bins=[]
    for first,last in [(1,10),(11,20),(21,30),(31,40)]:
        sample=[r for r in rows if first<=int(r['obstacleCount'])<=last]
        bins.append(cohort(f'{first}-{last}',sample))
    write_csv(study/'success_by_count.csv',[dict(countRange=b['cohort'],n=b['n'],success=b['success'],percent=b['percent'],
        wilsonLow=b['wilson95'][0],wilsonHigh=b['wilson95'][1],initialPlans=b['initialPlans']) for b in bins])
    catalog=read_csv(study/'obstacle_catalog.csv');counts=finite(rows,'obstacleCount');density=finite(rows,'nominalVolumeFraction')
    insertion=sum(int(r['addedCount'])>0 for r in rows)
    summary=dict(n=200,seeds=[2001,2200],algorithmVersion='2.3.1',cohorts=samples,outcomes=dict(reasons),
        overallSuccess=samples[0]['success'],overallPercent=samples[0]['percent'],wilson95=samples[0]['wilson95'],
        obstacleTotal=len(catalog),types=dict(Counter(r['type'] for r in catalog)),
        observedCount=dict(min=min(counts),max=max(counts),mean=statistics.mean(counts)),
        nominalVolumeFraction=dict(min=min(density),max=max(density),mean=statistics.mean(density),
            interpretation='Sum of individual volumes / workspace volume; overlaps are double-counted, not union occupancy.'),
        initialPlanFound=len(flights),adaptiveAttempted=sum(yes(r['adaptiveAttempted']) for r in rows),
        insertionTrials=insertion,maxInitialLayers=max(int(r['initialLayerCount']) for r in rows),
        maxPeakLayers=max(int(r['peakLayerCount']) for r in rows),budgetRejectedTrials=sum(int(r['budgetRejected'])>0 for r in rows),
        audited=True,sourceFiles=verify_sources(study),countBins=bins)
    write_json(study/'summary.json',summary)
    write_csv(study/'failures.csv',[r for r in rows if not yes(r['success'])],list(rows[0]))
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('command',choices=['test','prepare','run','audit','analyze'])
    parser.add_argument('--study',type=Path);parser.add_argument('--source-commit');args=parser.parse_args()
    if args.command=='test':
        code=f"restoredefaultpath;addpath('{(ROOT/'experiments3d').as_posix()}');test_unstructured_scene3d;"
        run_single('generator_test',code);return
    if args.command=='prepare':
        assert args.source_commit
        code=f"restoredefaultpath;addpath('{(ROOT/'experiments3d').as_posix()}');prepare_unstructured_sweep3d('{args.source_commit}');"
        run_single('prepare',code)
        study=Path((RUNTIME/'study_path.txt').read_text(encoding='utf-8-sig'));freeze_inputs(study)
        print(f'STUDY_FROZEN={study}',flush=True);return
    study=args.study or Path((RUNTIME/'study_path.txt').read_text(encoding='utf-8-sig'))
    if args.command=='run':run(study)
    elif args.command=='analyze':analyze(study)
    else:
        verify_inputs(study)
        code=f"restoredefaultpath;addpath('{(study/'source_snapshot'/'experiments3d').as_posix()}');audit_unstructured_sweep3d('{study.as_posix()}');"
        run_single('audit',code);verify_inputs(study)
if __name__=='__main__':main()
