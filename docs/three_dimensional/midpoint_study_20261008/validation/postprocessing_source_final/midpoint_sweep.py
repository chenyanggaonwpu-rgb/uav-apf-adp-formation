"""Midpoint-height experiment: immutable maps, resumable workers, paired analysis."""
from __future__ import annotations
import argparse, csv, hashlib, json, math, os, statistics, subprocess, time
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MATLAB=Path('C:/Program Files/MATLAB/R2024a/bin/matlab.exe')
RUNTIME=ROOT/'.runtime'/'adaptive_midpoint_20261008'
def read_csv(path):
    with Path(path).open(encoding='utf-8-sig',newline='') as f: return list(csv.DictReader(f))
def write_csv(path,rows,fieldnames=None):
    fields=fieldnames or (list(rows[0]) if rows else None)
    assert fields, 'CSV columns are required for an empty dataset.'
    with Path(path).open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
def write_json(path,data):
    Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def yes(value):return str(value).lower() in ('1','true')
def wilson(g,n):
    z=1.959963984540054;p=g/n;d=1+z*z/n;c=(p+z*z/(2*n))/d
    r=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/d
    return [max(0,c-r),min(1,c+r)]
def launch(name,code):
    directory=RUNTIME/name;directory.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env['MATLAB_PREFDIR']=str(directory/'prefs');Path(env['MATLAB_PREFDIR']).mkdir(exist_ok=True)
    log=(directory/'stdout.log').open('wb')
    info=subprocess.STARTUPINFO();info.dwFlags|=subprocess.STARTF_USESHOWWINDOW;info.wShowWindow=0
    p=subprocess.Popen([str(MATLAB),'-singleCompThread','-batch',code,'-logfile',str(directory/'matlab.log')],
        cwd=str(ROOT),env=env,stdout=log,stderr=subprocess.STDOUT,startupinfo=info,creationflags=subprocess.CREATE_NO_WINDOW)
    return p,log,directory

def run_single(name,code):
    p,log,directory=launch(name,code);started=time.monotonic();last=0
    try:
        while p.poll() is None:
            time.sleep(2)
            if time.monotonic()-started-last>=30:
                print(f'MATLAB {name} elapsed={time.monotonic()-started:.0f}s',flush=True);last=time.monotonic()-started
        if p.returncode:raise RuntimeError(f'{name} exited {p.returncode}; inspect {directory}')
    finally:log.close()
    print(f'MATLAB_COMPLETE {name}',flush=True)

def verify_sources(study):
    rows=read_csv(study/'source_sha256.csv')
    for r in rows:
        assert sha256(study/'source_snapshot'/r['file'])==r['sha256'],r['file']
    return len(rows)

def collect(study):
    allrows=[]
    for i in range(1,5):
        directory=study/'workers'/f'worker_{i:02d}'
        status=json.loads((directory/'status.json').read_text(encoding='utf-8-sig'))
        assert status['status']=='complete' and status['completed']==250
        rows=read_csv(directory/'trials.csv');assert len(rows)==250
        allrows+=rows
    allrows.sort(key=lambda r:int(r['seed']));assert [int(r['seed']) for r in allrows]==list(range(1,1001))
    write_csv(study/'adaptive_results.csv',allrows)
    return allrows

def run(study):
    verify_sources(study);procs=[];logs=[];started=time.monotonic();last=0
    for i in range(1,5):
        code=(f"restoredefaultpath; addpath('{(study/'source_snapshot'/'experiments3d').as_posix()}'); "
              f"run_midpoint_sweep3d('{study.as_posix()}',{i});")
        p,log,_=launch(f'worker_{i:02d}',code);procs.append(p);logs.append(log)
    try:
        while any(p.poll() is None for p in procs):
            time.sleep(2);elapsed=time.monotonic()-started
            if elapsed-last>=30:
                statuses=[]
                for path in (study/'workers').glob('worker_*/status.json'):
                    try:statuses.append(json.loads(path.read_text(encoding='utf-8-sig')))
                    except (OSError,json.JSONDecodeError):pass
                print(f'MIDPOINT_RUNNING elapsed={elapsed:.0f}s completed={sum(s["completed"] for s in statuses)}/1000 '
                      f'success={sum(s["success"] for s in statuses)} active={sum(p.poll() is None for p in procs)}',flush=True)
                last=elapsed
        codes=[p.wait() for p in procs]
        assert not any(codes),f'worker exit codes {codes}'
    finally:
        for log in logs:log.close()
    collect(study);print('ALL_1000_MIDPOINT_TRIALS_COMPLETE',flush=True)

def analyze(study):
    rows=collect(study);old=read_csv(study/'baseline_results.csv');manifest=read_csv(study/'map_manifest.csv')
    assert len(old)==1000 and sum(yes(r['success']) for r in old)==564
    oldby={int(r['seed']):r for r in old};maps={int(r['seed']):r for r in manifest}
    paired=[];families=[]
    for r in rows:
        seed=int(r['seed']);b=oldby[seed]
        assert r['mapHash']==maps[seed]['mapHash'] and yes(r['baselineSuccess'])==yes(b['success'])
        a= yes(b['success']);n=yes(r['success'])
        transition='retained_success' if a and n else 'rescued' if not a and n else 'lost_success' if a else 'both_failed'
        paired.append(dict(seed=seed,family=r['family'],baselineSuccess=int(a),adaptiveSuccess=int(n),transition=transition,
            baselineReason=b['reason'],adaptiveReason=r['reason'],addedHeights=r['addedHeights'],initialLayerCount=r['initialLayerCount']))
    for family in ['all','mixed','underpass','window','staggered_windows']:
        sample=rows if family=='all' else [r for r in rows if r['family']==family]
        bas=[oldby[int(r['seed'])] for r in sample];n=len(sample)
        g=sum(yes(r['success']) for r in sample);b=sum(yes(r['success']) for r in bas)
        lo,hi=wilson(g,n);blo,bhi=wilson(b,n)
        families.append(dict(family=family,n=n,baselineSuccess=b,adaptiveSuccess=g,baselinePercent=100*b/n,
            adaptivePercent=100*g/n,increasePercentagePoints=100*(g-b)/n,baselineLow=100*blo,baselineHigh=100*bhi,
            adaptiveLow=100*lo,adaptiveHigh=100*hi,initialPlanFound=sum(yes(r['initialPlanFound']) for r in sample)))
    counts=Counter(p['transition'] for p in paired);reasons=Counter(r['reason'] for r in rows)
    assert counts['retained_success']+counts['lost_success']==564
    assert counts['rescued']+counts['both_failed']==436
    errors=reasons['execution_error']
    assert errors==0,'Execution errors prevent a complete end-to-end rate conclusion.'
    records=dict(n=1000,baselineSuccess=564,adaptiveSuccess=sum(yes(r['success']) for r in rows),
        pairedTransitions=dict(counts),adaptiveOutcomes=dict(reasons),families=families,
        oldFailureCohort=dict(n=436,success=counts['rescued'],percent=100*counts['rescued']/436),
        adaptiveInitialPlanningSuccess=sum(yes(r['initialPlanFound']) for r in rows),
        layerDistribution=dict(Counter(r['initialLayerCount'] for r in rows)),
        insertionTrials=sum(int(r['addedCount'])>0 for r in rows),
        budgetRejectedTrials=sum(int(r['budgetRejected'])>0 for r in rows),
        minObstacleClearance=min(float(r['minObstacleClearance']) for r in rows if r['minObstacleClearance'] not in ('NaN','nan','')),
        minPairDistance=min(float(r['minPairDistance']) for r in rows if r['minPairDistance'] not in ('NaN','nan','')),
        maxInitialLayerCount=max(int(r['initialLayerCount']) for r in rows),maxPeakLayerCount=max(int(r['peakLayerCount']) for r in rows),
        medianInitialPlanSeconds=statistics.median(float(r['initialPlanSeconds']) for r in rows),
        sourceFilesVerified=verify_sources(study))
    write_csv(study/'paired_results.csv',paired);write_csv(study/'family_summary.csv',families)
    write_csv(study/'failures.csv',[r for r in rows if not yes(r['success'])],fieldnames=list(rows[0]))
    write_json(study/'summary.json',records)
    print(json.dumps(records,ensure_ascii=False,indent=2),flush=True)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('command',choices=['prepare','run','audit','analyze'])
    parser.add_argument('--study',type=Path);parser.add_argument('--source-commit');args=parser.parse_args()
    if args.command=='prepare':
        assert args.source_commit
        code=f"restoredefaultpath; addpath('{(ROOT/'experiments3d').as_posix()}'); prepare_midpoint_sweep3d('{args.source_commit}');"
        run_single('prepare',code)
        print((RUNTIME/'study_path.txt').read_text(encoding='utf-8-sig'),flush=True)
    else:
        study=args.study or Path((RUNTIME/'study_path.txt').read_text(encoding='utf-8-sig'))
        if args.command=='run':run(study)
        elif args.command=='analyze':analyze(study)
        elif args.command=='audit':
            verify_sources(study)
            repair_receipt=study/'audit_repair.json'
            if repair_receipt.exists():
                repair=json.loads(repair_receipt.read_text(encoding='utf-8-sig'))
                file=study/repair['repairFile']
                assert sha256(file)==repair['repairSha256']
                code=(f"restoredefaultpath; addpath('{file.parent.as_posix()}'); "
                      f"audit_midpoint_sweep_repair01_3d('{study.as_posix()}');")
                run_single('audit_repair01',code)
            else:
                code=(f"restoredefaultpath; addpath('{(study/'source_snapshot'/'experiments3d').as_posix()}'); "
                      f"audit_midpoint_sweep3d('{study.as_posix()}');")
                run_single('audit',code)
if __name__=='__main__':main()
