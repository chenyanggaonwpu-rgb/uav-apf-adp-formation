"""Paired 7 x 7 margin scan on the exact 200 previously frozen maps."""
from __future__ import annotations
import argparse,csv,hashlib,json,math,os,shutil,statistics,subprocess,time,zipfile
from collections import Counter
from datetime import datetime
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
MATLAB=Path('C:/Program Files/MATLAB/R2024a/bin/matlab.exe')
BASE=ROOT/'results/unstructured_random3d_v2.3.1_20261008_172031_791'
RUNTIME=ROOT/'.runtime/margin_grid49_20261008'
PATHS=[.8,1.2,1.8,2.5,3.5,4.2,5.6]
REFS=[.8,1.2,1.8,2.5,3.5,4.,4.4]

def read_csv(path):
    with Path(path).open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def write_csv(path,rows,fields=None):
    with Path(path).open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields or list(rows[0]));w.writeheader();w.writerows(rows)
def write_json(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1048576),b''):h.update(chunk)
    return h.hexdigest()
def yes(value):return str(value).lower() in ('1','true')
def number(value):
    try:v=float(value);return v if math.isfinite(v) else None
    except (ValueError,TypeError):return None
def mean(values):
    v=[x for x in values if x is not None];return statistics.mean(v) if v else None
def wilson(g,n):
    z=1.959963984540054;p=g/n;d=1+z*z/n;c=(p+z*z/(2*n))/d
    r=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/d;return [100*max(0,c-r),100*min(1,c+r)]
def git(*args):
    return subprocess.check_output(['git','-c',f'safe.directory={ROOT.as_posix()}',*args],cwd=ROOT,text=True,encoding='utf-8').strip()

def prepare(commit,workers):
    assert git('rev-parse','HEAD')==commit and not git('status','--porcelain')
    study=ROOT/'results'/f'margin_grid49_v2.3.1_{datetime.now():%Y%m%d_%H%M%S_%f}'
    study.mkdir();(study/'workers').mkdir();(study/'validation').mkdir();(study/'figures').mkdir()
    snap=study/'source_snapshot';snap.mkdir()
    for folder in ['src3d','config']:
        (snap/folder).mkdir()
        files=(ROOT/folder).glob('*.m') if folder=='src3d' else (ROOT/folder).glob('default_params*.m')
        for f in files:shutil.copy2(f,snap/folder/f.name)
    (snap/'experiments3d').mkdir()
    for name in ['run_margin_grid49_3d.m','audit_margin_grid49_worker3d.m','validate_margin_cache3d.m',
                 'plot_margin_grid49_3d.m','clearance_trial_record3d.m','midpoint_sha256_file3d.m','margin_grid49.py']:
        shutil.copy2(ROOT/'experiments3d'/name,snap/'experiments3d'/name)
    for r in read_csv(BASE/'source_sha256.csv'):
        if r['file'].startswith(('src3d/','config/')):assert digest(snap/r['file'])==r['sha256'],r['file']
    shutil.copytree(BASE/'maps',study/'maps')
    for name in ['map_manifest.csv','obstacle_catalog.csv']:
        shutil.copy2(BASE/name,study/name)
    shutil.copy2(BASE/'protocol.mat',study/'baseline_protocol.mat')
    source=[dict(file=f.relative_to(snap).as_posix(),sha256=digest(f)) for f in sorted(snap.rglob('*')) if f.is_file()]
    write_csv(study/'source_sha256.csv',source)
    maps=read_csv(study/'map_manifest.csv');assert len(maps)==200
    for r in maps:assert digest(study/r['mapFile'])==r['mapMatSha256']
    configs=[dict(configId=f'p{a:02d}_r{b:02d}',pathIndex=a,referenceIndex=b,pathClearance=x,desiredClear=y)
             for a,x in enumerate(PATHS,1) for b,y in enumerate(REFS,1)]
    write_csv(study/'configurations.csv',configs)
    jobs=[dict(jobId=f'{c["configId"]}_s{int(m["seed"]):06d}',configId=c['configId'],seed=int(m['seed']),
               pathClearance=c['pathClearance'],desiredClear=c['desiredClear'],mapHash=m['mapHash'],
               worker=(int(m['seed'])-2001)%workers+1) for m in maps for c in configs]
    write_csv(study/'jobs.csv',jobs);assert len(jobs)==9800
    protocol=dict(experiment='margin_grid49',algorithmVersion='2.3.1',sourceCommit=commit,
        baseStudy=BASE.as_posix(),baseCommit='f7787800e48b25c7e129203f8aead7b1ab6a1dba',
        pathValues=PATHS,referenceValues=REFS,nMaps=200,seeds=[2001,2200],nConfigurations=49,
        scheduledTasks=9800,nWorkers=workers,initialHardSafeMaps=sum(yes(m['initialHardSafe']) for m in maps),
        mapRule='Byte-identical copies of all 200 frozen maps; no regeneration, filtering or replacement.',
        parameterRule='Only pathClearance and desiredClear vary; isotropic mode; directional fields mirror those values but remain disabled.',
        heightRule='Unchanged 2.3.1 adaptive midpoint policy; base five layers; at most eight additions.',
        stochasticRule='Noise and disturbance remain zero; each trial resets ADP weights and RNG.',
        cacheRule='Prepare initial graph once for each map/planning margin, before any flight; never reuse a final flight cache.',
        timingRule='Concurrent single-compute-thread MATLAB processes and shared initial graphs; these timings are not isolated planning or hard-real-time benchmarks.',
        metricRule='Success means the complete original flight/hold criterion; primary denominator 200 and fixed initial-safe denominator 157.',
        plotRule='Show all 49 measured points at their physical numeric coordinates; linear facets only, no fitted smooth surface; intermediate parameters untested.',
        selectionRule='Exploratory grid maximum and all ties; no claim of continuous optimum or independent validation.',
        createdLocal=datetime.now().isoformat(timespec='seconds'))
    write_json(study/'protocol.json',protocol)
    names=['protocol.json','baseline_protocol.mat','map_manifest.csv','obstacle_catalog.csv','configurations.csv','jobs.csv','source_sha256.csv']
    names += [r['mapFile'] for r in maps]
    write_csv(study/'input_sha256.csv',[dict(file=n,sha256=digest(study/n)) for n in names])
    write_json(study/'pre_run_freeze.json',dict(noTrialsYet=True,maps=200,tasks=9800,sourceFiles=len(source),
        inputFiles=len(names),manifestSha256=digest(study/'input_sha256.csv'),sourceCommit=commit))
    RUNTIME.mkdir(parents=True,exist_ok=True);(RUNTIME/'study_path.txt').write_text(str(study),encoding='utf-8')
    print(f'MARGIN_STUDY_FROZEN={study}',flush=True);return study

def verify(study):
    for r in read_csv(study/'input_sha256.csv'):assert digest(study/r['file'])==r['sha256'],r['file']
    for r in read_csv(study/'source_sha256.csv'):assert digest(study/'source_snapshot'/r['file'])==r['sha256'],r['file']

def launch(study,name,code):
    directory=RUNTIME/study.name/name;directory.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy()
    for variable,folder in [('MATLAB_PREFDIR','prefs'),('TEMP','temp'),('TMP','temp'),('MCR_CACHE_ROOT','mcr')]:
        p=directory/folder;p.mkdir(exist_ok=True);env[variable]=str(p)
    log=(directory/'stdout.log').open('wb');info=subprocess.STARTUPINFO()
    info.dwFlags|=subprocess.STARTF_USESHOWWINDOW;info.wShowWindow=0
    process=subprocess.Popen([str(MATLAB),'-singleCompThread','-batch',code,'-logfile',str(directory/'matlab.log')],
        cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,startupinfo=info,creationflags=subprocess.CREATE_NO_WINDOW)
    return process,log

def single(study,name,call):
    verify(study)
    code=f"restoredefaultpath;addpath('{(study/'source_snapshot/experiments3d').as_posix()}');{call};"
    p,log=launch(study,name,code);last=time.monotonic();start=last
    try:
        while p.poll() is None:
            time.sleep(2)
            if time.monotonic()-last>=25:
                print(f'MARGIN_{name.upper()} elapsed={time.monotonic()-start:.0f}s',flush=True);last=time.monotonic()
        assert p.returncode==0,f'{name} exited {p.returncode}; see {RUNTIME/study.name/name}'
    finally:log.close()
    verify(study);print(f'MARGIN_{name.upper()}_COMPLETE',flush=True)

def run(study):
    verify(study);assert json.loads((study/'validation/cache_equivalence.json').read_text())['passed']
    protocol=json.loads((study/'protocol.json').read_text());processes=[];logs=[];start=time.monotonic();last=start
    for worker in range(1,protocol['nWorkers']+1):
        code=f"restoredefaultpath;addpath('{(study/'source_snapshot/experiments3d').as_posix()}');run_margin_grid49_3d('{study.as_posix()}',{worker});"
        p,log=launch(study,f'worker_{worker:02d}',code);processes.append(p);logs.append(log)
    try:
        while any(p.poll() is None for p in processes):
            time.sleep(2)
            if time.monotonic()-last>=25:
                states=[]
                for f in (study/'workers').glob('worker_*/status.json'):
                    try:states.append(json.loads(f.read_text(encoding='utf-8-sig')))
                    except (OSError,json.JSONDecodeError):pass
                print(f'MARGIN_GRID_RUNNING elapsed={time.monotonic()-start:.0f}s completed={sum(s["completed"] for s in states)}/9800 '
                      f'success={sum(s["success"] for s in states)} errors={sum(s["executionErrors"] for s in states)} '
                      f'active={sum(p.poll() is None for p in processes)}',flush=True);last=time.monotonic()
        assert all(p.returncode==0 for p in processes),[p.returncode for p in processes]
    finally:
        for log in logs:log.close()
    verify(study);collect(study);print('ALL_9800_MARGIN_TASKS_AND_RAW_AUDITS_COMPLETE',flush=True)

def collect(study):
    rows=[];audit=[];baselines=0
    p=json.loads((study/'protocol.json').read_text())
    for worker in range(1,p['nWorkers']+1):
        d=study/'workers'/f'worker_{worker:02d}'
        status=json.loads((d/'status.json').read_text(encoding='utf-8-sig'))
        a=json.loads((d/'audit.json').read_text(encoding='utf-8-sig'))
        assert status['status']=='complete' and a['passed'] and a['records']==status['scheduled']
        baselines+=a['baselineExactReplays'];rows+=read_csv(d/'trials.csv');audit+=read_csv(d/'raw_audit.csv')
    jobs={r['jobId'] for r in read_csv(study/'jobs.csv')}
    assert len(rows)==len(audit)==len(jobs)==9800 and {r['jobId'] for r in rows}==jobs
    assert len({r['jobId'] for r in rows})==9800 and baselines==200
    assert not any(yes(r['hasExecutionError']) or r['reason']=='execution_error' for r in rows)
    rows.sort(key=lambda r:(int(r['pathIndex']),int(r['referenceIndex']),int(r['seed'])))
    audit.sort(key=lambda r:r['jobId']);write_csv(study/'all_results.csv',rows);write_csv(study/'raw_mat_audit.csv',audit)
    write_json(study/'raw_mat_audit.json',dict(passed=True,records=9800,baselineExactReplays=200,
        allFrozenMapsUnchanged=True,allSourceFilesUnchanged=True,allParametersAndCsvPassed=True,
        independentVertexGeometryAndSuccessHoldPassed=True))
    return rows

def analyze(study):
    verify(study);rows=collect(study);configs=read_csv(study/'configurations.csv');summary=[]
    baseline={int(r['seed']):r for r in rows if float(r['pathXY'])==4.2 and float(r['refXY'])==4.0}
    assert len(baseline)==200 and sum(yes(r['success']) for r in baseline.values())==80
    for c in configs:
        sample=[r for r in rows if r['configId']==c['configId']];assert len(sample)==200
        assert {int(r['seed']) for r in sample}==set(range(2001,2201))
        safe=[r for r in sample if yes(r['initialHardSafe'])];success=[r for r in sample if yes(r['success'])]
        assert len(safe)==157 and sum(r['reason']=='invalid_scene' for r in sample)==43
        g=len(success);lo,hi=wilson(g,200);sl,sh=wilson(g,157);counts=Counter(r['reason'] for r in sample)
        s=dict(configId=c['configId'],pathIndex=int(c['pathIndex']),referenceIndex=int(c['referenceIndex']),
            pathClearance=float(c['pathClearance']),desiredClear=float(c['desiredClear']),n=200,initialSafeN=157,
            success=g,successPercent=100*g/200,wilsonLow=lo,wilsonHigh=hi,
            safeSuccessPercent=100*g/157,safeWilsonLow=sl,safeWilsonHigh=sh,
            initialPlanFound=sum(yes(r['initialPlanFound']) for r in sample),
            endpointEligible=sum(yes(r['initialHardSafe']) and yes(r['leaderEndpointsPlanClear']) for r in sample),
            initialUnsafe=counts['invalid_scene'],planningFailed=counts['planning_failed'],timeout=counts['timeout'],
            safetyUnresolved=counts['safety_unresolved'],collision=counts['collision'],boundaryViolation=counts['boundary_violation'],
            replanFailed=counts['replan_failed'],invalidState=counts['invalid_state'],
            rescued=sum(yes(r['success']) and not yes(baseline[int(r['seed'])]['success']) for r in sample),
            lost=sum(not yes(r['success']) and yes(baseline[int(r['seed'])]['success']) for r in sample))
        assert sum(counts.values())==200
        for field in ['time','path','verticalTravel','formationRms','commandEffort','referenceBlockedFraction','cycleP99ms']:
            s['successMean_'+field]=mean([number(r[field]) for r in success])
        for field in ['minObstacleClearance','minPairDistance']:
            v=[number(r[field]) for r in success];s['successMinimum_'+field]=min(v) if v else None
        common=[r for r in success if yes(baseline[int(r['seed'])]['success'])];s['commonSuccessN']=len(common)
        for field in ['time','path','verticalTravel','commandEffort']:
            s['commonMeanDelta_'+field]=mean([number(r[field])-number(baseline[int(r['seed'])][field]) for r in common])
        summary.append(s)
    write_csv(study/'grid_summary.csv',summary)
    best=max(s['success'] for s in summary);ties=[s for s in summary if s['success']==best]
    value=dict(tasks=9800,maps=200,configurations=49,baselineSuccess=80,baselinePercent=40,
        maximumSuccess=best,maximumPercent=100*best/200,maximumSafePercent=100*best/157,
        maximumConfigurations=ties,initialUnsafeMaps=43,initialSafetyOnlyCeilingPercent=78.5,
        outcomesAcrossTasks=dict(Counter(r['reason'] for r in rows)),auditPassed=True,
        interpretation='Finite exploratory grid on reused maps. All ties retained. No independent holdout, disturbance sweep, continuous optimum or real-flight guarantee.',
        surfaceRule='Linear facets join measured samples; unsampled combinations have not been simulated.',
        timingRule=json.loads((study/'protocol.json').read_text())['timingRule'])
    write_json(study/'summary.json',value)
    text=f'# 两项余量 49 组合扫描\n\n49 组 × 200 张完全相同的冻结地图，合计 9800 次完整任务。算法 2.3.1，高度/控制策略不变，各向同性余量，噪声和外扰为零。\n\n'
    text+=f'基线 4.2/4.0 m：80/200（40%）。本次网格最高 {best}/200（{best/2:.1f}%），初始安全子集 {best}/157（{100*best/157:.2f}%）；并列最高 {len(ties)} 组。\n\n'
    text+='43 张地图初始编队不安全，所有组合均保留。最高值属于这批地图上的探索性结果，尚无新 seed 独立验证；三维曲面只连接 49 个实测点，不证明中间参数的结果。\n\n'
    text+='## 并列最高组合\n\n| 规划余量/m | 参考余量/m | 成功/200 | 95% Wilson 区间 | 救回/丢失 | 碰撞 | 安全未解决 | 超时 |\n|---|---|---|---|---|---|---|---|\n'
    for s in ties:text+=f'| {s["pathClearance"]:g} | {s["desiredClear"]:g} | {s["success"]}/200 | {s["wilsonLow"]:.2f}%–{s["wilsonHigh"]:.2f}% | {s["rescued"]}/{s["lost"]} | {s["collision"]} | {s["safetyUnresolved"]} | {s["timeout"]} |\n'
    text+='\n## 查看图与数据\n\n- `figures/01_success_surface_all200.png`：全部地图的成功率曲面。\n- `figures/02_success_surface_safe157.png`：固定初始安全子集曲面。\n- `figures/03_success_heatmaps.png`：逐格精确成功率。\n- `figures/04_flight_failure_heatmaps.png`：飞行阶段失败分布。\n- 同名 PDF 为矢量导出，FIG 可在 MATLAB 中旋转与编辑。\n- `grid_summary.csv`：49 组汇总、区间、失败原因及同图救回/丢失。\n- `all_results.csv`：9800 条逐任务结果。\n- `raw_mat_audit.json`：全量原始文件审核，基线 200 张非计时轨迹精确复现。\n\n'
    text+='成功任务均值按各组自己的成功样本计算，不能直接当成性能改善；`commonMeanDelta_*` 在与基线共同成功的地图上配对计算。平方命令加速度积分与升降距离是代理指标，没有换算真实电池能耗。\n\n'
    text+='共享图缓存从初始规划取得，在飞行之前冻结；未把上一架次末尾的缓存带入下一次。每次重置学习权重及随机序列。并发和缓存计时不用于隔离规划或硬实时结论。\n'
    (study/'REPORT.md').write_text(text,encoding='utf-8')
    print(json.dumps(dict(maximum=best,ties=[(s['pathClearance'],s['desiredClear']) for s in ties],outcomes=value['outcomesAcrossTasks']),ensure_ascii=False),flush=True)

def deliver(study):
    destination=ROOT/'交付文件/余量49组合扫描_20261008';destination.mkdir(parents=True,exist_ok=True)
    for name in ['REPORT.md','protocol.json','summary.json','grid_summary.csv','all_results.csv','raw_mat_audit.json',
                 'raw_mat_audit.csv','source_sha256.csv','input_sha256.csv','pre_run_freeze.json','map_manifest.csv','configurations.csv']:
        shutil.copy2(study/name,destination/name)
    shutil.copytree(study/'figures',destination/'figures',dirs_exist_ok=True)
    shutil.copytree(study/'validation',destination/'validation',dirs_exist_ok=True)
    docs=ROOT/'docs/three_dimensional/margin_grid49_20261008';docs.mkdir(parents=True,exist_ok=True)
    for name in ['REPORT.md','protocol.json','summary.json','grid_summary.csv','all_results.csv','raw_mat_audit.json',
                 'source_sha256.csv','input_sha256.csv','pre_run_freeze.json','map_manifest.csv','configurations.csv']:
        shutil.copy2(study/name,docs/name)
    shutil.copytree(study/'figures',docs/'figures',dirs_exist_ok=True)
    shutil.copytree(study/'validation',docs/'validation',dirs_exist_ok=True)
    receipt=dict(study=study.as_posix(),delivery=destination.as_posix(),tasks=9800,
        baselineCommit=git('rev-parse','baseline-margin-grid-20261008'),executionCommit=json.loads((study/'protocol.json').read_text())['sourceCommit'])
    write_json(destination/'delivery_receipt.json',receipt);print(f'MARGIN_DELIVERY={destination}',flush=True)

def archive(study):
    assert not git('status','--porcelain');destination=ROOT/'交付文件/余量49组合扫描_20261008'
    archivePath=destination/'margin_grid49_full_data.zip'
    with zipfile.ZipFile(archivePath,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as z:
        for f in sorted(study.rglob('*')):
            if f.is_file():z.write(f,f.relative_to(study).as_posix())
    with zipfile.ZipFile(archivePath) as z:
        assert z.testzip() is None
        trials=[n for n in z.namelist() if '/trials/' in n and n.endswith('.mat')];assert len(trials)==9800
    bundle=destination/'uav_margin_grid49_history.bundle'
    git('bundle','create',str(bundle),'HEAD','baseline-margin-grid-20261008','margin-grid49-run-20261008','margin-grid49-study-20261008')
    git('bundle','verify',str(bundle))
    files=[dict(file=f.relative_to(destination).as_posix(),sha256=digest(f),bytes=f.stat().st_size)
           for f in sorted(destination.rglob('*')) if f.is_file() and f.name not in ['delivery_sha256.csv','archive_receipt.json']]
    write_csv(destination/'delivery_sha256.csv',files)
    write_json(destination/'archive_receipt.json',dict(tasks=9800,zipSha256=digest(archivePath),zipBytes=archivePath.stat().st_size,
        bundleSha256=digest(bundle),completionCommit=git('rev-parse','HEAD'),crcPassed=True))
    print('MARGIN_ARCHIVE_AND_HISTORY_COMPLETE',flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=['prepare','validate','run','analyze','plot','deliver','archive'])
    p.add_argument('--study',type=Path);p.add_argument('--source-commit');p.add_argument('--workers',type=int,default=6);a=p.parse_args()
    if a.command=='prepare':prepare(a.source_commit,a.workers);return
    study=a.study or Path((RUNTIME/'study_path.txt').read_text(encoding='utf-8'))
    if a.command=='validate':single(study,'validate',f"validate_margin_cache3d('{study.as_posix()}')")
    elif a.command=='run':run(study)
    elif a.command=='analyze':analyze(study)
    elif a.command=='plot':single(study,'plot',f"plot_margin_grid49_3d('{study.as_posix()}')")
    elif a.command=='deliver':deliver(study)
    else:archive(study)
if __name__=='__main__':main()
