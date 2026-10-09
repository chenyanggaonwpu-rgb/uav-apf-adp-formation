"""Release 2.3.3: predeclared normal maps, full flights, raw audit, reproducible rate."""
from __future__ import annotations
import argparse, csv, hashlib, json, math, os, statistics, subprocess, time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MATLAB = Path('C:/Program Files/MATLAB/R2024a/bin/matlab.exe')
NAMES = ['normal_seed_study.py', 'normal_seed_study_io3d.m', 'prepare_normal_seed_study3d.m',
         'run_normal_seed_study3d.m', 'audit_normal_seed_study3d.m', 'midpoint_sha256_file3d.m']

def read_csv(path):
    with Path(path).open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))

def write_csv(path, rows, fields=None):
    fields = fields or list(rows[0])
    with Path(path).open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)

def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False),
                          encoding='utf-8', newline='\n')

def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1048576), b''): h.update(chunk)
    return h.hexdigest()

def yes(value): return str(value).lower() in ('1', 'true')

def wilson(g, n):
    if not n: return [None, None]
    z = 1.959963984540054; p = g/n; d = 1+z*z/n
    c = (p+z*z/(2*n))/d; r = z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/d
    return [max(0, c-r), min(1, c+r)]

def quote_matlab(path): return str(path).replace('\\', '/').replace("'", "''")

def launch(study, name, code):
    runtime = ROOT/'.runtime'/'normal_seed_study_20261009'/study.name/name
    runtime.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    for key, folder in [('MATLAB_PREFDIR', 'prefs'), ('TEMP', 'temp'), ('TMP', 'temp')]:
        directory = runtime/folder; directory.mkdir(exist_ok=True); env[key] = str(directory)
    log = (runtime/'stdout.log').open('wb')
    info = subprocess.STARTUPINFO(); info.dwFlags |= subprocess.STARTF_USESHOWWINDOW; info.wShowWindow = 0
    process = subprocess.Popen([str(MATLAB), '-singleCompThread', '-batch', code,
                                '-logfile', str(runtime/'matlab.log')],
                               cwd=str(ROOT), env=env, stdout=log, stderr=subprocess.STDOUT,
                               startupinfo=info, creationflags=subprocess.CREATE_NO_WINDOW)
    return process, log, runtime

def run_single(study, name, code):
    process, log, runtime = launch(study, name, code); start = time.monotonic(); last = 0
    try:
        while process.poll() is None:
            time.sleep(2); elapsed = time.monotonic()-start
            if elapsed-last >= 25:
                print(f'MATLAB {name} elapsed={elapsed:.0f}s', flush=True); last = elapsed
        assert process.returncode == 0, f'{name} exited {process.returncode}; inspect {runtime}'
    finally: log.close()

def prepare(study, source_commit, first, count, workers):
    import shutil
    assert not study.exists(), f'Existing study must be resumed, never overwritten: {study}'
    study.mkdir(parents=True); (study/'maps').mkdir(); (study/'workers').mkdir()
    snap = study/'source_snapshot'
    for folder in ('config', 'src3d', 'experiments3d'): (snap/folder).mkdir(parents=True)
    for folder in ('config', 'src3d'):
        for file in (ROOT/folder).glob('*.m'): shutil.copy2(file, snap/folder/file.name)
    for name in NAMES: shutil.copy2(ROOT/'experiments3d'/name, snap/'experiments3d'/name)
    files = sorted(p for p in snap.rglob('*') if p.is_file())
    write_csv(study/'source_sha256.csv', [dict(file=p.relative_to(snap).as_posix(), sha256=sha256(p)) for p in files])
    code = (f"restoredefaultpath;addpath('{quote_matlab(snap/'experiments3d')}');"
            f"prepare_normal_seed_study3d('{quote_matlab(study)}','{source_commit}',{first},{count},{workers});")
    run_single(study, 'prepare', code)
    protocol = json.loads((study/'protocol.json').read_text(encoding='utf-8-sig'))
    assert protocol['seeds'] == list(range(first, first+count))
    print(f'PROTOCOL_PREDECLARED={study}', flush=True)

def verify_sources(study):
    rows = read_csv(study/'source_sha256.csv')
    for row in rows: assert sha256(study/'source_snapshot'/row['file']) == row['sha256'], row['file']
    return len(rows)

def verify_inputs(study):
    for row in read_csv(study/'input_sha256.csv'): assert sha256(study/row['file']) == row['sha256'], row['file']
    return verify_sources(study)

def collect(study, phase):
    protocol = json.loads((study/'protocol.json').read_text(encoding='utf-8-sig')); rows = []
    for worker, (first, last) in enumerate(protocol['ranges'], 1):
        directory = study/'workers'/f'worker_{worker:02d}'
        status = json.loads((directory/f'{phase}_status.json').read_text(encoding='utf-8-sig'))
        assert status['status'] == 'complete' and status['completed'] == last-first+1
        part = read_csv(directory/f'{phase}.csv'); assert len(part) == last-first+1
        rows += part
    rows.sort(key=lambda r: int(r['seed']))
    assert [int(r['seed']) for r in rows] == protocol['seeds']
    write_csv(study/f'all_{phase}.csv', rows)
    return rows

def freeze_inputs(study):
    rows = collect(study, 'generate'); proposals = []
    protocol = json.loads((study/'protocol.json').read_text(encoding='utf-8-sig'))
    for worker in range(1, protocol['nWorkers']+1):
        proposals += read_csv(study/'workers'/f'worker_{worker:02d}'/'proposals.csv')
    proposals.sort(key=lambda r: (int(r['seed']), int(r['attempt'])))
    assert len(proposals) == sum(int(r['attempts']) for r in rows)
    assert sum(yes(r['accepted']) for r in proposals) == sum(yes(r['accepted']) for r in rows)
    manifest = [dict(**r, mapMatSha256=sha256(study/r['mapFile'])) for r in rows]
    write_csv(study/'map_manifest.csv', manifest); write_csv(study/'all_proposals.csv', proposals)
    names = ['protocol.mat', 'protocol.json', 'source_sha256.csv', 'all_generate.csv',
             'map_manifest.csv', 'all_proposals.csv']+[r['mapFile'] for r in rows]
    write_csv(study/'input_sha256.csv', [dict(file=name, sha256=sha256(study/name)) for name in names])
    assert not any((study/'workers').glob('worker_*/trials/*.mat')), 'Flight started before freeze barrier'
    write_json(study/'pre_run_freeze.json', dict(status='all_maps_frozen_before_trials',
        seeds=protocol['seeds'], maps=len(rows), accepted=sum(yes(r['accepted']) for r in rows),
        proposals=len(proposals), sourceCommit=protocol['sourceCommit'], sourceFiles=verify_sources(study),
        inputFiles=len(names), inputManifestSha256=sha256(study/'input_sha256.csv'),
        noTrialsYet=True, frozenUTC=datetime.now(timezone.utc).isoformat()))
    print(f'MAPS_FROZEN n={len(rows)} accepted={sum(yes(r["accepted"]) for r in rows)} proposals={len(proposals)}', flush=True)

def run_phase(study, phase):
    verify_sources(study)
    if phase != 'generate': verify_inputs(study)
    protocol = json.loads((study/'protocol.json').read_text(encoding='utf-8-sig'))
    processes = []; logs = []; start = time.monotonic(); last = 0
    for worker in range(1, protocol['nWorkers']+1):
        code = f"restoredefaultpath;addpath('{quote_matlab(study/'source_snapshot'/'experiments3d')}');"
        if phase == 'audit': code += f"audit_normal_seed_study3d('{quote_matlab(study)}',{worker});"
        else: code += f"run_normal_seed_study3d('{quote_matlab(study)}',{worker},'{phase}');"
        process, log, _ = launch(study, f'{phase}_worker_{worker:02d}', code)
        processes.append(process); logs.append(log)
    try:
        while any(p.poll() is None for p in processes):
            time.sleep(2); elapsed = time.monotonic()-start
            if elapsed-last >= 25:
                statuses = []
                for path in (study/'workers').glob(f'worker_*/{phase}_status.json'):
                    try: statuses.append(json.loads(path.read_text(encoding='utf-8-sig')))
                    except (OSError, json.JSONDecodeError): pass
                print(f'NORMAL_RUNNING phase={phase} elapsed={elapsed:.0f}s '
                      f'completed={sum(s["completed"] for s in statuses)}/{protocol["nSeeds"]} '
                      f'accepted={sum(s.get("accepted", 0) for s in statuses)} '
                      f'success={sum(s["success"] for s in statuses)} '
                      f'active={sum(p.poll() is None for p in processes)}', flush=True)
                last = elapsed
        exits = [p.wait() for p in processes]
        assert not any(exits), f'Worker exit codes {exits}; inspect retained F: runtime logs'
    finally:
        for log in logs: log.close()
    rows = collect(study, phase)
    if phase == 'generate': freeze_inputs(study)
    else: verify_inputs(study)
    write_json(study/f'{phase}_phase_receipt.json', dict(phase=phase, complete=True, records=len(rows),
        wallSeconds=time.monotonic()-start, workers=protocol['nWorkers'], endedUTC=datetime.now(timezone.utc).isoformat()))
    print(f'NORMAL_PHASE_COMPLETE phase={phase} n={len(rows)}', flush=True)

def cohort(name, rows):
    n = len(rows); g = sum(yes(r['success']) for r in rows)
    return dict(cohort=name, n=n, success=g, percent=100*g/n if n else None, wilson95=wilson(g, n),
                outcomes=dict(Counter(r['reason'] for r in rows)))

def numeric(rows, key):
    values = []
    for r in rows:
        try: value = float(r[key])
        except (ValueError, KeyError): continue
        if math.isfinite(value): values.append(value)
    return values

def stats(values):
    if not values: return None
    values = sorted(values); n = len(values)
    return dict(n=n, min=values[0], max=values[-1], mean=statistics.mean(values), median=statistics.median(values),
                p95=values[math.ceil(.95*n)-1], p99=values[math.ceil(.99*n)-1])

def analyze(study):
    verify_inputs(study); rows = collect(study, 'flight'); audit = collect(study, 'audit')
    protocol = json.loads((study/'protocol.json').read_text(encoding='utf-8-sig'))
    proposals = read_csv(study/'all_proposals.csv'); manifests = read_csv(study/'map_manifest.csv')
    assert len(rows) == len(audit) == protocol['nSeeds']
    for r, a, m in zip(rows, audit, manifests):
        assert r['seed'] == a['seed'] == m['seed'] and yes(r['success']) == yes(a['success'])
        assert r['reason'] == a['reason'] and a['mapMatSha256'] == m['mapMatSha256']
        for key in ('parametersPassed', 'allProposalsRegenerated', 'csvPassed'): assert yes(a[key])
    outcomes = Counter(r['reason'] for r in rows); assert not outcomes['execution_error']
    accepted = [r for r in rows if yes(r['accepted'])]; success = [r for r in rows if yes(r['success'])]
    cohorts = [cohort('all_predeclared_seeds', rows), cohort('accepted_normal_maps', accepted)]
    bins = []
    for first, last in [(1, 10), (11, 20), (21, 30), (31, 40)]:
        c = cohort(f'{first}-{last}', [r for r in accepted if first <= int(r['obstacleCount']) <= last]); bins.append(c)
    write_csv(study/'success_by_obstacle_count.csv', [dict(countRange=c['cohort'], n=c['n'], success=c['success'],
        percent=c['percent'], wilsonLow=c['wilson95'][0], wilsonHigh=c['wilson95'][1]) for c in bins])
    failures = [r for r in rows if not yes(r['success'])]
    write_csv(study/'failures.csv', failures, list(rows[0]))
    rejection_counts = Counter(r['reason'] for r in proposals if not yes(r['accepted']))
    write_csv(study/'generation_rejection_counts.csv', [dict(reason=k, n=v) for k, v in sorted(rejection_counts.items())])
    summary = dict(algorithmVersion=protocol['version'], sourceCommit=protocol['sourceCommit'], n=len(rows),
        seedRange=[protocol['seeds'][0], protocol['seeds'][-1]], cohorts=cohorts, outcomes=dict(outcomes),
        generation=dict(requested=len(rows), accepted=len(accepted), failed=len(rows)-len(accepted),
            rawProposals=len(proposals), rejectedProposals=len(proposals)-len(accepted),
            proposalAcceptancePercent=100*len(accepted)/len(proposals),
            rejectionReasons=dict(rejection_counts), attempts=stats(numeric(rows, 'attempts')),
            seconds=stats(numeric(rows, 'generationSeconds'))),
        acceptedMapDistribution=dict(obstacleCount=stats(numeric(accepted, 'obstacleCount')),
            estimatedUnionVolumeFraction=stats(numeric(accepted, 'estimatedUnionVolumeFraction')),
            nominalVolumeFraction=stats(numeric(accepted, 'nominalVolumeFraction'))),
        successfulFlight=dict(simulatedSeconds=stats(numeric(success, 'time')), pathMetres=stats(numeric(success, 'path')),
            leaderVerticalTravelMetres=stats(numeric(success, 'verticalTravel'))),
        initialPlanFound=sum(yes(r['initialPlanFound']) for r in rows),
        adaptiveAttempted=sum(yes(r['adaptiveAttempted']) for r in rows),
        addedHeightTrials=sum(int(r['addedCount']) > 0 for r in rows),
        maxPeakLayerCount=max(int(r['peakLayerCount']) for r in rows), countBins=bins,
        audited=True, allFrozenInputsUnchanged=True, sourceFiles=verify_sources(study),
        confidenceIntervalRule='95% Wilson interval for independent requested seeds; no interval for adaptive raw-proposal acceptance.',
        modelScope=protocol['modelScope'])
    write_json(study/'summary.json', summary)
    write_json(study/'raw_mat_audit.json', dict(passed=True, records=len(audit),
        success=len(success), allFrozenInputsUnchanged=True, allProposalSeedsAndGeometryReproduced=True,
        allParametersAndCsvPassed=True, independentWitnessGeometryChecked=True, successHoldChecked=True))
    main = cohorts[0]; ci = main['wilson95']; generation = summary['generation']
    report = [f'# 2.3.3 正常随机场景 {len(rows)} seed 完整任务测试', '',
        f'完整任务成功 **{len(success)}/{len(rows)} = {main["percent"]:.2f}%**，95% Wilson 区间 **{ci[0]*100:.2f}%–{ci[1]*100:.2f}%**。', '',
        f'预定 seed {protocol["seeds"][0]}–{protocol["seeds"][-1]}；运行前冻结源码、参数和全部地图。源码提交 `{protocol["sourceCommit"]}`。', '',
        f'成功生成正常地图 {len(accepted)}/{len(rows)}，生成预算耗尽 {len(rows)-len(accepted)}。接受地图上的飞行成功率 {len(success)}/{len(accepted)} = {cohorts[1]["percent"]:.2f}%。', '',
        f'生成阶段提出 {len(proposals)} 张完整候选，拒绝 {generation["rejectedProposals"]} 张，候选接受率 {generation["proposalAcceptancePercent"]:.2f}%。这是生成筛选效率，不能当作飞行成功率。', '',
        '| 完整任务结果 | 数量 | 占全部预定 seed |', '|---|---:|---:|']
    for reason, n in sorted(outcomes.items()): report.append(f'| {reason} | {n} | {100*n/len(rows):.2f}% |')
    report += ['', '判据沿用当前代码：长机误差≤1 m，四架僚机相对动态参考误差≤1 m，所有速度≤0.25 m/s，连续保持1 s，任务上限120 s。实际运行 run_simulation3d，所有失败和拒绝候选留档；每次任务从零 ADP 权重及独立规划缓存开始。', '',
        '场景约束：初始五机净空≥2 m；5 m 离场/到达段净空≥1.2 m；目标固定朝向 V 形五机净空≥4 m；边界净空≥1.2 m；独立三维平移见证满足净空、速度、加速度和120 s时限。每个seed最多100张完整候选。飞行规划/参考余量为1.2/4.0 m，原控制器未调参。', '',
        '原始 MAT 审核覆盖全部预定seed、全部候选的随机种子/障碍重现、冻结地图哈希、参数/CSV一致性、独立见证连续几何与运动时限、速度上限，以及所有成功任务最后1 s的达标状态。执行错误为0；所有冻结输入哈希未变。', '',
        '本结果只适用于此条件化正常场景分布及无噪声空中出发的五机质点模型。接受图的障碍数量不再服从原始均匀分布；不能直接和旧无约束/结构化批次比较，也不是地面起飞、真实机体或电池续航的成功率。', '',
        '| 接受图障碍数量 | 样本数 | 成功数 | 成功率 |', '|---|---:|---:|---:|']
    for c in bins:
        value = f'{c["percent"]:.2f}%' if c['percent'] is not None else '无样本'
        report.append(f'| {c["cohort"]} | {c["n"]} | {c["success"]} | {value} |')
    report += ['', '生成拒绝原因按首个未通过门槛计数：', '', '| 原因 | 数量 |', '|---|---:|']
    for k, v in sorted(rejection_counts.items()): report.append(f'| {k} | {v} |')
    report += ['', '证据：protocol.json（协议），source_snapshot/与source_sha256.csv（冻结源码），map_manifest.csv与input_sha256.csv（冻结地图/输入），all_proposals.csv及maps/*.mat（全部候选），all_flight.csv与workers/*/trials/*.mat（完整结果），all_audit.csv与raw_mat_audit.json（审核），failures.csv（全部失败），summary.json（汇总）。', '', f'原始实验目录：`{study}`。']
    (study/'REPORT.md').write_text('\n'.join(report)+'\n', encoding='utf-8', newline='\n')
    print(f'AUDITED_SUCCESS {len(success)}/{len(rows)}={main["percent"]:.2f}% CI95={ci[0]*100:.2f}%-{ci[1]*100:.2f}%', flush=True)

def main():
    parser = argparse.ArgumentParser(); parser.add_argument('command', choices=['prepare', 'generate', 'flight', 'audit', 'analyze', 'all'])
    parser.add_argument('--study', type=Path, required=True); parser.add_argument('--source-commit')
    parser.add_argument('--first', type=int, default=10001); parser.add_argument('--count', type=int, default=2000)
    parser.add_argument('--workers', type=int, default=6); args = parser.parse_args()
    study = args.study.resolve(); assert study.is_relative_to(ROOT/'results'), 'Every output must stay on F: project results'
    if args.command in ('prepare', 'all'):
        assert args.source_commit; prepare(study, args.source_commit, args.first, args.count, args.workers)
    if args.command == 'all':
        for phase in ('generate', 'flight', 'audit'): run_phase(study, phase)
        analyze(study)
    elif args.command in ('generate', 'flight', 'audit'): run_phase(study, args.command)
    elif args.command == 'analyze': analyze(study)

if __name__ == '__main__': main()
