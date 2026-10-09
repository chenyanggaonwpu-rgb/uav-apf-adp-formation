"""F-drive evidence delivery for the unconditioned 200-seed experiment."""
from __future__ import annotations
import argparse,csv,json,math,shutil,subprocess,zipfile
from collections import Counter
from pathlib import Path
from unstructured_sweep import ROOT,RUNTIME,read_csv,write_csv,write_json,sha256,verify_inputs,yes,run_single
DELIVERY=ROOT/'交付文件'/'无结构随机障碍_200seed_20261008'
DOCS=ROOT/'docs'/'three_dimensional'/'unstructured_random200_20261008'
GIT=['git',f'--git-dir={ROOT.as_posix()}/.git',f'--work-tree={ROOT.as_posix()}','-c',f'safe.directory={ROOT.as_posix()}']

def git(*args):return subprocess.check_output(GIT+list(args),cwd=ROOT,text=True,encoding='utf-8').strip()
def pct(g,n):return f'{100*g/n:.2f}%' if n else 'NA'
def diagnose(study,rows):
    categories=[]
    for row in rows:
        if yes(row['success']):label='success'
        elif row['reason']=='invalid_scene':label='initial_formation_unsafe'
        elif row['reason']=='planning_failed' and not yes(row['leaderEndpointsPlanClear']):label='planning_failed_endpoint_margin'
        elif row['reason']=='planning_failed':label='planning_failed_with_clear_endpoints'
        else:label='flight_'+row['reason']
        categories.append(dict(seed=row['seed'],reason=row['reason'],diagnosticGroup=label,
            initialHardSafe=int(yes(row['initialHardSafe'])),leaderEndpointsPlanClear=int(yes(row['leaderEndpointsPlanClear'])),
            initialPlanFound=int(yes(row['initialPlanFound'])),obstacleCount=row['obstacleCount']))
    write_csv(study/'failure_diagnostics.csv',categories)
    counts=dict(Counter(r['diagnosticGroup'] for r in categories))
    write_json(study/'failure_diagnostics.json',dict(counts=counts,
        interpretation='Endpoint-margin categories use independent preflight geometry, not a proof of global infeasibility.'))
    return counts

def report(study):
    verify_inputs(study);summary=json.loads((study/'summary.json').read_text(encoding='utf-8-sig'))
    timing=json.loads((study/'timing_summary.json').read_text(encoding='utf-8-sig'))
    protocol=json.loads((study/'protocol.json').read_text(encoding='utf-8-sig'))
    rows=read_csv(study/'all_results.csv');assert len(rows)==200 and summary['audited']
    counts=diagnose(study,rows);g=summary['overallSuccess'];n=summary['n'];lo,hi=summary['wilson95']
    c=summary['cohorts'];observation=summary['observedCount'];types=summary['types'];examples=[]
    if (study/'example_manifest.json').exists():
        value=json.loads((study/'example_manifest.json').read_text(encoding='utf-8-sig'));examples=value if isinstance(value,list) else [value]
    cohort_names={'all_200':'全部随机地图','initial_formation_hard_safe':'初始五机均满足障碍安全距',
        'initial_safe_and_leader_endpoints_4p2m':'初始安全，且长机起终点满足 4.2 m 净空','flight_started':'实际进入飞行阶段'}
    lines=['# 无结构随机三维场景','',f'## {g} / {n}　·　{pct(g,n)}','',
        f'**200 个新 seed：2001–2200。当前 2.3.1 算法。全部随机地图均保留。**',
        '',f'主指标的 Wilson 95% 区间为 **{100*lo:.2f}%–{100*hi:.2f}%**。它描述本轮规定的随机分布，不能外推任意空域或真实无人机。',
        '', '![全部任务结局](figures/01_outcomes.png)', '', '## 1. 成功率与分母', '',
        '| 地图范围 | 成功 / 样本 | 比例 |','| --- | ---: | ---: |']
    for row in c:lines.append(f"| {cohort_names[row['cohort']]} | {row['success']} / {row['n']} | {pct(row['success'],row['n'])} |")
    lines += ['', '后面三行是解释性条件结果；主指标始终使用全部 200 张地图。初始编队不安全的地图保留为 `invalid_scene`，不称作飞行中碰撞。执行异常为 0，缺失 seed 为 0。',
        '', '完整成功要求：长机目标误差≤1 m、最大僚机对动态参考的跟踪误差≤1 m、全机速度≤0.25 m/s，连续保持 1 s，且全程没有安全失败或超时。最大仿真时长 120 s。',
        '', '## 2. 失败发生在哪一阶段', '', '| 结局 | 数量 |', '| --- | ---: |']
    labels={'success':'完整成功','invalid_scene':'起飞前编队不安全','planning_failed':'初始规划失败','timeout':'飞行超时','collision':'飞行碰撞',
        'safety_unresolved':'安全约束未解决','replan_failed':'飞行重规划失败','boundary_violation':'越界','invalid_state':'状态无效'}
    for reason,total in summary['outcomes'].items():lines.append(f'| {labels.get(reason,reason)} | {total} |')
    rescued=sum(yes(r['success']) and yes(r['adaptiveAttempted']) for r in rows)
    successful_insertions=sum(yes(r['success']) and int(r['addedCount'])>0 for r in rows)
    margin=counts.get('planning_failed_endpoint_margin',0);clear=counts.get('planning_failed_with_clear_endpoints',0)
    lines += ['',f'在初始规划失败中，**{margin} 张**长机起点或目标点本身不满足 4.2 m 规划净空；**{clear} 张**起终点都满足该净空，但当前有限层规划仍未找到路线。后者可能涉及通道连通性、栅格或补层覆盖限制，不能仅凭本轮记录判为连续空间无解。',
        '', '补高度层可以补上遗漏的可用高度，不能把被实体占据的起点变成安全起点。起终点净空不足与规划覆盖问题分别记录，避免把不同失败都归因于高度选择。',
        '', '[逐 seed 失败记录](failures.csv) · [阶段诊断](failure_diagnostics.csv) · [全部结果](all_results.csv)',
        '', '## 3. 障碍物怎样随机', '', '| 变量 | 预先冻结的采样范围 |', '| --- | --- |',
        '| 数量 | 每图整数 1–40，均匀抽样 |', '| 类型 | 球、竖直圆柱、轴对齐箱体，逐障碍独立等概率 |',
        '| 球 | 半径 1–6 m，连续均匀 |', '| 圆柱 | 半径 1–6 m、高度 2–24 m，连续均匀 |',
        '| 箱体 | 长宽各 2–20 m、高度 2–24 m，连续均匀 |',
        '| 三维位置 | 尺寸确定后，中心在能够容纳该实体的场地区域中均匀抽样 |',
        '| 重叠、起终点、可达性 | 允许重叠；不预留起终点；不筛选可达性；不替换失败地图 |',
        '',f"实际生成 **{summary['obstacleTotal']} 个实体**：球 {types.get('sphere',0)}、圆柱 {types.get('cylinder',0)}、箱体 {types.get('box',0)}。实际数量范围 {int(observation['min'])}–{int(observation['max'])} 个，平均 {observation['mean']:.3f} 个／图。",
        '', '这是有明确边界的随机实验：场地 80×80×30 m，所有实体完整位于场地中。尺寸与可放置中心范围受边界共同约束，因此不声称它们完全独立，也不把本轮范围当作真实空域统计模型。圆柱不再全部从地面长出，球和箱体的高度也随机，没有保证最高层能越过所有障碍的规则。',
        '', '![随机输入分布](figures/03_random_input_distribution.png)',
        '', '![按数量分组的成功率](figures/02_success_by_count.png)',
        '', '分组误差线为各组的 Wilson 95% 区间。各组分母包含该组全部地图，包括无法安全起飞的地图；没有在看到结果后挑选地图。名义体积分数按各实体体积相加计算，重叠部分重复计入，不是空间并集占用率。',
        '', '[200 张地图清单](map_manifest.csv) · [全部障碍的尺寸和坐标](obstacle_catalog.csv)',
        '', '## 4. 算法与参数', '',
        '- 沿用 2.3.1：先尝试 5/10/15/20/25 m；固定层失败时补上未覆盖可用区间的中点，最多补 8 层。',
        '- 规划与参考余量维持各向同性 4.2/4.0 m；硬障碍距离 0.6 m、机间距离 1 m。',
        '- 五机质点模型；速度与命令加速度上限 3 m/s、3 m/s²；ADP 和安全层开启，噪声和外扰均为 0。',
        '- 本轮改变随机环境，保持飞行算法和余量。有限采样、栅格和层数预算没有连续空间完备性保证。',
        '',f"本轮初始规划找到路线 **{summary['initialPlanFound']}/200**；{summary['adaptiveAttempted']} 个任务尝试自适应补层，{summary['insertionTrials']} 个任务实际补层。初始／全阶段最大工作层数为 {summary['maxInitialLayers']}/{summary['maxPeakLayers']}。",
        '', f'成功任务中，{rescued} 个在初始规划时尝试了补层，{successful_insertions} 个实际增加了初始高度层。若为 0，表示这批无结构地图的成功路径由基础五层找到，不能把全部成功归因于补层策略。',
        '', '上轮 1000 张结构化历史地图回放为 100%，其障碍数量与窗口结构不同。两轮成功率不能用来隔离高度策略的因果收益；同条件比较需要对同一批冻结地图运行不同策略。',
        '', '## 5. 计算性能', '',
        f"四个单计算线程 MATLAB 进程并发。共 {timing['planningCalls']} 次初始规划：中位 {timing['initialPlanMedianSeconds']:.3f} s、P95 {timing['initialPlanP95Seconds']:.3f} s、最大 {timing['initialPlanMaxSeconds']:.3f} s。",
        '',f"已记录 {timing['measuredCycles']} 个正耗时飞行周期。逐飞行 P99 耗时的中位数为 {timing['perFlightP99MedianMs']:.3f} ms；最大单周期 {timing['maxCycleMs']:.3f} ms，其中 {timing['cyclesAboveDt']} 个周期超过 50 ms 仿真步长。",
        '', '初始规划时间和在线控制周期分开统计；这些是本机并发时测量值，不是隔离基准或硬实时保证。本轮本地算力可完成，无需租算力。',
        '', '![计算时间](figures/04_computation_time.png)', '', '[逐 seed 耗时](timing_by_seed.csv)',
        '', '## 6. 看示例与 MP4', '']
    for item in examples:
        directory=item['path'];seed=item['seed'];isflight=item['rule']!='smallest_planning_failure'
        label='最小成功 seed' if item['rule']=='smallest_success' else '最小有飞行历史的失败 seed' if isflight else '最小初始规划失败 seed'
        lines += [f"### seed {seed} · {label}",'',f"结局 `{item['reason']}`，障碍 {item['obstacles']} 个。按预先规则选取，不参与筛选统计。",'']
        image='trajectory_3d.png' if isflight else 'map_3d.png'
        lines.append(f'![seed {seed}](examples/seed_{seed:06d}/{image})')
        if item.get('mp4'):
            lines += ['',f"[播放 MP4](<{(DELIVERY/directory/'flight_3d.mp4').as_posix()}>)。视频直接由正式 MAT 导出，不重新仿真。"]
    lines += ['', '其余有飞行历史的 seed 可从正式 MAT 导出动画。起飞前失败或规划失败没有飞行轨迹，可查看地图与高度区间决策。',
        '', '## 7. 证据、复现与回滚', '',
        f"数值源码冻结提交：`{protocol['sourceCommit']}`。执行标签：`unstructured-random-run-v2.3.1-20261008`；回滚标签：`baseline-unstructured-random-20261008`。",
        '', f"完整原始目录：`{study}`。所有地图在正式试验前冻结，205 份输入文件和 38 份源码的 SHA-256 校验通过；200 个原始 MAT 与 CSV、原地图逐条核对。独立顶点障碍距离、机间距离、初始几何和成功终端保持检查通过。连续航段净空使用冻结几何函数核对，未另行证明真实机体误差界。",
        '', '[原始记录审核](raw_mat_audit.csv) · [审核摘要](raw_mat_audit.json) · [冻结协议](protocol.json) · [输入校验值](input_sha256.csv) · [源码校验值](source_sha256.csv)',
        '', '数据归档和 Git bundle 保存于同一 F 盘交付目录，校验值见交付收据。恢复只读取已有 seed 的检查点，算法与原始记录均可回滚；没有推送远端。']
    (study/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    post=study/'postprocess_source';post.mkdir(exist_ok=True)
    for filename in ['plot_unstructured_sweep3d.m','package_unstructured_study.py']:
        shutil.copy2(ROOT/'experiments3d'/filename,post/filename)
    write_csv(study/'postprocess_sha256.csv',[dict(file=p.name,sha256=sha256(p)) for p in sorted(post.iterdir()) if p.is_file()])
    validation=study/'validation';validation.mkdir(exist_ok=True)
    for name in ['generator_test','prepare','audit','figures','figures_layout_repair']:
        file=RUNTIME/name/'matlab.log'
        if file.exists():shutil.copy2(file,validation/f'{name}.log')
    files=['REPORT.md','all_results.csv','failures.csv','failure_diagnostics.csv','failure_diagnostics.json','summary.json',
        'protocol.json','map_manifest.csv','obstacle_catalog.csv','input_sha256.csv','source_sha256.csv','pre_run_freeze.json',
        'success_by_count.csv','raw_mat_audit.csv','raw_mat_audit.json','timing_by_seed.csv','timing_summary.json',
        'example_manifest.json','postprocess_sha256.csv']
    for destination in [DELIVERY,DOCS]:
        destination.mkdir(parents=True,exist_ok=True)
        for name in files:
            if (study/name).exists():shutil.copy2(study/name,destination/name)
        for name in ['figures','examples','validation','postprocess_source']:
            if (study/name).is_dir():shutil.copytree(study/name,destination/name,dirs_exist_ok=True,
                ignore=shutil.ignore_patterns('*.mp4') if destination==DOCS else None)
    write_csv(DELIVERY/'evidence_sha256.csv',[dict(file=p.relative_to(DELIVERY).as_posix(),sha256=sha256(p))
        for p in sorted(DELIVERY.rglob('*')) if p.is_file() and p.name!='evidence_sha256.csv'])
    print(f'REPORT_DELIVERED success={g}/200 percent={pct(g,n)} directory={DELIVERY}',flush=True)

def archive(study):
    verify_inputs(study);assert not git('status','--porcelain'), 'Commit the final evidence before bundling.'
    head=git('rev-parse','HEAD');source=json.loads((study/'protocol.json').read_text(encoding='utf-8-sig'))['sourceCommit']
    bundle=DELIVERY/'uav_unstructured_history.bundle'
    subprocess.run(GIT+['bundle','create',str(bundle),'experiment/unstructured-random-200-20261008',
        'baseline-unstructured-random-20261008','unstructured-random-run-v2.3.1-20261008','unstructured-random-study-v2.3.1-20261008'],cwd=ROOT,check=True,capture_output=True)
    checked=subprocess.run(GIT+['bundle','verify',str(bundle)],cwd=ROOT,check=True,capture_output=True,text=True)
    receipt=dict(finalCommit=head,sourceCommit=source,branch=git('branch','--show-current'),workingTreeClean=True,
        bundleSha256=sha256(bundle),bundleBytes=bundle.stat().st_size,bundleVerified=True,completeHistory='complete history' in checked.stdout+checked.stderr,
        remotelyPushed=False)
    write_json(DELIVERY/'git_receipt.json',receipt);write_json(study/'validation'/'git_receipt.json',receipt)
    archive=DELIVERY/'random200_full_data.zip'
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_STORED,allowZip64=True) as z:
        for file in sorted(study.rglob('*')):
            if file.is_file():z.write(file,Path('results')/study.name/file.relative_to(study))
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None;names=z.namelist();assert len(names)==len(set(names))
        raw=[name for name in names if '/workers/' in name and '/trials/random_' in name and name.endswith('.mat')]
        maps=[name for name in names if '/maps/seed_' in name and name.endswith('.mat')]
        assert len(raw)==200 and len(maps)==200
    receipt=dict(status='complete',records=200,rawMatFiles=200,frozenMaps=200,zipEntries=len(names),
        dataZipBytes=archive.stat().st_size,dataZipSha256=sha256(archive),zipCrcPassed=True,
        bundleSha256=sha256(bundle),finalCommit=head,sourceCommit=source)
    write_json(DELIVERY/'delivery_receipt.json',receipt)
    paths=[p for p in sorted(DELIVERY.rglob('*')) if p.is_file() and p.name not in ('delivery_sha256.csv','evidence_sha256.csv')]
    write_csv(DELIVERY/'delivery_sha256.csv',[dict(file=p.relative_to(DELIVERY).as_posix(),sha256=sha256(p),bytes=p.stat().st_size) for p in paths])
    print(json.dumps(receipt,ensure_ascii=False,indent=2),flush=True)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('command',choices=['figures','report','archive']);parser.add_argument('--study',type=Path)
    args=parser.parse_args();study=args.study or Path((RUNTIME/'study_path.txt').read_text(encoding='utf-8-sig'))
    if args.command=='figures':
        verify_inputs(study)
        run_single('figures',f"restoredefaultpath;addpath('{(ROOT/'experiments3d').as_posix()}');plot_unstructured_sweep3d('{study.as_posix()}',true);")
    elif args.command=='report':report(study)
    else:archive(study)
if __name__=='__main__':main()
