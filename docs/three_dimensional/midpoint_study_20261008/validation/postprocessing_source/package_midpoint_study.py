"""Package the verified midpoint replay with standalone research figures."""
from __future__ import annotations
import argparse,csv,hashlib,json,shutil,time,zipfile
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def read_csv(path):
    with Path(path).open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def load_json(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))
def write_json(path,x):Path(path).write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
def yes(x):return str(x).lower() in ('true','1')
def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def build(study):
    s=load_json(study/'summary.json');audit=load_json(study/'raw_mat_audit.json');p=load_json(study/'protocol.json')
    rows=read_csv(study/'adaptive_results.csv');paired=read_csv(study/'paired_results.csv')
    assert audit['passed'] and audit['records']==1000 and audit['success']==s['adaptiveSuccess']
    assert len(rows)==len(paired)==1000 and sum(yes(r['success']) for r in rows)==s['adaptiveSuccess']
    assert dict(Counter(r['reason'] for r in rows))==s['adaptiveOutcomes']
    assert sum(yes(r['baselineSuccess']) for r in paired)==564
    counts=Counter(r['transition'] for r in paired);assert dict(counts)==s['pairedTransitions']
    charts=[('01_success_by_family','整体与各地图族成功率'),('02_paired_outcomes','同图配对结局'),
            ('03_initial_layer_counts','初始规划工作层数'),('04_outcome_breakdown','完整任务结局')]
    for stem,_ in charts:
        for ext in ['.png','.pdf','.fig','_data.csv']:assert (study/'figures'/f'{stem}{ext}').is_file()
    # A zero-failure export has a header and zero rows; no fabricated placeholder.
    with (study/'failures.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(r for r in rows if not yes(r['success']))
    good=s['adaptiveSuccess'];percent=good/10;gain=percent-56.4;ci=s['families'][0]
    lines=['# 临时高度补中点：原 1,000 张地图回放实验','',
      '> 完成日期：2026-10-08。使用原保存地图；全部1,000个预定seed纳入分母。','',
      '## 主要结果','',
      f'固定五层 **564/1000（56.4%）**；补中点策略 **{good}/1000（{percent:.1f}%）**，变化 **{gain:+.1f}个百分点**。',
      f'新结果 Wilson 95%区间：{ci["adaptiveLow"]:.4f}%–{ci["adaptiveHigh"]:.4f}%。该区间针对历史回放样本计算。',
      f'原436个失败seed救回 **{counts["rescued"]}/436**；原564个成功seed失去 **{counts["lost_success"]}** 个。',
      f'初始规划通过 **{s["adaptiveInitialPlanningSuccess"]}/1000**；完整飞行成功 **{good}/1000**，两个指标分别记录。','',
      '| 地图族 | 样本数 | 固定五层成功 | 补中点成功 | 新成功率 |','|---|---:|---:|---:|---:|']
    labels={'mixed':'混合三维障碍','underpass':'顶棚下穿','window':'单窗口墙','staggered_windows':'交错双窗口墙'}
    for f in s['families'][1:]:lines.append(f'| {labels[f["family"]]} | {f["n"]} | {f["baselineSuccess"]} | {f["adaptiveSuccess"]} | {f["adaptivePercent"]:.1f}% |')
    lines+=['','### 全部结局','', '| 结局 | 次数 |','|---|---:|---:|']
    for name,n in s['adaptiveOutcomes'].items():lines.append(f'| {name} | {n} |')
    lines+=['','## 实验条件与可比性','',
      '- 原地图来自 `seed_sweep3d_v2.1.0_20261005_113155_970`，seed 1–1000；逐图SHA核对。不重新生成、不换seed、不删失败。',
      '- 水平规划/参考余量仍为4.2/4.0 m；传统模式下的竖直规划/参考余量仍为4.2/4.0 m。本轮未叠加上次5.6/4.4/2 m的余量候选。',
      '- 五个基础高度为5/10/15/20/25 m，五架二阶质点；速度/命令加速度上限3 m/s、3 m/s²；障碍硬距离0.6 m、机间硬距离1 m。',
      '- ADP与预测安全层开启；每个seed共享学习权重按原参数重置；位置噪声和外部加速度均为0。',
      '- 成功仍要求目标误差≤1 m、最大参考跟踪误差≤1 m、全机速度≤0.25 m/s，连续满足1 s，且未碰撞、越界或超时。终端成功不代表全程刚性V形。',
      '- 本批是已知历史地图的同图回放，不能称为全新独立地图验证，也不外推到任意三维地图或真实机体。','',
      '## 策略与计算预算','',
      '先用基础五层规划；无路线时，计算障碍水平投影中心、投影内5×5有限采样、两障碍投影重叠内3×3采样处的竖直可用区间。球、圆柱和长方体按当前余量形成禁行高度区间，合并后取补集。',
      '缺层区间补入中点；已被基础层或先补的临时层覆盖时不重复插入。按投影面积、区间宽度、距起点高度和坐标固定排序。临时层只用于当前地图；输入基础层不改写。所有图边、起终连接与简化航段继续检查三维净空。',
      '路径评分仍为三维路长加1倍累计升降高度，在候选层之间比较整条路线。升降高度是能耗代理指标，当前质点模型没有电池功率模型，不能把该代价直接换算成真实耗电。',
      '默认最多补8层，总层数上限13，1 m水平网格容量上限81×81×13=85,293节点。有限截面和中点采样不构成连续空间完备性证明；局部区间存在也不能代替完整飞行验证。',
      f'本批初始补层任务 **{s["insertionTrials"]}**；初始最大层数 **{s["maxInitialLayerCount"]}**；含重规划/终端规划的最大层数 **{s["maxPeakLayerCount"]}**；触发初始层数预算拒绝的任务 **{s["budgetRejectedTrials"]}**。',
      f'已记录最小障碍距离 **{s["minObstacleClearance"]:.6f} m**、最小机间距离 **{s["minPairDistance"]:.6f} m**。',
      f'初始规划耗时中位 **{s["medianInitialPlanSeconds"]:.4f} s**，含固定层尝试和必要的补层重建。四个并发单计算线程MATLAB进程的计时只作成本记录，不作为硬实时证据。','',
      '## 首轮覆盖修复与验证','',
      '首轮仅取投影中心，会被窗口旁的其他障碍遮住可用截面。首轮中止，保留280条已写CSV和285个MAT文件；中断时尚未完成保存的记录不作正式结果。首轮不并入最终1000图分母。修复增加通用分布截面采样，重新冻结2.3.1并完整重跑。',
      '四组几何/三维测试通过。前16图关闭补层后与旧版非计时历史逐字段完全一致，原试跑补层16/16成功。覆盖修复另检验中心被柱体遮挡的通用窗口以及首轮5个漏解；验证日志保留。初次试跑空高度CSV格式问题已修复，首个日志保留。','',
      '## 科研图表','']
    if (study/'timing_summary.json').exists():
        t=load_json(study/'timing_summary.json')
        assert t['records']==1000
        insert=lines.index('## 首轮覆盖修复与验证')
        lines[insert:insert]=['## 本轮耗时记录','',
          f"初始规划中位 {t['initialPlanMedianSeconds']:.4f} s、P95 {t['initialPlanP95Seconds']:.4f} s、最大 {t['initialPlanMaxSeconds']:.4f} s。",
          f"每条实际飞行的P99控制周期耗时中位 {t['perFlightP99MedianMs']:.4f} ms，最大记录周期 {t['maxCycleMs']:.4f} ms。共 {t['measuredCycles']} 个正耗时周期，其中 {t['cyclesAboveDt']} 个超过 {t['cycleDeadlineMs']:.0f} ms仿真步长。",
          '采用逐飞行最近秩P99再取中位，不等于合并全部周期的P99；计时包括周期内事件规划。四个MATLAB进程并发，不能据此宣称硬实时。',
          '[逐seed计时CSV](timing_by_seed.csv) · [计时口径](timing_summary.json)','']
    for stem,label in charts:
        lines +=[f'### {label}','',f'![{label}](figures/{stem}.png)','',
          f'[矢量PDF](figures/{stem}.pdf) · [MATLAB FIG](figures/{stem}.fig) · [源CSV](figures/{stem}_data.csv)','']
    if (study/'example_selection.json').exists():
        e=load_json(study/'example_selection.json');folder=f'examples/seed_{e["seed"]:06d}'
        assert (study/folder/'flight_3d.mp4').exists()
        lines+=['## 真实飞行示例','',f'按“交错双窗口族中最小救回seed”固定顺序选择 **seed {e["seed"]}**；取自正式原始MAT，不另计一次试验。',
          '临时高度：'+', '.join(f'{z:.6f} m' for z in e['addedHeights'])+'。',
          f'[飞行MP4]({folder}/flight_3d.mp4) · [区间与决策CSV]({folder}/height_intervals.csv) · [全机实际高度CSV]({folder}/actual_heights.csv)','']
    lines+=['## 原始数据与审核','',
      f'原始MAT **1000/1000** 审核通过，冻结源码 **{audit["sourceFiles"]}** 个文件哈希匹配。检查参数、原地图与基线文件SHA、CSV/MAT一致性、初始航段净空、补层中点规则、成功末端保持，以及独立顶点物理距离与原连续安全记录。未另行证明所有连续状态的误差界。',
      '[逐seed结果](adaptive_results.csv) · [配对结果](paired_results.csv) · [旧基线](baseline_results.csv) · [失败清单](failures.csv)',
      '[MAT审计](raw_mat_audit.csv) · [地图来源](map_manifest.csv) · [源码哈希](source_sha256.csv)','',
      '## Git与复现','',f'数值冻结提交：`{p["sourceCommit"]}`，标签 `adaptive-midpoint-run-v{p["version"]}-20261008`。',
      '回滚基线：`baseline-adaptive-height-v2.2.0-20261008`；实验分支：`experiment/adaptive-midpoint-20261008`。完成标签、Git bundle和数据ZIP在交付目录；未推送远端。',
      '默认 `default_params3d()` 已启用补层；设置 `p.adaptiveHeight.enable=false` 恢复固定层规划。缺少该字段的历史参数默认关闭补层。',
      '复核本批使用冻结原地图和源码。运行 `experiments3d/midpoint_sweep.py run --study <结果目录>` 可恢复检查点；`audit` 审核原MAT，`analyze` 重新统计。直接重新生成地图属于另一批实验。','']
    (study/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8')
    guide=['# 阅读说明','', '先打开REPORT.md；四组科研图的PNG/PDF/FIG与源CSV在figures/。',
      '正式数据在workers/worker_01至worker_04/trials/，每seed一个MAT；原地图及参数在maps/。',
      '本轮只改变补层策略，余量保持旧批次4.2/4.0 m；全部1000图纳入分母。',
      f'数值版本：{p["version"]}，冻结提交：{p["sourceCommit"]}。',
      '回滚基线标签：baseline-adaptive-height-v2.2.0-20261008。',
      f'完成标签：midpoint-study-v{p["version"]}-20261008。',
      '保存Git bundle和experiment_data_full.zip可恢复代码历史及完整本轮数据；ZIP CRC/SHA见delivery_receipt.json。',
      'ZIP的results/是本轮正式数据，baseline_original/是完整原基线，aborted_attempt/是首轮中止记录。复原审核前，把baseline_original/下的原基线目录放回项目results/，保留原文件内容。']
    (study/'阅读说明.md').write_text('\n\n'.join(guide),encoding='utf-8')
    mirror=ROOT/'docs'/'three_dimensional'/'midpoint_study_20261008';delivery=ROOT/'交付文件'/'临时高度补层_1000seed_20261008'
    selected=['REPORT.md','阅读说明.md','summary.json','protocol.json','raw_mat_audit.json','raw_mat_audit.csv',
      'adaptive_results.csv','baseline_results.csv','paired_results.csv','family_summary.csv','failures.csv','map_manifest.csv','source_sha256.csv','example_selection.json','timing_by_seed.csv','timing_summary.json']
    validation=dict(figures=4,rawRecords=1000,allRowsReconciled=True,allPairedCountsReconciled=True,allFigureExportsPresent=True)
    write_json(study/'report_validation.json',validation);selected.append('report_validation.json')
    for dest in [mirror,delivery]:
        dest.mkdir(parents=True,exist_ok=True)
        for name in selected:
            if (study/name).is_file():shutil.copy2(study/name,dest/name)
        for name in ['figures','examples','validation']:
            if (study/name).exists():shutil.copytree(study/name,dest/name,dirs_exist_ok=True)
    print('REPORT_BUILT',str(delivery),flush=True)
def archive(study):
    delivery=ROOT/'交付文件'/'临时高度补层_1000seed_20261008';target=delivery/'experiment_data_full.zip'
    files=[(p,Path('results')/study.name/p.relative_to(study)) for p in study.rglob('*') if p.is_file()]
    baseline=Path(load_json(study/'protocol.json')['baselineDir'])
    assert baseline.is_dir()
    baseline_files=[p for p in baseline.rglob('*') if p.is_file()]
    files +=[(p,Path('baseline_original')/baseline.name/p.relative_to(baseline)) for p in baseline_files]
    first=ROOT/'.runtime/adaptive_midpoint_20261008/attempt01_study_path.txt'
    if first.exists():
        attempt=Path(first.read_text(encoding='utf-8-sig'))
        files +=[(p,Path('aborted_attempt')/attempt.name/p.relative_to(attempt)) for p in attempt.rglob('*') if p.is_file()]
    with zipfile.ZipFile(target,'w',compression=zipfile.ZIP_STORED,allowZip64=True) as z:
        for i,(p,name) in enumerate(files,1):
            z.write(p,name)
            if i%500==0:print(f'ARCHIVE_PROGRESS {i}/{len(files)}',flush=True)
    with zipfile.ZipFile(target) as z:
        assert z.testzip() is None;names=z.namelist();assert len(names)==len(set(names))
        raw=[n for n in names if n.startswith('results/') and '/trials/adaptive_' in n and n.endswith('.mat')];assert len(raw)==1000
        original=[n for n in names if n.startswith('baseline_original/') and '/trials/C_random_' in n and n.endswith('.mat')];assert len(original)==1000
    receipt=dict(archiveBytes=target.stat().st_size,archiveSha256=sha256(target),crcPassed=True,formalRawMats=1000,
      originalBaselineRawMats=1000,archiveEntries=len(files),study=str(study),sourceCommit=load_json(study/'protocol.json')['sourceCommit'])
    bundle=delivery/'uav_midpoint_history.bundle'
    if bundle.exists():receipt['gitBundleSha256']=sha256(bundle)
    write_json(delivery/'delivery_receipt.json',receipt)
    values=[dict(file=p.relative_to(delivery).as_posix(),bytes=p.stat().st_size,sha256=sha256(p))
      for p in sorted(delivery.rglob('*')) if p.is_file() and p.name not in ('experiment_data_full.zip','delivery_sha256.csv')]
    with (delivery/'delivery_sha256.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['file','bytes','sha256']);w.writeheader();w.writerows(values)
    print('ARCHIVE_COMPLETE',json.dumps(receipt,ensure_ascii=False),flush=True)
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('command',choices=['build','archive']);parser.add_argument('--study',type=Path)
    args=parser.parse_args();study=args.study or Path((ROOT/'.runtime/adaptive_midpoint_20261008/study_path.txt').read_text(encoding='utf-8-sig'))
    if args.command=='build':build(study)
    else:archive(study)
