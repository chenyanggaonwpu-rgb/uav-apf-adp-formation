# 统一Actor全量重验复现

先从工程根目录运行MATLAB：

```matlab
addpath('experiments3d/unified_actor_20261029');
output=run_all_unified(fullfile(pwd,'results','unified_actor_20261029'));
% 结束全部并行仿真后，单独运行串行计时：
measure_serial_unified(fullfile(pwd,'results','unified_actor_20261029','serial_timing'));
```

支持分片：在不同MATLAB进程传入相同output和shard=1..N、shards=N。已完成任务按记录跳过。密集图并发规划可能耗尽内存，可减少进程数并保留错误后用相同输入重试。模型不重新训练。

Python需要numpy；分析脚本从工程根目录推导固定结果目录，依次运行：

```text
python -B experiments3d/unified_actor_20261029/analyze_campaign.py
python -B experiments3d/unified_actor_20261029/export_traces.py
python -B experiments3d/unified_actor_20261029/summarize_for_reports.py
```

只重建已交付报告时，无需再跑仿真：将results/unified_actor_20261029中的records_3840.zip和independent_audits_3840.zip解压到同一目录，再运行报告目录的build_reports.py。已提交报告图表采用的4份MAT轨迹、report_facts.json、串行周期CSV和原始摘要已随附。生成Word需要python-docx和reportlab，输出PDF另见render_reports.py；当前固定版式PDF和逐页检查凭据已随报告提交。

完整3840份MAT与209份逐步CSV保存在F盘原工程，Git保存全部地图输入、3840份原始摘要/独立复核压缩包和4份作图MAT。复现实验会重新生成全部物理轨迹。运行前把TEMP、TMP、MATLAB_PREFDIR设为F盘目录。
