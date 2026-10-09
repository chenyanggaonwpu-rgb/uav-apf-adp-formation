# 正常随机场景完整任务成功率

**最新完成状态：**正式正常场景2000次完整任务1955成功（97.75%），全部原始MAT审核通过；[当前状态](../docs/CURRENT_STATUS.md)、[结果与CSV](../docs/three_dimensional/normal_random_2000_20261009/README.md)、[单步计时](../docs/three_dimensional/step_timing_20261009/README.md)。当前版本2.3.3，单进程复测仍待执行。

`normal_seed_study.py` 对当前 2.3.3 正常生成器与完整飞行进行一次冻结实验，不做控制参数搜索。正式预定 seed 为 10001–12000，2000 个；飞行规划/参考余量为 1.2/4.0 m。每个 seed 最多生成 100 张候选图，保留每张被拒绝候选的完整几何和首个拒绝原因。生成失败不补 seed。

所有实验目录在项目 F 盘 `results/` 内，MATLAB 临时文件、偏好目录和日志在 `.runtime/normal_seed_study_20261009/` 内。按 `prepare → generate → flight → audit → analyze` 执行：

1. 冻结 config、src3d、实验运行器，记录 SHA-256、源码提交、全部参数、seed 范围和固定分工。
2. 每个请求 seed 独立生成；在最后一张地图冻结并形成 `input_sha256.csv` 和 `pre_run_freeze.json` 之前，禁止开始任何被评估的飞行。
3. 从冻结的地图和参数运行 `run_simulation3d`。每次从默认零 ADP 权重和独立空规划缓存开始，不继承上个 seed 的学习状态。完整任务 MAT、CSV、日志逐 seed 保存，支持断点恢复。执行错误保留证据并阻止正式结论。
4. 对每个原始任务 MAT 审核，重现全部候选的原始障碍抽样，校验参数、冻结输入、几何、独立见证、速度上限、CSV 与 MAT 的一致性及成功任务连续保持条件。独立见证连续段用单独的段最小距离算法复核，不重复执行原见证搜索。
5. 全部 seed 完成并审核通过后，生成 `summary.json`、`REPORT.md`、全部结果/失败 CSV、障碍数量分组和拒绝原因计数。成功率采用完整任务 `result.success`，不采用初始规划是否找到路径。

分母分别为全部预定 seed、成功生成的正常地图。候选接受率另报，用于说明生成效率；不作为飞行成功率。95% Wilson 区间只用于独立请求 seed 的完整任务结果，不用于同一 seed 下提前停止的原始候选比例。已接受场景是约束条件下的分布，其障碍数量不再均匀。

示例（需替换 Python 路径和完整本地源码提交）：

```powershell
& '<python.exe>' -B experiments3d/normal_seed_study.py all `
  --study 'F:/UAV project/UAV_APF_ADP_Formation/results/normal_random_v2.3.3_2000_20261009' `
  --source-commit '<full local commit>' --first 10001 --count 2000 --workers 6
```

恢复时使用同一冻结目录执行 `generate`、`flight`、`audit` 或 `analyze`，不重新 prepare、不替换地图、不修改冻结源码。如果某个任务有未解决执行错误，先保存原错误证据再修复实验工具；不能把错误默默删除或当成控制器失败。

试运行 seed 9001–9008 与正式 seed 不重叠，只用于验证运行器和审核流程。试运行结果不并入 2000 个正式任务。
