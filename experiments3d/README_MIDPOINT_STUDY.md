# 临时高度补层实验：运行、回放与回滚

## 当前策略

基础高度保持 5/10/15/20/25 m。固定层找不到路线时，计算局部竖直可用区间；区间缺少已有高度层时加入中点，最多增加 8 层。完整三维航段仍检查净空。默认参数启用此策略；`p.adaptiveHeight.enable=false` 使用固定层。

本轮正式任务是原 seed 1–1000 的同图回放，原始余量为规划 4.2 m、参考 4.0 m。它与上次分方向余量实验分别统计。

## 完整批量运行

在项目根目录执行 Python `experiments3d/midpoint_sweep.py prepare --source-commit <完整Git提交>`，准备原地图和数值源码快照。准备成功后执行 `run`；四个单计算线程 MATLAB 进程分别负责原连续 250 个 seed。

运行中每个 seed 单独保存 MAT，每 5 个任务写 CSV 和状态。中断后用 `run --study <正式结果绝对路径>` 恢复已完成检查点。不要使用首轮中止目录作为最终结果，不要把中止记录与新批次合并。

全部结束后按顺序执行 `audit --study <路径>`、`analyze --study <路径>`。审核读取原始 MAT，核对地图 SHA、输入参数、源码哈希、CSV、航段净空及终端保持，不重新运行仿真。

## 看任意 seed 的 MP4

正式目录：

`F:/UAV project/UAV_APF_ADP_Formation/results/adaptive_midpoint3d_v2.3.1_20261008_154420_015`

以下代码读取已保存结果并导出动画，不另计一次试验；选择 seed 1–1000：

```matlab
addpath('src3d');
study = 'F:/UAV project/UAV_APF_ADP_Formation/results/adaptive_midpoint3d_v2.3.1_20261008_154420_015';
seed = 68;
worker = ceil(seed/250);
matFile = fullfile(study,'workers',sprintf('worker_%02d',worker), ...
    'trials',sprintf('adaptive_%06d.mat',seed));
videoFile = fullfile(study,'replays',sprintf('seed_%06d.mp4',seed));
render_3d_animation(matFile,videoFile);
```

动画入口适用于有完整飞行历史的任务。初始规划失败记录无飞行轨迹，应查看其原地图、区间决策和失败原因。

交付报告中的示例直接来自正式 MAT；全机实际高度曲线和补层决策 CSV 与该示例对应。批量仿真不为全部 1000 个 seed 渲染视频，避免绘图成为计算瓶颈。

## 结果结构

- `protocol.json`：分母、参数、来源提交和预定采样规则。
- `maps/`：原保存地图及原输入，不重新生成。
- `workers/worker_XX/trials/adaptive_XXXXXX.mat`：每 seed 的输入、输出、完整飞行历史及几何来源。
- `adaptive_results.csv`、`paired_results.csv`：全部任务结局及与原基线的配对。
- `raw_mat_audit.csv/json`：原始记录审核与逐文件 SHA。
- `figures/`：科研图 PNG、矢量 PDF、FIG 和对应源 CSV。
- `examples/`：按明确顺序选出的示例与 MP4。

`results/` 被 Git 忽略。交付归档包含正式数据、完整原基线和首轮中止记录；首轮记录不进入最终成功率。

## 版本与回滚

数值冻结提交：`e093ba9a90e0053a6d4ca9a9f6adafc1552c2667`。
数值执行标签：`adaptive-midpoint-run-v2.3.1-20261008`。
旧版回滚标签：`baseline-adaptive-height-v2.2.0-20261008`。
实验分支：`experiment/adaptive-midpoint-20261008`。

在干净工作树中可切换至上述标签查看历史源码。结果目录按版本保存，切换源码不会改变其统计口径。完成后的 Git bundle 保存本分支全部历史和关键标签；没有远端推送。

有限截面、中点及层数预算不保证任意连续三维地图一定有解。规划成功与完整编队飞行成功分别记录；本次历史同图结果不能代替独立地图或真实机体验证。
