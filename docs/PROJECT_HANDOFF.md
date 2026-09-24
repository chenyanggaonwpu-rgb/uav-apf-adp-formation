# UAV 编队项目当前交接说明

更新日期：2026-09-24。此文档描述当前 F 盘原项目；历史记录在 `docs/archive/`，实验原始快照不得改写成新参数。

## 当前项目

- 路径：`F:\UAV project\UAV_APF_ADP_Formation`。
- 软件：无人机编队智能避障仿真软件，版本标识 V1.1.1。
- MATLAB R2024a；二维单位质量质点模型，5 架无人机、静态圆障碍。
- 算法：膨胀栅格引导、APF、领航轨迹跟随、Actor–Critic ADP、预测安全修正。
- GitHub 远端名为 `github`，Gitee 为 `origin`；两者不会自动同步。实际提交状态用 Git 查询，文档不嵌入会失效的“当前提交号”。

## 当前默认参数

| 参数 | 数值 |
| --- | --- |
| vmax / amax | 3 m/s / 3 m/s² |
| nObstacles / corridorBand / marginObs | 16 / 50 m（半宽）/ 1 m |
| pathClearance | 4.2 m（长机规划） |
| 场景生成有效净空 | 5.6 m = max(4.2,5.0+0.6) |
| seed / nUAV | 25 / 5 |
| dt / Tmax | 0.05 s / 500 s |
| workspace | [0,80] × [0,80] m |
| 起点 / 目标 | (10,10) / (70,62) m |
| 成功阈值 | 目标误差≤1 m、各僚机参考跟踪误差≤1 m、全机速度≤0.25 m/s，连续保持1 s，全程无碰撞 |

`config/default_params.m` 是参数权威来源。`corridorBand` 表示左右各 B 米的采样半宽，圆心越界后会被截回场地。`marginObs` 是边缘间距下限。巡航队形误差针对动态参考，不能等同刚性 V 形误差；末端使用固定 V 形作为基准，但仍对参考点执行避障修正。当前没有主动的障碍两侧分流决策。

## 当前实验依据

1. 主扫描：`results/obstacle_sweep_v1.1.1_20260923_215708_187/`。M∈{4,8,12,16,20}，B∈{10,15,20,25,30} m，g∈{1,2,3,4,5} m，各组合 seed 1–200。共25,000次：24,783成功、1碰撞、7超时、209生成失败、0执行异常。条件飞行成功率99.9677%，计划任务整体成功比例99.132%。
2. 默认环境补测：`results/obstacle_sweep_v1.1.1_20260924_165543_653/`。M=16、B=50 m、g=1 m，seed 1–200，200/200成功。其结果独立于主扫描。
3. 两批均限速3 m/s、长机规划净空4.2 m。原始记录、MAT检查点、源代码快照及校验值归档于 `docs/障碍物扫描原始结果_20260923_20260924.zip`。
4. 对外使用 `docs/导师报告_20260924/` 及同名“报告及数据”ZIP。原始试验值与历史快照不能为匹配新配置而改写。呈现数字按统一精度四舍五入，完整精度以CSV为准。
5. 旧100-seed基准、2026-09-17运行记录、2026-09-22的4.5 m/s报告仅是历史资料。不能当作当前默认值的测量，也不能把所有成功率混为一个指标。

## 运行和验证

双击 `Start_UAV.cmd` 演示，`Test_UAV.cmd` 核心测试，`Start_Obstacle_Sweep.cmd` 启动默认6线程扫描。

```matlab
main
run_batch(20)
cleanup = setup_project(); run_all_tests;
addpath('experiments','tests'); test_obstacle_sweep;
outputDir = run_obstacle_sweep('Workers',4);
```

并行扫描需要 Parallel Computing Toolbox；无该工具箱可用 `run_obstacle_sweep('UseParallel',false)`。恢复实验使用其冻结源码及原始参数；新实验才使用当前源码。完整参数、恢复和统计定义见 `docs/OBSTACLE_SWEEP.md`。

`results/`、`.runtime/`、MATLAB环境缓存及LaTeX编译缓存被Git忽略；正式结果通过docs中的压缩包和报告附件提交。修改当前说明后需同步更新报告PDF和压缩包。不要只改README或只替换PDF。
