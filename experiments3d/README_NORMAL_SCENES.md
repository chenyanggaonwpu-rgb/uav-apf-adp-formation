# 正常起飞随机场景（2.3.3，2026-10-09）

**最新完成状态：**正式正常场景2000次完整任务1955成功（97.75%），全部原始MAT审核通过；[当前状态](../docs/CURRENT_STATUS.md)、[结果与CSV](../docs/three_dimensional/normal_random_2000_20261009/README.md)、[单步计时](../docs/three_dimensional/step_timing_20261009/README.md)。当前版本2.3.3，单进程复测仍待执行。

本轮按用户要求把初始有效性、离场空间和目标区域的约束写入随机生成入口。当前评估模型从空中零速度状态开始，默认高度 10 m；这不是地面垂直起飞模型。

## 使用入口

```matlab
cd('F:/UAV project/UAV_APF_ADP_Formation');
addpath('config','src','src3d','experiments3d');
p = default_params3d();
p.sceneId = 'random_normal';
p.seed = 3001;
r = main3d(p);
```

`scenario3d('random',seed,p)`、`scenario3d('random_normal',seed,p)` 和两参数的 `unstructured_scene3d(seed,p)` 现在默认生成正常随机场景。固定默认演示仍为 S8_window；控制器、高度策略、1.2/4.0 m 规划/参考余量及成功判据沿用既有设置。

配置在 `config/default_normal_scene_options3d.m`，经 `p.normalScene` 传入。环境协议的参数独立于 `pathClearance`、`desiredClear` 和 `heightLayers`，对照实验仍必须复用已冻结地图。

```matlab
p.normalScene.startClearance = 2.0;
[scene,metadata,audit] = normal_scene3d(3001,p);
% 只检查局部环境时可显式关闭见证；此时 taskFeasible=false，状态 not_checked。
p.normalScene.requireWitness = false;
```

## 默认约束

| 条件 | 默认规则 |
|---|---|
| 初始位置 | 实际五架机各自净空至少 2 m，机间距离严格大于 1 m |
| 工作空间 | 全部轨迹到工作空间边界至少 1.2 m |
| 离场 | 沿起终点方向同步移动 5 m，五条连续运动段净空至少 1.2 m |
| 终端配置 | 初始朝向对应的目标 V 形五个位置各自净空至少 4 m |
| 到达段 | 与离场相同方向的最后 5 m 连续净空至少 1.2 m |
| 长机端点 | 起点与目标点净空至少 1.2 m |
| 整任务见证 | 默认要求一个独立三维网格找到并经连续检查的五机同步平移见证 |
| 动力学/时间 | 见证各段采用先加速再减速、转角停稳的速度曲线，速度≤3 m/s、加速度≤3 m/s²，含 2 s 终端参考过渡和 1 s 保持时间不超过 120 s |
| 候选预算 | 每个请求 seed 最多尝试 100 张完整候选图，失败时明确 generation_failed |

目标 4 m 只约束目标停留配置，配合现有参考点规则；没有把 4 m 强加到整条路线。2 m 起始净空是公开的实验设定，并非航空标准或真实机体误差界。

## 独立见证与结论边界

见证搜索在 XYZ 三维网格上使用六邻接，默认网格间隔 2 m、最多展开 40000 个节点，初始朝向的 V 形偏移在同步平移中保持不变。它不调用现有有限高度层规划器、APF、ADP 或飞行控制器。障碍距离函数的 1-Lipschitz 性质用于自适应细分连续线段；不能得到足够净空下界的线段不会被认定安全。

见证是一个充分条件：实际质点系统存在满足净空、机间距离、边界和名义运动限制的运动方案。它不证明当前 APF/ADP 控制器一定能完成，也不等同于真实六自由度飞行器或有外扰时的可行性。

六邻接、网格间隔、固定 V 形平移和搜索预算均有局限。`no_grid_witness`、`search_budget_exhausted` 和 `time_budget_not_certified` 表示本方法尚未给出合格见证，不是连续空间无解的证明。它们会使默认“已确认可行”的生成协议拒绝该候选，但所有候选地图及原因保留。

全空间见证检查用于离线生成/审核；其耗时不能算作现有机载规划器的实时性能，也不改变飞行阶段的高度层数。

## 随机分布、留档与分母

每次先生成完整无约束候选：障碍数 1–40，球/圆柱/箱体等概率，尺寸及中心位置规则沿用原无结构分布，允许障碍重叠。随后按公开条件拒绝整张地图，不删减障碍，不根据当前算法的飞行结果筛图。

请求 seed 的专用 RandStream 产生 proposalSeed 序列；每张候选再用独立流生成。请求 seed、全部 proposalSeed、障碍几何、审核结果和最终接受记录都保存在 `generationAudit`。超出预算没有接受图时保留最后候选和全部拒绝记录，不能换 seed 掩盖生成失败。

接受分布是经过条件筛选的分布，其障碍数量、尺度和密度分布可能改变。没有声称接受图仍然具有无条件的均匀数量分布。`nominalVolumeFraction` 是个体体积之和，会重复计算重叠；`estimatedUnionVolumeFraction` 是规则网格中心采样得到的并集占据率估计，不是精确体积。

`main3d` 在飞行前保存 `scene_generation.mat`（含全部候选）、`generation_proposals.csv` 和 `generation_receipt.json`，再保存正式的 `simulation_result.mat`。全部路径位于 F 盘项目 results 目录。

后续批次应同时报告：候选接受数/候选总数；成功生成的请求 seed 数/预定 seed 数；完整飞行成功数/接受场景数；若生成失败，另报告完整飞行成功数/全部预定 seed 数。拒绝原因和任务失败原因分开统计。

## 历史入口

```matlab
% 原无结构、无约束分布，仍保留无效起点。
raw = scenario3d('random_unconditioned',2001,p);
[raw,meta] = unstructured_scene3d(2001,p,struct('mode','unconditioned'));
% 旧已存协议的完整平面 size/count 选项没有 mode 时，兼容旧无约束语义。
% 原四类结构化地图：混合、顶棚、窗口、交错双窗口。
structured = scenario3d('random_structured',1,p);
```

历史冻结源码、200 张旧地图及 9800 条任务记录保持原状。新条件场景不能与旧无约束场景混合汇总，也不能把旧条件子集的 92% 当作新生成器已经获得的成功率。

## 验证与回滚

验证入口 `validate_normal_scene_release3d(outputDir)` 包含负例、12 个新 seed 的几何与见证检查、旧无约束生成器检查及 200 张历史地图 SHA 一致性检查、四个既有三维回归套件，以及预定 seed 3001–3003 的三次完整任务冒烟运行。第一架次通过公开 main3d 入口运行。三次任务只检查接入，不用于正式成功率估计。

```matlab
addpath('experiments3d');
validate_normal_scene_release3d('F:/UAV project/UAV_APF_ADP_Formation/results/normal_scene_validation_new');
```

修改前标签：`baseline-normal-scenes-20261009`，指向 2.3.2 交接提交。开发分支：`feature/normal-random-scenes-20261009`。完整回滚需先保存个人未提交改动，再切回标签或原分支；不执行破坏性重置。
