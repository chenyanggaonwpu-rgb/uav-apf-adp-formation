当前共享入口：[最新交接](HANDOFF_CURRENT_20261009.md)与[当前状态](CURRENT_STATUS.md)。软件2.3.3，正常2000次任务97.75%，并行步骤超50ms比例0.3848%；单进程复测尚未开始。下方为按日期保留的阶段记录。

---

最新正式测试已完成：2.3.3正常场景seed10001–12000，1955/2000（97.75%），原始MAT审核通过。当前接手请先读 `docs/HANDOFF_NORMAL_2000_20261009.md`。以下为先前阶段记录，保留供追溯。

---

# 无人机编队智能避障项目｜新对话交接

**更新时间：2026-10-09（北京时间）**

**最新后续状态：2.3.3，正常起飞随机约束已接入。请先阅读 [2.3.3 最新交接](HANDOFF_NORMAL_SCENES_20261009.md)。**

**本文下文保留 2.3.2 交接基线；涉及“当前”的代码、分支及随机入口以最新交接为准。**

**真实项目根目录：`F:\UAV project\UAV_APF_ADP_Formation`**

> 先阅读本文，再根据用户的新指令工作。当前请求只生成交接文档；没有待自动执行的新实验。应用提供的 C 盘工作目录不是项目根目录，全部项目输出继续存入 F 盘。

## 1. 接手时先知道这几件事

1. 三维默认参数已按用户要求改为 **规划余量 `pathClearance=1.2 m`、参考余量 `desiredClear=4.0 m`**，版本为 2.3.2。
2. 当前高度策略是 **5 个基础高度 + 规划失败后补可用区间中点**，最多补 8 层、合计最多 13 层。基础高度是 **5、10、15、20、25 m**，最高基础层不是 20 m。
3. 同一批无结构随机地图上的 49 组合扫描已全部完成：**49×200=9800 个正式完整任务**。最高组合为 1.2/4.0 m，成功 **121/200=60.5%**；原 4.2/4.0 m 为 **80/200=40.0%**。
4. 60.5% 是用来选择参数的旧扫描结果，**尚未用新 seed 独立验证新默认参数**。最新默认场景完整任务和四个已有回归套件已通过。
5. 不同地图生成分布必须分别统计。旧结构化 1000 图补层后 100% 与新无结构随机 200 图的 60.5% 不能直接作为同条件对照。
6. 数值代码、地图、参数、MAT、CSV、图表和归档均有留档。不要覆盖历史冻结源码，不要重跑已完成的 9800 个任务来“补交付”。

## 2. 用户目标与长期约定

项目从二维扩展为三维，主要面向航空航天应用背景，围绕 **可解释性、实时性、鲁棒性、轻量性** 展开。三维采用有限高度层规划，控制计算规模，避免直接使用密集的 80×80×80 体素搜索。

用户特别关心高度选择、余量的依据、随机地图成功率，以及失败究竟来自环境不可起飞、规划保守、离散漏解还是飞行控制。实验需要探索小余量和系统边界，不能只围绕一个局部最高点展开。

工作约定：

- **全部交付和运行输出放 F 盘**：MAT、CSV、figure、PNG/PDF/FIG、MP4、报告、日志、临时目录和 MATLAB 偏好目录。C 盘应用工作区不作为交付位置。
- 代码或参数修改做好 Git 提交和回滚入口；不要擅自推送远端。
- 对照实验复用冻结地图，保持 seed、初始状态、随机序列和学习权重重置规则一致。改变 `desiredClear` 后重新调用地图生成器，不一定还是原来那张图。
- 保留全部预定 seed 和失败结果，不替换失败地图，不通过筛掉困难环境抬高主要成功率。
- 说明主要分母、条件分母和样本来源；规划成功不等于完整任务成功。
- 数据结论能追溯到原始 MAT/CSV，图表保留源数据和绘图代码。报告用清楚的中文，风格简洁、美观。
- MP4 从真实数值轨迹导出；用户曾因找不到动画追问，交付时提供明确的 F 盘入口。
- 若后续仿真确实超出本机资源，再告知用户租算力；本轮 9800 任务已在本机完成，无需补租。
- 老师意见 PDF 是项目背景材料，其内容与用户当前授权区分；当前授权范围以用户消息为准。

## 3. 当前默认参数与模型

实际配置：[default_params3d.m](<F:/UAV project/UAV_APF_ADP_Formation/config/default_params3d.m>)。三维配置继承二维公共控制参数，再覆盖三维字段；旧二维 `default_params.m` 仍保留 1.1.1 和历史余量，复核三维不能只看二维文件。

| 项目 | 当前值 / 含义 |
|---|---|
| 无人机数量 | **5 架**，1 架长机、4 架僚机 |
| 工作空间 | 80×80×30 m |
| 默认长机起点 / 目标 | `[10;10;10]` / `[70;62;10]` m |
| 默认场景 / seed | `S8_window` / 5 |
| 基础高度 | `[5 10 15 20 25]` m |
| 临时补层 | 开启，最多新增 8 层，仅作用于本次地图/规划 |
| 水平网格分辨率 | 1 m |
| `pathClearance` | **1.2 m**，路线规划净空 |
| `desiredClear` | **4.0 m**，编队参考点调整的软余量 |
| 方向余量模式 | `directionalClearance.enable=false`，当前各向同性 |
| 竖直规划 / 参考余量 | 与 XY 同为 **1.2 / 4.0 m**；不是独立竖直余量方案 |
| 硬障碍 / 机间安全门限 | `obsSafe=0.6 m` / `uavSafe=1.0 m` |
| 速度 / 命令加速度上限 | 向量模长 3 m/s / 3 m/s² |
| 仿真步长 / 最长时间 | 0.05 s / 120 s |
| 累计升降代价权重 | `verticalCostWeight=1` |
| ADP / 预测安全层 | 开启；安全层最多 5 次迭代 |
| 当前噪声 / 加速度外扰 | 均为 0 |
| 默认动画 | `animate=false`，默认不自动导出 MP4 |

### 3.1 高度选择与规划

- 首先用基础五层建立图并寻找路线；失败后，计算若干水平采样位置的可用高度区间。
- 区间已扣除本次规划所需净空。区间内缺少现有高度时，取中点作为临时层候选。
- 当前策略标识为 `midpoint_footprint_v2`，包含投影中心、5×5 投影内采样和 3×3 重叠区域采样，按投影面积、区间宽度及升降代价安排候选，受新增 8 层预算限制。
- 每层平面内使用 8 邻接，层间主要使用同一水平位置的相邻高度连接；边要做连续线段净空检查。
- 图缓存依赖地图、数值参数和高度策略。补层不永久改写 `p.heightLayers`。
- 路线代价为 **三维路径长度 + 权重×累计高度变化**。有限采样、有限层数和离散连边没有任意连续空间完备性保证，也不能保证每张随机图顺利起飞。

### 3.2 飞行控制与成功判据

模型是静态、已知障碍地图中的三维二阶质点运动。路线提供导航目标；APF、编队参考跟踪及共享 Actor-Critic ADP 生成修正，再经过速度/加速度限制和预测安全检查。三维 ADP 的共享参数为 6 个 Critic 权重与 3×6 个 Actor 权重，共 24 个标量；6 是特征维数，不是无人机数量。

完整任务成功要求：长机目标误差≤1 m、最大参考跟踪误差≤1 m、所有无人机速度≤0.25 m/s，连续保持 1 s，并满足安全、时间及参考未阻塞等条件。判断完整任务使用 `result.success` / `result.reason`，不能只统计 `result.plan.status`。

路长、累计升降和平方命令加速度积分是性能代理量，**不是实测电池能量**。任务成功也不代表全程保持刚性 V 形、真实机体碰撞验证、六自由度飞行器验证或硬实时保证。

## 4. 最近完成的默认参数更新（2.3.2）

最近用户指令：**“那你先把两个余量都改成成功率最高的那个组合吧”**。已经完成。

- 三维 `pathClearance` 从 4.2 改为 1.2 m；`desiredClear` 原本继承 4.0 m，现在三维配置中显式写为 4.0 m。
- `VERSION` 与三维版本字段为 2.3.2；`main`、`main3d`、`Start_UAV.cmd` 使用新默认配置。
- 高度策略、控制器、ADP、硬安全门限、速度/加速度及成功判据沿用原设置。
- S8/S10 的文字描述改为不依赖某个余量数值的几何说明，未修改障碍几何。
- 历史越顶、漏层及重规划反例的测试显式使用原 4.2 m 条件；实际新默认配置另做完整任务验证。

已通过的验证：

| 验证项 | 结果 |
|---|---|
| 新默认 `S8_window`，seed 5，完整任务 | 成功 |
| 仿真任务时间 | 30.65 s |
| 最小障碍距离 | 4.2103666848 m |
| 最小机间距离 | 3.6671911555 m |
| 最高速度 | 约 3 m/s，浮点尾差按验证容差处理 |
| 四个已有回归套件 | `test_3d`、`test_environment3d`、`test_adaptive_height3d`、`test_directional_clearance3d` 全通过 |

证据：[验证说明与快照](<F:/UAV project/UAV_APF_ADP_Formation/docs/three_dimensional/default_margin_update_20261009/README.md>)、[验证收据](<F:/UAV project/UAV_APF_ADP_Formation/docs/three_dimensional/default_margin_update_20261009/validation_receipt.json>)、[正式验证 MAT](<F:/UAV project/UAV_APF_ADP_Formation/results/default_margin_v2.3.2_20261009/simulation_result.mat>)。

本轮没有重新跑新随机地图、噪声或外扰实验。不能把这个默认场景的成功写成对 60.5% 的独立确认。

## 5. 49 组合扫描：已完成，下一轮的数值依据

协议和复现：[README_MARGIN_GRID49.md](<F:/UAV project/UAV_APF_ADP_Formation/experiments3d/README_MARGIN_GRID49.md>)。详细结论：[扫描报告](<F:/UAV project/UAV_APF_ADP_Formation/docs/three_dimensional/margin_grid49_20261008/REPORT.md>)。

- 冻结版本 2.3.1；各向同性规划/参考余量分别扫描。
- 规划候选：`[0.8 1.2 1.8 2.5 3.5 4.2 5.6]` m。
- 参考候选：`[0.8 1.2 1.8 2.5 3.5 4.0 4.4]` m。
- 同一批冻结无结构随机 **seed 2001–2200**，每组 200 张地图，共 9800 个正式任务。
- 五机、高度策略、控制、ADP、安全门限及终端成功判据一致，噪声/外扰为 0。

| 组合 / 口径 | 结果 |
|---|---|
| 原基线 4.2/4.0 m，全部 200 图 | **80/200=40.0%** |
| 网格最高 1.2/4.0 m，全部 200 图 | **121/200=60.5%** |
| 同图变化 | +20.5 个百分点；救回 43、丢失 2 |
| 最高组合，固定初始安全子集 | **121/157=77.07%** |
| 最高组合的总体 Wilson 95% 区间 | 53.59%–67.02% |

最高点是这个有限网格上的唯一最高记录；多个邻近组合为 120/200，不能据 1 个 seed 的差距宣称连续参数空间唯一最优。探索数据上选出的参数还没有独立 holdout 结果。

最高组合 79 个失败：

| 原因 | 数量 |
|---|---:|
| 初始编队不安全 `invalid_scene` | 43 |
| 初始规划失败 `planning_failed` | 24 |
| 超时 `timeout` | 9 |
| 安全层未解决 `safety_unresolved` | 3 |
| 碰撞 / 越界 / 无效状态 | 0 |

审核情况：9800 个原始 MAT、每组 200 个唯一 seed、38 份冻结数值源码和 207 份输入 SHA 通过；全部 200 个原基线的非计时轨迹精确复现。缓存按每图/规划余量在飞行前冻结，供 7 个参考余量复用；不复用前一架次结束后的缓存，每次重置学习权重和随机序列。

原始任务最初 6 个单计算线程 MATLAB 进程，辅助调度时最多 10 个；运行与原始审核约 4.52 小时。2156 条辅助结果进入预定 9800，另有 91 条辅助记录保留但不进入分母。并发与共享缓存计时不能代表隔离规划性能或硬实时性。

**9800 次执行复用了 200 张图，不是 9800 张独立随机地图。** 全网格失败计数可以分析参数分布，不能把不同组合合并为一个算法的独立成功率样本。

## 6. 对“是不是起飞环境太恶劣”的已有诊断

用户在调整默认值前询问，失败率高是否主要因为障碍完全随机、起飞环境太恶劣。已有冻结地图与 CSV 复核结果：

- **43/200 张图初始编队不满足 0.6 m 硬障碍门限**；其中 37 张至少一架无人机在障碍内部或表面，6 张虽在外部但净空不足。
- 按当前固定初始状态与安全协议，这 43 张图不能有效起飞；即使余下全部完成，总体成功率也最多为 157/200=78.5%。这是该批地图与协议的上限，不是所有环境的上限。
- 在最高组合的 79 个失败里，43 个初始不安全占约 54.43%，环境有效性确实是主要来源之一。
- 最高组合全部 24 个初始规划失败都不满足当前 **1.2 m 长机起终点规划净空**：20 个仅目标不满足、3 个仅起点不满足、1 个两端都不满足；规划状态为 `connector_failed`。
- 其中 17 张目标点在障碍内部。单个目标点在障碍中不等于整个 1 m 目标接受球都无解；尚未完成独立的整任务可行性审查。
- 原 4.2/4.0 m 基线的 77 个规划失败也全部有端点净空不满足：21 个仅起点、42 个仅目标、14 个两端。

因此不能把所有失败归因于高度层漏解，也不能把所有规划失败直接当作连续空间无解。同一批图通过改变参数从 40% 到 60.5%，说明参数保守程度和飞行控制也影响结果。

若后续用户要求研究，应分别记录初始状态有效性、独立参考方法确认的可行性，以及可行任务中的算法成功率，同时保留原全部地图分母。**这只是后续研究建议，尚未获得新一轮实验指令。**

## 7. 地图分布与历史结果：必须分开理解

### 7.1 新无结构随机 200 图

本批由实验入口 `unstructured_scene3d` 生成，**不是直接使用 `scenario3d('random')`**。规则为：

- 数量在 1–40 整数中均匀抽取。
- 每个实体独立等概率选择球、圆柱、长方体。
- 球半径 1–6 m；圆柱半径 1–6 m、高 2–24 m；长方体两个水平尺寸 2–20 m、高 2–24 m。
- 先抽尺寸，再在能容纳实体的 80×80×30 m 有界区域均匀抽中心。
- 允许重叠，不预留起终点、不筛路径可达性、不替换失败地图；没有最高层必通保证。

这是声明了边界和概率分布的随机生成，不是数学意义上任意障碍分布。200 图共 4053 个实体：球 1269、圆柱 1352、箱体 1432，均值 20.265 个。

原 2.3.1、4.2/4.0 m 的结果为 80/200=40%，43 个初始不安全、77 个初始规划失败；全部成功图的初始路线由基础五层找到。本批原结果不能把成功归功于补中点。

证据：[200 图报告](<F:/UAV project/UAV_APF_ADP_Formation/docs/three_dimensional/unstructured_random200_20261008/REPORT.md>)、[随机实验与动画复现](<F:/UAV project/UAV_APF_ADP_Formation/experiments3d/README_UNSTRUCTURED_STUDY.md>)。

### 7.2 原四类结构化 1000 图

当前 `scenario3d('random')` 仍是四类结构化生成器：混合障碍、顶棚、单窗口、交错双窗口。它与上面的无结构生成器不同。

相同冻结 seed 1–1000：原固定五层 2.1.0 为 **564/1000=56.4%**，2.3.1 补中点后 **1000/1000=100%**，救回 436、丢失 0。该结论适用于这批结构化图，不是任意三维障碍下的保证。

证据：[补层报告](<F:/UAV project/UAV_APF_ADP_Formation/docs/three_dimensional/midpoint_study_20261008/REPORT.md>)、[补层复现说明](<F:/UAV project/UAV_APF_ADP_Formation/experiments3d/README_MIDPOINT_STUDY.md>)。

### 7.3 更早的方向余量与二维阶段

2.2.0 曾研究规划 XY=5.6 m、参考 XY=4.4 m、共享 Z=2 m 的候选，在独立结构窗口图 seed 1001–2000 上为 1000/1000，对照 581/1000；这是另一套地图和冻结模型，**不是当前默认参数**。该阶段还保留噪声、外扰、队形瞬态和参考投影局限的专门审计。

2.0.0 的较容易地图、二维 1.1.1 的报告及参数属于历史证据。更完整时间线见 [PROJECT_HANDOFF.md](<F:/UAV project/UAV_APF_ADP_Formation/docs/PROJECT_HANDOFF.md>)；其中旧段落的“当前”“默认”按各自日期理解，不能覆盖本文的 2.3.2 状态。

## 8. 文件入口与交付位置

| 用途 | F 盘项目内位置 |
|---|---|
| 当前代码、配置和 Git | `F:\UAV project\UAV_APF_ADP_Formation` |
| 当前配置验证 | `docs/three_dimensional/default_margin_update_20261009/` |
| 当前默认场景原始结果 | `results/default_margin_v2.3.2_20261009/` |
| 49 扫描全部原始结果 | `results/margin_grid49_v2.3.1_20261008_183125_139266/` |
| 49 扫描紧凑证据 | `docs/three_dimensional/margin_grid49_20261008/` |
| 49 扫描完整交付 | `交付文件/余量49组合扫描_20261008/` |
| 原无结构 200 图结果 | `results/unstructured_random3d_v2.3.1_20261008_172031_791/` |
| 原无结构 200 图交付 | `交付文件/无结构随机障碍_200seed_20261008/` |
| 原 1000 图补层结果 | `results/adaptive_midpoint3d_v2.3.1_20261008_154420_015/` |
| 仿真运行、缓存与临时目录 | 项目 `.runtime/` |

49 扫描紧凑证据中，`grid_summary.csv` 为 49 行组合汇总，`all_results.csv` 为 9800 条逐任务记录；`summary.json`、`raw_mat_audit.json/csv`、协议、源码/输入 SHA 和 validation 留档可用于复核。

### 8.1 图与交互页面

[三维曲面：全部 200 图](<F:/UAV project/UAV_APF_ADP_Formation/docs/three_dimensional/margin_grid49_20261008/figures/01_success_surface_all200.png>)、[曲面：初始安全 157 图](<F:/UAV project/UAV_APF_ADP_Formation/docs/three_dimensional/margin_grid49_20261008/figures/02_success_surface_safe157.png>)、[成功率热力图](<F:/UAV project/UAV_APF_ADP_Formation/docs/three_dimensional/margin_grid49_20261008/figures/03_success_heatmaps.png>)、[飞行失败热力图](<F:/UAV project/UAV_APF_ADP_Formation/docs/three_dimensional/margin_grid49_20261008/figures/04_flight_failure_heatmaps.png>)。

四组图都有 PNG/PDF/FIG 和源 CSV；PNG 已逐图检查，PDF 含部分嵌入位图，不应称为纯矢量。FIG 可在 MATLAB 中打开和旋转。

[离线交互曲面 HTML](<F:/UAV project/UAV_APF_ADP_Formation/docs/three_dimensional/margin_grid49_20261008/margin_success_surface.html>) 的嵌入数据、分母及事件逻辑检查通过，但本机 CUA/Edge 自动启动失败，**实际浏览器显示及交互未完成核验**。已有失败记录在 `validation/viewer_qa/`，不能写成浏览器验收通过。优先提供已核验的静态 PNG；用户另要求修复网页时再处理浏览器核验。

### 8.2 全量数据包与 Git bundle

完整交付目录有：

- `margin_grid49_full_data.zip`：2,260,931,515 字节，包含恰好 9800 个正式任务 MAT、200 个冻结地图、源码、原基线和辅助留档；CRC 通过。
- ZIP SHA-256：`5d23cd95f533e717127cb985ff7ed4380f834ade02451c3214c990fca6d62db1`。
- `uav_margin_grid49_history.bundle`：Git 校验通过。
- bundle SHA-256：`2efdc545f495468c152d043bdda8b695dfe35aec96a54061f345ce558caf206f`。
- `archive_receipt.json` 与 `delivery_sha256.csv` 为归档/交付收据。

**这个 bundle 截止 49 扫描完成提交 `23db08d`，不包含随后 `e91653d` 的 2.3.2 默认参数更新，也不包含本次新交接文档。** 最新状态在当前项目仓库中。`results/` 与 `交付文件/` 被 Git 忽略，迁移电脑时需单独携带原始数据/ZIP；只复制 Git 历史不会获得 MAT 和 MP4。

## 9. Git 状态与回滚入口

交接前已核对：工作树干净，当前分支 **`config/default-margin-20261009`**。本次文档会在此分支增加一个文档提交，数值代码锚点不变。

| 入口 | 提交 / 用途 |
|---|---|
| 2.3.2 代码和默认配置提交 | `e91653ddbe2d75a80d950ce434bf8e462463fc36` |
| 2.3.2 完成标签 | `default-margin-v2.3.2-20261009` → 上述代码提交 |
| 默认参数更新前标签 | `baseline-default-margin-20261009` → `23db08df418c44c0bc81e048d781f2882543a667` |
| 49 扫描完成标签 | `margin-grid49-study-20261008` → `23db08df418c44c0bc81e048d781f2882543a667` |
| 49 扫描数值冻结标签 | `margin-grid49-run-20261008` → `7bc3bb0439074f315f3ff944fa93fcfe5be5ef79` |
| 49 扫描前标签 | `baseline-margin-grid-20261008` → `f7787800e48b25c7e129203f8aead7b1ab6a1dba` |
| 49 扫描研究分支 | `experiment/margin-grid49-20261008` |

接手先检查实际状态和提交，后续用户可能已经改动：

```powershell
git -c 'safe.directory=F:/UAV project/UAV_APF_ADP_Formation' -C 'F:/UAV project/UAV_APF_ADP_Formation' status --short
git -c 'safe.directory=F:/UAV project/UAV_APF_ADP_Formation' -C 'F:/UAV project/UAV_APF_ADP_Formation' branch --show-current
git -c 'safe.directory=F:/UAV project/UAV_APF_ADP_Formation' -C 'F:/UAV project/UAV_APF_ADP_Formation' log -3 --oneline
```

在保存个人修改、确认工作树干净后，可通过 `git switch --detach <标签>` 查看历史版本；回到当前分支使用 `git switch config/default-margin-20261009`。不要未经用户要求执行破坏性重置，不要改写旧实验的冻结快照。

若新会话出现 `Permission denied` 或 `this operation must be run in a work tree`，本机此前是 F 盘项目权限未完整授予造成的。通过权限工具申请实际项目目录的写权限；不需要为此改 `core.bare` 或全局 Git 配置。`.git` 为隐藏目录，PowerShell 检查需 `Get-Item -Force`。新会话的权限应以该会话实际授予为准。

## 10. 代码入口与运行 / 动画方法

主要代码：

- 配置：[config/default_params3d.m](<F:/UAV project/UAV_APF_ADP_Formation/config/default_params3d.m>)。
- 入口：[main3d.m](<F:/UAV project/UAV_APF_ADP_Formation/main3d.m>)、[src3d/run_simulation3d.m](<F:/UAV project/UAV_APF_ADP_Formation/src3d/run_simulation3d.m>)。
- 高度与图搜索：[plan_layers3d.m](<F:/UAV project/UAV_APF_ADP_Formation/src3d/plan_layers3d.m>)、[adaptive_height_layers3d.m](<F:/UAV project/UAV_APF_ADP_Formation/src3d/adaptive_height_layers3d.m>)、[available_height_intervals3d.m](<F:/UAV project/UAV_APF_ADP_Formation/src3d/available_height_intervals3d.m>)、[plan_fixed_layers3d.m](<F:/UAV project/UAV_APF_ADP_Formation/src3d/plan_fixed_layers3d.m>)。
- 飞行和安全：[formation_reference3d.m](<F:/UAV project/UAV_APF_ADP_Formation/src3d/formation_reference3d.m>)、[control3d.m](<F:/UAV project/UAV_APF_ADP_Formation/src3d/control3d.m>)、[resolve_safety3d.m](<F:/UAV project/UAV_APF_ADP_Formation/src3d/resolve_safety3d.m>)、[check_swept3d.m](<F:/UAV project/UAV_APF_ADP_Formation/src3d/check_swept3d.m>)。
- 图和动画：[plot_result3d.m](<F:/UAV project/UAV_APF_ADP_Formation/src3d/plot_result3d.m>)、[render_3d_animation.m](<F:/UAV project/UAV_APF_ADP_Formation/src3d/render_3d_animation.m>)。
- 扫描与后处理：`experiments3d/margin_grid49.py`、`plot_margin_grid49_reviewed3d.m`、`finalize_margin_grid49.py`、`render_margin_surface.py`。

本机 MATLAB R2024a：`C:\Program Files\MATLAB\R2024a\bin\matlab.exe`。软件安装在 C 盘不等于输出应存 C 盘。批量运行使用已有 Python 启动器，将 TEMP/TMP、MATLAB 偏好目录和日志设到 F 盘 `.runtime`，后台启动隐藏窗口。MATLAB 新进程启动可能约 2 分钟，不要因短时间没有日志便判为死锁。

以下是接手后按用户指令可用的方法，**本次交接没有执行这些命令**。

### 10.1 新跑默认三维演示

```matlab
cd('F:/UAV project/UAV_APF_ADP_Formation');
r = main;
```

输出到项目 `results/` 下的新运行目录，控制台会显示绝对目录。需要新任务同时导出 MP4 时：

```matlab
cd('F:/UAV project/UAV_APF_ADP_Formation');
addpath('config','src','src3d');
p = default_params3d();
p.animate = true;
r = main3d(p);
```

### 10.2 已有 MAT 直接导出 MP4，不重算数值轨迹

```matlab
cd('F:/UAV project/UAV_APF_ADP_Formation');
addpath('config','src','src3d');
render_3d_animation( ...
    'F:/UAV project/UAV_APF_ADP_Formation/results/default_margin_v2.3.2_20261009/simulation_result.mat', ...
    'F:/UAV project/UAV_APF_ADP_Formation/交付文件/default_margin_2.3.2.mp4');
```

渲染器支持结果结构，以及包含 `result` 或 `r` 的 MAT。并非 9800 个任务都已预渲染成 MP4；有飞行历史的任务可按需导出。初始不安全/初始规划失败任务通常没有飞行轨迹，应查看地图及失败诊断，不能伪造飞行动画。任意 seed 的任务定位与历史动画导出见对应实验 README。

## 11. 尚未解决的问题与下一步边界

以下问题值得研究，但用户还没有指定下一轮任务：

- 最高组合在新 seed、不同密度/尺度、不同起终点和扰动下是否仍有效。
- 在保留主要分母的前提下，区分初始无效、目标/任务不可行与可行任务上的算法失败。
- 对 24 个端点净空失败做更细的目标区域与连续空间诊断；对 9 个超时、3 个安全未解决分析控制过程。
- 现有中点补层的有限采样、层数预算与图连边限制是否漏掉真实可行路线。
- 是否需要独立竖直余量及新的轻量高度选择策略，需重新冻结、比较并记录实验。
- 离线 HTML 的实际浏览器显示和交互仍待验收。

目前没有待续跑的正式扫描。先读取本文和新用户指令，再确定修改或实验；不要把建议列表当作自动执行授权。

---

**新对话可直接使用的提示：**

> 请阅读 `F:\UAV project\UAV_APF_ADP_Formation\docs\HANDOFF_FOR_NEW_CHAT_20261009.md`，以当前 2.3.2 状态接手。所有项目输出保存在 F 盘。阅读后等待我的下一步指令。
