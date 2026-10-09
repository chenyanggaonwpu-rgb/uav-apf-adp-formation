# 无结构随机三维场景：200 个新 seed

## 预先确定的规则

- seed 为 2001–2200，全部保留；四个单计算线程 MATLAB 进程各运行 50 个任务。
- 场地 80×80×30 m，5 架无人机，沿用 2.3.1 的起终点、控制器、余量及成功判据。
- 障碍数量在整数 1–40 上均匀抽样，每个障碍的球／竖直圆柱／轴对齐箱体类型独立等概率抽样。
- 球半径 1–6 m；圆柱半径 1–6 m、高度 2–24 m；箱体长宽各 2–20 m、高度 2–24 m，连续均匀抽样。
- 先抽尺寸，再在能够容纳该实体的场地区域内均匀抽取三维中心。尺寸与边界约束决定位置范围，因此不声称尺寸与中心完全独立。
- 障碍可重叠；不保留起终点区域，不预设窗口，不限制障碍间距，不检查可达性后重抽，不替换失败 seed。
- 圆柱上下端均随机，箱体和球中心高度也随机；没有保证最高层可通过的生成规则。范围是本轮实验定义，不代表真实空域的统计分布。

## 统计口径

主指标为完整飞行成功数 / 全部 200 个预定 seed，包含起飞前已不满足障碍安全距离的地图。另报初始五机均满足 0.6 m 障碍安全距的地图成功率，以及初始安全且长机起终点满足 4.2 m 规划净空的条件结果。后两者都不用于替换主指标。

`invalid_scene` 为起飞前初始编队不安全，不能描述成飞行中碰撞；`planning_failed` 为当前有限层规划未找到路线，不证明连续空间无解。`timeout` 等飞行结局单独保留。出现 `execution_error` 时暂停有效成功率结论，先处理数据完整性问题。

成功仍要求长机目标误差≤1 m、最大僚机对动态参考的跟踪误差≤1 m、全机速度≤0.25 m/s，连续保持 1 s，且无安全失败或超时。ADP 与安全层开启，噪声与外扰为 0。规划/参考余量为各向同性 4.2/4.0 m。

## 运行与断点恢复

项目根目录下，用 Python 运行 `experiments3d/unstructured_sweep.py`：先 `test`，再 `prepare --source-commit <完整提交号>`，最后 `run`。准备完成时冻结 200 个地图、输入和源码；逐文件 SHA 位于 `input_sha256.csv` 和 `source_sha256.csv`。

恢复时使用 `run --study <原结果绝对路径>`；从已保存 MAT 检查点继续，保持原地图和参数。完成后依次运行 `audit`、`analyze`。审核读取全部原始 MAT 并独立计算顶点物理距离，核对原文件、随机地图重建、CSV、净空和末端保持；不重新仿真。

## 数据与回放

每个 seed 的地图在 `maps/seed_XXXXXX.mat`，完整结果在 `workers/worker_XX/trials/random_XXXXXX.mat`，障碍尺寸/位置/类型在 `obstacle_catalog.csv`。地图清單记录数量、体积与起终点净空。

只有存在飞行历史的任务能导出 MP4。MATLAB 中添加 `src3d` 路径后调用：

```matlab
render_3d_animation('F:/.../workers/worker_01/trials/random_002001.mat', ...
    'F:/.../replays/seed_002001.mp4');
```

请替换为实际完整 F 盘路径。起飞前失败或初始规划失败应查看静态地图和失败原因。所有输出、缓存、MATLAB 独立偏好设置都在 F 盘项目中。

前一轮 1000 个结构化地图的成功率不能作为这次分布改变的同条件算法对照。名义障碍体积分数是各实体体积之和除以场地体积，重叠部分重复计入，不是实体并集的空间占用率。

## 本轮完成记录与实际 MP4 路径

正式原始目录：`F:/UAV project/UAV_APF_ADP_Formation/results/unstructured_random3d_v2.3.1_20261008_172031_791`。

结果80/200（40.0%），77规划失败、43起飞前不安全；初始安全子集80/157（50.96%）。全部200原始MAT审核通过。报告在 `docs/three_dimensional/unstructured_random200_20261008/REPORT.md`，交付在F盘 `交付文件/无结构随机障碍_200seed_20261008`。

下例读取本轮任意有轨迹的seed（80个）导出视频；规划失败与起飞前失败没有轨迹。

```matlab
addpath('F:/UAV project/UAV_APF_ADP_Formation/src3d');
study='F:/UAV project/UAV_APF_ADP_Formation/results/unstructured_random3d_v2.3.1_20261008_172031_791';
seed=2001;
worker=ceil((seed-2000)/50);
source=fullfile(study,'workers',sprintf('worker_%02d',worker),'trials',sprintf('random_%06d.mat',seed));
output=fullfile(study,'replays',sprintf('seed_%06d.mp4',seed));
render_3d_animation(source,output);
```

数值冻结提交 `abedb59a18fdff0c26ee88c795b5086bfb50c269`，执行标签 `unstructured-random-run-v2.3.1-20261008`，回滚标签 `baseline-unstructured-random-20261008`。完成标签 `unstructured-random-study-v2.3.1-20261008` 保存报告与绘图源码，交付Git bundle包含完整历史。运行 `package_unstructured_study.py figures/report/archive` 依次导出图表、报告和完整归档，`archive` 需要已提交的干净工作树及完成标签。
