# 三参数障碍环境扫描

本轮研究障碍物数量、航线两侧散布半宽及障碍物最小边缘间距与任务成功率的关系。

## 实验设计

| 参数 | 默认扫描值 |
| --- | --- |
| 障碍物数量 `nObstacles` | 4、8、12、16、20 |
| 两侧散布半宽 `corridorBand` | 10、15、20、25、30 m |
| 最小边缘间距 `marginObs` | 1、2、3、4、5 m |
| seed | 每个组合 1–200 |
| 总数 | 125 组 × 200 = 25,000 次计划试验 |
| 最大速度 | 固定 3 m/s |
| 规划膨胀净空 | 当前默认 4.2 m |

其它参数取启动时 `default_params()` 的实际内容，保存到 `experiment.mat`。
不自动修改日常演示参数；不增加障碍物均匀分段采样等算法变化。
本轮是粗扫描，不自动对优胜组追加 1000/2000 个 seed。

## 运行与恢复

在项目根目录的 MATLAB 命令窗口运行：

```matlab
outputDir = run_obstacle_sweep('Workers',4);
```

也可双击 `Start_Obstacle_Sweep.cmd`，默认启动 6 线程后台运行。
启动记录和日志保存在 `.runtime/latest_obstacle_sweep.json` 及其 `log` 路径；
日志中的 `OUTPUT_DIR` 是本次结果目录。后台运行时保持电脑开机并避免休眠。
PowerShell 恢复可用 `./launch_obstacle_sweep.ps1 -ResumeDir '此前结果目录'`。

每 100 次试验保存一批，可调整 `BatchSize`。输出目录为项目的
`results/obstacle_sweep_v1.1.1_时间戳/`。这个目录包括：

- `experiment.mat`：参数、125 组设计、25,000 个任务 ID、随机生成器、MATLAB 版本。
- `source_snapshot/`：本轮使用的源码副本，与原工程同在 F 盘结果目录。
- `source_sha256.csv`：源码快照校验值；恢复时拒绝混用变动后的代码。
- `chunks/`：按批保存的 MAT 检查点，作为恢复时的权威数据。
- `trials.csv`：每个组合 × seed 的结果、时间、路径代价、队形误差与异常。
- `summary.csv`：每组计数、分母、置信区间和成功样本的效率指标。
- `progress.json`：进度、当前会话吞吐量和预计剩余时间。
- `RESULTS.md`：扫描完成（或设置有限试运行返回）后的数据说明及关系表。
- `figures/`：最终 PNG 与矢量 PDF 图表。

中断后恢复，已完成的检查点不会重跑：

```matlab
run_obstacle_sweep('ResumeDir',outputDir,'Workers',4);
```

中途在结果目录中创建空文件 `STOP_REQUESTED.txt`，程序会在保存当前批次后停止。
恢复前删除该停止文件。普通 Ctrl+C 同样可用，但当前尚未保存的一批会重算。
`RUNNING.lock` 防止两个进程同时写同一结果；若 MATLAB 意外退出而遗留锁，
确认锁文件中 PID 对应的旧进程已经结束后再删除该锁文件。

只更新图表、不重新仿真：

```matlab
addpath('experiments');
plot_obstacle_sweep(outputDir);
```

## 统计口径

- **飞行成功率** = 成功数 / 已生成且完成判定的场景数。
  分母包括 success、collision、timeout、invalid_state，不含生成失败。
- **全部试验成功比例** = 成功数 / 已执行计划 seed 数。
  生成失败计为未成功，但必须与飞行碰撞区分。
- **场景生成失败率** = 耗尽内部生成重试的 seed 数 / 已执行 seed 数。
  它不是单次候选布局拒绝率，也不是随机地图不可通行概率。
- 普通程序异常为 `execution_error`，保存错误 ID 后中止扫描；
  对应组合的成功率标为 NA，避免将软件故障当作飞行失败。
- 每组飞行成功率附 Wilson 95% 置信区间。区间是逐组的，不是 125 组的同时置信区间。
- 完成时间、路径系数和队形 RMS 的汇总仅使用成功 seed；超时结束时间不混入成功完成时间。
- 相同 seed 在不同参数下不代表相同地图。障碍数量及拒绝采样会改变随机数消耗；
  因此不能直接宣称严格配对或将同 seed 视为完全相同环境。
- 随机生成器仍预先筛选通道；场景有效筛选净空为
  `max(pathClearance, 最大编队横向偏移 + obsSafe)`，默认五机时为 5.6 m。
  结论限于此生成器接受的场景分布，无法证明任意高密度环境的成功率。
- `corridorBand=B` 表示航线左右各 B m，总采样带宽 2B；采样越界后由现有生成器
  截到场地边缘。CSV 记录靠边障碍物数，分析时应说明宽散布参数与边界截断的交互。
- 最小间距为下限，非实际平均间距；CSV 另记每张地图的实际最小障碍边缘间距。
- 从 125 组里挑出的最高成功率存在筛选偏差；最终候选需要独立新 seed 验证。

## 验证

```matlab
addpath('tests');
run_all_tests;
test_obstacle_sweep;
```

专项测试对比串行/并行结果、断点续跑去重、失败类型分母、Wilson 区间及真正不可生成的场景。
本机进程池因中文主机名触发 MATLAB 字符编码异常，默认使用内存开销更低的线程池。
逐任务显式设置 twister seed，并通过串行/并行结果一致性测试验证复现。
其它支持进程池的机器可用 `run_obstacle_sweep('PoolType','Processes','Workers',4)`。
