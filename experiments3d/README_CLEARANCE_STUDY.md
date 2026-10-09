# 分方向余量实验：运行、留档与回滚

## 数据目录

本次正式实验：`F:\UAV project\UAV_APF_ADP_Formation\results\clearance_boundary3d_v2.2.0_20261006_203154_442`。

- `protocol.mat/json`：冻结地图生成参数、物理模型、阶段设计、数值源码提交号。
- `source_snapshot/`、`source_sha256.csv`：所有数值函数、执行器的不可变副本及哈希。
- `maps/`、`map_manifest.csv`：2000 张随机地图及 45 张控制窗口地图；不随参数重新生成。
- `阶段名.csv`：预先列出的完整任务，不按成功结局改动。
- `stages/阶段/worker_XX/trials/*.mat`：每个任务的原始轨迹、参数、地图、随机序列和结局。
- `阶段名_results.csv`、`阶段名_summary.json`：包括所有计划任务和失败的汇总。
- `selection.json`、`selection_freeze.json`：最终候选与在独立验证前生成的 SHA-256 收据。
- `raw_mat_audit.csv/json`：逐原始记录物理距离、参数和终端保持审计。
- `raw_mat_sha256.csv`、`file_integrity.json`：全部原始 MAT 的完整性清单。
- `analysis.json`、`REPORT.md/html`、`figures/`：配对统计、报告、PNG/矢量 PDF/可编辑 FIG 与图表源数据。
- `examples/`：按声明规则选出的真实轨迹；交错窗口示例另含 MP4、抽帧与 `replay_validation.json` 对应的复现验收。

## 完整运行与中断恢复

PowerShell 中使用实际的 Python 和 MATLAB 安装路径。MATLAB 每个进程单计算线程，最多四个并行进程；独立计时只用一个进程。运行时文件和 MATLAB 偏好设置都在 F 盘项目 `.runtime/margin_study_20261006/` 下。

```powershell
$study = 'F:/UAV project/UAV_APF_ADP_Formation/results/clearance_boundary3d_v2.2.0_20261006_203154_442'
$python = 'C:/Users/24910/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
& $python 'F:/UAV project/UAV_APF_ADP_Formation/experiments3d/execute_clearance_protocol.py' $study --start screen --workers 4
```

恢复时 `--start` 指定被中断阶段。执行器读取已有任务 MAT，继续未完成的任务，并重建 worker CSV；同一阶段恢复保持 worker 数量不变。不要同时启动同阶段或在已有四个 worker 时另开计算阶段。独立验证之后不能重新执行选择来换掉候选；从 `holdout` 或之后的阶段恢复，冻结收据会验证候选没变。

原粗筛执行器：`clearance_study.py screen ...`。后续连续执行器使用快照中的原执行器，新增脚本只安排阶段、检查候选冻结收据；不修改数值模型。后处理源文件的版本与数值源文件分开记录。

`postprocess_clearance_study.py study` 可以提前启动等待数值完成，随后冻结后处理源码，依次审计 MAT、生成图表/案例、核对哈希并制作报告。只有逐任务无执行错误、审计通过，报告才能作为有效实验交付。后处理修正必须与数值源码分开记录，原始快照保持不变。

数值任务和隔离计时全部完成后，在 MATLAB 调用 `validate_clearance_replay3d(study)`，它通过用户复现入口生成 MP4，对照原始案例的全部非计时历史字段并抽帧。随后使用同一后处理快照的 `analyze_clearance_study.py` 重建报告，加入动画验收信息。这次示例复现另计，不进入 8422 次分母。

## 版本管理与回滚

- 基线：`baseline-3d-v2.1.0-20261006`，提交 `f2b0c73`。
- 数值执行版本：`experiment-clearance-run-v2.2.0-20261006`，提交 `da6b2db`。
- 实验分支：`experiment/clearance-boundary-20261006`。
- 完成交付标签：`clearance-study-v2.2.0-20261006`；最终交接记录给出完整提交号。

需要查看基线时，新建一个工作树保留当前源码和实验：

```powershell
git --git-dir='F:/UAV project/UAV_APF_ADP_Formation/.git' --work-tree='F:/UAV project/UAV_APF_ADP_Formation' worktree add --detach 'F:/UAV project/UAV_APF_ADP_Formation/.runtime/rollback_baseline' baseline-3d-v2.1.0-20261006
```

仅查看不会改变当前分支；完成后用 `git worktree remove` 清理该工作树。不要用 `reset --hard` 回滚，因为它会丢弃当前未提交改动。Git 保存源码、协议和小型统计；原始 MAT 和运行缓存保存在忽略的结果目录，通过哈希与归档另行留存。交付目录的 Git bundle 可恢复本地提交和标签，未自动推送远端。

## 指标口径

成功率按所有计划任务为分母，规划成功与完整编队飞行成功分开。名义 1000 图和独立 1000 图是两个数据集；扰动重复和控制窗口不并入随机地图成功率。性能差值仅比较同图两方案共同成功的地图；升降和平方加速度积分不是电池能耗。固定层之外加入窗口中线仅作诊断，不能冒充默认算法成功。

## 图表与数据归档

8 组图分别展示：全域参数扫描、独立分场景成功率、任务结局、噪声/外扰边界、竖向开口与高度采样、隔离计时、余量消融、实际高度同步。窗口图区分初始无路径和已飞行后失败。高度同步和原始 V 形的真实终端变形另行审计，避免把最终任务成功写成全程刚性编队保证。

`package_clearance_study.py study delivery` 将小型证据复制到 F 盘交付目录和 Git 的 `docs/three_dimensional/clearance_study_20261006/`。提交完成并生成 Git bundle 后，追加 `--archive --bundle bundle路径` 保存完整本次实验与旧 2.1.0 基线数据，执行 ZIP CRC 和 SHA-256 校验。原始结果目录保留。归档收据 `delivery_receipt.json`、交付文件哈希 `delivery_sha256.csv` 存在交付目录内。
