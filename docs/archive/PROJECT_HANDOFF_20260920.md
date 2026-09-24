# UAV 编队项目交接说明

> 历史交接快照（2026-09-20）：以下版本号、提交号和默认参数已过时。当前状态请以 `config/default_params.m`、`docs/OBSTACLE_SWEEP.md` 和 `docs/导师报告_20260924/` 为准。

> 更新日期：2026-09-20。供新对话快速接手；具体行为以当前源码和重新运行的测试为准。

## 一句话概况

项目位于 `F:\UAV project\UAV_APF_ADP_Formation`，当前版本为 **v1.1.0**。它是 MATLAB R2024a 的二维质点仿真：默认 5 架无人机采用领航—跟随 V 形编队，使用栅格路径引导、改进人工势场（APF）、Actor–Critic 自适应动态规划（ADP）和预测式避碰，在随机静态圆形障碍物环境中抵达目标并重新收敛成队。

## 当前已核实的状态

- 本地 Git 分支：`main`；检查交接文档写入前，工作区干净，提交为 `9caf45d`（`Import UAV formation project v1.1.0`）。本文件是新增加的本地文档，**尚未提交或推送**。
- `origin` 指向 Gitee：`https://gitee.com/chenyanggaonwpu/uav_-apf_-adp_-formation.git`；`github` 指向 GitHub：`https://github.com/chenyanggaonwpu-rgb/uav-apf-adp-formation.git`。本地记录的 `origin/main` 与 `github/main` 均为 `9caf45d`；本次没有联网核查远端最新状态。
- 用户已确认 Gitee 上传成功。GitHub 协作者是否已邀请、是否已接受邀请，尚未确认。GitHub 与 Gitee **不会自动同步**；后续提交需要分别推送。
- 本机存在 `C:\Program Files\MATLAB\R2024a\bin\matlab.exe`。本次交接仅查阅源码和现有结果，**未重新运行 MATLAB 仿真或测试**。
- `.gitignore` 排除 `MathWorks/`、`.runtime/`、`results/` 及 MATLAB 临时文件。这些目录不应当作已上传的项目源码。

## 如何运行与验证

在 Windows 上双击根目录的 `Start_UAV.cmd` 运行演示，双击 `Test_UAV.cmd` 运行核心测试。`launch_project.ps1` 会使用项目独立的 `.runtime/preferences_R2024a/` 作为 `MATLAB_PREFDIR`，以规避此机器曾出现的 `failed to load settings errors_warnings plugin` 启动错误；输出日志在 `.runtime/`，仿真图和 MAT 结果在 `results/`。入口支持 `Run`、`Test`、`Batch`、`Verify` 模式。

在 MATLAB 命令窗口也可运行：

```matlab
main                              % 使用 default_params 的单次演示
run_batch(20)                     % 20 个 seed 的批量统计
cleanup = setup_project(); run_all_tests  % 11 项核心回归测试
```

完整复验（测试、默认仿真、绘图/动画、20 个 seed 和静态检查）可在项目目录的 PowerShell 中运行：

```powershell
.\launch_project.ps1 -Mode Verify
```

当前 `config/default_params.m` 的默认 `seed=33`、`dt=0.05 s`、`Tmax=500 s`、`nUAV=5`、`nObstacles=8`，场地为 `80×80 m`。**旧版** `docs/RUN_VERIFICATION.md` 记录的是 2026-09-17 的 v1.0.0、`seed=666` 验证，不能当作当前默认参数的最新实测结果。

## 代码入口与控制流程

| 文件 | 作用 |
| --- | --- |
| `main.m` | 单次运行、保存 `simulation_result.mat`、轨迹/指标图及动画。 |
| `run_batch.m` | 按 seed 批量运行并保存成功率、失败原因和可复现结果。 |
| `config/default_params.m` | 全部场景、编队、APF、安全层与 ADP 参数。 |
| `src/run_simulation.m` | 主时间步循环；计算期望队形、控制、状态更新、在线学习与完成判定。 |
| `src/plan_grid_path.m` | 基于障碍净空的栅格路径规划和航点简化。 |
| `src/leader_control.m`、`src/follower_control.m` | 长机导航、僚机队形保持与避障控制。 |
| `src/adp_features.m`、`src/adp_actor.m`、`src/adp_critic.m`、`src/adp_update.m` | 状态特征、Actor/Critic 输出及在线更新。 |
| `src/resolve_collisions.m`、`src/check_collisions.m` | 预测避碰、提前制动和连续扫掠碰撞判定。 |
| `tests/run_all_tests.m`、`tests/verify_project.m` | 核心回归测试和完整复验。 |

控制大致按“长机/僚机基础控制量 → ADP 加速度修正 → 加速度限幅 → 预测式碰撞安全层 → 运动积分 → ADP 更新”执行。长机预先规划具有编队净空的路径；僚机跟随长机轨迹，长机到目标后再绕障归队。成功要求长机进入目标容差、队形误差及全机速度低于阈值，并连续保持 `holdDuration`，全过程无碰撞且不超过 `Tmax`。

## 已有性能证据与边界

`docs/benchmark_baseline.csv` 和 `docs/benchmark_improved.csv` 各有 100 个相同 seed 的场景记录：v1.0.0 为 **95/100 成功**（3 次碰撞、2 次超时），v1.1.0 为 **100/100 成功**。详细口径、图表见 `docs/性能对比报告_设计升级版_v1.0.0_vs_v1.1.0.pdf` 和 `docs/性能测试复现说明.md`；原版源码快照在 `docs/baseline_v1.0.0_source_20260919.zip`。这些是历史测试结果，本次交接未重跑。

100/100 仅描述已测的二维、静态圆障碍场景，不等于任意随机种子、动态障碍或真实无人机的安全保证。v1.1.0 同时改进了规划、归队和避碰；现有对比**不能单独证明 ADP 的增益**。如果继续研究算法，建议先固定 seed、障碍布局和评估口径，做 APF、规划、ADP、安全层的消融对比，并扩展高密度障碍与失败场景。

## 后续工作与协作注意

1. 新对话先阅读本文件、`README.md`、`config/default_params.m` 和与任务相关的 `src/` 文件；动代码后运行核心测试，必要时再做完整验证。
2. 如果要让队友在 GitHub 修改私有仓库，需要在 GitHub 仓库的 **Settings → Collaborators** 发邀请，并由队友接受；当前是否完成尚未确认。多人修改建议用各自分支和 Pull request 合并。
3. 新文档目前只在本机；若希望队友从 GitHub/Gitee 读到，需要在审核后提交并分别推送到两处远端。推送前复查待提交文件，特别是不要包含 `.runtime/`、`results/`、凭据或本机环境数据。
4. 本机用户账户与项目目录所有者曾不一致，Git 可能提示 `dubious ownership`。既有处理方式是在每条 Git 命令上使用项目专属的 `-c "safe.directory=F:/UAV project/UAV_APF_ADP_Formation"`；不要因为交接文档而改全局信任设置。已有 Gitee 上传辅助脚本位于被忽略的 `.runtime/gitee-upload/`。
5. 若后续准备软件著作权材料，还需由实际权利人确认著作权人及许可/授权信息；`docs/DEPENDENCIES.md` 记载了运行依赖和第三方代码情况。

## 给新对话的开场语

> 请先阅读 `F:\UAV project\UAV_APF_ADP_Formation\docs\PROJECT_HANDOFF.md`，再核对项目当前 Git 状态和相关源码。项目是 MATLAB R2024a 的 UAV APF+ADP 编队避障仿真。请以当前文件和实测结果为准，不把历史 100/100 测试推广为普遍保证。我接下来想做的是：〔在这里写你的新任务〕。
