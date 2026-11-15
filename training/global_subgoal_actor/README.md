# schema6 全局子目标规划 Actor 训练工程

这是任务书v1.0所需的独立训练工程。仓库内相对路径：**`training/global_subgoal_actor`**。`python/uav_rl/` 等路径均相对于本目录。它与仓库根目录2.3.4/2.3.6旧MATLAB部署入口分开。

- `actor_schema=6`，`action_semantics=normalized_global_subgoal_xyz`，`task_protocol=global_subgoal_v1`。
- 3340输入，512→512→256→256，LayerNorm/ReLU，直接SAC，BC/专家=0；Actor从t=0规划，1.6秒持有，事件最短0.4秒重选。
- 默认RTX4060/8GB：CUDA，batch32，双原生worker；所有训练数据/缓存默认F盘。
- 包含VAL256、MICRO32、TEST512、16类标签/几何哈希/种子清单、原生R2024a worker、两组MAT及验证记录。
- 大体积完整检查点、优化器、RNG与1932份Replay episode在[同版本Release](https://github.com/chenyanggaonwpu-rgb/uav-apf-adp-formation/releases/tag/schema6-training-handoff-20261115)，下载后SHA校验。

## 先读

1. [六项答复与开工确认](docs/六项答复与开工确认.md)：MATLAB版本、三种子24run、方法口径、分期验收。
2. [原任务书v1.0](docs/任务书_v1.0.md)。
3. [模型与检查点说明](docs/模型与检查点.md)、[预算与数据封存](docs/预算与数据封存.md)。

## Windows快速开始

安装Git、64位Python3.12（3.11–3.13可用）、NVIDIA驱动和**MATLAB Runtime R2024a x64**。已有MATLAB R2026a可以保留；现成EXE仍需配套R2024a Runtime。

```powershell
git clone https://github.com/chenyanggaonwpu-rgb/uav-apf-adp-formation.git
cd uav-apf-adp-formation/training/global_subgoal_actor
git rev-parse HEAD
powershell -ExecutionPolicy Bypass -File .\00_setup.ps1 -WorkRoot F:\UAV_Schema6
powershell -ExecutionPolicy Bypass -File .\01_preflight.ps1 -WorkRoot F:\UAV_Schema6
powershell -ExecutionPolicy Bypass -File .\02_smoke.ps1 -WorkRoot F:\UAV_Schema6
```

需要显式指定已安装Runtime时，为00脚本加 `-MatlabRuntimeRoot "C:\Program Files\MATLAB\MATLAB Runtime\R2024a"`，以实际目录为准。脚本不会自动启动付费云，也不会恢复甲方已暂停的任务。

自检和小预算确认后，新训练：

```powershell
powershell -ExecutionPolicy Bypass -File .\03_train.ps1 -WorkRoot F:\UAV_Schema6 -Run F_s20261110 -Seed 20261110
powershell -ExecutionPolicy Bypass -File .\04_monitor.ps1 -WorkRoot F:\UAV_Schema6 -Run F_s20261110
```

05暂停，06续训。源码、配置、worker改变后新建run，不修改哈希绕过恢复检查。其余种子显式传 `-Seed 20261111`、`-Seed 20261112`。`config/ABLATION_RUN_MATRIX.csv`登记24个run；A1–A7仍需乙方按任务书实现和双端验证，**矩阵文件不代表消融代码或结果已经完成**。

## 复现149/158

Python路径是工作根venv；下例在00脚本创建环境后运行：

```powershell
F:\UAV_Schema6\venv\Scripts\python.exe -B tools/reproduce.py --model retained149 --split micro --limit 2 --work-root F:\UAV_Schema6 --audit
F:\UAV_Schema6\venv\Scripts\python.exe -B tools/reproduce.py --model candidate158 --split validation --work-root F:\UAV_Schema6
powershell -ExecutionPolicy Bypass -File .\09_fetch_checkpoint_assets.ps1 -WorkRoot F:\UAV_Schema6
```

不带limit且split=validation才是完整VAL256复现。结果写入工作根 `reproductions`，同一模型/worker/code/case哈希绑定缓存。

需要保留学习状态续训时，下载检查点后执行：

```powershell
F:\UAV_Schema6\venv\Scripts\python.exe -B tools/import_checkpoint.py --asset retained149 --run continuation149 --work-root F:\UAV_Schema6 --device cuda
powershell -ExecutionPolicy Bypass -File .\06_resume.ps1 -WorkRoot F:\UAV_Schema6 -Run continuation149
```

导入建立有来源记录的新阶段，保留Actor/Critic/target/优化器/Replay/RNG，并计入历史物理预算。**正式24run不能用成熟模型暖启动**；全部从对应随机种子共同初始化方案开始。158是被拒绝候选，不是默认续训或交付起点。

## 数据、预算与验证

甲方直接提供TEST512；乙方不自行另生成替换。测试前用 `tools/lock_panel.py`锁定完整模型面板与唯一主模型，之后07脚本运行一次独立测试/冻结。VAL256与MICRO32可开发使用，TEST512不选模。

正式首档每run **B=10,000,000联合物理步、SAC/Critic更新上限200,000、更新/新决策转移比0.25**，每flight最多32次更新。物理步包括暖机、失败、探索、回退前的实际训练；VAL/TEST另记。每次联合步包含五架无人机的一个0.05秒物理区间。完整任务不截断，剩余不足2400步时结束，少用最多2399步。

每100万联合训练步MICRO32、每200万步VAL256、预算末尾补一次完整VAL；以实际步数记录节点，整任务/并行组可带来小幅节点偏移。主选模规则见任务书，不用MICRO选最终模型。已提供的F实现不等于完成任务书新增的全部报告与多种子统计。

## 验证范围

甲方本机仅有CPU与R2024a。移植工程需通过本包 `provenance/PORTABLE_CHECKS.json`记录的实际检查；RTX4060 CUDA及R2026a重编译由乙方完成设备侧检查，不能把甲方CPU通过写成GPU通过。

原训练源码、配置和模型记录与便携改动分开保存。本机原两套长训练仍保持暂停。本包是可复现工程与交接材料，不是新训练已完成或性能达标证明。
