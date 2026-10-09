# 组员协作与复现指南

## 获取当前版本

GitHub仓库：[uav-apf-adp-formation](https://github.com/chenyanggaonwpu-rgb/uav-apf-adp-formation)。仓库为私有，使用已获得仓库访问权限的GitHub账号。

```powershell
git clone https://github.com/chenyanggaonwpu-rgb/uav-apf-adp-formation.git
cd uav-apf-adp-formation
```

已有克隆先保存个人修改，再从默认`main`分支获取更新：

```powershell
git switch main
git pull --ff-only
```

先读[当前状态](CURRENT_STATUS.md)，再读[实现接口](three_dimensional/IMPLEMENTATION.md)。需要开发时从当前main建立自己的分支，并在提交说明中记录改动和验证。

## 运行

MATLAB R2024a，在项目根目录运行`main`或`main3d`。Windows启动器为`Start_UAV.cmd`。三维基础仿真不要求Parallel Computing Toolbox；Python实验管理脚本使用标准库启动独立MATLAB进程。

```matlab
cleanup = setup_project();
p = default_params3d();
p.sceneId = 'random_normal';
p.seed = 10001;
p.animate = true;
r = main3d(p);
```

输出在当前项目内results目录。当前维护者的真实根目录是F:/UAV project/UAV_APF_ADP_Formation，所有维护者运行、临时目录与交付继续位于F盘。其它电脑先克隆自己的工作目录，再按脚本说明设置MATLAB路径和临时目录。

## 在GitHub能查看什么

- 当前config、src、src3d、tests、experiments3d和入口脚本。
- 正常2000次任务的完整结局CSV、拒绝候选CSV、失败清单、参数协议、审核与冻结哈希。
- 全部2000次任务的计时汇总与5478条超过50 ms的步骤。
- 历史实验的科研图、FIG/PDF/PNG、静态网页和示例MP4；结果入口见[索引](three_dimensional/RESULT_INDEX.md)。

## 完整MAT数据

正常2000批次的maps和workers原始文件合计约596 MB，保存在维护者F盘results/normal_random_v2.3.3_2000_20261009。完整MAT、其它历史原始结果、日志、偏好及临时目录被Git忽略。克隆仓库可获得源码和紧凑证据；读取现有完整轨迹需另行取得这批数据。原始地图/任务完整性以协议中的SHA-256和审核记录为准。

需要重新运行完整实验时，按[运行器说明](../experiments3d/README_NORMAL_SEED_STUDY.md)新建结果目录，先冻结所有地图，再执行飞行与审核。不要覆盖旧目录，不替换失败seed，不把试运行并入正式统计，不继承上一个任务的ADP权重或规划缓存。

有完整MAT后，复算单步计时：

```matlab
cleanup = setup_project();
addpath('experiments3d');
folder = summarize_step_timing3d( ...
    fullfile(pwd,'results','normal_random_v2.3.3_2000_20261009'), ...
    fullfile(pwd,'results','step_timing_recomputed'));
```

该函数只读取已保存的2.3.3、seed10001–12000数据。单进程超时率需要新的独占复测，目前尚未执行。

## 版本与历史

软件版本保持2.3.3。本次更新集中在说明和证据；已有实验的冻结源码、原始数值和成功判据保持各自版本。

本地原始开发历史、数值发布标签与回滚标签保存在维护者仓库。GitHub发布与本地同步方式、目录树校验及远端提交见[同步说明](GITHUB_SYNC_20261009.md)。实验协议里的源码提交是运行时本地锚点，复核对应源码优先使用随证据保存的source_snapshot和source_sha256。

## 待处理问题

- 单进程复测与模块耗时分解尚未开始。
- 在线重规划与终端规划仍会阻塞同步循环，当前没有严格50 ms上界。
- 本批45个失败仍需按安全修正、终端控制和重规划原因分别诊断。
- 当前没有动态障碍、六自由度或实际电池能量模型。
