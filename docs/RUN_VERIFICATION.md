# 运行验证（2026-09-17，Windows / MATLAB R2024a）

## 启动修复

`launch_project.ps1` 将 MATLAB_PREFDIR 设置到项目的 `.runtime/preferences_R2024a`。
原用户配置不变。默认使用 `-batch` 和 `-softwareopengl`，显示图窗并在关闭所有结果
图窗后退出。`Start_UAV.cmd` 是双击启动入口，`Test_UAV.cmd` 是测试入口。

## 实测结果

- 9项核心测试全部通过；MATLAB checkcode 对全部 .m 文件给出0条消息。
- 默认seed=666：success；仿真36.65秒；长机误差约0.183米；最大队形误差约0.829米。
- main入口、PNG轨迹图、PNG指标图、MAT保存以及动画完整回放均已实际执行。
- 双击入口使用的Run模式已完成一次默认演示，日志为
  `.runtime/startup_20260917_165555_266.log`。
- seed=1..20：成功17次、碰撞2次（4、12）、超时1次（11），无未处理的仿真异常。
- 完整验证日志：`.runtime/startup_20260917_165201_293.log`。
- 批量结果：`results/batch_v1.0.0_20260917_165239_354/`。

## 结论与边界

已验证默认演示可直接运行，随机场景仍有算法失败情况。代码可执行不等同于任意
场景必定无碰撞或收敛。正式材料中的成功率应引用实测结果，不应写成100%。

复验命令：在项目目录运行 `powershell -NoProfile -File .\launch_project.ps1 -Mode Verify`。
