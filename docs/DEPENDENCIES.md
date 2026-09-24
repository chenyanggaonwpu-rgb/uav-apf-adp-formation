# 运行依赖与第三方说明

- 运行环境：MATLAB R2024a。单次仿真和串行扫描使用基础 MATLAB；并行扫描及 `test_obstacle_sweep` 的并行一致性检查需要 Parallel Computing Toolbox。
- 无并行工具箱时可运行 `run_obstacle_sweep('UseParallel',false)`；默认启动器使用线程池。
- 项目源码未内嵌或分发第三方源代码。
- 绘图、数值计算和文件操作使用 MATLAB 自带函数。
- `MathWorks/` 目录属于本机 MATLAB 生成的环境数据，不属于本软件源码，打包或提交
  软件著作权材料时应排除。

著作权人和最终许可条款需由项目实际权利人确认后填写，项目不会自动推定个人、团队
或学校的权属。
