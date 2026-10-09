# 运行依赖与第三方说明

## 当前三维2.3.3

- Windows与MATLAB R2024a；基础三维仿真、绘图、串行测试使用MATLAB自带功能。
- 三维实验管理脚本使用Python标准库，启动独立的MATLAB进程；每进程可设singleCompThread，不要求Parallel Computing Toolbox。
- 运行器默认MATLAB路径为C:/Program Files/MATLAB/R2024a/bin/matlab.exe，组员电脑路径不同时按运行器说明调整。
- 批量仿真时间、独立地图见证、初始规划与在线循环计时分别报告。
- 维护者全部结果、日志和临时目录保存F盘项目results及.runtime。

## 历史二维

二维并行扫描及test_obstacle_sweep的并行一致性检查需要Parallel Computing Toolbox；无工具箱时可使用run_obstacle_sweep('UseParallel',false)。旧二维入口与证据按其冻结版本复现。

项目源码未内嵌第三方源代码。MathWorks/属于本机MATLAB环境数据，.runtime属于本机运行数据；均不作为项目源码提交。最终权属和许可由项目实际权利人确认。
