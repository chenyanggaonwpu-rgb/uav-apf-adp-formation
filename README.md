# 无人机编队智能避障仿真软件 V1.1.1

这是一个 MATLAB 2D 仿真工程，用质点模型表示无人机，五架无人机以 V 字形编队
飞行，采用领航-跟随结构。长机向目标点飞行，编队内各机在改进人工势场法（APF）
和自适应动态规划（ADP）协同下避开随机生成的圆形障碍物。当长机到达目标点、
各僚机收敛到当前参考位置、全机减速并连续保持一段时间后，任务判定完成。终端参考以初始 V 形为基准，遇障时仍会调整。

现行版本标识为 V1.1.1（2026-09-24 配置），采用领航-跟随编队控制，最大速度为 **3 m/s**，长机规划膨胀净空为 **4.2 m**。巡航跟踪动态队形参考，终端以固定 V 形为参考基准并保留避障修正；不保证全航程刚性队形。

## 当前配置与结果入口

以下数值以 `config/default_params.m` 为准。扫描范围是实验自变量，不等于日常演示默认值。

| 参数 | 当前默认值 | 三参数扫描 |
| --- | --- | --- |
| 最大速度 `vmax` | 3 m/s | 固定 3 m/s |
| 最大加速度 `amax` | 3 m/s² | 固定 3 m/s² |
| 障碍物数量 `nObstacles` | 16 | 4、8、12、16、20 |
| 航线两侧散布半宽 `corridorBand` | 50 m（左右各 50 m） | 10、15、20、25、30 m |
| 最小障碍边缘间距 `marginObs` | 1 m | 1、2、3、4、5 m |
| 长机规划膨胀净空 `pathClearance` | 4.2 m | 固定 4.2 m |
| 场景筛选有效净空 | 5.6 m | 固定 5.6 m |
| 随机种子 `seed` | 25 | 每组合 1–200 |
| 时间步长 / 时间上限 | 0.05 s / 500 s | 同默认 |
| 工作空间 / 无人机数量 | 80×80 m / 5 架 | 同默认 |

生成器有效净空为 `max(pathClearance, 最大编队横向偏移 + obsSafe)`，即 `max(4.2,5.0+0.6)=5.6 m`；它与长机实际规划的 4.2 m 是不同环节。散布宽度为采样前半宽，超出工作空间时障碍圆心会被截回场地内。

- [现行导师报告及数据](docs/导师报告_20260924/)：仅使用 3 m/s 的主扫描和默认环境补测。
- [报告及数据压缩包](docs/导师报告_20260924_报告及数据.zip)。
- [完整原始结果压缩包](docs/障碍物扫描原始结果_20260923_20260924.zip)：两批实验的 MAT 检查点、参数与源码快照、CSV 和图表。
- [三参数扫描与统计口径](docs/OBSTACLE_SWEEP.md)、[当前交接说明](docs/PROJECT_HANDOFF.md)。

主扫描 25,000 次计划试验：24,783 成功、1 碰撞、7 超时、209 场景生成失败，无无效状态或执行异常。条件飞行成功率为 24,783/24,791 = **99.9677%**；含生成失败的整体成功比例为 24,783/25,000 = **99.132%**。当前默认环境（16 个障碍、50 m 半宽、1 m 间距）另测 seed 1–200，**200/200 成功**，不与主扫描混合。上述均为特定生成器下的仿真观察，不是普遍安全保证。

## 运行方式

Windows 下直接双击项目根目录的 `Start_UAV.cmd`，会通过 MATLAB 批处理启动运行
`main`、显示轨迹/指标图窗和播放动画，无需先打开 MATLAB 桌面。运行结束后图窗保留，
关闭全部结果图窗后该 MATLAB 进程退出。启动器针对本机 MATLAB R2024a 自动设置
`MATLAB_PREFDIR`，使用 `.runtime/preferences_R2024a/`，解决原有
`failed to load settings errors_warnings plugin` 启动故障。无需手动配置环境变量。
原有用户配置不被删除，启动日志保存在 `.runtime/startup_时间戳.log`。
启动器使用软件 OpenGL，以避开本机图形驱动初始化问题。

在另一台电脑使用时，可在 PowerShell 指定安装路径：

```powershell
.\launch_project.ps1 -MatlabExe 'C:\Program Files\MATLAB\R2024a\bin\matlab.exe'
```

在 MATLAB 中打开项目根目录，运行：

```matlab
main
```

也可打开 `main.m` 后点击 MATLAB 的“运行”。`main` 会自动加载源码和参数目录。
需要自定义参数并取得结果时：

```matlab
cleanup = setup_project();
p = default_params();
p.animate = false;
result = main(p);
```

默认运行一次随机障碍物仿真，在 `results/run_v1.1.1_时间戳/` 下保存轨迹图、
指标图和完整 MAT 结果，并在命令行
打印任务结果。运行结束后会自动弹出一个动态过程动画窗口，可以直观看到五机编队
绕障飞行和队形收敛过程。如果只想重新播放某次仿真结果，可以单独调用：

```matlab
cleanup = setup_project();
result = run_simulation(default_params());
play_flight_animation(result);
```

批量统计不同随机场景的成功率：

```matlab
run_batch(20)
```

每次批量运行都会写入独立的 `results/batch_v1.1.1_时间戳/` 目录，不覆盖历史结果。
批量结果包含参数、随机种子、错误信息和可复现的 MAT 数据。`run_batch` 的成功率以全部请求 seed 为分母（含异常），不能直接与三参数扫描排除生成失败后的“条件飞行成功率”混用。

障碍环境三参数扫描（固定限速 3 m/s；125 组、每组 seed 1–200）：

```matlab
outputDir = run_obstacle_sweep('Workers',4);
% 中断后继续：run_obstacle_sweep('ResumeDir',outputDir,'Workers',4)
```

支持并行计算、按批检查点、源码快照、逐组 95% 置信区间和 PNG/PDF 图表。
完整参数、统计分母及恢复方法见 [三参数扫描说明](docs/OBSTACLE_SWEEP.md)。

## 目录结构

```text
UAV_APF_ADP_Formation/
  main.m                单次演示入口
  run_batch.m           随机场景批量统计
  setup_project.m       临时配置 MATLAB 搜索路径
  config/
    default_params.m    全部仿真参数
  src/
    run_simulation.m    主仿真循环与停滞逃逸逻辑
    validate_params.m   参数合法性检查
    init_formation.m    初始 V 形编队
    generate_obstacles.m 随机障碍生成与可行性过滤
    formation_desired_positions.m 领航轨迹跟踪期望位置
    leader_control.m    长机控制
    follower_control.m  僚机控制
    apf_attractive_force.m 目标制导力
    apf_repulsive_force.m 改进斥力场
    inter_uav_repulsion.m 机间避让
    adp_features.m      ADP 状态特征
    adp_actor.m         ADP Actor 网络
    adp_critic.m        ADP Critic 网络
    adp_update.m        ADP 在线学习与策略更新
    resolve_collisions.m 预测式碰撞安全层
    play_flight_animation.m 动态过程动画播放
    plot_scene.m        场景绘制
    plot_trajectory.m   轨迹图
    plot_metrics.m      指标图
  results/              运行后自动生成
  tests/
    run_all_tests.m     核心回归测试
```

## 算法说明

### 领航-跟随编队

无人机 1 是长机，其余四架是僚机。长机实时记录历史轨迹；僚机在长机历史轨迹上
取滞后一定距离的点，再叠加 V 形横向偏移，得到期望位置；随后按障碍表面4 m净空修正期望点。该机制倾向沿长机走廊跟随，但不保证僚机实际轨迹始终安全。

长机进入目标保持区后，队形期望点切换回初始 V 形相对偏移，并以到点瞬间的航向
固定队形朝向，参考点仍保留避障修正。记录的队形误差是各僚机到这些参考点的最大距离，不能等同于刚性 V 形几何误差。

### 改进人工势场

长机使用“按剩余距离限速”的目标制导，接近目标时自动减速。障碍斥力采用带目标
距离衰减的改进势场，并加入切向分量，降低目标不可达和局部极小的影响。

### ADP

ADP 采用 Actor-Critic 结构：

- 状态特征包含最近障碍距离、目标距离、速度、队形误差和航向对齐程度；
- Critic 用线性网络逼近长期代价 J；
- Actor 输出二维修正加速度，叠加到 APF/编队控制量上；
- 在线用归一化时序差分误差更新 Critic，并用即时控制代价和下一状态代价对实际控制量
  的梯度更新 Actor；运行增益与学习梯度使用一致尺度，权重范数受到限制。

可在 `config/default_params.m` 中关闭：

```matlab
params.adp.enable = false;
```

### 局部极小逃逸与碰撞安全

长机先在带编队净空的占据栅格上规划路径，再沿经过简化的航点飞行。若出现局部
停滞，仍可选择临时逃逸航点；进入保持模式时会清除旧航点。

长机进入目标保持区后，僚机会对归队目标规划局部绕障路径；归队过程保留障碍避让
权重，不再为了快速归队主动削弱避障。安全层还会根据当前接近速度和剩余净空提前
制动，并在保持阶段保留原有预警余量。受有限加速度和路径规划离散化影响，任意
场景的无碰撞和收敛仍不能保证；检测到碰撞或达到时间上限时如实返回 `collision`
或 `timeout`。

控制量计算完成后还有一层预测式碰撞安全层：迭代预测下一步扫掠轨迹，若连续轨迹
会进入障碍安全区或机间安全区，则追加径向避让和闭合速度抵消项。最终碰撞判定也
检查相邻离散状态之间的连续线段，避免只检查采样点造成“穿透”。

### 随机障碍可行性

随机障碍生成后会在二维占据网格上搜索从起点到终点的通道，并按编队最大横向尺寸
预留净空。单个障碍放置失败时会重新生成整组场景；超过最大尝试次数会明确报错，
不再静默返回不合格布局。

## 任务完成条件

满足以下全部条件且保持 `holdDuration` 秒后任务成功：

1. 长机与目标点距离不大于 `goalTol`；
2. 所有僚机到各自当前期望位置的误差不大于 `formTol`；
3. 全机速度不大于 `vTol`；
4. 仿真过程中没有发生碰撞，且未超过 `Tmax`。

## 常用参数

所有参数在 `config/default_params.m` 中集中配置：

```text
seed              随机场景种子
nObstacles        障碍物数量
formSpacing       V 形纵向间距
formWing          每排横向偏移增量（默认 2.5 m，第二排为 5 m）
kRep              斥力增益
rho0              斥力作用半径
navGain           目标接近速度增益
closeBoostDist    近目标引力增强距离
closeBoostGain    近目标引力增强倍数
holdTriggerDist   长机开始保持的距离
escapeRadius      局部极小逃逸航点距离
collisionMargin   碰撞安全层触发余量
adp.lrC           Critic 学习率
adp.lrA           Actor 学习率
```

修改后重新运行 `main` 即可。长机栅格膨胀4.2 m，常规路径简化检查净空为3.4 m，端点连接检查为0.75 m；僚机归队栅格规划净空为1.9 m。这些是不同阶段的参数，不是实际轨迹全程净空保证。

## 测试

Windows 下双击 `Test_UAV.cmd` 可运行核心测试。完整验证（包含绘图、动画及20个
固定种子的批量场景）可在 PowerShell 运行 `.\launch_project.ps1 -Mode Verify`。

在 MATLAB 中运行：

```matlab
cleanup = setup_project();
run_all_tests
```

11 项测试覆盖参数校验、任意机数 V 形初始化、障碍物内部斥力、连续碰撞检测、重合无人机
分离、成功条件连续保持、ADP 权重有限更新、短时仿真、绕障路径和提前制动。

历史 v1.0.0/v1.1.0 的 100 个配对场景结果见 `docs/性能对比报告_v1.0.0_vs_v1.1.0.pdf`，仅代表当时版本与配置。它们不作为当前 3 m/s、16 障碍默认环境的性能结果；复现必须使用对应历史源码，见 [历史性能复现说明](docs/性能测试复现说明.md)。历史导师报告归档于 [docs/archive](docs/archive/)。

## 运行环境

- MATLAB R2024a；单次仿真可直接运行；并行扫描需要 Parallel Computing Toolbox，无该工具箱时指定 `UseParallel=false`；
- 项目不包含或分发第三方源代码；
- `MathWorks/` 若由 MATLAB 自动生成，属于本机环境数据，不是项目源码。
