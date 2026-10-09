# 三维默认余量更新（2.3.2，2026-10-09）

按用户指令，将当前三维默认值设为上一轮49组合扫描的最高组合。

| 参数 | 更新前 | 当前默认 |
|---|---:|---:|
| 规划净空 `pathClearance` | 4.2 m | 1.2 m |
| 参考净空 `desiredClear` | 4.0 m（继承二维配置） | 4.0 m（三维配置显式赋值） |
| 各向同性竖直规划/参考余量 | 4.2/4.0 m | 1.2/4.0 m |

权威配置为 `config/default_params3d.m`，默认 `main`、`main3d`及`Start_UAV.cmd`经同一配置生效。版本字段与根目录VERSION更新为2.3.2。`directionalClearance.enable=false`，其规划/参考Z字段直接取两项默认值。

五个基础高度层、缺层补中点预算、控制与ADP、障碍硬安全距离0.6m、机间硬安全距离1m、速度3m/s和命令加速度3m/s²沿用原规则。旧二维配置仍由`default_params()`提供。

## 选择依据

上一轮冻结的200张无结构随机地图（seed2001–2200），1.2/4.0m完整成功121/200（60.5%），4.2/4.0m为80/200（40%）。这次仅采用该参数组合，未重新估计随机地图成功率。邻近多组60%，尚无新seed独立验证。此前实验的冻结源码、原始记录、CSV、报告及ZIP按其2.3.1版本保存。

## 验证记录

验证脚本读取实际默认配置，执行默认S8_window完整任务，并运行四个现有三维回归套件。全部通过：默认场景完整成功，仿真时间30.65s，最小障碍距离4.210367m、最小机间距离3.667191m。执行结果与源码哈希见`validation_receipt.json`和`verification_manifest.json`；完整MAT位于F盘 `results/default_margin_v2.3.2_20261009/simulation_result.mat`。

历史漏层、分方向对照、越顶及在线重规划的测试显式设为4.2m，保留原断言的几何条件；新默认场景另行执行，避免用历史测试参数代替默认配置验证。S8和S10的场景描述同步改为几何事实，不再把4.2m时的截面结果写成所有配置的结论。

## Git回滚

- 分支：`config/default-margin-20261009`。
- 更新前：`baseline-default-margin-20261009`，指向23db08df418c44c0bc81e048d781f2882543a667。
- 更新完成：`default-margin-v2.3.2-20261009`。

可在保存个人未提交修改后，用`git switch --detach baseline-default-margin-20261009`查看更新前版本，或用`git switch --detach default-margin-v2.3.2-20261009`查看更新完成版本。
