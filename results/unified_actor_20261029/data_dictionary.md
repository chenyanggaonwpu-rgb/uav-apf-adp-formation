# 全量重验数据说明

版本：2.3.4-unified-actor-20261029。3840 条是正式任务数，528 是地图目录项数；多种控制方法、相同地图的噪声条件和随机流不能作为新增独立地图计算。

## 文件与单位

- `trials.csv`：每个正式任务一行，3840 行；`id` 是连接其他记录的唯一任务键。
- `catalogue.mat`：全部地图几何、参数和任务，复现输入；`jobs.json` 是展开后的任务列表。
- `records/<id>.json` 与 `flight/<id>.mat`：逐任务原始摘要和完整物理轨迹；每条轨迹的 SHA256 保存在 `flightSha256`。完整 MAT 保存在 F 盘，不逐个提交到 Git。
- `independent_audit.csv`：Python 从 MAT 重新计算物理推进、限幅和扰动的检查，共3840条；动态场景另有200次独立几何核验。
- `traces/*.csv`：全部200条动态试验与9条选定抗扰/接入试验的逐步可读轨迹；`trace_index.json` 保存209个文件的路径、行数和指纹。
- `paired_records.csv`：同图配对；代价对比采用共同成功任务，成功/失败结局单独统计。
- `recovery.csv`：外扰关闭后的恢复判定，与同地图、方法、安全开关和噪声流的无外扰基线配对；位置差≤0.5m、速度差≤0.25m/s，连续1秒，并在关闭后5秒内完成。
- `serial_timing/`：串行复测的21次计时，每步周期CSV；初始化、热身、保存和绘图不计入周期，周期包含同步重规划。

位置和距离单位m，时间s，速度m/s，加速度m/s²，计算周期ms。`noiseStd` 是每个轴的位置测量噪声标准差；`gustAmplitude` 是指令限幅后叠加的外部加速度模长。

## 控制组与触发

`method=none/actor/rule` 分别为不启用脱困器、统一冻结Actor、规则脱困。`safety=0/1` 同时关闭/开启预测修正和共同制动。所有组仍执行真实物理积分、指令限幅、碰撞检测。`routePolicy=global/local` 区分全局航点与直接局部导航。`actorLoaded` 与 `actorCalls` 分别表示加载模型和实际网络调用，加载不等于接管；`activations` 统计监督器介入次数。`legacyADPCalls` 全部为0。

## 单机控制指令的记录口径

单机诊断原始 `episode.commands`、`history.command` 以及原始 JSON 的 `maxCommand` 记录进入 `motion_step` 前的请求，可能达到6m/s²。真实系统始终先按模长限到3m/s²再推进。`trials.csv` 的 `maxRequestedCommand` 保留原请求，`maxCommand` 是独立复算得到的实际限幅指令；`commandRecordScope=before_plant_limit` 标记这一区别。五机 `history.command` 已是限幅后的最终指令，标记为 `after_command_limit`。外扰加速度在限幅后叠加，因此 `maxActualAcceleration` 可以超过3m/s²。不能将请求值或外扰后的加速度当作执行器违反限幅。

## 距离、摇摆与失败

`minObstacleDistance` 和 `minPairDistance` 按原系统的真实轨迹连续段检查口径报告；障碍安全距离和机间安全距离分别按任务配置判定，不以轨迹图的视觉间隔代替。`surfaceContact`、`obstacleSafetyViolation`、`pairSafetyViolation` 分别保留实体接触与安全间隔越界。摇摆、停滞、控制换向和恢复的预先固定判据见 `analysis_protocol.json`；现象时间段和对应无人机见 `dynamic_phenomena.json`。

## 执行异常

正式3840项最终记录全部完成并通过独立复核；另保留1次内存不足的失败尝试，同源码、同模型、同输入串行重试成功。另有1次进度JSON占用引起的工作器进程退出，已保存任务未丢失，恢复后补齐。原异常记录、退出码和日志在 `attempts/`，注册表不将异常从历史中删除。早先被中断的在飞计算不当作完成任务增加样本量。
