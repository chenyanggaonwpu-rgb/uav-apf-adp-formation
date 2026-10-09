# 项目文档索引

## 当前三维 2.3.3 · 2026-10-09

- [当前状态](CURRENT_STATUS.md)：参数、模块、正式结果、实时性与待完成事项。
- [组员协作与复现](TEAM_GUIDE.md)：GitHub、运行命令、随机入口、原始数据边界。
- [最新完整交接](HANDOFF_CURRENT_20261009.md)：2.3.3、2000 seed 与单步计时；此前交接按历史日期保存。
- [实现说明](three_dimensional/IMPLEMENTATION.md)与[可编辑信息流](three_dimensional/flow.mmd)。
- [正常场景 2000 seed](three_dimensional/normal_random_2000_20261009/README.md)：1955/2000（97.75%），全部任务与审核记录。
- [单步耗时](three_dimensional/step_timing_20261009/README.md)：平均11.67 ms；5478/1423679步超过50 ms；六进程计时，单进程复测待做。
- [正常场景规则](../experiments3d/README_NORMAL_SCENES.md)和[冻结实验运行器](../experiments3d/README_NORMAL_SEED_STUDY.md)。
- [正常场景功能验收](three_dimensional/normal_scene_constraints_20261009/README.md)。
- [所有分版本结果入口](three_dimensional/RESULT_INDEX.md)。
- [运行依赖](DEPENDENCIES.md)。

## 三维历史证据

- [2.3.2余量更新](three_dimensional/default_margin_update_20261009/README.md)。
- [2.3.1的49组扫描](three_dimensional/margin_grid49_20261008/REPORT.md)与[复现/回滚](../experiments3d/README_MARGIN_GRID49.md)。
- [无约束随机200图](three_dimensional/unstructured_random200_20261008/REPORT.md)。
- [中点补层1000图同图回放](three_dimensional/midpoint_study_20261008/REPORT.md)。
- [2.2.0分方向余量与边界](three_dimensional/clearance_study_20261006/REPORT.md)。
- [2.1.0障碍与高度验证](three_dimensional/ENVIRONMENT_UPGRADE_20261005.md)。
- [2.0.0阶段技术报告](three_dimensional/TECHNICAL_REPORT.md)、[当时教师意见反馈](three_dimensional/TASK_FEEDBACK.md)及2026-10-03代码介绍书。

## 历史交接与二维材料

- [完整阶段交接](PROJECT_HANDOFF.md)：最新状态置顶，下方按日期保留历史。
- [2000 seed阶段交接](HANDOFF_NORMAL_2000_20261009.md)。
- [正常场景接入阶段交接](HANDOFF_NORMAL_SCENES_20261009.md)。
- [2.3.2阶段交接](HANDOFF_FOR_NEW_CHAT_20261009.md)。
- [二维1.1.1说明](archive/README_v1.1.1_2d.md)与[2026-09-24导师报告](导师报告_20260924/)。
- [二维障碍扫描](OBSTACLE_SWEEP.md)与历史性能报告、CSV、数据ZIP。

各批次保留独立分母、场景分布与冻结版本。当前源码和紧凑证据可随Git迁移；完整MAT、日志与临时目录保存在F盘并被Git忽略。
