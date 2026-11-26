# 全局子目标规划 Actor：训练交接入口

schema6完整训练工程已上传到 [training/global_subgoal_actor](training/global_subgoal_actor/README.md)。包含python/uav_rl、MATLAB原生接口、RTX4060配置、VAL256/MICRO32/TEST512、149/158 MAT与复现证据。完整训练checkpoint/Replay见 [同版本Release](https://github.com/chenyanggaonwpu-rgb/uav-apf-adp-formation/releases/tag/schema6-training-handoff-20261115)。

乙方先读 [六项答复与开工确认](training/global_subgoal_actor/docs/六项答复与开工确认.md)。模块路径均相对训练工程目录；仓库根目录的MATLAB部署入口仍是历史版本。

当前仿真验收版本见 [全图规则规划＋Actor Z＋APF](experiments3d/global_rule_actor_z/README.md)，附 MATLAB 演示、模型校验和同图对照结果。

---

# 无人机编队智能避障 2.3.4 统一Actor版本

当前版本 `2.3.4-unified-actor-20261029`。执行 `main` 或 `main3d`，所有已知地图统一加载10月15日完成2000回合训练的同一个冻结Actor；模型随源码交付。

3840项全量重验和两份报告已完成。五机接入成功数83→123/128；局部导航71→122/144，规则脱困为124/144。Actor有明确脱困收益，当前效果仍略低于规则；抗扰组Actor未触发，强扰动快速恢复和密集场景实时性仍需改进。

- [学生版PDF](交付文件/项目整体报告_20261029/无人机编队项目整体报告_学生版.pdf) / [Word](交付文件/项目整体报告_20261029/无人机编队项目整体报告_学生版.docx)
- [教师版PDF](交付文件/项目整体报告_20261029/无人机编队项目技术报告_教师版.pdf) / [Word](交付文件/项目整体报告_20261029/无人机编队项目技术报告_教师版.docx)
- [逐任务结果](results/unified_actor_20261029/trials.csv) / [数据字典](results/unified_actor_20261029/data_dictionary.md)
- [控制器说明和复现](docs/KNOWN_MAP_UNIFIED_ACTOR.md) / [任务书逐项反馈](交付文件/统一Actor全量重验_20261029/修订记录与任务书反馈.md)

完整MAT轨迹、209份逐步CSV和历史计算异常保存在F盘项目 `results/unified_actor_20261029`。GitHub保存源码、模型、全部地图输入、3840条原始JSON摘要压缩包、逐任务结果、独立复核和完整报告。
