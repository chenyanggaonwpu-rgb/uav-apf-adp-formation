# 全图规则规划＋Actor Z＋APF

在已知三维障碍地图中，规则规划器生成领机全图航线。Actor 只建议下一航点的高度。候选高度通过五机两段参考线的净空检查后才会采用，否则沿用规则航点。领机用 APF 跟踪航点，从机跟踪编队参考。

## 运行

使用 MATLAB R2024a，在 MATLAB 中进入本目录并运行以下命令。

    run_demo

程序运行一张单窗口图和一张多窗口图，使用 [retained149.mat](../../training/global_subgoal_actor/assets/reference_models/retained149.mat)。它会在项目根目录的 demo_output 下创建时间戳子目录，保存 summary.json、flights.jsonl 和 raw 目录中的轨迹 MAT 文件。也可调用 run_demo(outDir) 指定位置。

批量复验使用 run_bench(campaign, casesFile, outDir)。campaign 指向项目的 training/global_subgoal_actor，casesFile 为地图和扰动参数的 JSON 文件。默认方案为 global_xy_z。默认模型的 SHA-256 为 b2c051cacb795849dcfe223e85230f2657a9e17ab201809b33f974bef16c674a。

## 768 张新图的同图对照

| 方案 | 安全成功 | 安全违规 | 严重事件 |
| --- | ---: | ---: | ---: |
| 全图规则规划＋APF | 740/768 | 12 | 0 |
| 全图规则规划＋Actor Z＋APF | 747/768 | 13 | 0 |
| 全图规则规划＋G 开关 Actor＋APF | 736/768 | 16 | 0 |

相对全图规则规划＋APF，Actor Z 组的安全成功率增加 0.91 个百分点，95% 配对区间为 −0.13 至 +1.95 个百分点。结果使用同一份冻结模型。
