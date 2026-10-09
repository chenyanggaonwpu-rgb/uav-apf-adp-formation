# GitHub同步说明 · 2026-10-09

## 协作入口

现有私有仓库：[chenyanggaonwpu-rgb/uav-apf-adp-formation](https://github.com/chenyanggaonwpu-rgb/uav-apf-adp-formation)。默认发布分支为`main`，组员使用已授权账号访问。[远端提交记录](https://github.com/chenyanggaonwpu-rgb/uav-apf-adp-formation/commits/main)

本次用户授权更新全部当前项目入口并同步GitHub。软件保持2.3.3，数值规划、控制、参数与正式实验成功判据没有调整。最新文件包括当前源码、接口说明、随机生成规则、2000次任务紧凑证据与单步计时CSV。

## 同步方式与历史

本机Git凭据工具枚举Windows凭据时返回Access denied；已连接GitHub工具确认具有该仓库写权限。本次通过GitHub连接发布当前文件目录树，远端新提交接续原main，采用预期SHA校验更新，保留旧分支备份。

同步前GitHub main为`229d74a123c51d8d4282aeee2c2741830a63cf82`。远端保留分支`baseline-before-v2.3.3-20261009`作为更新前入口。本地更新前标签`baseline-github-sync-20261009`指向`030ab91`，完整开发历史和原数值/实验标签仍保存在F盘仓库。

**本次GitHub发布为当前文件快照；原本地开发提交与标签没有逐条上传。**远端发布提交与本地原始开发提交的SHA不同，文件目录树会逐路径、模式和Git blob SHA核对。正式实验协议中的源码提交57d735e、数据完成提交030ab91是当时本地锚点，组员复核数值源码使用证据目录中的source_snapshot和source_sha256。

GitHub支持沿远端main历史回到更新前版本；本地可从实验分支和已有标签回到各三维阶段。不要对有个人未保存修改的目录执行破坏性重置。

## 交付范围

- 当前所有Git管理的源码、文档、紧凑CSV/JSON及历史图表、PDF和示例视频。
- 2000个seed的全部结局、拒绝候选清单、失败清单、协议、哈希和审核记录。
- 全部2000次任务的计时汇总与5478条>50 ms记录。
- 原始完整MAT、所有历史results、运行日志与临时目录继续在F盘并被Git忽略，取得完整轨迹数据的方法见[组员指南](TEAM_GUIDE.md)。

## 校验范围

发布前核对当前版本与参数、相对文档链接、统计分母及源码锚点；新可移植计时入口从原2000条MAT复算，并与已核验统计逐项比较。冻结证据目录使用Git属性保持字节，以便不同系统克隆后校验SHA-256。

上传的Git blob返回SHA与本地对象逐一比较；创建的目录树与本地树核对；main更新使用同步前远端SHA作为预期值。发布后重新读取远端main及完整目录，确认每个当前路径、模式、blob SHA一致。实际同步与校验收据保存在维护者F盘.runtime/github_sync_20261009，不在此文写入自身提交SHA。

单进程2000次耗时复测尚未启动。
