# 环境依据

- Python 3.12推荐，3.11–3.13 64位可用；依赖锁在requirements-base.txt。
- PyTorch固定2.8.0，GPU默认官方cu126轮子；CPU轮子只用于显式CPU方案。命令依据：[PyTorch官方历史版本](https://pytorch.org/get-started/previous-versions/)。CUDA是否实际可用由01脚本的真实计算验证。
- 预编译worker为R2024a x64；Runtime主版本必须与编译版本一致：[MathWorks安装说明](https://www.mathworks.com/help/compiler/install-the-matlab-runtime.html)。
- 官方Runtime下载入口：[MathWorks MATLAB Runtime](https://www.mathworks.com/products/compiler/matlab-runtime.html)。本仓库不打包MATLAB Runtime、Python或NVIDIA驱动。
- MATLAB R2026a源码/重编译尚未由甲方实测；乙方须在新run记录release/worker SHA并按任务书做设备侧核验。
