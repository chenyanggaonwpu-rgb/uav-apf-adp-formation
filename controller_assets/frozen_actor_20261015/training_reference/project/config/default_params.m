function params = default_params()
% DEFAULT_PARAMS 返回一份默认仿真参数
% 所有物理量都使用国际单位制，工作空间是二维正方形
% 无人机用质点表示，1 号机是长机，其余 4 架是僚机

    params.softwareName   = '无人机编队智能避障仿真软件';
    params.version        = '1.1.1';

    params.dt             = 0.05;   % 仿真积分步长，单位秒
    params.Tmax           = 500;     % 最大仿真时间，单位秒
    params.nUAV           = 5;      % 无人机总数
    params.leaderIdx      = 1;      % 长机编号，1 号机是长机
    params.mass           = 1;      % 无人机质量，质点模型取 1
    params.vmax           = 3;    % 最大飞行速度，单位 m/s
    params.amax           = 3.0;    % 最大加速度，单位 m/s^2

    params.workspace      = [0 0; 80 80]; % 二维工作空间，左下角和右上角坐标
    params.start          = [10; 10];     % 长机起点坐标
    params.target         = [70; 62];     % 任务目标点坐标
    params.seed           = 5;      % 随机场景种子，保留当前设置
    params.animate        = true;   % 仿真结束后是否播放动态动画

    % 编队几何参数：长机在最前面，僚机在后面组成 V 形
    params.formSpacing    = 3.0;    % V 形前后两排之间的纵向间距
    params.formWing       = 2.5;    % 僚机相对长机中心线的横向偏移
    params.kForm          = 2.6;    % 编队位置控制增益，越大回位越快
    params.kFormD         = 1.8;    % 编队速度阻尼增益，防止振荡
    params.formGainBoost  = 0.5;    % 归队阶段温和提高编队位置增益
    params.formDampBoost  = 0.8;    % 归队阶段温和提高编队阻尼
    params.formErrScale   = 4.0;    % 队形误差的归一化尺度，供 ADP 特征使用
    params.formTol        = 1.0;    % 任务成功允许的最大队形误差，单位米
    params.vTol           = 0.25;   % 任务成功允许的最大飞行速度
    params.holdDuration   = 1.0;    % 成功条件需要连续保持的时间

    % APF 人工势场相关参数
    params.navGain        = 0.8;    % 长机期望速度与剩余距离的比例增益
    params.navAccel       = 2.0;    % 长机向期望速度逼近的加速度增益
    params.closeBoostDist = 8.0;    % 距离目标小于该值时启用近目标引力增强
    params.closeBoostGain = 2.0;    % 近目标引力增强倍数
    params.kRep           = 20.0;   % 障碍物斥力增益
    params.rho0           = 6.0;    % 障碍物斥力作用半径
    params.obsSafe        = 0.6;    % 无人机与障碍物之间的硬碰撞安全距离
    params.desiredClear   = 4.0;    % 编队期望点被障碍物挡住时向外推的安全距离
    params.uavSafe        = 1.0;    % 无人机之间的硬碰撞距离
    params.uavAvoidSafe   = 3.0;    % 无人机之间开始避让的作用半径
    params.kUAV           = 10.0;   % 无人机之间的斥力增益
    params.kUAVVel        = 5.0;    % 无人机相对靠近速度的阻尼增益
    params.collisionAvoidGain = 15.0; % 碰撞安全层的径向推力增益
    params.collisionVelGain   = 4.0;  % 碰撞安全层抵消靠近速度的增益
    params.collisionMargin    = 1.5;  % 预测式碰撞避让提前触发的距离余量
    params.useTangential  = true;   % 是否启用切向绕障力
    params.kTang          = 1.5;    % 切向绕障力增益
    params.goalDecay      = 0.035;  % 目标距离衰减系数，缓解目标不可达问题
    params.minGoalRepFactor = 0.20; % 即使接近目标也保留的最小障碍斥力比例
    params.goalTol        = 1.0;    % 长机到达目标点的判定距离
    params.holdTriggerDist = 3.0;   % 长机提前进入保持控制的距离
    params.goalScale      = 25.0;   % 目标距离特征归一化尺度，供 ADP 使用
    params.escapeThresh   = 0.2;    % 判定长机是否取得进展的距离阈值
    params.escapeWindow   = 2.0;    % 停滞多少秒后触发逃逸
    params.escapeFlipTime = 4.0;    % 逃逸方向每隔多少秒切换一次
    params.escapeMaxTime  = 5.0;    % 单次逃逸最大持续时间
    params.escapeRadius   = 8.0;    % 逃逸虚拟航点到长机的距离
    params.escapeWaypointTol = 2.0; % 逃逸虚拟航点的到达判定距离
    params.escapeTargetBias = 2.0;  % 逃逸航点选择时朝目标方向的倾向权重

    % 编队控制与障碍避让的加权融合参数
    params.wObsMax        = 0.90;   % 危险程度最高时障碍力占据的最大权重

    % 长机到达目标点后的保持控制器参数
    params.kHold          = 6.0;    % 保持控制器位置增益
    params.kHoldD         = 4.0;    % 保持控制器速度阻尼

    % 随机静态障碍物参数
    params.nObstacles     = 16;      % 障碍物数量
    params.obsRMin        = 1.2;    % 障碍物最小半径
    params.obsRMax        = 2.6;    % 障碍物最大半径
    params.corridorBand   = 50;     % 障碍物在起点到目标走廊两侧的散布宽度
    params.marginStart    = 5.0;    % 障碍物离起点的最小距离
    params.marginGoal     = 5.0;    % 障碍物离目标点的最小距离
    params.marginObs      = 1.0;    % 两个障碍物之间的最小间距
    params.pathGridResolution = 1.0; % 障碍物场景可行性检查的网格分辨率
    params.pathClearance  = 4.2;    % 规划通道需要为编队预留的横向安全距离
    params.maxObstacleSetAttempts = 60; % 重新生成整组障碍物的最大次数
    params.maxObstaclePlacementAttempts = 300; % 单个障碍物的最大放置次数

    % ADP Actor-Critic 参数
    params.adp.enable     = true;   % 是否启用 ADP 在线学习
    params.adp.gain       = 0.5;    % ADP 修正加速度的整体缩放系数
    params.adp.uMax       = 0.8;    % ADP 修正加速度的最大幅值
    params.adp.gamma      = 0.95;   % ADP 长期代价折扣因子
    params.adp.lrC        = 0.05;   % Critic 权重的学习率
    params.adp.lrA        = 0.05;   % Actor 权重的学习率
    params.adp.eps        = 0.02;   % 计算代价梯度的微小扰动
    params.adp.qRisk      = 1.0;    % 代价函数中风险项权重
    params.adp.qForm      = 0.7;    % 代价函数中队形误差项权重
    params.adp.qCtrl      = 0.03;   % 代价函数中控制量项权重
    params.adp.qTime      = 0.01;   % 代价函数中时间项权重
    params.adp.wC         = zeros(6,1); % Critic 权重初始化为零向量
    params.adp.wA         = zeros(2,6); % Actor 权重初始化为零矩阵
    params.adp.maxWeightNorm = 100; % ADP 权重异常增长时的范数上限

    params.debug          = false;  % 是否打印详细碰撞诊断信息

    % 图片输出目录：项目根目录下的 results
    rootDir = fileparts(fileparts(mfilename('fullpath'))); % 从 config 目录向上得到项目根目录
    params.resultsRoot = fullfile(rootDir, 'results');    % 所有运行结果的根目录
    params.figDir = params.resultsRoot;                   % 兼容旧调用，可由入口改为独立运行目录
end                          % 结束 default_params 函数
