function phi = adp_features(state, i, target, pDes, obstacles, params)
% ADP_FEATURES 提取第 i 架无人机的状态特征向量，供 Actor/Critic 使用
    pos = state(1:2, i);              % 读取该无人机的位置
    vel = state(3:4, i);              % 读取该无人机的速度

    dObs = params.rho0;               % 最近障碍距离先初始化为斥力作用半径
    for k = 1:numel(obstacles)        % 遍历所有障碍物
        d = norm(pos - obstacles(k).pos) - obstacles(k).r; % 到障碍物表面的距离
        dObs = min(dObs, max(d, 0));  % 取所有障碍物中的最近距离
    end                               % 结束障碍物循环
    risk = max(0, 1 - dObs / params.rho0); % 归一化风险值，越靠近障碍越大

    dGoal = norm(target - pos);       % 到目标点的距离
    goalDist = min(1, dGoal / params.goalScale); % 归一化目标距离特征
    speed = min(1, norm(vel) / params.vmax);     % 归一化速度特征

    if i == params.leaderIdx          % 如果是长机
        formErr = 0;                  % 长机不计算队形误差
    else                              % 如果是僚机
        formErr = min(1, norm(pos - pDes(:, i)) / params.formErrScale); % 归一化队形误差
    end                               % 结束 if

    align = 0;                        % 航向对齐度初始化为 0
    if norm(vel) > 0.05 && dGoal > 0.05 % 如果速度和对目标距离足够大
        vHat = vel / norm(vel);       % 速度方向的单位向量
        gHat = (target - pos) / dGoal; % 指向目标点的单位向量
        align = 0.5 * (1 - dot(vHat, gHat)); % 两方向越不齐，对齐度越大
    end                               % 结束 if

    phi = [risk; goalDist; speed; formErr; align; 1]; % 组装特征向量，最后一项是常数偏置
end                                   % 结束 adp_features 函数
