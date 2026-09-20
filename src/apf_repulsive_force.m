function [a, risk] = apf_repulsive_force(p, pGoal, obstacles, params)
% APF_REPULSIVE_FORCE 计算障碍物斥力和当前风险值
    a = zeros(2, 1);                  % 初始化斥力加速度
    risk = 0;                         % 初始化风险值
    dGoal = norm(pGoal - p);          % 到当前期望目标点的距离
    goalFactor = max(params.minGoalRepFactor, ...
        1 - exp(-params.goalDecay * dGoal^2)); % 接近目标时仍保留最低安全斥力

    for k = 1:numel(obstacles)        % 遍历所有障碍物
        delta = p - obstacles(k).pos; % 从障碍物中心指向无人机
        d = norm(delta);              % 到障碍物中心的距离
        if d < 1e-6                   % 防止无人机恰好落在障碍物中心
            delta = [1e-6; 0];        % 给一个极小方向
            d = 1e-6;                 % 避免除零
        end                           % 结束 if
        dObs = d - obstacles(k).r;    % 到障碍物表面的有符号距离
        if dObs <= params.rho0        % 进入斥力范围或障碍物内部时均计算
            if dObs <= 0
                risk = 1;
                penetration = min(-dObs, params.rho0) / params.rho0;
                mag = params.kRep * (1 + penetration); % 内部状态使用强制向外推力
            else
                mag = params.kRep * (params.rho0 - dObs) / params.rho0;
                risk = max(risk, 1 - dObs / params.rho0);
            end
            mag = mag * goalFactor;
            a = a + mag * delta / d;  % 斥力方向是从障碍物指向无人机

            if params.useTangential && dObs > 0 && ...
                    dObs < 0.5 * params.rho0 && dGoal > 1e-3 % 靠近障碍物时加入切向力
                uHat = delta / d;     % 径向单位向量
                gHat = (pGoal - p) / dGoal; % 指向目标的单位向量
                crossVal = uHat(1) * gHat(2) - uHat(2) * gHat(1); % 计算叉积，判断绕行方向
                turnSign = sign(crossVal);
                if turnSign == 0
                    turnSign = 1;     % 完全共线时也给出确定的绕行方向
                end
                tang = [-uHat(2); uHat(1)] * turnSign; % 垂直于径向的切向单位向量
                a = a + params.kTang * (1 - dObs / params.rho0) * tang; % 叠加切向绕障力
            end                       % 结束 if
        end                           % 结束 if
    end                               % 结束障碍物循环
end                                   % 结束 apf_repulsive_force 函数
