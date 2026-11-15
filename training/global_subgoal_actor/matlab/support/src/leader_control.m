function [u, info] = leader_control(state, target, obstacles, params, leaderReached)
% LEADER_CONTROL 计算长机控制量，包含目标制导和障碍避让
    p = state(1:2, params.leaderIdx); % 长机位置
    v = state(3:4, params.leaderIdx); % 长机速度

    if leaderReached                  % 如果长机已经进入目标保持区
        att = -params.kHold * (p - target) - params.kHoldD * v; % 用 PD 控制把长机稳定在目标点
    else                              % 否则
        att = apf_attractive_force(p, v, target, params); % 用目标制导力飞向目标
    end                               % 结束 if

    [rep, risk] = apf_repulsive_force(p, target, obstacles, params); % 计算障碍物斥力
    uUAV = inter_uav_repulsion(p, v, state(1:2, :), state(3:4, :), ...
        params, params.leaderIdx); % 计算机间避让力
    wObs = params.wObsMax * sqrt(risk); % 根据风险决定障碍力权重
    u = (1 - wObs) * att + wObs * rep + uUAV; % 融合目标力、障碍力和机间避让力

    info.att = att;                   % 输出目标力，便于调试
    info.rep = rep;                   % 输出障碍力
    info.uav = uUAV;                  % 输出机间避让力
    info.risk = risk;                 % 输出当前风险
end                                   % 结束 leader_control 函数
