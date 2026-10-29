function [u, info] = follower_control(p, v, pDes, vDes, obstacles, ...
                                      allPos, allVel, params, selfIdx)
% FOLLOWER_CONTROL 计算僚机控制量，融合编队力和障碍避让力
    kForm = params.kForm * (1 + params.formGainBoost * params.holdMode); % 长机到点后提高编队位置增益
    kFormD = params.kFormD * (1 + params.formDampBoost * params.holdMode); % 长机到点后提高编队阻尼
    uForm = kForm * (pDes - p) + kFormD * (vDes - v); % 编队 PD 控制：位置差加速度差
    [rep, risk] = apf_repulsive_force(p, pDes, obstacles, params); % 计算障碍物斥力
    uUAV = inter_uav_repulsion(p, v, allPos, allVel, params, selfIdx); % 计算机间避让力

    wObs = params.wObsMax * sqrt(risk); % 根据风险决定障碍力权重
    % 归队阶段不再削弱障碍避让，避免把安全性让位给队形误差。
    wForm = 1 - wObs;                 % 编队力权重，危险时让位给避障力
    if risk > 0.35                    % 如果已经比较靠近障碍物
        wForm = max(0, 1 - (risk - 0.35) / 0.65); % 风险越高编队力越小
    end                               % 结束 if
    u = wForm * uForm + wObs * rep + uUAV; % 融合编队力、障碍力和机间力

    info.form = uForm;                % 输出编队控制力
    info.rep = rep;                   % 输出障碍力
    info.risk = risk;                 % 输出当前风险
end                                   % 结束 follower_control 函数
