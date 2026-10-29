function a = apf_attractive_force(p, v, pTarget, params)
% APF_ATTRACTIVE_FORCE 长机目标制导力：按剩余距离限制期望速度
    d = norm(pTarget - p);            % 当前到目标点的距离
    if d < params.goalTol             % 如果已经非常接近目标
        vDes = zeros(2, 1);           % 期望速度设为 0，准备停车
    else                              % 否则
        vDes = min(params.vmax, params.navGain * d) * (pTarget - p) / d; % 距离越近期望速度越小
    end                               % 结束 if
    accelGain = params.navAccel;      % 默认使用基础加速度增益
    if d < params.closeBoostDist      % 如果已经比较接近目标
        accelGain = params.navAccel * params.closeBoostGain; % 增强引力，让长机更快进入保持区
    end                               % 结束 if
    a = accelGain * (vDes - v);       % 用比例控制把实际速度拉向期望速度
end                                   % 结束 apf_attractive_force 函数
