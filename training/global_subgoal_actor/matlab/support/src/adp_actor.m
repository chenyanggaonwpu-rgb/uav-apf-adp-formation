function u = adp_actor(phi, params)
% ADP Actor 网络：根据状态特征输出修正加速度
    if ~params.adp.enable
        u = zeros(2, 1);
        return;
    end
    u = params.adp.wA * phi;          % 用 Actor 权重矩阵乘特征向量
    uMax = params.adp.uMax;           % 读取最大修正加速度
    u = max(-uMax, min(uMax, u));     % 把修正加速度限制在允许范围内
end                                   % 结束 adp_actor 函数
