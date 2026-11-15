function a = inter_uav_repulsion(p, v, allPos, allVel, params, selfIdx)
% INTER_UAV_REPULSION 计算当前无人机与其他无人机的避让力
    a = zeros(2, 1);                  % 初始化机间斥力
    for j = 1:size(allPos, 2)         % 遍历所有其他无人机
        if j == selfIdx
            continue;
        end
        d = norm(allPos(:, j) - p);   % 两机之间的距离
        if d < 1e-9                   % 如果两机几乎重合
            pairAngle = 2 * pi * (min(selfIdx, j) - 1) / max(size(allPos, 2), 1);
            direction = [cos(pairAngle); sin(pairAngle)];
            if selfIdx > j
                direction = -direction;
            end
        else
            delta = p - allPos(:, j); % 从另一架无人机指向当前无人机
            direction = delta / d;
        end                           % 结束 if
        if d < params.uavAvoidSafe    % 如果进入机间避让范围
            strength = params.kUAV * (params.uavAvoidSafe - d) / params.uavAvoidSafe; % 距离越近推力越大
            a = a + strength * direction; % 沿远离对方的方向施加斥力
        end                           % 结束 if
        relVel = v - allVel(:, j);    % 两机相对速度
        closing = dot(relVel, direction); % 沿连线的接近速度，正为远离负为靠近
        if closing < 0 && d < params.uavAvoidSafe + 2 % 只对近距离相向运动施加阻尼
            a = a + params.kUAVVel * (-closing) * direction; % 增加抵消靠近速度的力
        end                           % 结束 if
    end                               % 结束循环
end                                   % 结束 inter_uav_repulsion 函数
