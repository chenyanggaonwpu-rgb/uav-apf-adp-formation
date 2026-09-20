function state = motion_update(state, u, params)
% MOTION_UPDATE 用欧拉法更新无人机速度和位置
    state(3:4, :) = state(3:4, :) + u * params.dt; % 速度加上加速度乘时间步长
    for i = 1:size(state, 2)          % 遍历所有无人机
        v = state(3:4, i);            % 读取该机速度
        sp = norm(v);                 % 计算速度大小
        if sp > params.vmax           % 如果速度超过上限
            state(3:4, i) = v * params.vmax / sp; % 按比例缩到最大速度
        end                           % 结束 if
    end                               % 结束循环
    state(1:2, :) = state(1:2, :) + state(3:4, :) * params.dt; % 位置加上速度乘时间步长
    state = apply_boundary(state, params); % 把无人机限制在工作空间内
end                                   % 结束 motion_update 函数
