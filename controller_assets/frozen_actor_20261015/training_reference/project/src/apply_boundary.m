function state = apply_boundary(state, params)
% APPLY_BOUNDARY 把无人机限制在工作空间边界内
    lo = params.workspace(1, :);      % 工作空间左下角坐标
    hi = params.workspace(2, :);      % 工作空间右上角坐标
    for i = 1:size(state, 2)          % 遍历所有无人机
        p = state(1:2, i);            % 读取位置
        v = state(3:4, i);            % 读取速度
        if p(1) < lo(1)               % 如果 x 坐标越过左边界
            p(1) = lo(1);             % 拉回左边界
            v(1) = max(v(1), 0);      % 让速度不再向左
        elseif p(1) > hi(1)           % 如果 x 坐标越过右边界
            p(1) = hi(1);             % 拉回右边界
            v(1) = min(v(1), 0);      % 让速度不再向右
        end                           % 结束 if
        if p(2) < lo(2)               % 如果 y 坐标越过下边界
            p(2) = lo(2);             % 拉回下边界
            v(2) = max(v(2), 0);      % 让速度不再向下
        elseif p(2) > hi(2)           % 如果 y 坐标越过上边界
            p(2) = hi(2);             % 拉回上边界
            v(2) = min(v(2), 0);      % 让速度不再向上
        end                           % 结束 if
        state(1:2, i) = p;            % 写回修正后的位置
        state(3:4, i) = v;            % 写回修正后的速度
    end                               % 结束无人机循环
end                                   % 结束 apply_boundary 函数
