function [state, params] = init_formation(params)
% INIT_FORMATION 根据起点和目标点初始化五机 V 形编队
    psi = atan2(params.target(2) - params.start(2), ...
                params.target(1) - params.start(1)); % 初始航向指向目标点
    params.psi = psi;                 % 保存初始航向
    params.formationOffsets = formation_offsets(params); % 计算 V 形相对偏移

    R = [cos(psi), -sin(psi); sin(psi), cos(psi)]; % 根据航向构造旋转矩阵
    v0 = 0.35 * params.vmax * [cos(psi); sin(psi)]; % 初始速度沿航向方向
    state = zeros(4, params.nUAV);    % 初始化状态矩阵，每列是一架无人机
    for i = 1:params.nUAV             % 遍历所有无人机
        p = params.start + R * params.formationOffsets(:, i); % 长机在起点，僚机按偏移展开
        state(:, i) = [p; v0];        % 写入位置和速度
    end                               % 结束循环
    lo = params.workspace(1, :).' + params.obsSafe;
    hi = params.workspace(2, :).' - params.obsSafe;
    targetFormation = params.target + R * params.formationOffsets;
    if any(state(1:2, :) <= lo, 'all') || any(state(1:2, :) >= hi, 'all') || ...
            any(targetFormation <= lo, 'all') || any(targetFormation >= hi, 'all')
        error('UAV:FormationOutOfBounds', ...
            '当前机数和编队间距会使起点或目标队形越出工作空间。');
    end
end                                   % 结束 init_formation 函数

function offsets = formation_offsets(params)
% FORMATION_OFFSETS 返回 V 形编队的相对位置偏移
    offsets = zeros(2, params.nUAV);  % 初始化偏移矩阵
    offsets(:, 1) = [0; 0];           % 1 号长机偏移为 0
    for i = 2:params.nUAV
        followerNumber = i - 1;
        row = ceil(followerNumber / 2);
        side = -1;
        if mod(followerNumber, 2) == 0
            side = 1;
        end
        offsets(:, i) = [-row * params.formSpacing; ...
                          side * row * params.formWing];
    end
end                                   % 结束 formation_offsets 函数
