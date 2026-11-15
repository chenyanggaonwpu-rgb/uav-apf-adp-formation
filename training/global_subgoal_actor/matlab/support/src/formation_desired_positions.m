function [pDes, vDes, psi] = formation_desired_positions(state, obstacles, params)
% FORMATION_DESIRED_POSITIONS 计算每架僚机应飞向的期望位置
    pL = state(1:2, params.leaderIdx); % 长机当前位置
    vL = state(3:4, params.leaderIdx); % 长机当前速度
    speed = norm(vL);                 % 长机速度大小
    if params.holdMode && isfield(params, 'holdPsi') % 如果进入保持阶段
        psi = params.holdPsi;         % 使用到点瞬间固定下来的编队航向
    elseif speed > 0.05               % 如果长机还在移动
        psi = atan2(vL(2), vL(1));    % 用速度方向作为编队航向
    else                              % 否则
        psi = params.psi;             % 沿用上一次保存的航向
    end                               % 结束 if

    pDes = zeros(2, params.nUAV);     % 初始化期望位置矩阵
    vDes = zeros(2, params.nUAV);     % 初始化期望速度矩阵
    pDes(:, 1) = pL;                  % 长机期望位置就是当前位置
    vDes(:, 1) = vL;                  % 长机期望速度就是当前速度

    if params.holdMode && isfield(params, 'holdPsi') % 保持阶段使用固定 V 形
        R = [cos(psi), -sin(psi); sin(psi), cos(psi)]; % 按固定航向构造旋转矩阵
        for i = 2:params.nUAV         % 遍历所有僚机
            pDes(:, i) = pL + R * params.formationOffsets(:, i); % 用初始 V 形相对偏移计算期望位置
            pDes(:, i) = obstacle_clear_desired(pDes(:, i), obstacles, params); % 避开位于障碍内的期望点
            vDes(:, i) = vL;          % 期望速度沿用长机速度
        end                           % 结束僚机循环
    else                              % 飞行阶段使用长机历史轨迹
        for i = 2:params.nUAV         % 遍历所有僚机
            row = floor((i - 2) / 2) + 1; % 计算僚机所在排数，2、3 号是第一排，4、5 号是第二排
            side = -1;                % 默认排在左侧
            if mod(i, 2) == 1         % 如果是奇数编号
                side = 1;             % 排在右侧
            end                       % 结束 if
            distBack = row * params.formSpacing; % 该僚机相对长机的纵向滞后距离
            [pTrail, dir, trailLen] = trail_point(params.leaderTrail, ...
                params.leaderTrailCumLen, params.leaderTrailCount, distBack, psi); % 在长机轨迹上取滞后点
            lateral = side * row * params.formWing; % 横向偏移量
            pDes(:, i) = pTrail + lateral * [-dir(2); dir(1)] ...
                         - max(0, distBack - trailLen) * dir; % 滞后点加横向偏移得到期望位置
            pDes(:, i) = obstacle_clear_desired(pDes(:, i), obstacles, params); % 避开位于障碍内的期望点
            vDes(:, i) = speed * dir; % 期望速度沿滞后轨迹的局部切向
        end                           % 结束僚机循环
    end                               % 结束分支
end                                   % 结束 formation_desired_positions 函数

function [pTrail, dir, trailLen] = trail_point(trail, cumLen, n, distBack, psi)
% TRAIL_POINT 在长机历史轨迹上找到滞后 distBack 米的点
    if n == 1                         % 如果轨迹只有一个点
        pTrail = trail(:, 1);         % 直接返回该点
        dir = [cos(psi); sin(psi)];   % 方向用当前航向
        trailLen = 0;                 % 轨迹长度为 0
        return;                       % 返回
    end                               % 结束 if

    trailLen = cumLen(n);             % 使用主循环增量维护的累计弧长
    sTarget = max(0, trailLen - distBack); % 目标弧长位置

    left = 1;
    right = n;
    while left < right
        middle = floor((left + right) / 2);
        if cumLen(middle) >= sTarget
            right = middle;
        else
            left = middle + 1;
        end
    end
    idx = left; % 二分查找也适用于长时间停滞导致的重复弧长
    if idx == 1                       % 如果目标位置在第一个点
        pTrail = trail(:, 1);         % 直接使用第一个点
    else                              % 否则
        segmentLength = cumLen(idx) - cumLen(idx - 1);
        frac = (sTarget - cumLen(idx - 1)) / max(segmentLength, eps); % 在线段内插值比例
        pTrail = trail(:, idx - 1) + frac * (trail(:, idx) - trail(:, idx - 1)); % 插值得到精确点
    end                               % 结束 if

    i0 = max(1, idx - 1);             % 取前一个轨迹点
    i1 = min(n, idx + 1);             % 取后一个轨迹点
    d = trail(:, i1) - trail(:, i0);  % 用前后点构造方向向量
    if norm(d) > 1e-6                 % 如果方向有效
        dir = d / norm(d);            % 归一化得到方向
    else                              % 否则
        dir = [cos(psi); sin(psi)];   % 用航向兜底
    end                               % 结束 if
end                                   % 结束 trail_point 函数

function p = obstacle_clear_desired(p, obstacles, params)
% OBSTACLE_CLEAR_DESIRED 如果期望点落在障碍内，就把它向外推
    lo = params.workspace(1, :).' + params.obsSafe;
    hi = params.workspace(2, :).' - params.obsSafe;
    maxIterations = max(3, 2 * numel(obstacles) + 1);
    for iter = 1:maxIterations
        changed = false;
        p = max(lo, min(hi, p));
        for k = 1:numel(obstacles)
            v = p - obstacles(k).pos;
            d = norm(v);
            minD = obstacles(k).r + params.desiredClear;
            if d < minD
                if d < 1e-6
                    angle = 2 * pi * (k - 1) / max(numel(obstacles), 1);
                    v = [cos(angle); sin(angle)];
                    d = 1;
                end
                p = obstacles(k).pos + v * minD / d;
                changed = true;
            end
        end
        if ~changed
            break;
        end
    end
    p = max(lo, min(hi, p));
    if ~desired_point_is_safe(p, obstacles, params)
        bestPoint = p;
        bestSlack = desired_point_slack(p, obstacles, params, lo, hi);
        angles = linspace(0, 2 * pi, 25);
        for k = 1:numel(obstacles)
            radius = obstacles(k).r + params.desiredClear;
            for angle = angles(1:end-1)
                candidate = obstacles(k).pos + radius * [cos(angle); sin(angle)];
                candidate = max(lo, min(hi, candidate));
                slack = desired_point_slack(candidate, obstacles, params, lo, hi);
                if slack > bestSlack
                    bestSlack = slack;
                    bestPoint = candidate;
                end
            end
        end
        if bestSlack < -1e-6
            error('UAV:DesiredPositionBlocked', ...
                '无法在工作空间内为僚机找到满足净空要求的期望位置。');
        end
        p = bestPoint;
    end
end                                   % 结束 obstacle_clear_desired 函数

function safe = desired_point_is_safe(p, obstacles, params)
    safe = true;
    for k = 1:numel(obstacles)
        if norm(p - obstacles(k).pos) < obstacles(k).r + params.desiredClear - 1e-9
            safe = false;
            return;
        end
    end
end

function slack = desired_point_slack(p, obstacles, params, lo, hi)
    slack = min([p - lo; hi - p]);
    for k = 1:numel(obstacles)
        slack = min(slack, norm(p - obstacles(k).pos) - ...
            obstacles(k).r - params.desiredClear);
    end
end
