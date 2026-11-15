function obstacles = generate_obstacles(params)
% GENERATE_OBSTACLES 生成合法障碍物，并用编队净空网格验证通道。

    if params.nObstacles == 0
        obstacles = struct('pos', {}, 'r', {});
        return;
    end

    for attempt = 1:params.maxObstacleSetAttempts
        [candidate, placedAll] = generate_one(params);
        if placedAll && has_clear_corridor(params, candidate)
            obstacles = candidate;
            return;
        end
    end

    error('UAV:ObstacleGenerationFailed', ...
        ['在 %d 次尝试后仍无法生成满足净空要求的障碍物场景。' ...
         '请减少 nObstacles、减小 pathClearance 或扩大 workspace。'], ...
        params.maxObstacleSetAttempts);
end

function [obstacles, placedAll] = generate_one(params)
% GENERATE_ONE 生成一组互不重叠且不阻挡起终区域的圆形障碍物。

    [initState, paramsWithFormation] = init_formation(params);
    start = params.start;
    target = params.target;
    corridor = target - start;
    corridorLen = norm(corridor);
    uDir = corridor / corridorLen;
    nDir = [-uDir(2); uDir(1)];

    obstacles = struct('pos', {}, 'r', {});
    placedAll = true;
    for k = 1:params.nObstacles
        placed = false;
        for attempt = 1:params.maxObstaclePlacementAttempts
            t = 0.12 + 0.76 * rand;
            along = start + t * corridor;
            perp = (2 * rand - 1) * params.corridorBand;
            pos = along + nDir * perp;
            r = params.obsRMin + rand * (params.obsRMax - params.obsRMin);
            pos = clamp_to_workspace(pos, r, params);
            if is_obstacle_valid(pos, r, start, target, initState, ...
                    obstacles, paramsWithFormation)
                placed = true;
                break;
            end
        end
        if ~placed
            placedAll = false;
            obstacles = struct('pos', {}, 'r', {});
            return;
        end
        obstacles(end + 1).pos = pos; %#ok<AGROW>
        obstacles(end).r = r;
    end
end

function ok = has_clear_corridor(params, obstacles)
% HAS_CLEAR_CORRIDOR 在二维占据网格上搜索具有编队横向净空的通道。

    lo = params.workspace(1, :);
    hi = params.workspace(2, :);
    resolution = params.pathGridResolution;
    xs = lo(1):resolution:hi(1);
    ys = lo(2):resolution:hi(2);
    [xx, yy] = meshgrid(xs, ys);

    clearance = formation_clearance(params);
    blocked = xx < lo(1) + clearance | xx > hi(1) - clearance | ...
              yy < lo(2) + clearance | yy > hi(2) - clearance;
    for k = 1:numel(obstacles)
        radius = obstacles(k).r + clearance;
        blocked = blocked | ...
            (xx - obstacles(k).pos(1)).^2 + (yy - obstacles(k).pos(2)).^2 <= radius^2;
    end

    [~, startCol] = min(abs(xs - params.start(1)));
    [~, startRow] = min(abs(ys - params.start(2)));
    [~, goalCol] = min(abs(xs - params.target(1)));
    [~, goalRow] = min(abs(ys - params.target(2)));
    if blocked(startRow, startCol) || blocked(goalRow, goalCol)
        ok = false;
        return;
    end

    [nRows, nCols] = size(blocked);
    visited = false(nRows, nCols);
    queueRows = zeros(numel(blocked), 1);
    queueCols = zeros(numel(blocked), 1);
    head = 1;
    tail = 1;
    queueRows(1) = startRow;
    queueCols(1) = startCol;
    visited(startRow, startCol) = true;
    neighbors = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];

    ok = false;
    while head <= tail
        row = queueRows(head);
        col = queueCols(head);
        head = head + 1;
        if row == goalRow && col == goalCol
            ok = true;
            return;
        end
        for ni = 1:size(neighbors, 1)
            dr = neighbors(ni, 1);
            dc = neighbors(ni, 2);
            nr = row + dr;
            nc = col + dc;
            if nr < 1 || nr > nRows || nc < 1 || nc > nCols || ...
                    blocked(nr, nc) || visited(nr, nc)
                continue;
            end
            if dr ~= 0 && dc ~= 0 && ...
                    (blocked(row + dr, col) || blocked(row, col + dc))
                continue;
            end
            visited(nr, nc) = true;
            tail = tail + 1;
            queueRows(tail) = nr;
            queueCols(tail) = nc;
        end
    end
end

function clearance = formation_clearance(params)
% FORMATION_CLEARANCE 取配置净空与编队最大横向半径中的较大者。
    if isfield(params, 'formationOffsets')
        lateralRadius = max(abs(params.formationOffsets(2, :)));
    else
        lateralRadius = ceil(max(params.nUAV - 1, 0) / 2) * params.formWing;
    end
    clearance = max(params.pathClearance, lateralRadius + params.obsSafe);
end

function p = clamp_to_workspace(p, r, params)
    lo = params.workspace(1, :).' + r + 1;
    hi = params.workspace(2, :).' - r - 1;
    p = max(lo, min(hi, p));
end

function ok = is_obstacle_valid(pos, r, start, target, initState, obstacles, params)
    clearance = formation_clearance(params);
    if norm(pos - start) < r + max(params.marginStart, clearance) || ...
            norm(pos - target) < r + max(params.marginGoal, clearance)
        ok = false;
        return;
    end
    for i = 1:size(initState, 2)
        if norm(pos - initState(1:2, i)) < r + params.obsSafe + 1
            ok = false;
            return;
        end
    end
    for k = 1:numel(obstacles)
        if norm(pos - obstacles(k).pos) < r + obstacles(k).r + params.marginObs
            ok = false;
            return;
        end
    end
    ok = true;
end
