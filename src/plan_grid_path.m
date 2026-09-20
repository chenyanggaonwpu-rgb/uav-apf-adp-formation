function waypoints = plan_grid_path(startPoint, goalPoint, obstacles, params, clearance)
% PLAN_GRID_PATH 在膨胀障碍物的栅格上寻找并简化安全路径。
% clearance 是相对障碍物表面的规划净空；规划失败时返回空数组。

    startPoint = startPoint(:);
    goalPoint = goalPoint(:);
    waypoints = zeros(2, 0);
    if ~all(isfinite([startPoint; goalPoint]))
        return;
    end
    if segment_clear(startPoint, goalPoint, obstacles, clearance)
        waypoints = goalPoint;
        return;
    end

    lo = params.workspace(1, :);
    hi = params.workspace(2, :);
    step = params.pathGridResolution;
    xs = lo(1):step:hi(1);
    ys = lo(2):step:hi(2);
    [xx, yy] = meshgrid(xs, ys);
    blocked = xx < lo(1) + clearance | xx > hi(1) - clearance | ...
        yy < lo(2) + clearance | yy > hi(2) - clearance;
    for k = 1:numel(obstacles)
        r = obstacles(k).r + clearance;
        blocked = blocked | (xx - obstacles(k).pos(1)).^2 + ...
            (yy - obstacles(k).pos(2)).^2 < r^2;
    end
    [nr, nc] = size(blocked);
    ids = reshape(1:numel(blocked), nr, nc);
    maxEdges = 4 * numel(blocked);
    src = zeros(maxEdges, 1);
    dst = zeros(maxEdges, 1);
    weight = zeros(maxEdges, 1);
    count = 0;
    directions = [0 1; 1 0; 1 1; 1 -1];
    for c = 1:nc
        for r = 1:nr
            if blocked(r, c)
                continue;
            end
            for d = 1:4
                rr = r + directions(d, 1);
                cc = c + directions(d, 2);
                if rr < 1 || rr > nr || cc < 1 || cc > nc || blocked(rr, cc)
                    continue;
                end
                if d >= 3 && (blocked(rr, c) || blocked(r, cc))
                    continue;
                end
                count = count + 1;
                src(count) = ids(r, c);
                dst(count) = ids(rr, cc);
                weight(count) = step * hypot(directions(d,1), directions(d,2));
            end
        end
    end
    if count == 0
        return;
    end
    map = graph(src(1:count), dst(1:count), weight(1:count), numel(blocked));
    joinMargin = params.obsSafe + 0.15;
    startNode = nearest_visible_grid_node(startPoint, xx, yy, blocked, ...
        obstacles, joinMargin);
    goalNode = nearest_visible_grid_node(goalPoint, xx, yy, blocked, ...
        obstacles, joinMargin);
    if startNode == 0 || goalNode == 0
        return;
    end
    route = shortestpath(map, startNode, goalNode);
    if isempty(route)
        return;
    end
    routePoints = [reshape(xx(route),1,[]); reshape(yy(route),1,[])];
    path = [startPoint, routePoints, goalPoint];
    smoothMargin = max(joinMargin, clearance - 0.8 * step);
    keep = 1;
    while keep(end) < size(path, 2)
        last = keep(end);
        next = last + 1;
        for j = size(path,2):-1:last+1
            margin = smoothMargin;
            if j == last + 1 && (last == 1 || j == size(path,2))
                margin = joinMargin;
            end
            if segment_clear(path(:,last),path(:,j),obstacles,margin)
                next = j;
                break;
            end
        end
        keep(end+1) = next; %#ok<AGROW>
    end
    waypoints = path(:, keep(2:end));
end

function id = nearest_visible_grid_node(p, xx, yy, blocked, obstacles, margin)
    distance = hypot(xx-p(1), yy-p(2));
    distance(blocked) = inf;
    [~, order] = sort(distance(:));
    id = 0;
    for k = 1:min(100, numel(order))
        candidate = order(k);
        if ~isfinite(distance(candidate))
            return;
        end
        q = [xx(candidate); yy(candidate)];
        if segment_clear(p, q, obstacles, margin)
            id = candidate;
            return;
        end
    end
end

function ok = segment_clear(a, b, obstacles, margin)
    ok = true;
    segment = b-a;
    lengthSquared = dot(segment,segment);
    for k = 1:numel(obstacles)
        if lengthSquared <= eps
            closest = a;
        else
            fraction = max(0,min(1,dot(obstacles(k).pos-a,segment)/lengthSquared));
            closest = a + fraction*segment;
        end
        if norm(closest-obstacles(k).pos) < obstacles(k).r + margin - 1e-9
            ok = false;
            return;
        end
    end
end
