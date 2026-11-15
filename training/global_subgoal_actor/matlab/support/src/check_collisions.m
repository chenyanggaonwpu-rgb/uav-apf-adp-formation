function hit = check_collisions(state, obstacles, params, previousState)
% CHECK_COLLISIONS 检查当前状态及上一步运动线段上的连续碰撞。

    if nargin < 4 || isempty(previousState)
        previousState = state;
    end
    hit = false;
    n = size(state, 2);
    for i = 1:n
        p0 = previousState(1:2, i);
        p1 = state(1:2, i);
        for k = 1:numel(obstacles)
            d = point_to_segment_distance(obstacles(k).pos, p0, p1);
            if d <= obstacles(k).r + params.obsSafe
                hit = true;
                return;
            end
        end
        for j = i + 1:n
            relative0 = previousState(1:2, i) - previousState(1:2, j);
            relative1 = state(1:2, i) - state(1:2, j);
            if minimum_segment_norm(relative0, relative1) <= params.uavSafe
                hit = true;
                return;
            end
        end
    end
end

function d = point_to_segment_distance(point, a, b)
    segment = b - a;
    denom = dot(segment, segment);
    if denom <= eps
        d = norm(point - a);
        return;
    end
    t = max(0, min(1, dot(point - a, segment) / denom));
    d = norm(point - (a + t * segment));
end

function d = minimum_segment_norm(a, b)
    delta = b - a;
    denom = dot(delta, delta);
    if denom <= eps
        d = norm(a);
        return;
    end
    t = max(0, min(1, -dot(a, delta) / denom));
    d = norm(a + t * delta);
end
