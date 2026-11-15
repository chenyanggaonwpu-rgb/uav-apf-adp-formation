function [u, info] = resolve_collisions(state, u, obstacles, params)
% RESOLVE_COLLISIONS 基于下一步扫掠轨迹迭代修正控制量。
% 该层同时检查障碍物和机间连续距离；每轮都会用更新后的控制量重新预测。

    n = size(state, 2);
    margin = params.collisionMargin;
    % 不因进入归队阶段而缩小安全余量；归队时同样需要制动距离。
    u = preemptive_brake(state, u, obstacles, params);

    maxIterations = 5;
    for iter = 1:maxIterations
        [predictedPos, predictedVel] = predict_motion(state, u, params);
        correction = zeros(size(u));

        for i = 1:n
            p0 = state(1:2, i);
            p1 = predictedPos(:, i);
            for k = 1:numel(obstacles)
                [d, closest] = point_to_segment_distance(obstacles(k).pos, p0, p1);
                minD = obstacles(k).r + params.obsSafe + margin;
                if d < minD
                    direction = closest - obstacles(k).pos;
                    if norm(direction) < 1e-9
                        direction = p0 - obstacles(k).pos;
                    end
                    if norm(direction) < 1e-9
                        angle = 2 * pi * (i - 1) / max(n, 1);
                        direction = [cos(angle); sin(angle)];
                    else
                        direction = direction / norm(direction);
                    end
                    closing = dot(predictedVel(:, i), direction);
                    correction(:, i) = correction(:, i) + ...
                        params.collisionAvoidGain * (minD - d) * direction;
                    if closing < 0
                        correction(:, i) = correction(:, i) - ...
                            params.collisionVelGain * closing * direction;
                    end
                end
            end
        end

        for i = 1:n-1
            for j = i+1:n
                relative0 = state(1:2, i) - state(1:2, j);
                relative1 = predictedPos(:, i) - predictedPos(:, j);
                [d, relativeClosest] = minimum_segment_vector(relative0, relative1);
                minD = params.uavSafe + margin;
                if d < minD
                    if d < 1e-9
                        pairAngle = 2 * pi * (i - 1) / max(n, 1);
                        direction = [cos(pairAngle); sin(pairAngle)];
                    else
                        direction = relativeClosest / d;
                    end
                    closing = dot(predictedVel(:, i) - predictedVel(:, j), direction);
                    pairCorrection = params.collisionAvoidGain * ...
                        (minD - d) * direction;
                    if closing < 0
                        pairCorrection = pairCorrection - ...
                            params.collisionVelGain * closing * direction;
                    end
                    correction(:, i) = correction(:, i) + 0.5 * pairCorrection;
                    correction(:, j) = correction(:, j) - 0.5 * pairCorrection;
                end
            end
        end

        if max(vecnorm(correction)) < 1e-10
            break;
        end
        u = u + correction;
        for i = 1:n
            u(:, i) = clamp_acceleration(u(:, i), params.amax);
        end
    end

    [predictedPos, ~] = predict_motion(state, u, params);
    info.predictedSafe = ~check_collisions([predictedPos; zeros(2, n)], ...
        obstacles, params, [state(1:2, :); zeros(2, n)]);
    info.iterations = iter;
end

function u = preemptive_brake(state, u, obstacles, params)
% 将径向接近速度限制在剩余净空所允许的制动速度以下。
% 这是保守的提前减速启发式；受输入限幅影响，不构成安全性证明。
    n = size(state,2);
    dt = params.dt;
    reserve = max(0.35, params.vmax*dt);
    effectiveBrake = 0.7*params.amax;
    for i = 1:n
        p = state(1:2,i);
        v = state(3:4,i);
        correction = zeros(2,1);
        for k = 1:numel(obstacles)
            radial = p-obstacles(k).pos;
            dist = norm(radial);
            if dist < 1e-9
                continue;
            end
            direction = radial/dist;
            clearance = dist-obstacles(k).r-params.obsSafe;
            allowedClosing = sqrt(2*effectiveBrake*max(clearance-reserve,0));
            closing = -dot(v+u(:,i)*dt,direction);
            if closing > allowedClosing
                correction = correction + ...
                    ((closing-allowedClosing)/dt)*direction;
            end
        end
        u(:,i) = clamp_acceleration(u(:,i)+correction,params.amax);
    end
    for i = 1:n-1
        for j = i+1:n
            relative = state(1:2,i)-state(1:2,j);
            dist = norm(relative);
            if dist < 1e-9
                continue;
            end
            direction = relative/dist;
            clearance = dist-params.uavSafe;
            allowedClosing = sqrt(4*effectiveBrake*max(clearance-reserve,0));
            relativeVelocity = state(3:4,i)-state(3:4,j) + ...
                dt*(u(:,i)-u(:,j));
            closing = -dot(relativeVelocity,direction);
            if closing > allowedClosing
                correction = ((closing-allowedClosing)/dt)*direction/2;
                u(:,i) = clamp_acceleration(u(:,i)+correction,params.amax);
                u(:,j) = clamp_acceleration(u(:,j)-correction,params.amax);
            end
        end
    end
end

function [position, velocity] = predict_motion(state, u, params)
    velocity = state(3:4, :) + u * params.dt;
    for i = 1:size(state, 2)
        speed = norm(velocity(:, i));
        if speed > params.vmax
            velocity(:, i) = velocity(:, i) * params.vmax / speed;
        end
    end
    position = state(1:2, :) + velocity * params.dt;
end

function [d, closest] = point_to_segment_distance(point, a, b)
    segment = b - a;
    denom = dot(segment, segment);
    if denom <= eps
        closest = a;
    else
        t = max(0, min(1, dot(point - a, segment) / denom));
        closest = a + t * segment;
    end
    d = norm(point - closest);
end

function [d, closest] = minimum_segment_vector(a, b)
    delta = b - a;
    denom = dot(delta, delta);
    if denom <= eps
        closest = a;
    else
        t = max(0, min(1, -dot(a, delta) / denom));
        closest = a + t * delta;
    end
    d = norm(closest);
end

function a = clamp_acceleration(a, amax)
    nrm = norm(a);
    if nrm > amax
        a = a * amax / nrm;
    end
end
