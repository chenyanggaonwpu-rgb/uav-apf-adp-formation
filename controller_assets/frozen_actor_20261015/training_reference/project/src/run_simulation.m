function result = run_simulation(params)
% RUN_SIMULATION 主仿真函数：输入参数，输出包含轨迹和结果的结构体
    params = validate_params(params);     % 在分配内存和进入循环前检查配置
    if isfield(params, 'seed')            % 如果参数中有随机种子
        rng(params.seed);                 % 设置随机种子，保证障碍物布局可复现
    end                                   % 结束 if
    [state, params] = init_formation(params); % 初始化五机 V 形编队
    obstacles = generate_obstacles(params); % 随机生成障碍物，并过滤不可行场景
    leaderRoute = plan_grid_path(state(1:2, params.leaderIdx), ...
        params.target, obstacles, params, params.pathClearance);
    leaderRouteIndex = 1;

    n = params.nUAV;                      % 无人机数量
    nSteps = ceil(params.Tmax / params.dt); % 根据最大时间和步长计算总步数
    params.leaderTrail = zeros(2, nSteps + 1); % 预分配长机轨迹，避免逐步拼接
    params.leaderTrail(:, 1) = state(1:2, params.leaderIdx);
    params.leaderTrailCumLen = zeros(1, nSteps + 1); % 增量维护累计弧长
    params.leaderTrailCount = 1;
    hist.t = zeros(nSteps, 1);            % 预分配时间记录数组
    hist.leaderDist = zeros(nSteps, 1);   % 预分配长机到目标距离记录
    hist.formationErr = zeros(nSteps, 1); % 预分配队形误差记录
    hist.critic = zeros(nSteps, 1);       % 预分配 ADP Critic 代价记录
    hist.adpNorm = zeros(nSteps, 1);      % 预分配 ADP 修正量大小记录
    hist.controlNorm = zeros(nSteps, 1);  % 预分配总控制量大小记录
    hist.psi = zeros(nSteps, 1);          % 预分配编队航向记录
    hist.state = zeros(4, n, nSteps);     % 预分配无人机状态记录
    hist.escapeActive = zeros(nSteps, 1); % 预分配逃逸状态记录
    hist.stallTime = zeros(nSteps, 1);    % 预分配停滞时间记录
    hist.successHoldTime = zeros(nSteps, 1); % 连续满足全部成功条件的时长
    hist.predictedSafe = false(nSteps, 1); % 安全层最终预测是否通过硬碰撞检查
    hist.cycleMs = zeros(nSteps, 1);       % 完整在线周期耗时（不含绘图/磁盘）

    leaderReached = false;                % 长机是否已进入目标保持区
    holdTime = 0;                         % 成功条件已连续保持的时间
    success = false;                      % 任务是否成功
    reason = 'timeout';                   % 默认结束原因为超时
    kStop = nSteps;                       % 实际结束的步数，默认到最大步数
    bestDist = inf;                       % 长机距目标点的历史最优距离
    stallTime = 0;                        % 长机未取得进展的累计时间
    escapeActive = false;                 % 是否处于逃逸状态
    escapeActiveTime = 0;                 % 当前逃逸已持续的时间
    escapePhaseTime = 0;                  % 当前逃逸方向已持续的时间
    escapeWaypoint = [];                  % 逃逸虚拟航点，为空表示未设置
    followerBestErr = inf(1, n);          % 每架僚机的历史最优队形误差
    followerStallTime = zeros(1, n);      % 每架僚机未取得进展的累计时间
    followerEscapeActive = false(1, n);   % 每架僚机是否正在逃逸
    followerEscapeTime = zeros(1, n);     % 每架僚机当前逃逸已持续的时间
    followerRoutes = cell(1, n);
    followerRouteIndex = ones(1, n);
    followerRouteGoal = nan(2, n);
    followerPlanTime = -inf(1, n);
    params.holdMode = false;              % 初始不在保持队形模式

    for k = 1:nSteps                      % 主仿真循环，逐时间步推进
        cycleClock = tic;
        t = (k - 1) * params.dt;          % 当前仿真时刻
        params.holdMode = leaderReached;  % 先更新模式，再计算本步期望队形
        [pDes, vDes, psi] = formation_desired_positions(state, obstacles, params); % 计算每架僚机的期望位置和期望速度
        pDesControl = pDes;
        vDesControl = vDes;
        targetForLeader = params.target;
        if ~leaderReached && ~isempty(leaderRoute)
            while leaderRouteIndex < size(leaderRoute, 2) && ...
                    norm(state(1:2, params.leaderIdx) - leaderRoute(:, leaderRouteIndex)) < 2
                leaderRouteIndex = leaderRouteIndex + 1;
                bestDist = inf;
            end
            targetForLeader = leaderRoute(:, leaderRouteIndex);
        end
        if leaderReached
            for i = 2:n
                goalMoved = any(~isfinite(followerRouteGoal(:,i))) || ...
                    norm(followerRouteGoal(:,i)-pDes(:,i)) > 1.5;
                if (goalMoved || isempty(followerRoutes{i})) && ...
                        t-followerPlanTime(i) >= 2
                    followerRoutes{i} = plan_grid_path(state(1:2,i), ...
                        pDes(:,i),obstacles,params,params.obsSafe+1.3);
                    followerRouteIndex(i) = 1;
                    followerRouteGoal(:,i) = pDes(:,i);
                    followerPlanTime(i) = t;
                end
                route = followerRoutes{i};
                if ~isempty(route)
                    while followerRouteIndex(i) < size(route,2) && ...
                            norm(state(1:2,i)-route(:,followerRouteIndex(i))) < 1.5
                        followerRouteIndex(i) = followerRouteIndex(i)+1;
                    end
                    if followerRouteIndex(i) < size(route,2)
                        pDesControl(:,i) = route(:,followerRouteIndex(i));
                    end
                    delta = pDesControl(:,i)-state(1:2,i);
                    if norm(delta)>0.05
                        speedRef = min(params.vmax, ...
                            sqrt(2*0.6*params.amax*norm(delta)));
                        vDesControl(:,i) = speedRef*delta/norm(delta);
                    else
                        vDesControl(:,i) = zeros(2,1);
                    end
                end
            end
        end
        params.psi = psi;                 % 保存当前编队航向

        uBase = zeros(2, n);              % 初始化 APF 和编队控制量
        uAdp = zeros(2, n);               % 初始化 ADP 修正量
        u = zeros(2, n);                  % 初始化最终控制量
        criticSum = 0;                    % 用于累加五架机的 Critic 代价

        for i = 1:n                       % 逐架无人机计算控制量
            if i == params.leaderIdx      % 如果是长机
                if escapeActive && ~isempty(escapeWaypoint) && ~leaderReached % 如果正在逃逸
                    targetForLeader = escapeWaypoint; % 把临时目标换成逃逸虚拟航点
                end                       % 结束 if
                [uBase(:, i), ~] = leader_control(state, targetForLeader, ...
                    obstacles, params, leaderReached); % 计算长机控制量
            else                          % 如果是僚机
                paramsFollower = params;   % 复制一份参数给僚机使用
                if followerEscapeActive(i) % 如果这架僚机正在强制归队
                    paramsFollower.formGainBoost = max(paramsFollower.formGainBoost,0.7);
                    paramsFollower.formDampBoost = max(paramsFollower.formDampBoost,1.0);
                end                       % 结束 if
                [uBase(:, i), ~] = follower_control(state(1:2, i), ...
                    state(3:4, i), pDesControl(:, i), vDesControl(:, i), obstacles, ...
                    state(1:2, :), state(3:4, :), paramsFollower, i); % 计算僚机控制量
            end                           % 结束长机/僚机分支
            phi = adp_features(state, i, params.target, pDes, obstacles, params); % 提取 ADP 状态特征
            criticSum = criticSum + adp_critic(phi, params); % 累加 Critic 代价
            uAdp(:, i) = params.adp.gain * adp_actor(phi, params); % ADP Actor 输出修正加速度
            u(:, i) = clamp_acceleration(uBase(:, i) + uAdp(:, i), params.amax); % 合成控制量并限幅
        end                               % 结束控制量计算循环
        [u, safetyInfo] = resolve_collisions(state, u, obstacles, params); % 扫掠轨迹预测避碰

        stateOld = state;                 % 保存本步开始时的状态，供 ADP 学习使用
        state = motion_update(state, u, params); % 用最终控制量推进无人机状态
        if any(~isfinite(state), 'all')
            reason = 'invalid_state';
            kStop = k;
            hist.t(k) = t + params.dt;
            hist.leaderDist(k) = inf;
            hist.formationErr(k) = inf;
            hist.state(:, :, k) = state;
            hist.cycleMs(k) = 1000 * toc(cycleClock);
            break;
        end
        params.leaderTrailCount = params.leaderTrailCount + 1;
        trailIndex = params.leaderTrailCount;
        previousLeader = params.leaderTrail(:, trailIndex - 1);
        currentLeader = state(1:2, params.leaderIdx);
        params.leaderTrail(:, trailIndex) = currentLeader;
        params.leaderTrailCumLen(trailIndex) = ...
            params.leaderTrailCumLen(trailIndex - 1) + norm(currentLeader - previousLeader);
        [pDesNew, ~] = formation_desired_positions(state, obstacles, params); % 使用最新轨迹计算新期望位置

        params = adp_update(stateOld, state, uBase, uAdp, params.target, ...
                            pDes, pDesNew, obstacles, params); % 在线更新 ADP 的 Actor 和 Critic 权重

        leaderDist = norm(state(1:2, params.leaderIdx) - params.target); % 计算长机到目标点距离
        formErr = formation_error(state, pDesNew, params); % 计算当前队形误差

        if leaderDist <= params.holdTriggerDist && ~leaderReached % 如果长机进入保持区且之前未触发
            leaderReached = true;         % 标记长机已到保持区
            params.holdPsi = psi;         % 固定保持阶段的编队航向
            escapeActive = false;
            escapeWaypoint = [];
        end                               % 结束 if
        maxSpeed = max(sqrt(sum(state(3:4, :).^2, 1))); % 计算全机最大速度
        allSuccessConditions = leaderReached && leaderDist <= params.goalTol && ...
            formErr <= params.formTol && maxSpeed <= params.vTol;
        holdTime = update_success_hold(holdTime, allSuccessConditions, params.dt);

        if ~leaderReached                 % 只有长机尚未到点时才检测停滞
            progressDist = norm(state(1:2,params.leaderIdx)-targetForLeader);
            if progressDist < bestDist - params.escapeThresh % 如果距离明显优于历史最优
                bestDist = progressDist;    % 更新历史最优距离
                stallTime = 0;            % 停滞时间清零
                escapeActive = false;     % 关闭逃逸
                escapeActiveTime = 0;     % 逃逸持续时间清零
                escapePhaseTime = 0;      % 逃逸方向持续时间清零
                escapeWaypoint = [];      % 清空逃逸航点
            else                          % 否则说明长机没有取得明显进展
                stallTime = stallTime + params.dt; % 累加停滞时间
            end                           % 结束 if
            if ~escapeActive && stallTime >= params.escapeWindow % 只在进入逃逸时选择一次航点
                escapeActive = true;      % 开启逃逸模式
                dGoal = max(leaderDist, 1e-3); % 目标距离，避免除零
                gDir = (params.target - state(1:2, params.leaderIdx)) / dGoal; % 指向目标点的单位向量
                escapeWaypoint = pick_escape_waypoint( ...
                    state(1:2, params.leaderIdx), gDir, obstacles, params); % 选择绕障虚拟航点
            end                           % 结束 if
            if escapeActive               % 如果正在逃逸
                escapeActiveTime = escapeActiveTime + params.dt; % 累加逃逸总时间
                escapePhaseTime = escapePhaseTime + params.dt;   % 累加当前方向持续时间
                if escapePhaseTime >= params.escapeFlipTime      % 如果当前方向持续过久
                    escapePhaseTime = 0;  % 方向计时清零
                    dGoal = max(leaderDist, 1e-3);
                    gDir = (params.target - state(1:2, params.leaderIdx)) / dGoal;
                    escapeWaypoint = pick_escape_waypoint( ...
                        state(1:2, params.leaderIdx), gDir, obstacles, params); % 换一个方向重新选航点
                end                       % 结束 if
                if ~isempty(escapeWaypoint) && ...
                        norm(state(1:2, params.leaderIdx) - escapeWaypoint) <= params.escapeWaypointTol % 如果已飞到虚拟航点
                    escapeActive = false; % 关闭逃逸
                    escapeActiveTime = 0; % 清空逃逸时间
                    escapePhaseTime = 0;  % 清空方向时间
                    stallTime = 0;        % 清空停滞时间
                    escapeWaypoint = [];  % 清空虚拟航点
                end                       % 结束 if
                if escapeActiveTime >= params.escapeMaxTime % 如果逃逸时间过长仍无进展
                    escapeActive = false; % 强制关闭逃逸
                    escapeActiveTime = 0; % 清空逃逸时间
                    escapePhaseTime = 0;  % 清空方向时间
                    stallTime = 0;        % 清空停滞时间
                    escapeWaypoint = [];  % 清空虚拟航点
                end                       % 结束 if
            end                           % 结束逃逸状态处理
        end                               % 结束停滞检测

        errs = formation_errors(state, pDesNew, params); % 计算每架僚机当前的队形误差
        for i = 2:n                       % 逐架僚机检查是否停滞
            if ~leaderReached             % 长机尚未到保持区前，僚机继续沿轨迹飞行
                continue;                 % 不启用僚机逃逸，避免破坏队形
            end                           % 结束 if
            if ~followerEscapeActive(i)   % 如果这架僚机当前没有在逃逸
                if errs(i) < followerBestErr(i) - params.escapeThresh % 如果队形误差明显优于历史最优
                    followerBestErr(i) = errs(i); % 更新该机历史最优误差
                    followerStallTime(i) = 0;     % 停滞时间清零
                else                      % 否则说明该机没有取得进展
                    followerStallTime(i) = followerStallTime(i) + params.dt; % 累加停滞时间
                end                       % 结束 if
                if followerStallTime(i) >= params.escapeWindow && ...
                        errs(i) > 2 * params.formTol % 只有误差确实很大时才触发逃逸
                    followerEscapeActive(i) = true; % 开启该机逃逸
                    followerEscapeTime(i) = 0;  % 清空逃逸时间
                end                       % 结束 if
            else                          % 如果这架僚机正在逃逸
                followerEscapeTime(i) = followerEscapeTime(i) + params.dt; % 累加逃逸总时间
                if errs(i) < followerBestErr(i) - params.escapeThresh % 如果强制归队后重新取得进展
                    followerEscapeActive(i) = false; % 关闭该机逃逸
                    followerEscapeTime(i) = 0;  % 清空逃逸时间
                    followerStallTime(i) = 0;   % 清空停滞时间
                    followerBestErr(i) = errs(i); % 更新历史最优误差
                elseif errs(i) <= params.formTol % 如果该机已经成功归队
                    followerEscapeActive(i) = false; % 关闭该机逃逸
                    followerEscapeTime(i) = 0;  % 清空逃逸时间
                    followerStallTime(i) = 0;   % 清空停滞时间
                    followerBestErr(i) = errs(i); % 更新历史最优误差
                elseif followerEscapeTime(i) >= params.escapeMaxTime % 如果逃逸时间过长仍无进展
                    followerEscapeActive(i) = false; % 强制关闭该机逃逸
                    followerEscapeTime(i) = 0;  % 清空逃逸时间
                    followerStallTime(i) = 0;   % 清空停滞时间
                    followerBestErr(i) = errs(i); % 从当前状态重新评估后续进展
                end                       % 结束 if
            end                           % 结束僚机逃逸状态处理
        end                               % 结束僚机停滞检测循环

        hist.t(k) = t + params.dt;        % 记录本步结束时刻
        hist.leaderDist(k) = leaderDist;  % 记录长机到目标距离
        hist.formationErr(k) = formErr;   % 记录队形误差
        hist.critic(k) = criticSum / n;   % 记录五机平均 Critic 代价
        hist.adpNorm(k) = mean(sqrt(sum(uAdp.^2, 1))); % 记录平均 ADP 修正量大小
        hist.controlNorm(k) = mean(sqrt(sum(u.^2, 1))); % 记录平均总控制量大小
        hist.psi(k) = psi;                % 记录编队航向
        hist.state(:, :, k) = state;      % 记录当前无人机状态
        hist.escapeActive(k) = double(escapeActive); % 记录是否处于逃逸状态
        hist.stallTime(k) = stallTime;    % 记录停滞时间
        hist.successHoldTime(k) = holdTime;
        hist.predictedSafe(k) = safetyInfo.predictedSafe;

        if check_collisions(state, obstacles, params, stateOld) % 检查整步扫掠轨迹
            if params.debug
                fprintf('DEBUG k=%d t=%.2f detected swept collision.\n', ...
                    k, t + params.dt);
            for di = 1:n                 % 逐架检查端点碰撞来源
                dp = state(1:2, di);     % 该机位置
                for dk = 1:numel(obstacles) % 检查障碍物碰撞
                    if norm(dp - obstacles(dk).pos) <= obstacles(dk).r + params.obsSafe
                        fprintf('DEBUG k=%d t=%.2f UAV%d-obstacle%d pos=[%.2f %.2f] obs=[%.2f %.2f] r=%.2f formErr=%.2f leaderDist=%.2f\n', ...
                            k, t + params.dt, di, dk, dp(1), dp(2), ...
                            obstacles(dk).pos(1), obstacles(dk).pos(2), obstacles(dk).r, ...
                            formErr, leaderDist); % 打印碰撞细节
                    end                  % 结束障碍物检查
                end                      % 结束障碍物循环
                for dj = di + 1:n        % 检查机间碰撞
                    if norm(dp - state(1:2, dj)) <= params.uavSafe
                        fprintf('DEBUG k=%d t=%.2f UAV%d-UAV%d p1=[%.2f %.2f] p2=[%.2f %.2f] formErr=%.2f leaderDist=%.2f\n', ...
                            k, t + params.dt, di, dj, dp(1), dp(2), ...
                            state(1, dj), state(2, dj), formErr, leaderDist); % 打印机间碰撞细节
                    end                  % 结束机间检查
                end                      % 结束机间循环
                if di > 1                % 僚机额外打印期望点和归队状态
                    fprintf('DEBUG UAV%d pDes=[%.2f %.2f] esc=%d stall=%.2f\n', ...
                        di, pDes(1, di), pDes(2, di), ...
                        followerEscapeActive(di), followerStallTime(di)); % 打印诊断信息
                end                      % 结束僚机诊断
            end                          % 结束无人机循环
            end                          % 结束详细诊断
            success = false;              % 任务失败
            reason = 'collision';         % 结束原因设为碰撞
            kStop = k;                    % 记录结束步数
            hist.cycleMs(k) = 1000 * toc(cycleClock);
            break;                        % 跳出主循环
        end                               % 结束 if

        if allSuccessConditions && holdTime >= params.holdDuration
            success = true;               % 任务成功
            reason = 'success';           % 结束原因设为成功
            kStop = k;                    % 记录结束步数
            hist.cycleMs(k) = 1000 * toc(cycleClock);
            break;                        % 跳出主循环
        end                               % 结束 if
        hist.cycleMs(k) = 1000 * toc(cycleClock);
    end                                   % 结束主仿真循环

    result.success = success;             % 保存是否成功
    result.reason = reason;               % 保存结束原因
    result.time = hist.t(kStop);          % 保存仿真结束时间
    result.leaderDistFinal = hist.leaderDist(kStop); % 保存最终长机到目标距离
    result.formErrFinal = hist.formationErr(kStop); % 保存最终队形误差
    result.obstacles = obstacles;         % 保存障碍物信息
    result.state = state;                 % 保存最终状态
    params.leaderTrail = params.leaderTrail(:, 1:params.leaderTrailCount);
    params.leaderTrailCumLen = ...
        params.leaderTrailCumLen(1:params.leaderTrailCount);
    result.params = params;               % 保存更新后的参数
    result.history.t = hist.t(1:kStop);   % 截取实际使用的时间记录
    result.history.leaderDist = hist.leaderDist(1:kStop); % 截取长机距离记录
    result.history.formationErr = hist.formationErr(1:kStop); % 截取队形误差记录
    result.history.critic = hist.critic(1:kStop); % 截取 Critic 代价记录
    result.history.adpNorm = hist.adpNorm(1:kStop); % 截取 ADP 修正量记录
    result.history.controlNorm = hist.controlNorm(1:kStop); % 截取控制量记录
    result.history.psi = hist.psi(1:kStop); % 截取航向记录
    result.history.state = hist.state(:, :, 1:kStop); % 截取状态记录
    result.history.escapeActive = hist.escapeActive(1:kStop); % 截取逃逸状态记录
    result.history.stallTime = hist.stallTime(1:kStop); % 截取停滞时间记录
    result.history.successHoldTime = hist.successHoldTime(1:kStop);
    result.history.predictedSafe = hist.predictedSafe(1:kStop);
    result.history.cycleMs = hist.cycleMs(1:kStop);
end                                       % 结束 run_simulation 函数

function wp = pick_escape_waypoint(p, goalDir, obstacles, params)
% PICK_ESCAPE_WAYPOINT 在长机周围选择最空旷的侧向虚拟航点
    baseAngle = atan2(goalDir(2), goalDir(1)); % 目标方向角度
    candidates = [baseAngle + pi/2, baseAngle - pi/2, ...
                  baseAngle + pi/2 + 0.6, baseAngle - pi/2 - 0.6, ...
                  baseAngle + pi/2 - 0.6, baseAngle - pi/2 + 0.6, ...
                  baseAngle + pi, baseAngle]; % 生成一组候选逃逸角度
    bestScore = -inf;                     % 最优评分初始化为负无穷
    wp = p + params.escapeRadius * [cos(baseAngle + pi/2); sin(baseAngle + pi/2)]; % 默认航点取目标方向右侧
    for c = 1:numel(candidates)           % 遍历所有候选角度
        dvec = [cos(candidates(c)); sin(candidates(c))]; % 候选方向单位向量
        cand = p + params.escapeRadius * dvec; % 计算候选航点
        cand = clamp_to_workspace_point(cand, params); % 把航点限制在工作空间内
        score = workspace_clearance(cand, params); % 计算航点到工作空间边界的距离
        for k = 1:numel(obstacles)        % 遍历所有障碍物
            score = min(score, norm(cand - obstacles(k).pos) - obstacles(k).r); % 评分取到障碍物表面的最小距离
        end                               % 结束障碍物循环
        progress = dot((cand - p) / max(norm(cand - p), eps), goalDir); % 该候选方向在目标方向上的投影
        score = score + params.escapeTargetBias * progress; % 优先选择既空旷又朝目标前进的航点
        if score > bestScore              % 如果这个候选航点更空旷
            bestScore = score;            % 更新最优评分
            wp = cand;                    % 更新最优航点
        end                               % 结束 if
    end                                   % 结束候选循环

end                                       % 结束 pick_escape_waypoint

function s = workspace_clearance(p, params)
% WORKSPACE_CLEARANCE 计算点到工作空间四条边的最短距离
    lo = params.workspace(1, :);          % 工作空间左下角
    hi = params.workspace(2, :);          % 工作空间右上角
    s = min([p(1) - lo(1), hi(1) - p(1), p(2) - lo(2), hi(2) - p(2)]); % 取四个方向距离的最小值
end                                       % 结束 workspace_clearance

function p = clamp_to_workspace_point(p, params)
% CLAMP_TO_WORKSPACE_POINT 把点限制在工作空间内部
    lo = params.workspace(1, :) + 2;      % 左边界内缩 2 米
    hi = params.workspace(2, :) - 2;      % 右边界内缩 2 米
    p = max(lo', min(hi', p));            % 逐坐标截断到合法范围
end                                       % 结束 clamp_to_workspace_point

function a = clamp_acceleration(a, amax)
% CLAMP_ACCELERATION 对加速度向量限幅
    nrm = norm(a);                        % 计算加速度大小
    if nrm > amax                         % 如果超过最大加速度
        a = a * amax / nrm;               % 按比例缩小到最大加速度
    end                                   % 结束 if
end                                       % 结束 clamp_acceleration

function err = formation_error(state, pDes, params)
% FORMATION_ERROR 计算所有僚机到期望位置的最大误差
    err = 0;                              % 初始化误差为 0
    for i = 1:params.nUAV                 % 遍历所有无人机
        if i == params.leaderIdx          % 长机不需要计算队形误差
            continue;                     % 跳过长机
        end                               % 结束 if
        err = max(err, norm(state(1:2, i) - pDes(:, i))); % 取所有僚机中的最大误差
    end                                   % 结束循环
end                                       % 结束 formation_error

function errs = formation_errors(state, pDes, params)
% FORMATION_ERRORS 计算每架僚机到期望位置的误差向量
    errs = zeros(1, params.nUAV);         % 初始化误差数组
    for i = 2:params.nUAV                 % 只遍历僚机
        errs(i) = norm(state(1:2, i) - pDes(:, i)); % 计算该僚机到期望位置的距离
    end                                   % 结束循环
end                                       % 结束 formation_errors
