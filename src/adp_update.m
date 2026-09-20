function params = adp_update(stateOld, stateNew, uBase, uAdp, target, ...
                             pDesOld, pDesNew, obstacles, params)
% ADP_UPDATE 在线更新 ADP 的 Critic 权重和 Actor 权重
    adp = params.adp;                 % 取出 ADP 参数，简化后面代码
    if ~adp.enable                    % 如果 ADP 被关闭
        return;                       % 直接返回，不更新
    end                               % 结束 if

    dwC = zeros(size(adp.wC));        % Critic 权重累计增量初始化为零
    dwA = zeros(size(adp.wA));        % Actor 权重累计增量初始化为零
    n = params.nUAV;                  % 无人机数量

    for i = 1:n                       % 遍历所有无人机
        phiOld = adp_features(stateOld, i, target, pDesOld, obstacles, params); % 更新前状态的特征
        phiNew = adp_features(stateNew, i, target, pDesNew, obstacles, params); % 更新后状态的特征

        ctrl = norm(uAdp(:, i))^2;    % 该机 ADP 修正量的平方，作为控制代价
        r = adp.qRisk * phiOld(1) + adp.qForm * phiOld(4) + ...
            adp.qCtrl * ctrl + adp.qTime * params.dt; % 计算即时代价

        JOld = adp.wC' * phiOld;      % 用 Critic 估计旧状态长期代价
        JNew = adp.wC' * phiNew;      % 用 Critic 估计新状态长期代价
        delta = r + adp.gamma * JNew - JOld; % 计算时序差分误差 TD error
        featureScale = 1 + phiOld' * phiOld;
        dwC = dwC + adp.lrC * delta * phiOld / featureScale; % 归一化半梯度更新

        grad = zeros(2, 1);           % 初始化代价对控制量的梯度
        for ax = 1:2                  % 分别对 x 和 y 两个方向求梯度
            e = zeros(2, 1);          % 构造扰动向量
            e(ax) = adp.eps;          % 只在当前方向加微小扰动
            stPlus  = one_step(stateOld, i, uBase(:, i) + uAdp(:, i) + e, params); % 正向扰动后的状态
            stMinus = one_step(stateOld, i, uBase(:, i) + uAdp(:, i) - e, params); % 负向扰动后的状态
            phiPlus  = adp_features(stPlus,  i, target, pDesNew, obstacles, params); % 正向特征
            phiMinus = adp_features(stMinus, i, target, pDesNew, obstacles, params); % 负向特征
            JPlus  = adp.wC' * phiPlus;   % 正向扰动后的长期代价
            JMinus = adp.wC' * phiMinus;  % 负向扰动后的长期代价
            grad(ax) = adp.gamma * (JPlus - JMinus) / (2 * adp.eps); % 下一状态代价梯度
        end                           % 结束梯度循环

        grad = grad + 2 * adp.qCtrl * uAdp(:, i); % 加入即时控制代价的解析梯度
        % 运行时实际修正量为 gain*(wA*phi)，因此对权重的梯度要乘 gain。
        % 这里只使用一次学习率，避免旧实现中无意形成 lrA^2。
        dwA = dwA - adp.lrA * adp.gain * grad * phiOld' / featureScale;
    end                               % 结束无人机循环

    params.adp.wC = limit_weight_norm(adp.wC + dwC / n, adp.maxWeightNorm);
    params.adp.wA = limit_weight_norm(adp.wA + dwA / n, adp.maxWeightNorm);
end                                   % 结束 adp_update 函数

function s = one_step(state, i, u, params)
% ONE_STEP 只推进第 i 架无人机一步，用于 ADP 梯度计算
    s = state;                        % 复制完整状态
    s(3:4, i) = s(3:4, i) + u * params.dt; % 更新该机速度
    v = s(3:4, i);                    % 读取新速度
    sp = norm(v);                     % 计算速度大小
    if sp > params.vmax               % 如果超速
        v = v * params.vmax / sp;     % 限幅
        s(3:4, i) = v;                % 写回限幅后的速度
    end                               % 结束 if
    s(1:2, i) = s(1:2, i) + s(3:4, i) * params.dt; % 更新位置
end                                   % 结束 one_step 函数

function w = limit_weight_norm(w, maxNorm)
% LIMIT_WEIGHT_NORM 防止在线学习中的非有限值或权重爆炸。
    if any(~isfinite(w), 'all')
        error('UAV:ADPDiverged', 'ADP 权重出现 NaN 或 Inf。');
    end
    nrm = norm(w(:));
    if nrm > maxNorm
        w = w * (maxNorm / nrm);
    end
end
