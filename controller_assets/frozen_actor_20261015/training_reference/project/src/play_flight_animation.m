function play_flight_animation(result, params, pauseTime)
% PLAY_FLIGHT_ANIMATION 播放无人机飞行避障过程动画
% 用法：
%   play_flight_animation(result)
%   play_flight_animation(result, params)
%   play_flight_animation(result, params, pauseTime)
% 静态场景只绘制一次，之后只更新无人机位置、轨迹和队形连线

    if nargin < 2 || isempty(params)  % 如果没有传入参数
        params = result.params;       % 从结果中取出参数
    end                               % 结束 if
    if nargin < 3 || isempty(pauseTime) % 如果没有传入播放间隔
        pauseTime = 0.02;             % 默认每帧暂停 0.02 秒
    end                               % 结束 if

    h = result.history;               % 取出历史记录
    nFrames = size(h.state, 3);       % 总帧数
    nUAV = params.nUAV;               % 无人机数量
    step = max(1, floor(nFrames / 400)); % 如果帧数太多，每隔 step 帧播放一次

    fig = figure('Name', 'UAV formation obstacle avoidance animation', ...
                 'Color', 'w');       % 新建动画窗口
    ax = axes('Parent', fig);         % 创建坐标轴
    hold(ax, 'on');                   % 保持当前图形，允许叠加元素

    lo = params.workspace(1, :);      % 工作空间左下角
    hi = params.workspace(2, :);      % 工作空间右上角
    rectangle(ax, 'Position', [lo, hi - lo], 'EdgeColor', [0.3 0.3 0.3], ...
              'LineWidth', 1);        % 画工作空间外框

    for k = 1:numel(result.obstacles) % 遍历所有障碍物
        pg = nsidedpoly(100, 'Center', result.obstacles(k).pos(:).', ...
                        'Radius', result.obstacles(k).r); % 用多边形近似圆形障碍物
        plot(ax, pg, 'FaceColor', [0.86 0.35 0.35], 'FaceAlpha', 0.75, ...
             'EdgeColor', 'none');    % 画红色半透明障碍圆
        th = linspace(0, 2 * pi, 80); % 生成圆周采样角度
        influenceRadius = result.obstacles(k).r + params.rho0;
        plot(ax, result.obstacles(k).pos(1) + influenceRadius * cos(th), ...
             result.obstacles(k).pos(2) + influenceRadius * sin(th), ':', ...
             'Color', [0.7 0.3 0.3]); % 画虚线表示的斥力作用范围
    end                               % 结束障碍物循环

    plot(ax, params.start(1), params.start(2), 's', 'MarkerSize', 10, ...
         'MarkerFaceColor', [0.2 0.7 0.3], 'MarkerEdgeColor', 'k'); % 画起点
    plot(ax, params.target(1), params.target(2), 'p', 'MarkerSize', 14, ...
         'MarkerFaceColor', [0.95 0.75 0.15], 'MarkerEdgeColor', 'k'); % 画目标点

    colors = lines(nUAV);             % 为每架无人机分配颜色
    trailHandles = gobjects(1, nUAV); % 预分配轨迹线句柄
    uavHandles = gobjects(1, nUAV);   % 预分配无人机标记句柄
    labelHandles = gobjects(1, nUAV); % 预分配编号文本句柄
    linkHandles = gobjects(1, nUAV - 1); % 预分配队形连线句柄

    state0 = h.state(:, :, 1);        % 取第一帧状态
    for i = 1:nUAV                    % 遍历所有无人机
        trailHandles(i) = plot(ax, NaN, NaN, 'Color', colors(i, :), ...
                               'LineWidth', 1.2); % 初始化轨迹线，数据为空
        uavHandles(i) = plot(ax, state0(1, i), state0(2, i), 'o', ...
                             'MarkerSize', 9, 'MarkerFaceColor', colors(i, :), ...
                             'MarkerEdgeColor', 'k'); % 初始化无人机圆点标记
        labelHandles(i) = text(ax, state0(1, i) + 0.6, state0(2, i) + 0.6, ...
                               sprintf('U%d', i), 'FontWeight', 'bold'); % 初始化无人机编号
    end                               % 结束循环
    for i = 2:nUAV                    % 遍历除长机外的僚机
        linkHandles(i - 1) = plot(ax, NaN(1, 2), NaN(1, 2), ...
                                  'Color', [0.6 0.6 0.6], 'LineWidth', 0.8); % 初始化长机到僚机的连线
    end                               % 结束循环

    infoText = text(ax, 0.02, 0.98, '', 'Units', 'normalized', ...
                    'VerticalAlignment', 'top', 'FontWeight', 'bold'); % 创建左上角信息文本
    axis(ax, 'equal');                % 保证横纵坐标比例一致
    xlim(ax, [lo(1) - 1, hi(1) + 1]); % 设置 x 轴范围
    ylim(ax, [lo(2) - 1, hi(2) + 1]); % 设置 y 轴范围
    grid(ax, 'on');                   % 打开网格
    hold(ax, 'off');                  % 关闭叠加模式

    frames = unique([1:step:nFrames, nFrames]);
    for k = frames                   % 始终包含最后一帧
        if ~isgraphics(fig)
            return; % 用户关闭窗口即停止回放
        end
        state = h.state(:, :, k);     % 取当前帧状态
        for i = 1:nUAV                % 遍历所有无人机
            set(trailHandles(i), 'XData', squeeze(h.state(1, i, 1:k)), ...
                                 'YData', squeeze(h.state(2, i, 1:k))); % 更新该机历史轨迹
            set(uavHandles(i), 'XData', state(1, i), 'YData', state(2, i)); % 更新该机位置
            set(labelHandles(i), 'Position', ...
                [state(1, i) + 0.6, state(2, i) + 0.6]); % 让编号跟随无人机
        end                           % 结束无人机循环
        pL = state(1:2, params.leaderIdx); % 取长机位置
        for i = 2:nUAV                % 遍历僚机
            set(linkHandles(i - 1), 'XData', [pL(1), state(1, i)], ...
                                    'YData', [pL(2), state(2, i)]); % 更新长机到僚机的连线
        end                           % 结束僚机循环

        set(infoText, 'String', sprintf( ...
            't = %.1f / %.1f s   leader dist = %.1f m   formation error = %.1f m', ...
            h.t(k), h.t(end), h.leaderDist(k), h.formationErr(k))); % 更新左上角实时信息
        title(ax, sprintf('%s V%s', params.softwareName, params.version));
        drawnow;                      % 强制刷新画面
        if ~isgraphics(fig)
            return;
        end
        pause(pauseTime);             % 暂停指定时间，控制播放速度
    end                               % 结束播放循环

    title(ax, sprintf('%s V%s — %s', params.softwareName, ...
        params.version, result.reason)); % 播放结束后显示任务结果
end                                   % 结束 play_flight_animation 函数
