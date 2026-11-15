function plot_scene(ax, state, obstacles, params, historyPos)
% PLOT_SCENE 绘制一次完整场景：工作空间、障碍物、起点、目标点和无人机
    hold(ax, 'on');                   % 保持当前图形，允许叠加元素
    lo = params.workspace(1, :);      % 工作空间左下角
    hi = params.workspace(2, :);      % 工作空间右上角
    rectangle(ax, 'Position', [lo, hi - lo], 'EdgeColor', [0.3 0.3 0.3], ...
              'LineWidth', 1);        % 画工作空间外框

    for k = 1:numel(obstacles)        % 遍历所有障碍物
        pg = nsidedpoly(100, 'Center', obstacles(k).pos(:).', ...
                        'Radius', obstacles(k).r); % 用多边形近似圆形障碍物
        plot(ax, pg, 'FaceColor', [0.86 0.35 0.35], 'FaceAlpha', 0.75, ...
             'EdgeColor', 'none');    % 画红色半透明障碍圆
        th = linspace(0, 2 * pi, 80); % 生成圆周采样角度
        influenceRadius = obstacles(k).r + params.rho0;
        plot(ax, obstacles(k).pos(1) + influenceRadius * cos(th), ...
                 obstacles(k).pos(2) + influenceRadius * sin(th), ':', ...
             'Color', [0.7 0.3 0.3]); % 画虚线表示的斥力作用范围
    end                               % 结束障碍物循环

    plot(ax, params.start(1), params.start(2), 's', 'MarkerSize', 10, ...
         'MarkerFaceColor', [0.2 0.7 0.3], 'MarkerEdgeColor', 'k'); % 画起点
    plot(ax, params.target(1), params.target(2), 'p', 'MarkerSize', 14, ...
         'MarkerFaceColor', [0.95 0.75 0.15], 'MarkerEdgeColor', 'k'); % 画目标点

    if nargin >= 5 && ~isempty(historyPos) % 如果传入了历史位置
        colors = lines(params.nUAV);  % 为每架无人机分配颜色
        for i = 1:params.nUAV         % 遍历所有无人机
            plot(ax, squeeze(historyPos(1, i, :)), ...
                 squeeze(historyPos(2, i, :)), 'Color', colors(i, :), ...
                 'LineWidth', 1.2);   % 画该无人机的完整轨迹
        end                           % 结束循环
    end                               % 结束 if

    pL = state(1:2, params.leaderIdx); % 取长机位置
    for i = 2:params.nUAV             % 遍历所有僚机
        plot(ax, [pL(1), state(1, i)], [pL(2), state(2, i)], ...
             'Color', [0.6 0.6 0.6], 'LineWidth', 0.8); % 画长机到僚机的队形连线
    end                               % 结束循环

    colors = lines(params.nUAV);      % 为每架无人机分配颜色
    for i = 1:params.nUAV             % 遍历所有无人机
        plot(ax, state(1, i), state(2, i), 'o', 'MarkerSize', 9, ...
             'MarkerFaceColor', colors(i, :), 'MarkerEdgeColor', 'k'); % 画无人机圆点
        text(ax, state(1, i) + 0.6, state(2, i) + 0.6, ...
             sprintf('U%d', i), 'FontWeight', 'bold'); % 标注无人机编号
    end                               % 结束循环

    axis(ax, 'equal');                % 保证横纵坐标比例一致
    xlim(ax, [lo(1) - 1, hi(1) + 1]); % 设置 x 轴范围
    ylim(ax, [lo(2) - 1, hi(2) + 1]); % 设置 y 轴范围
    grid(ax, 'on');                   % 打开网格
    hold(ax, 'off');                  % 关闭叠加模式
end                                   % 结束 plot_scene 函数
