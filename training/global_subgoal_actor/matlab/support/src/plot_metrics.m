function fig = plot_metrics(result, params)
% PLOT_METRICS 绘制并保存仿真指标图
    fig = figure('Name', 'Metrics', 'Color', 'w'); % 新建指标图窗口
    h = result.history;               % 取出仿真历史记录
    t = h.t;                          % 取出时间序列

    subplot(2, 2, 1);                 % 创建左上子图
    plot(t, h.leaderDist, 'LineWidth', 1.5); % 画长机到目标距离曲线
    title('Leader distance to target'); % 子图标题
    xlabel('t (s)');                  % x 轴标签
    ylabel('m');                      % y 轴标签
    grid on;                          % 打开网格

    sgtitle(sprintf('%s V%s', params.softwareName, params.version));

    subplot(2, 2, 2);                 % 创建右上子图
    plot(t, h.formationErr, 'LineWidth', 1.5); % 画队形误差曲线
    title('Formation error');         % 子图标题
    xlabel('t (s)');                  % x 轴标签
    ylabel('m');                      % y 轴标签
    grid on;                          % 打开网格

    subplot(2, 2, 3);                 % 创建左下子图
    plot(t, h.critic, 'LineWidth', 1.5); % 画 ADP Critic 代价曲线
    title('ADP critic value');        % 子图标题
    xlabel('t (s)');                  % x 轴标签
    ylabel('J');                      % y 轴标签
    grid on;                          % 打开网格

    subplot(2, 2, 4);                 % 创建右下子图
    plot(t, h.adpNorm, 'LineWidth', 1.5); % 画 ADP 修正量大小曲线
    hold on;                          % 保持当前图形，便于叠加曲线
    plot(t, h.controlNorm, 'LineWidth', 1.5); % 叠加总控制量大小曲线
    legend('ADP correction', 'total control', 'Location', 'best'); % 添加图例
    title('Control effort');          % 子图标题
    xlabel('t (s)');                  % x 轴标签
    ylabel('m/s^2');                  % y 轴标签
    grid on;                          % 打开网格

    if ~exist(params.figDir, 'dir')   % 如果结果目录不存在
        mkdir(params.figDir);         % 创建结果目录
    end                               % 结束 if
    print(fig, fullfile(params.figDir, 'metrics.png'), '-dpng', '-r150'); % 保存指标图为 PNG
end                                   % 结束 plot_metrics 函数
