function fig = plot_trajectory(result, params)
% PLOT_TRAJECTORY 绘制并保存最终轨迹图
    fig = figure('Name', 'Trajectory', 'Color', 'w'); % 新建轨迹图窗口
    ax = axes('Parent', fig);         % 在窗口中创建坐标轴
    historyPos = result.history.state(1:2, :, :); % 取出所有无人机的历史位置
    plot_scene(ax, result.state, result.obstacles, params, historyPos); % 调用场景绘制函数
    title(ax, sprintf('%s V%s (%s)', params.softwareName, ...
          params.version, result.reason)); % 标题中显示统一软件名和版本
    xlabel(ax, 'x (m)');              % x 轴标签
    ylabel(ax, 'y (m)');              % y 轴标签

    if ~exist(params.figDir, 'dir')   % 如果结果目录不存在
        mkdir(params.figDir);         % 创建结果目录
    end                               % 结束 if
    print(fig, fullfile(params.figDir, 'trajectory.png'), '-dpng', '-r150'); % 保存轨迹图为 PNG
end                                   % 结束 plot_trajectory 函数
