function result = main(params)
% MAIN 程序入口：运行一次完整仿真并输出结果
    projectRoot = fileparts(mfilename('fullpath')); % 获取本项目根目录
    oldPath = path;
    pathCleanup = onCleanup(@() path(oldPath));
    addpath(fullfile(projectRoot, 'src'), fullfile(projectRoot, 'src3d'), ...
            fullfile(projectRoot, 'config'));       % 把源码目录和配置目录加入 MATLAB 搜索路径
    if nargin < 1
        params = default_params3d();
    end
    if numel(params.start) == 3
        result = main3d(params);
        return;
    end
    params = validate_params(params);
    params.figDir = create_run_output_dir(params.resultsRoot, 'run', params.version);
    result = run_simulation(params);     % 执行主仿真循环
    params = result.params;              % 取回仿真过程中更新过的参数，例如 ADP 权重
    save(fullfile(params.figDir, 'simulation_result.mat'), 'result', '-v7.3');
    plot_trajectory(result, params);     % 绘制并保存最终轨迹图
    plot_metrics(result, params);        % 绘制并保存仿真指标图
    if params.animate                    % 判断是否需要播放动态动画
        animate_demo(result, params);    % 播放无人机飞行避障过程动画
    end                                  % 结束 if 判断
    fprintf('\n===== APF + ADP formation simulation =====\n'); % 打印分隔标题
    fprintf('Software      : %s V%s\n', params.softwareName, params.version);
    fprintf('Result        : %s\n', result.reason);            % 打印最终任务结果
    fprintf('Success       : %d\n', result.success);           % 打印是否成功
    fprintf('Elapsed time  : %.2f s\n', result.time);          % 打印仿真用时
    fprintf('Final leader distance : %.2f m\n', result.leaderDistFinal); % 打印终点时到目标距离
    fprintf('Final formation error : %.2f m\n', result.formErrFinal);   % 打印最终队形误差
    fprintf('Figures saved to      : %s\n', params.figDir);    % 打印图片保存目录
end                                  % 结束 main 函数
