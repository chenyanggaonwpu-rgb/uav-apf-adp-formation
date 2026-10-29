function animate_demo(result, params)
% ANIMATE_DEMO 播放仿真结果动画的兼容入口
    if nargin < 2                     % 如果没有传入参数
        params = result.params;       % 从结果中取参数
    end                               % 结束 if
    play_flight_animation(result, params); % 调用正式动画函数
end                                   % 结束 animate_demo 函数
