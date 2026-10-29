function [waypoint, info] = adp3d_decode_action(state, action, maxOffsetMeters)
% ADP3D_DECODE_ACTION 将 3 维归一化动作解码为三维临时航点。
% action 三项分别沿世界坐标 x、y、z 轴，单项范围 [-1,1]。
% 先把动作投影到单位球内，再乘最大偏移距离，保证三维总偏移不超限。
% 本函数只提出航点；端点、路径、动力学及后备制动安全要另行检查。

    validateattributes(state, {'numeric'}, {'real','finite','size',[6,1]});
    validateattributes(action, {'numeric'}, {'real','finite','size',[3,1]});
    validateattributes(maxOffsetMeters, {'numeric'}, {'real','finite','scalar','positive'});
    assert(all(abs(action)<=1), 'ADP3D:ActionRange', ...
        '动作的三个分量必须分别在 [-1,1] 内。');
    boundedAction = double(action)/max(1,norm(double(action)));
    offset = maxOffsetMeters*boundedAction;
    waypoint = double(state(1:3))+offset;
    info = struct('rawAction',double(action), 'boundedAction',boundedAction, ...
        'maxOffsetMeters',maxOffsetMeters, 'offsetMeters',offset);
end
