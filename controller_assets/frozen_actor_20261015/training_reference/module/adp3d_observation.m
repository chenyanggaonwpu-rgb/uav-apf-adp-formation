function [observation, info] = adp3d_observation(state, obstacles, p, rangeMeters)
% ADP3D_OBSERVATION 三维单机脱困的第 1 版观测接口，输出 32x1。
% 1:3 为原始目标相对位置/p.goalScale；4:6 为速度/p.vmax；7:32 为 26 束测距。
% 目前为局部观测，不能据此宣称完全可观测或已实现真实机载感知。

    validateattributes(state, {'numeric'}, {'real','finite','size',[6,1]});
    validateattributes(p.goalScale, {'numeric'}, {'real','finite','scalar','positive'});
    validateattributes(p.vmax, {'numeric'}, {'real','finite','scalar','positive'});
    goalError = p.target-state(1:3);
    [scan,scanInfo] = adp3d_scan26(state(1:3),obstacles,p.workspace,rangeMeters);
    observation = [goalError/p.goalScale;state(4:6)/p.vmax;scan];
    assert(isequal(size(observation),[32,1]) && all(isfinite(observation)));
    info = struct('goalErrorMeters',goalError, 'goalScaleMeters',p.goalScale, ...
        'velocityScaleMetersPerSecond',p.vmax, 'scan',scanInfo);
end
