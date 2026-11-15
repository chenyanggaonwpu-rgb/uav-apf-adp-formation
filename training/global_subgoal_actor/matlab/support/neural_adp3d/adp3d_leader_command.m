function [command, info] = adp3d_leader_command(state, waypoint, obstacles, p)
% ADP3D_LEADER_COMMAND 原工程长机非保持阶段的三维 APF 控制公式。
% state = [x;y;z;vx;vy;vz]，位置单位 m，速度单位 m/s。
% waypoint 为当前三维控制航点，单位 m。
% 单机隔离接口：使用 control3d.m 的长机导航/障碍公式，省去机间项和旧 ADP。
% 本函数不调用全局规划，也不包含尚未完成的神经网络安全过滤器。
% command 是限幅前加速度命令；限幅由原 motion_step3d.m 完成。

    validateattributes(state, {'numeric'}, {'real','finite','size',[6,1]});
    validateattributes(waypoint, {'numeric'}, {'real','finite','size',[3,1]});
    position = state(1:3);
    velocity = state(4:6);
    delta = waypoint-position;
    distance = norm(delta);
    desiredVelocity = min(p.vmax, sqrt(2*0.6*p.amax*distance)) ...
        * delta/max(distance, eps);
    attraction = p.navAccel*(desiredVelocity-velocity);
    [repulsion, risk, degenerate] = apf_repulsion3d(position, waypoint, obstacles, p);
    obstacleWeight = p.wObsMax*sqrt(risk);
    command = (1-obstacleWeight)*attraction + obstacleWeight*repulsion;
    if degenerate
        command = command-0.5*velocity;
    end
    info = struct('desiredVelocity',desiredVelocity, 'attraction',attraction, ...
        'repulsion',repulsion, 'risk',risk, 'obstacleWeight',obstacleWeight, ...
        'tangentDegenerate',degenerate);
end
