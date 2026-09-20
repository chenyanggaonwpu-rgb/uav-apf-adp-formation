function run_all_tests()
% RUN_ALL_TESTS 运行不依赖额外工具箱的核心回归测试。

    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    oldPath = path;
    cleanup = onCleanup(@() path(oldPath));
    addpath(fullfile(projectRoot, 'src'), fullfile(projectRoot, 'config'));

    tests = {@test_parameter_validation, @test_variable_formation_size, ...
        @test_apf_inside_obstacle, @test_continuous_collision_detection, ...
        @test_overlap_resolution, @test_success_hold_reset, ...
        @test_adp_update_is_finite, @test_short_simulation, @test_disabled_actor, ...
        @test_grid_route_avoids_obstacle, @test_early_braking};

    fprintf('Running %d UAV project tests...\n', numel(tests));
    for k = 1:numel(tests)
        name = func2str(tests{k});
        tests{k}();
        fprintf('  PASS %s\n', name);
    end
    fprintf('All tests passed.\n');
end

function test_grid_route_avoids_obstacle()
    p = validate_params(default_params());
    obstacle = struct('pos',[20;10],'r',2);
    startPoint = [10;10];
    goalPoint = [30;10];
    route = plan_grid_path(startPoint,goalPoint,obstacle,p,p.obsSafe+1.3);
    assert(size(route,2)>=2,'直线路径被挡住时应找到绕障航点。');
    points = [startPoint route];
    for i = 1:size(points,2)-1
        a = points(:,i);
        b = points(:,i+1);
        ab = b-a;
        t = max(0,min(1,dot(obstacle.pos-a,ab)/dot(ab,ab)));
        d = norm(a+t*ab-obstacle.pos)-obstacle.r;
        assert(d>=p.obsSafe-1e-6,'规划线段不得穿过硬安全边界。');
    end
end

function test_early_braking()
    p = validate_params(default_params());
    p.nUAV = 1;
    p.holdMode = true;
    obstacle = struct('pos',[14;10],'r',1);
    state = [10;10;3;0];
    [safeInput,info] = resolve_collisions(state,[3;0],obstacle,p);
    assert(safeInput(1)<3,'接近障碍时应提前降低向障碍方向的加速度。');
    assert(info.predictedSafe,'此测试的下一步应仍处于硬安全区外。');
end

function test_parameter_validation()
    p = default_params();
    validate_params(p);
    p.dt = 0;
    didFail = false;
    try
        validate_params(p);
    catch exception
        didFail = strcmp(exception.identifier, 'UAV:InvalidParameter');
    end
    assert(didFail, '无效 dt 应被参数校验拒绝。');
end

function test_variable_formation_size()
    p = validate_params(default_params());
    p.nUAV = 7;
    p.start = [20; 20];
    p.target = [62; 58];
    [state, p] = init_formation(p);
    assert(isequal(size(state), [4 7]));
    for i = 1:6
        for j = i+1:7
            assert(norm(state(1:2, i) - state(1:2, j)) > p.uavSafe);
        end
    end
end

function test_apf_inside_obstacle()
    p = validate_params(default_params());
    obstacle.pos = [10; 10];
    obstacle.r = 2;
    [force, risk] = apf_repulsive_force([10.5; 10], [20; 10], obstacle, p);
    assert(risk == 1, '障碍物内部风险必须为 1。');
    assert(force(1) > 0 && all(isfinite(force)), '内部斥力必须有限且指向障碍物外。');
end

function test_continuous_collision_detection()
    p = validate_params(default_params());
    p.nUAV = 1;
    obstacle.pos = [5; 0];
    obstacle.r = 1;
    oldState = [0; 0; 0; 0];
    newState = [10; 0; 0; 0];
    assert(check_collisions(newState, obstacle, p, oldState), ...
        '运动线段穿过障碍物时必须判为碰撞。');
end

function test_overlap_resolution()
    p = validate_params(default_params());
    p.nUAV = 2;
    p.holdMode = false;
    state = zeros(4, 2);
    [u, ~] = resolve_collisions(state, zeros(2, 2), ...
        struct('pos', {}, 'r', {}), p);
    assert(norm(u(:, 1) - u(:, 2)) > 0, '重合无人机应获得相反的分离控制量。');
    assert(norm(u(:, 1) + u(:, 2)) < 1e-9, '成对分离修正应保持对称。');
end

function test_success_hold_reset()
    holdTime = update_success_hold(0, true, 0.1);
    holdTime = update_success_hold(holdTime, true, 0.1);
    assert(abs(holdTime - 0.2) < 1e-12);
    holdTime = update_success_hold(holdTime, false, 0.1);
    assert(holdTime == 0, '任一成功条件失效时保持时间必须清零。');
end

function test_adp_update_is_finite()
    p = validate_params(default_params());
    p.nUAV = 1;
    p.adp.wC = [0.3; -0.1; 0.2; 0; 0.1; 0];
    [state, p] = init_formation(p);
    stateNew = state;
    stateNew(1, 1) = stateNew(1, 1) + 0.05;
    pDes = state(1:2, :);
    updated = adp_update(state, stateNew, zeros(2, 1), [0.1; -0.1], ...
        p.target, pDes, pDes, struct('pos', {}, 'r', {}), p);
    assert(all(isfinite(updated.adp.wC)) && all(isfinite(updated.adp.wA), 'all'));
    assert(norm(updated.adp.wA - p.adp.wA, 'fro') > 0, 'Actor 权重应获得有限更新。');
end

function test_short_simulation()
    p = validate_params(default_params());
    p.nObstacles = 0;
    p.Tmax = 0.5;
    p.animate = false;
    p.adp.enable = false;
    result = run_simulation(p);
    assert(numel(result.history.t) == ceil(p.Tmax / p.dt));
    assert(all(isfinite(result.history.state), 'all'));
    assert(isfield(result.history, 'successHoldTime'));
    assert(isfield(result.history, 'predictedSafe'));
end

function test_disabled_actor()
    p = default_params();
    p.adp.wA = ones(2, 6);
    p.adp.enable = false;
    assert(isequal(adp_actor(ones(6, 1), p), zeros(2, 1)), ...
        'Disabling ADP must suppress saved nonzero policy weights.');
end
