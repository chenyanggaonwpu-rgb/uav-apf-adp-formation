function d3 = adp3d_project_demo(showFigure, projectRoot, action)
% ADP3D_PROJECT_DEMO 原三维工程的单机 APF 停滞基线与三维学习接口。
% 使用：d3 = adp3d_project_demo;
% 复用原 S8_window 场景、参数、斥力、运动更新和连续碰撞判定。
% 单机隔离测试：关闭本次副本的旧 ADP，且不调用全局规划。
% 测试起点取原 start 的 x/z 和原 target 的 y，消除侧向误差以检查力平衡。
% 三维位置/速度均可自由变化；没有固定高度或禁用 z 轴。
% 输出 32 维局部观测与 3 维航点动作。当前不创建/训练神经网络。

    if nargin<1, showFigure=true; end
    if nargin<2, defaults=adp3d_config(); projectRoot=defaults.projectRoot; end
    if nargin<3, action=[0;0;0.6]; end
    validateattributes(showFigure,{'logical','numeric'},{'scalar','real','finite'});
    assert(showFigure==0 || showFigure==1,'ADP3D:FigureFlag','作图开关应为 true 或 false。');
    validateattributes(action,{'numeric'},{'real','finite','size',[3,1]});
    projectRoot=char(projectRoot);
    assert(isfolder(projectRoot),'ADP3D:ProjectMissing','找不到原三维工程：%s',projectRoot);
    oldPath=path;
    restorePath=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(projectRoot,'config'),fullfile(projectRoot,'src'), ...
        fullfile(projectRoot,'src3d'),'-begin');
    expectedMotion=fullfile(projectRoot,'src3d','motion_step3d.m');
    assert(strcmpi(which('motion_step3d'),expectedMotion), ...
        'ADP3D:WrongMotion','当前调用的运动模型不是指定原工程文件。');

    originalParameters=default_params3d();
    p=originalParameters;
    p.nUAV=1;
    p.adp.enable=false;
    p.sceneId='S8_window';
    scene=scenario3d(p.sceneId,p.seed,p);
    initialState=[p.start(1);p.target(2);p.start(3);0;0;0];
    baseline=simulateBaseline(initialState,scene.obstacles,p);

    state=baseline.states(:,end);
    scanRangeMeters=12;
    [observation,observationInfo]=adp3d_observation(state,scene.obstacles,p,scanRangeMeters);
    [waypoint,actionInfo]=adp3d_decode_action(state,action,p.escapeRadius);
    [candidateCommand,commandInfo]=adp3d_leader_command(state,waypoint,scene.obstacles,p);
    [nextState,actualAcceleration]=motion_step3d(state,candidateCommand,zeros(3,1),p);
    [nextStepCollision,nextStepObstacleDistance,~,nextStepBoundaryHit]= ...
        check_swept3d(state,nextState,scene.obstacles,p);
    [nextObservation,nextObservationInfo]=adp3d_observation(nextState,scene.obstacles,p,scanRangeMeters);
    endpointState=[waypoint;zeros(3,1)];
    [waypointCollision,waypointObstacleDistance,~,waypointBoundaryHit]= ...
        check_swept3d(endpointState,endpointState,scene.obstacles,p);
    [straightSegmentBlocked,straightSegmentDistance]= ...
        check_swept3d(state,endpointState,scene.obstacles,p);

    % 这个折线只证明几何上存在升高穿窗的通道，没有输入基线控制器。
    witness=[state(1:3),[state(1);state(2);15],[50;state(2);15],p.target];
    witnessMinimum=inf;
    witnessCollision=false;
    for j=1:size(witness,2)-1
        from=[witness(:,j);zeros(3,1)];
        to=[witness(:,j+1);zeros(3,1)];
        [hit,distance]=check_swept3d(from,to,scene.obstacles,p);
        witnessCollision=witnessCollision || hit;
        witnessMinimum=min(witnessMinimum,distance);
    end

    source=struct('projectRoot',projectRoot, 'motion',which('motion_step3d'), ...
        'repulsion',which('apf_repulsion3d'), 'collision',which('check_swept3d'), ...
        'scene',which('scenario3d'), 'leaderFormulaReference',which('control3d'));
    step=struct('candidateCommand',candidateCommand, 'commandInfo',commandInfo, ...
        'actualAcceleration',actualAcceleration, 'collision',nextStepCollision, ...
        'boundaryHit',nextStepBoundaryHit, 'minimumObstacleDistance',nextStepObstacleDistance, ...
        'hardObstacleMargin',nextStepObstacleDistance-p.obsSafe, ...
        'deltaPosition',nextState(1:3)-state(1:3), ...
        'previewOnly',true);
    proposalGeometry=struct('waypointCollision',waypointCollision, ...
        'waypointBoundaryHit',waypointBoundaryHit, ...
        'waypointObstacleDistance',waypointObstacleDistance, ...
        'straightSegmentBlocked',straightSegmentBlocked, ...
        'straightSegmentObstacleDistance',straightSegmentDistance);
    d3=struct('stage','three_dimensional_project_interface', 'source',source, ...
        'originalParameters',originalParameters, 'parameters',p, 'scene',scene, ...
        'initialState',initialState, 'baseline',baseline, 'state',state, ...
        'observation',observation, 'observationInfo',observationInfo, ...
        'action',double(action), 'waypoint',waypoint, 'actionInfo',actionInfo, ...
        'nextState',nextState, 'nextObservation',nextObservation, ...
        'nextObservationInfo',nextObservationInfo, 'step',step, ...
        'proposalGeometry',proposalGeometry, 'witness',witness, ...
        'witnessCollision',witnessCollision, 'witnessMinimumObstacleDistance',witnessMinimum, ...
        'witnessHardMargin',witnessMinimum-p.obsSafe, 'neuralTrainingStarted',false, ...
        'trainingEnvironmentComplete',false);

    fprintf('\n--- Original project: 3-D APF and learning interface ---\n');
    fprintf('Scene: S8_window; single-UAV APF test; no global planning\n');
    fprintf('dt: %.2f s; vmax: %.1f m/s; amax: %.1f m/s^2; obstacle threshold: %.2f m\n', ...
        p.dt,p.vmax,p.amax,p.obsSafe);
    fprintf('APF reached goal: %d\n',baseline.reachedGoal);
    fprintf('APF stalled:      %d\n',baseline.stalled);
    fprintf('APF collision:    %d\n',baseline.collision);
    fprintf('Final APF command norm: %.9g m/s^2\n',baseline.finalCommandNorm);
    fprintf('Stalled position: [%+.4f; %+.4f; %+.4f] m\n',state(1:3));
    fprintf('State size:       %d x %d\n',size(state));
    fprintf('Observation size: %d x %d\n',size(observation));
    fprintf('Action size:      %d x %d\n',size(action));
    fprintf('Proposed action:  [%+.2f; %+.2f; %+.2f]\n',action);
    fprintf('Proposed waypoint:[%+.4f; %+.4f; %+.4f] m\n',waypoint);
    fprintf('Predicted next-step dz: %+.8f m\n',step.deltaPosition(3));
    fprintf('Predicted next-step collision: %d\n',nextStepCollision);
    fprintf('Geometric escape witness collision: %d\n',witnessCollision);
    fprintf('Geometric escape witness margin: %.4f m\n',d3.witnessHardMargin);
    fprintf('Neural training started: 0\n\n');
    if showFigure
        d3.figure=makePreview(d3);
    end
end

function baseline=simulateBaseline(initialState,obstacles,p)
    maxSteps=ceil(p.Tmax/p.dt);
    states=zeros(6,maxSteps+1);
    goalDistance=zeros(1,maxSteps+1);
    obstacleDistance=inf(1,maxSteps+1);
    states(:,1)=initialState;
    goalDistance(1)=norm(p.target-initialState(1:3));
    [initialHit,obstacleDistance(1)]=check_swept3d(initialState,initialState,obstacles,p);
    assert(~initialHit,'ADP3D:InvalidInitialState','三维单机测试起点碰撞。');
    collision=false;
    reachedGoal=false;
    last=maxSteps+1;
    for k=1:maxSteps
        command=adp3d_leader_command(states(:,k),p.target,obstacles,p);
        states(:,k+1)=motion_step3d(states(:,k),command,zeros(3,1),p);
        [collision,obstacleDistance(k+1)]=check_swept3d(states(:,k),states(:,k+1),obstacles,p);
        goalDistance(k+1)=norm(p.target-states(1:3,k+1));
        reachedGoal=goalDistance(k+1)<=p.goalTol && norm(states(4:6,k+1))<=p.vTol;
        if collision || reachedGoal
            last=k+1;
            break;
        end
    end
    states=states(:,1:last);
    goalDistance=goalDistance(1:last);
    obstacleDistance=obstacleDistance(1:last);
    time=(0:last-1)*p.dt;
    speed=vecnorm(states(4:6,:),2,1);
    windowStart=max(1,last-round(p.escapeWindow/p.dt));
    windowDistance=goalDistance(windowStart:end);
    stalled=~collision && ~reachedGoal && time(end)>=p.escapeWindow ...
        && max(windowDistance)-min(windowDistance)<0.05 ...
        && max(speed(windowStart:end))<0.05;
    finalCommand=adp3d_leader_command(states(:,end),p.target,obstacles,p);
    baseline=struct('time',time, 'states',states, 'goalDistance',goalDistance, ...
        'obstacleDistance',obstacleDistance, 'speed',speed, 'collision',collision, ...
        'reachedGoal',reachedGoal, 'stalled',stalled, ...
        'stallDistanceRangeMeters',max(windowDistance)-min(windowDistance), ...
        'stallWindowMaxSpeed',max(speed(windowStart:end)), ...
        'finalCommand',finalCommand, 'finalCommandNorm',norm(finalCommand));
end

function fig=makePreview(d3)
    fig=figure('Color','w','Name','Original project: 3-D APF and learning interface', ...
        'Position',[80,100,1420,650]);
    layout=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
    ax=nexttile(layout,1,[2,2]);
    hold(ax,'on');
    draw_obstacles3d(ax,d3.scene.obstacles,true,0.22);
    states=d3.baseline.states;
    actual=plot3(ax,states(1,:),states(2,:),states(3,:), ...
        'Color',[0.05,0.35,0.75],'LineWidth',2.2);
    witness=plot3(ax,d3.witness(1,:),d3.witness(2,:),d3.witness(3,:),'--', ...
        'Color',[0.08,0.52,0.28],'LineWidth',1.7);
    start=plot3(ax,states(1,1),states(2,1),states(3,1),'ko','MarkerFaceColor','w','MarkerSize',7);
    stalled=plot3(ax,d3.state(1),d3.state(2),d3.state(3),'o', ...
        'Color',[0.75,0.12,0.10],'MarkerFaceColor',[0.75,0.12,0.10],'MarkerSize',7);
    target=d3.parameters.target;
    goal=plot3(ax,target(1),target(2),target(3),'p','MarkerSize',12, ...
        'MarkerEdgeColor',[0.35,0.22,0.05],'MarkerFaceColor',[0.95,0.70,0.15]);
    candidate=plot3(ax,d3.waypoint(1),d3.waypoint(2),d3.waypoint(3),'d', ...
        'MarkerSize',8,'MarkerEdgeColor',[0.45,0.15,0.65],'MarkerFaceColor',[0.68,0.45,0.82]);
    plot3(ax,[d3.state(1),d3.waypoint(1)],[d3.state(2),d3.waypoint(2)], ...
        [d3.state(3),d3.waypoint(3)],':','Color',[0.55,0.28,0.70],'LineWidth',1.2);
    xlabel(ax,'x (m)');ylabel(ax,'y (m)');zlabel(ax,'z (m)');
    axis(ax,'equal');xlim(ax,[0,80]);ylim(ax,[0,80]);zlim(ax,[0,30]);
    view(ax,-50,22);grid(ax,'on');box(ax,'on');
    title(ax,'S8\_window: original 3-D wall and vertical opening');
    legend(ax,[actual,witness,start,stalled,goal,candidate], ...
        {'APF trajectory','Geometric escape witness','Start','APF stall','Final goal','Candidate waypoint'}, ...
        'Location','southoutside','NumColumns',2,'FontSize',9);
    axHeight=nexttile(layout,3);
    plot(axHeight,d3.baseline.time,states(3,:),'Color',[0.05,0.35,0.75],'LineWidth',1.7);
    hold(axHeight,'on');
    yline(axHeight,d3.waypoint(3),'--','Candidate z (not flown)', ...
        'Color',[0.55,0.28,0.70],'LabelHorizontalAlignment','left');
    xlabel(axHeight,'Time (s)');ylabel(axHeight,'Height z (m)');
    ylim(axHeight,[8,17]);grid(axHeight,'on');title(axHeight,'Height remains at the trap plane');
    axDistance=nexttile(layout,6);
    plot(axDistance,d3.baseline.time,d3.baseline.goalDistance, ...
        'Color',[0.05,0.35,0.75],'LineWidth',1.7);
    xlabel(axDistance,'Time (s)');ylabel(axDistance,'Distance to final goal (m)');
    grid(axDistance,'on');title(axDistance,sprintf('Stalled = %d; collision = %d', ...
        d3.baseline.stalled,d3.baseline.collision));
    title(layout,'3-D project interface: APF baseline and an upward waypoint proposal', ...
        'FontSize',14,'FontWeight','bold');
    drawnow;
end
