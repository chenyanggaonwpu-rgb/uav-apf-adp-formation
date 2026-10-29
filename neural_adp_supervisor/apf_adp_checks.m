function checks = apf_adp_checks(actorNet,cfg,s)
% Boundary and state-machine checks complement the actual full-flight comparisons.
    p=adp3d_setup(cfg); trap=adp3d_make_case(10001,cfg,p); p=trap.parameters;
    root=trap.initialState; stationary=repmat(root,1,round(s.historySeconds/p.dt)+1);
    c=apf_adp_init();
    [~,c,d]=apf_adp_decide(c,stationary(:,1:end-1),s.historySeconds-p.dt, ...
        trap.obstacles,p,cfg,s,actorNet);
    assert(c.activations==0 && ~d.stalled,'APFADP:EarlyTrigger');
    [waypoint,c,d]=apf_adp_decide(c,stationary,s.historySeconds,trap.obstacles,p,cfg,s,actorNet);
    assert(c.mode==2 && c.activations==1 && d.stalled && c.actorCalls==1);
    assert(norm(waypoint-root(1:3))<=cfg.maxOffset+1e-6);
    moving=stationary; moving(1,:)=moving(1,:)+0.08*(0:size(moving,2)-1)*p.dt;
    moving(4,:)=0.08;
    [~,c,d]=apf_adp_decide(apf_adp_init(),moving,s.historySeconds,trap.obstacles,p,cfg,s,actorNet);
    assert(c.activations==0 && ~d.stalled,'APFADP:SlowMovementFalseTrigger');
    % Executed cooldown cannot immediately activate again, even at a trap.
    c=apf_adp_init(); c.mode=3; c.modeSince=s.historySeconds;
    [~,c,~]=apf_adp_decide(c,stationary,s.historySeconds,trap.obstacles,p,cfg,s,actorNet);
    assert(c.activations==0 && c.mode==3);
    % Safe local exit requires a sustained corridor, not one clear observation.
    p.target=[70;40;15]; root=[40;40;15;1;0;0];
    empty=scenario3d('S0_empty',0,p); history=repmat(root,1,61);
    c=apf_adp_init(); c.mode=2; c.entryDistance=40; c.activations=1;
    [~,c,~]=apf_adp_decide(c,history,2,empty.obstacles,p,cfg,s,actorNet);
    assert(c.mode==2);
    [waypoint,c,~]=apf_adp_decide(c,history,3,empty.obstacles,p,cfg,s,actorNet);
    assert(c.mode==3 && isequal(waypoint,p.target));
    % A timeout and an activation limit are explicit failures, not false successes.
    p=trap.parameters; stationary=repmat(trap.initialState,1,61);
    c=apf_adp_init(); c.mode=2; c.entryDistance=norm(p.target-trap.initialState(1:3));
    [~,c,~]=apf_adp_decide(c,stationary,s.maximumActiveSeconds,trap.obstacles,p,cfg,s,actorNet);
    assert(c.mode==5 && strcmp(c.failureReason,'escape_timeout'));
    c=apf_adp_init(); c.activations=s.maximumActivations;
    [~,c,~]=apf_adp_decide(c,stationary,3,trap.obstacles,p,cfg,s,actorNet);
    assert(c.mode==5 && strcmp(c.failureReason,'activation_limit'));
    % Deliberately unsafe acceleration must be intercepted by physical braking.
    p=adp3d_setup(cfg); scene=scenario3d('S11_blocked',0,p);
    state=[33.8;40;10;3;0;0];
    [backup,ok]=adp3d_braking_plan(state,scene.obstacles,p,cfg.safety); assert(ok);
    interventions=0; minimumDistance=inf;
    for k=1:60
        previous=state;
        [state,backup,info]=adp3d_safe_step(state,[3;0;0],backup,scene.obstacles,p,cfg.safety);
        expected=motion_step3d(previous,info.executedCommand,zeros(3,1),p);
        assert(norm(state-expected)<1e-12 && ~info.collision && ~info.boundaryHit);
        interventions=interventions+info.intervention;
        minimumDistance=min(minimumDistance,info.minimumObstacleDistance);
    end
    assert(interventions>0 && minimumDistance>p.obsSafe);
    % Goal position AND low speed remain required by the physical rollout.
    checks=struct('stationaryTriggerAtThreeSeconds',true,'noPrematureTrigger',true, ...
        'slowMovingTraceNotTriggered',true,'cooldownPreventsImmediateReentry',true, ...
        'sustainedClearCorridorReturnsToAPF',true,'waypointOffsetBounded',true, ...
        'timeoutStopsWithFailure',true,'activationLimitStopsWithFailure',true, ...
        'unsafeAccelerationIntercepted',true,'forcedSafetyInterventions',interventions, ...
        'forcedSafetyMinimumDistance',minimumDistance,'noStateFreezingOrVelocityReset',true);
end
