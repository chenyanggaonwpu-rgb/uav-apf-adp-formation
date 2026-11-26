function [row,cycles,result]=step0_bench_flight(item,base,planner,arm,net,modelSha)
% Shared closed-loop arm definitions for the small and large Step-0 bench.
    started=tic;
    [p,scene]=v4_decode_case(item,base.p);
    p.v4Continuous=false;
    p.v4AllowDirectPlannerFallback=false;
    p.disturbanceMax=0;
    p.disturbanceHoldSteps=10;
    p.stochasticSeed=item.stochastic_seed;
    p.rlRayNoise=item.ray_noise_std;
    p.rlRayDropout=item.ray_dropout_probability;
    if isfield(item,'random_acceleration_max')
        p.disturbanceMax=item.random_acceleration_max;
    end
    assert(p.nUAV==5 && p.dt==.05 && p.Tmax==120 && p.safety.enable);

    useRouteHeight=strcmp(arm,'global_xy_z');
    useRouteGate=strcmp(arm,'global_gate');
    mode='actor';
    route='actor_global';
    planner.local_hop=false;
    planner.rule_requests_only=false;
    planner.request_mode='actor';
    switch arm
        case {'rule','global_xy_z','global_gate'}
            mode='none';
            route='global';
        case 'apf'
            mode='none';
            route='local_apf';
        case 'gate'
            planner.local_hop=true;
        case {'xy_z','xy_z_apfoff'}
            planner.local_hop=true;
            planner.request_mode='goal_xy';
        case 'rule_xy'
            planner.local_hop=true;
            planner.request_mode='goal';
            planner.rule_requests_only=true;
        case {'hop','apfoff'}
            % Historical arms use the original point projection.
        otherwise
            error('UAV:UnknownBenchArm','Unknown arm %s.',arm);
    end
    p.actorOnly=ismember(arm,{'apfoff','xy_z_apfoff'});
    initial=init_formation3d(p);
    planner.formation_offsets=initial(1:3,:)-p.start;
    cfg=base.cfg;
    cfg.physicsStepsPerDecision=4;
    cfg.planner=planner;
    s=base.s;
    s.maximumFlightSeconds=120;
    s.disableNatural=false;
    s.elapsedTakeover.enable=false;
    actor=[];
    if strcmp(mode,'actor') || useRouteHeight || useRouteGate
        options=struct('training',false,'teacher_probability',0, ...
            'stochastic',false,'seed',item.stochastic_seed, ...
            'ray_noise_std',0,'ray_dropout_probability',0);
        if planner.rule_requests_only
            % rl_simulate uses the schema-6 object for observation plumbing.
            % A schema marker with no weights cannot perform actor inference.
            actor=RLPolicy(struct('schema',6),options,cfg);
            modelSha='';
        else
            assert(isfield(net,'schema') && net.schema==6 && ~isempty(modelSha));
            if useRouteHeight
                actor=GlobalRouteHeightPolicy(net,options,cfg);
                p.globalRouteHeightPolicy=actor;
                routeHeightCleanup=onCleanup(@()delete(actor)); %#ok<NASGU>
            elseif useRouteGate
                actor=GlobalRouteGatePolicy(net,options,cfg);
                p.globalRouteGatePolicy=actor;
                routeGateCleanup=onCleanup(@()delete(actor)); %#ok<NASGU>
            else
                actor=RLPolicy(net,options,cfg);
            end
        end
    else
        modelSha='';
    end

    result=rl_simulate(p,scene,[],mode,actor,cfg,s,route,struct());
    audit=v4_audit(result);
    assert(audit.passed);
    assert(~ismember(result.reason,{'invalid_scene','invalid_state','generation_failed','not_started'}), ...
        'UAV:InvalidBenchFlight','Invalid simulation for %s: %s',item.id,result.reason);
    if isfield(result,'leaderRulePlannerCalls')
        globalCalls=result.leaderRulePlannerCalls;
    else
        assert(strcmp(result.reason,'planning_failed') && strcmp(route,'global'));
        globalCalls=1;
    end
    if ~strcmp(route,'global')
        assert(globalCalls==0);
    end
    if planner.rule_requests_only
        assert(result.controller.actorCalls==0 && result.controller.ruleCalls>0);
    end

    minObs=NaN;minPair=NaN;safe=true;critical=false;pathLength=0;fleetPath=0;
    cycles=zeros(0,1);
    if audit.steps>0
        minObs=result.metrics.minObstacleClearance;
        minPair=result.metrics.minPairDistance;
        boundary=strcmp(result.reason,'boundary_violation');
        safe=minObs>p.obsSafe && minPair>p.uavSafe && ~boundary;
        critical=minObs<=0 || minPair<=.25 || boundary;
        cycles=result.history.cycleMs(:);
        pathLength=result.metrics.leaderPathLength;
        fleetPath=result.metrics.fleetPathLength;
    else
        assert(strcmp(result.reason,'planning_failed') && strcmp(route,'global'));
    end
    row=struct('id',item.id,'group',item.group,'arm',arm,'noise_profile',item.noise_profile, ...
        'case_sha256',item.case_sha256,'geometry_sha256',item.geometry_sha256, ...
        'environment_seed',item.stochastic_seed,'success',double(result.success), ...
        'safe',double(safe),'safe_success',double(result.success && safe), ...
        'critical',double(critical),'reason',result.reason,'flight_seconds',result.time, ...
        'audit_passed',double(audit.passed),'steps',numel(cycles),'leader_path_meters',pathLength, ...
        'fleet_path_meters',fleetPath,'min_obstacle_m',minObs,'min_pair_m',minPair, ...
        'ms_median',median(cycles),'ms_mean',mean(cycles),'model_sha256',modelSha, ...
        'leader_global_route',strcmp(route,'global'),'leader_apf',~p.actorOnly, ...
        'leader_global_plan_calls',globalCalls,'actor_calls',result.controller.actorCalls, ...
        'rule_waypoint_calls',result.controller.ruleCalls,'decisions',0,'local_hops',0, ...
        'unverified_local_hops',0,'window_bypass_decisions',0,'xy_max_error_m',NaN,'z_max_error_m',NaN);
    if strcmp(route,'actor_global')
        record=result.trainingRecord;
        methods=record.projection_methods;
        row.decisions=numel(methods);
        row.local_hops=nnz(startsWith(methods,'local_hop'));
        row.unverified_local_hops=nnz(contains(methods,'unverified'));
        row.window_bypass_decisions=nnz(startsWith(methods,'window_bypass'));
        if ismember(arm,{'xy_z','xy_z_apfoff','rule_xy'})
            [row.xy_max_error_m,row.z_max_error_m]=verify_request(record,result.history,p,planner.rule_requests_only);
        end
    end
    if useRouteHeight
        row.actor_calls=actor.Requests;
        row.decisions=actor.Requests;
        row.actor_z_accepted=actor.AcceptedRequests;
        row.actor_z_revoked=actor.RevokedRequests;
        row.actor_z_applied_steps=actor.AppliedSteps;
    end
    if useRouteGate
        row.actor_calls=actor.Requests;
        row.decisions=actor.Requests;
        row.local_hops=actor.AcceptedLocalHops;
        row.window_bypass_decisions=actor.WindowRequests;
        row.g_local_hop_requests=actor.LocalHopRequests;
        row.g_unverified_requests_rejected=actor.UnverifiedHopRequests;
        row.g_window_held_requests=actor.WindowHeldRequests;
        row.g_window_accepted=actor.AcceptedWindows;
        row.g_accepted=actor.AcceptedRequests;
        row.g_revoked=actor.RevokedRequests;
        row.g_applied_steps=actor.AppliedSteps;
        row.g_off_route_xy_steps=actor.OffRouteXYSteps;
    end
    row.wall_seconds=toc(started);
end

function [xyError,zError]=verify_request(record,history,p,levelRequest)
    assert(~isempty(record.times));
    index=round(record.times/p.dt)+1;
    measured=reshape(history.measuredState(1:3,1,index),3,[]);
    low=p.workspace(1,:)';high=p.workspace(2,:)';
    direction=p.target(1:2)-measured(1:2,:);
    atGoal=vecnorm(direction)<1e-6;
    direction(1,atGoal)=1;
    direction(2,atGoal)=0;
    ruleXY=max(low(1:2),min(high(1:2),measured(1:2,:)+50*direction./vecnorm(direction)));
    if levelRequest
        expectedZ=measured(3,:);
    else
        expectedZ=low(3)+(max(-1,min(1,double(record.actions(3,:))))+1)*.5*(high(3)-low(3));
    end
    xyError=max(abs(record.requested_subgoals(1:2,:)-ruleXY),[],'all');
    zError=max(abs(record.requested_subgoals(3,:)-expectedZ));
    assert(xyError<1e-8 && zError<1e-5,'UAV:AblationRequestMismatch');
end
