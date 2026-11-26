function [waypoint,planner,controller,diagnostic]=rl_planner_decide(planner,controller,actor,measured,time,obstacles,p,settings)
% Only called at observation ticks; a new action is not emitted every tick.
% Step-0 variant: the executed subgoal goes through the local reachability
% executor when settings.local_hop is true; otherwise the frozen behaviour.
    waypoint=planner.waypoint;diagnostic=struct('decided',false);
    distance=norm(measured(1:3,1)-waypoint);
    if distance<planner.bestDistance-.15
        planner.bestDistance=distance;planner.stallSince=time;
    end
    elapsed=time-planner.lastDecision;reason='';
    if ~isfinite(planner.lastDecision),reason='mission_start';
    elseif elapsed>=settings.decision_seconds-1e-8,reason='periodic';
    elseif elapsed>=settings.minimum_decision_seconds-1e-8
        if distance<=settings.reached_meters,reason='subgoal_reached';
        elseif time-planner.stallSince>=settings.stall_seconds-1e-8,reason='stagnation';end
    end
    if isempty(reason),return;end
    assert(isa(actor,'RLPolicy')&&actor.Net.schema==6);
    useHop=isfield(settings,'local_hop') && settings.local_hop;
    ruleOnly=useHop && isfield(settings,'rule_requests_only') && settings.rule_requests_only;
    if ruleOnly
        assert(isfield(settings,'request_mode') && strcmp(settings.request_mode,'goal'));
        action=zeros(3,1);
        startReason='rule_waypoints_start';
        controller.ruleCalls=controller.ruleCalls+1;
    else
        action=actor.predict([]);
        startReason='global_planner_start';
        controller.actorCalls=controller.actorCalls+1;
    end
    if useHop
        if ~isfield(settings,'request_mode') || isempty(settings.request_mode)
            settings.request_mode='goal_xy';
        end
        settings.now=time;
        if isfield(planner,'bypassUntil'),settings.bypass_until=planner.bypassUntil;else,settings.bypass_until=-inf;end
        latch=6;
        if isfield(settings,'gate_latch_seconds'),latch=settings.gate_latch_seconds;end
        [waypoint,projection]=rl_local_hop(action,measured(1:3,1),planner.previous,obstacles,p,settings);
        if isfield(projection,'bypass')&&projection.bypass
            planner.bypassUntil=time+latch;
        elseif isfield(planner,'bypassUntil')&&time>=planner.bypassUntil
            planner.bypassUntil=-inf;
        end
    else
        [waypoint,projection]=rl_project_subgoal(action,measured(1:3,1),planner.previous,obstacles,p,settings);
    end
    if controller.mode~=2
        from=controller.mode;controller.mode=2;controller.modeSince=time;controller.activations=controller.activations+1;
        controller.events(end+1)=struct('time',time,'from',from,'to',2,'reason',startReason, ...
            'goalDistance',norm(p.target-measured(1:3,1)),'speed',norm(measured(4:6,1)),'position',measured(1:3,1));
    end
    planner.lastDecision=time;planner.waypoint=waypoint;planner.previous=waypoint;
    planner.bestDistance=norm(waypoint-measured(1:3,1));planner.stallSince=time;planner.decisions=planner.decisions+1;
    controller.proposalCalls=controller.proposalCalls+1;
    diagnostic=projection;diagnostic.decided=true;diagnostic.action=action;diagnostic.reason=reason;
end
