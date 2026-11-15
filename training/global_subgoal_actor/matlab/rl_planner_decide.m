function [waypoint,planner,controller,diagnostic]=rl_planner_decide(planner,controller,actor,measured,time,obstacles,p,settings)
% Only called at observation ticks; a new action is not emitted every tick.
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
    action=actor.predict([]);
    [waypoint,projection]=rl_project_subgoal(action,measured(1:3,1),planner.previous,obstacles,p,settings);
    if controller.mode~=2
        from=controller.mode;controller.mode=2;controller.modeSince=time;controller.activations=controller.activations+1;
        controller.events(end+1)=struct('time',time,'from',from,'to',2,'reason','global_planner_start', ...
            'goalDistance',norm(p.target-measured(1:3,1)),'speed',norm(measured(4:6,1)),'position',measured(1:3,1));
    end
    planner.lastDecision=time;planner.waypoint=waypoint;planner.previous=waypoint;
    planner.bestDistance=norm(waypoint-measured(1:3,1));planner.stallSince=time;planner.decisions=planner.decisions+1;
    controller.actorCalls=controller.actorCalls+1;controller.proposalCalls=controller.proposalCalls+1;
    diagnostic=projection;diagnostic.decided=true;diagnostic.action=action;diagnostic.reason=reason;
end
