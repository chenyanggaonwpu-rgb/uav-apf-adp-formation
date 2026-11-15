function row=fc_summarize(r,item,mode,routePolicy,audit,wallSeconds)
    row=struct('id',string(item.id),'family',string(item.family),'seed',item.seed, ...
        'routePolicy',string(routePolicy),'mode',string(mode),'success',r.success,'reason',string(r.reason), ...
        'flightSeconds',r.time,'steps',audit.steps,'activations',0,'actorCalls',0,'localExits',0,'terminalExits',0, ...
        'jointBrakeSteps',0,'originalSafetySteps',0,'minimumObstacleDistance',NaN,'minimumPairDistance',NaN, ...
        'finalLeaderGoalDistance',NaN,'finalFormationError',NaN,'maximumSpeed',NaN,'maximumAcceleration',NaN, ...
        'cycleMedianMs',NaN,'cycleP99Ms',NaN,'cycleMaxMs',NaN,'cyclesOver50Ms',0, ...
        'initialPlanSeconds',NaN,'preparationSeconds',NaN,'wallSeconds',wallSeconds,'collision',false,'auditPassed',audit.passed, ...
        'ruleCalls',0,'proposalCalls',0,'proposalSeconds',0,'observationSeconds',0, ...
        'leaderPathLength',NaN,'fleetPathLength',NaN,'formationRms',NaN,'maximumFormationError',NaN, ...
        'formationErrorOver1mSeconds',NaN,'commandEffort',NaN,'replanSeconds',0,'terminalPlanSeconds',0,'decisionSeconds',0);
    if isfield(r,'controller')
        c=r.controller;row.activations=c.activations;row.actorCalls=c.actorCalls;
        row.ruleCalls=c.ruleCalls;row.proposalCalls=c.proposalCalls;
        row.proposalSeconds=c.proposalSeconds;row.observationSeconds=c.observationSeconds;
        row.localExits=sum(strcmp({c.events.reason},'local_apf_corridor_clear'));
        row.terminalExits=sum(ismember({c.events.reason},{'terminal_handoff','formation_terminal_hold'}));
    end
    if isfield(r,'timing')
        row.initialPlanSeconds=r.timing.initialPlanSeconds;row.preparationSeconds=r.timing.preparationSeconds;
        row.replanSeconds=sum(r.timing.replanSeconds);row.terminalPlanSeconds=sum(r.timing.terminalPlanSeconds);
    end
    if isfield(r,'metrics')
        row.leaderPathLength=r.metrics.leaderPathLength;row.fleetPathLength=r.metrics.fleetPathLength;
        row.formationRms=r.metrics.formationRms;row.commandEffort=r.metrics.commandEffort;
    end
    if ~isfield(r.history,'state') || isempty(r.history.state),return;end
    h=r.history;p=item.parameters;row.jointBrakeSteps=nnz(h.jointBrake);row.originalSafetySteps=nnz(h.safetyTriggered);
    row.maximumFormationError=max(h.formationError);row.formationErrorOver1mSeconds=nnz(h.formationError>1)*p.dt;
    row.decisionSeconds=sum(h.actorDecisionMs)/1000;
    row.minimumObstacleDistance=min(h.obstacleClearance);row.minimumPairDistance=min(h.pairDistance);
    row.finalLeaderGoalDistance=norm(r.state(1:3,1)-p.target);row.finalFormationError=h.formationError(end);
    row.maximumSpeed=max(sqrt(sum(h.state(4:6,:,:).^2,1)),[],'all');
    row.maximumAcceleration=max(sqrt(sum(h.actualAcceleration.^2,1)),[],'all');
    row.cycleMedianMs=median(h.cycleMs);row.cycleP99Ms=prctile(h.cycleMs,99);row.cycleMaxMs=max(h.cycleMs);
    row.cyclesOver50Ms=nnz(h.cycleMs>1000*p.dt);
    row.collision=any(h.obstacleClearance<=p.obsSafe) || any(h.pairDistance<=p.uavSafe) || strcmp(r.reason,'boundary_violation');
end
