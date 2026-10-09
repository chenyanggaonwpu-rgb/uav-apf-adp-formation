function row=clearance_trial_record3d(job,q,scene,mapHash,r,errorText,wallSeconds)
% CLEARANCE_TRIAL_RECORD3D One row per scheduled job, including failed plans.
    row=struct('jobId',job.jobId,'stage',job.stage,'configId',job.configId, ...
        'seed',job.seed,'family',string(scene.family),'mapHash',string(mapHash), ...
        'enabled',job.enabled,'pathXY',job.pathXY,'refXY',job.refXY, ...
        'planZ',job.planZ,'refZ',job.refZ,'layerPolicy',job.layerPolicy, ...
        'layerCount',numel(q.heightLayers),'noiseStd',job.noiseStd, ...
        'disturbanceMax',job.disturbanceMax,'disturbanceHoldSteps',job.disturbanceHoldSteps, ...
        'repeat',job.repeat,'stochasticSeed',job.stochasticSeed, ...
        'success',false,'reason',"execution_error",'planStatus',"not_run", ...
        'time',nan,'path',nan,'formationRms',nan,'minObstacleClearance',nan, ...
        'minPairDistance',nan,'verticalTravel',nan,'ascent',nan,'descent',nan, ...
        'commandEffort',nan,'followerHeightRms',nan,'followerHeightMax',nan, ...
        'leaderRouteZMax',nan,'referenceBlockedFraction',nan,'safetyTriggerFraction',nan, ...
        'cycles',0,'initialPlanSeconds',nan,'buildSeconds',nan,'searchSeconds',nan, ...
        'availableNodes',nan,'edges',nan,'replanCount',0,'cycleP99ms',nan, ...
        'cycleMaxMs',nan,'cyclesOver50ms',0,'finalLeaderError',nan, ...
        'finalFormationError',nan,'finalMaxSpeed',nan,'wallSeconds',wallSeconds, ...
        'hasExecutionError',~isempty(errorText));
    if isempty(r), return; end
    row.reason=string(r.reason); row.success=r.success; row.time=r.time;
    row.planStatus=string(r.plan.status); row.initialPlanSeconds=r.timing.initialPlanSeconds;
    for name={'buildSeconds','searchSeconds','availableNodes','edges'}
        if isfield(r.plan,name{1}), row.(name{1})=r.plan.(name{1}); end
    end
    row.replanCount=numel(r.timing.replanSeconds);
    if ~isfield(r.history,'t') || isempty(r.history.t), return; end
    m=r.metrics; h=r.history; row.cycles=numel(h.t);
    row.path=m.leaderPathLength; row.formationRms=m.formationRms;
    row.minObstacleClearance=m.minObstacleClearance; row.minPairDistance=m.minPairDistance;
    row.verticalTravel=m.leaderVerticalTravel; row.ascent=m.leaderAscent;
    row.descent=m.leaderDescent; row.commandEffort=m.commandEffort;
    dz=reshape(h.state(3,2:end,:)-h.state(3,1,:),[],1);
    row.followerHeightRms=sqrt(mean(dz.^2)); row.followerHeightMax=max(abs(dz));
    row.referenceBlockedFraction=mean(h.referenceBlocked);
    row.safetyTriggerFraction=mean(h.safetyTriggered);
    timing=h.cycleMs;
    row.cycleP99ms=prctile(timing,99); row.cycleMaxMs=max(timing);
    row.cyclesOver50ms=nnz(timing>50);
    row.finalLeaderError=norm(r.state(1:3,1)-q.target);
    row.finalFormationError=h.formationError(end);
    row.finalMaxSpeed=max(vecnorm(r.state(4:6,:)));
    % Leader reference is its own measured position in formation_reference3d;
    % compute departure from the stored final planned polyline instead.
    % This is a geometric deviation diagnostic, not a tracking-error bound.
    xyz=reshape(h.state(1:3,1,:),3,[]); points=[q.start,r.route];
    best=inf(1,size(xyz,2)); err=zeros(size(best));
    for k=1:size(points,2)-1
        a=points(:,k); delta=points(:,k+1)-a;
        t=max(0,min(1,sum((xyz-a).*delta,1)/max(dot(delta,delta),eps)));
        projected=a+delta.*t; d=sum((xyz-projected).^2,1); keep=d<best;
        best(keep)=d(keep); err(keep)=abs(xyz(3,keep)-projected(3,keep));
    end
    row.leaderRouteZMax=max(err);
end
