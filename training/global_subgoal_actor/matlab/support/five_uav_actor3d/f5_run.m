function f5_run(project,output,phase)
    [actor,cfg,s,modelInfo]=rtadp_load_actor(project);
    originalWeights=actor.Learnables;originalCfg=cfg;originalSupervisor=s;
    production=load(fullfile(project,'results','planning_block_fix_20261013','inputs','trial_010001_S.mat'),'q');
    p=production.q;p.adp.enable=false;p.animate=false;
    assert(p.nUAV==5 && p.dt==0.05 && p.noiseStd==0 && p.disturbanceMax==0);
    p=validate_params3d(p);
    shard=str2double(getenv('F5_SHARD'));shards=str2double(getenv('F5_SHARDS'));
    if isnan(shard),shard=1;end
    if isnan(shards),shards=1;end
    fprintf('F5_PARAMETERS pathClearance=%.3f desiredClear=%.3f directional=%d vmax=%.3f amax=%.3f Tmax=%.1f\n', ...
        p.pathClearance,p.desiredClear,p.directionalClearance.enable,p.vmax,p.amax,p.Tmax);
    checks=f5_checks(p,cfg);writejson(fullfile(output,'joint_safety_checks.json'),checks);
    for i=1:20,adp3d_actor_action(actor,zeros(32,1));end
    protocol=struct('version','five-uav-frozen-leader-actor-v1.1','phase',phase, ...
        'nUAV',5,'actorRole','Leader temporary waypoint only; followers retain original formation/APF/pair control', ...
        'globalRoutePolicy','Original planner, prebuilt graphs, original replan and terminal plans', ...
        'localStressPolicy','Explicitly disable global leader planning for both paired modes', ...
        'successRule','Original five-UAV leader goal + maximum formation error + maximum speed + 1-second hold', ...
        'safety','Original predictive repair plus identical certified joint braking guardian in both modes', ...
        'intentionalHoldGate','Do not request Actor during original terminal formation hold', ...
        'timingScope','Unpaced batch CPU processing; initial planning/preparation separate; not wall-clock deadline verification', ...
        'shard',shard,'shards',shards,'concurrentBatchTimingIsNotRealtimeEvidence',shards>1, ...
        'trainingPerformed',false,'criticLoaded',false,'modelInfo',modelInfo, ...
        'parameters',p,'supervisor',s,'configuration',cfg);
    save(fullfile(output,'protocol.mat'),'protocol','-v7');
    writejson(fullfile(output,'protocol.json'),protocol);
    cases=f5_cases(phase,p,cfg);save(fullfile(output,'all_cases_before_flight.mat'),'cases','-v7');
    fullCatalogueCount=numel(cases);cases=cases(shard:shards:end);
    if strcmp(phase,'validation')
        warmItem=struct('parameters',p,'scene',scenario3d('S8_window',58000,p));
        for mode={'baseline','automatic'}
            warmResult=f5_simulate(warmItem.parameters,warmItem.scene,[],mode{1},actor,cfg,s,'global');
            save(fullfile(output,['warmup_' mode{1} '.mat']),'warmResult','-v7');
        end
    end
    fprintf('F5_CATALOGUE_LOCKED cases=%d pairedFlights=%d\n',numel(cases),2*numel(cases));
    records=struct([]);pairs=struct([]);auditedSteps=0;
    for i=1:numel(cases)
        item=cases{i};save(fullfile(output,'inputs',[item.id '.mat']),'item','-v7');
        outcomes=cell(1,2);rowPair=struct('id',string(item.id),'family',string(item.family), ...
            'seed',item.seed,'baselineSuccess',false,'automaticSuccess',false, ...
            'automaticActivations',0,'automaticActorCalls',0,'trajectoryAndCommandsExact',false);
        for m=1:2
            modes={'baseline','automatic'};mode=modes{m};clock=tic;errorText='';
            try
                result=f5_simulate(item.parameters,item.scene,[],mode,actor,cfg,s,item.routePolicy);
                audit=f5_audit(result,item,cfg);auditedSteps=auditedSteps+audit.steps;
                assert(isequaln(originalWeights,actor.Learnables),'F5:WeightsChanged');
                assert(isequaln(originalCfg,cfg) && isequaln(originalSupervisor,s));
            catch exception
                errorText=getReport(exception,'extended','hyperlinks','off');
                result=struct('success',false,'reason','execution_error','time',0,'history',struct());
                audit=struct('steps',0,'passed',false,'dynamicsError',NaN,'minimumCertifiedMargin',NaN);
                fprintf('F5_ERROR id=%s mode=%s %s\n',item.id,mode,exception.message);
            end
            wallSeconds=toc(clock);
            save(fullfile(output,'flight',sprintf('%s_%s.mat',item.id,mode)), ...
                'result','audit','item','wallSeconds','errorText','-v7');
            row=summarize(result,item,mode,audit,wallSeconds);records=[records;row]; %#ok<AGROW>
            outcomes{m}=result;
            writetable(struct2table(records),fullfile(output,'trials.csv'));
            fprintf('F5_FLIGHT %d/%d id=%s mode=%s reason=%s time=%.2f activations=%d calls=%d brake=%d minObs=%.4f minPair=%.4f maxCycle=%.3fms wall=%.2fs\n', ...
                numel(records),2*numel(cases),item.id,mode,result.reason,result.time,row.activations, ...
                row.actorCalls,row.jointBrakeSteps,row.minimumObstacleDistance,row.minimumPairDistance,row.cycleMaxMs,wallSeconds);
        end
        a=outcomes{1};b=outcomes{2};rowPair.baselineSuccess=a.success;rowPair.automaticSuccess=b.success;
        if isfield(b,'controller')
            rowPair.automaticActivations=b.controller.activations;rowPair.automaticActorCalls=b.controller.actorCalls;
        end
        if isfield(a.history,'state') && isfield(b.history,'state')
            rowPair.trajectoryAndCommandsExact=isequaln(a.history.state,b.history.state) && isequaln(a.history.command,b.history.command);
        end
        pairs=[pairs;rowPair];writetable(struct2table(pairs),fullfile(output,'pairs.csv')); %#ok<AGROW>
    end
    completion=struct('completed',true,'phase',phase,'cases',numel(cases),'flights',numel(records), ...
        'fullCatalogueCount',fullCatalogueCount,'shard',shard,'shards',shards, ...
        'baselineSuccesses',sum([pairs.baselineSuccess]),'automaticSuccesses',sum([pairs.automaticSuccess]), ...
        'rescued',sum(~[pairs.baselineSuccess] & [pairs.automaticSuccess]), ...
        'regressions',sum([pairs.baselineSuccess] & ~[pairs.automaticSuccess]), ...
        'baselineSuccessCasesWithActivation',sum([pairs.baselineSuccess] & [pairs.automaticActivations]>0), ...
        'baselineSuccessCasesExact',sum([pairs.baselineSuccess] & [pairs.trajectoryAndCommandsExact]), ...
        'collisions',sum([records.collision]),'executionErrors',sum(strcmp([records.reason],'execution_error')), ...
        'auditedSteps',auditedSteps,'allExecutedTrajectoryAuditsPassed',all([records.auditPassed]), ...
        'actorLearnablesUnchanged',isequaln(originalWeights,actor.Learnables), ...
        'parametersUnchanged',isequaln(originalCfg,cfg) && isequaln(originalSupervisor,s), ...
        'trainingPerformed',false,'criticLoaded',false,'timingScope',protocol.timingScope);
    writejson(fullfile(output,'completion.json'),completion);
    save(fullfile(output,'aggregate.mat'),'records','pairs','completion','checks','protocol','-v7');
    fprintf('FIVE_UAV_ACTOR_CHECK_COMPLETED cases=%d baseline=%d automatic=%d rescued=%d regressions=%d collisions=%d errors=%d auditedSteps=%d\n', ...
        completion.cases,completion.baselineSuccesses,completion.automaticSuccesses,completion.rescued, ...
        completion.regressions,completion.collisions,completion.executionErrors,completion.auditedSteps);
end
function row=summarize(r,item,mode,audit,wallSeconds)
    row=struct('id',string(item.id),'family',string(item.family),'seed',item.seed, ...
        'routePolicy',string(item.routePolicy),'mode',string(mode),'success',r.success,'reason',string(r.reason), ...
        'flightSeconds',r.time,'steps',audit.steps,'activations',0,'actorCalls',0,'localExits',0,'terminalExits',0, ...
        'jointBrakeSteps',0,'originalSafetySteps',0,'minimumObstacleDistance',NaN,'minimumPairDistance',NaN, ...
        'finalLeaderGoalDistance',NaN,'finalFormationError',NaN,'maximumSpeed',NaN,'maximumAcceleration',NaN, ...
        'cycleMedianMs',NaN,'cycleP99Ms',NaN,'cycleMaxMs',NaN,'cyclesOver50Ms',0, ...
        'initialPlanSeconds',NaN,'preparationSeconds',NaN,'wallSeconds',wallSeconds,'collision',false,'auditPassed',audit.passed);
    if isfield(r,'controller')
        c=r.controller;row.activations=c.activations;row.actorCalls=c.actorCalls;
        row.localExits=sum(strcmp({c.events.reason},'local_apf_corridor_clear'));
        row.terminalExits=sum(ismember({c.events.reason},{'terminal_handoff','formation_terminal_hold'}));
    end
    if isfield(r,'timing'),row.initialPlanSeconds=r.timing.initialPlanSeconds;row.preparationSeconds=r.timing.preparationSeconds;end
    if ~isfield(r.history,'state') || isempty(r.history.state),return;end
    h=r.history;p=item.parameters;row.jointBrakeSteps=nnz(h.jointBrake);row.originalSafetySteps=nnz(h.safetyTriggered);
    row.minimumObstacleDistance=min(h.obstacleClearance);row.minimumPairDistance=min(h.pairDistance);
    row.finalLeaderGoalDistance=norm(r.state(1:3,1)-p.target);row.finalFormationError=h.formationError(end);
    row.maximumSpeed=max(sqrt(sum(h.state(4:6,:,:).^2,1)),[],'all');
    row.maximumAcceleration=max(sqrt(sum(h.actualAcceleration.^2,1)),[],'all');
    row.cycleMedianMs=median(h.cycleMs);row.cycleP99Ms=prctile(h.cycleMs,99);row.cycleMaxMs=max(h.cycleMs);
    row.cyclesOver50Ms=nnz(h.cycleMs>1000*p.dt);
    row.collision=any(h.obstacleClearance<=p.obsSafe) || any(h.pairDistance<=p.uavSafe) || strcmp(r.reason,'boundary_violation');
end
function writejson(path,value)
    fid=fopen(path,'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s',jsonencode(value,PrettyPrint=true));
end
