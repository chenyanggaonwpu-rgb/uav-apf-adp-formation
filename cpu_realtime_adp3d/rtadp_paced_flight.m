function [episode,summary]=rtadp_paced_flight(item,architecture,actor,resident,pool,cfg,s,protocol,fault,paced)
% 20Hz APF/safety and 5Hz supervisor. Real process isolation for async trials.
% The plant retains the original 50ms integration. Whole missed physical cycles
% advance with the held command; a late computed command is never applied to a stale state.
    if nargin<11, paced=true; end
    p=item.parameters; obstacles=item.obstacles; state=item.initialState;
    [backup,ok]=adp3d_braking_plan(state,obstacles,p,cfg.safety); assert(ok);
    limit=round(protocol.clockBudgetSeconds/p.dt);
    states=zeros(6,limit+1); states(:,1)=state;
    commands=zeros(3,limit); accelerations=commands;
    modeLog=zeros(1,limit); braking=false(1,limit); staleHolding=false(1,limit);
    cycles=zeros(1,limit); releases=zeros(1,limit); finishes=zeros(1,limit);
    lateness=zeros(1,limit); deadlineMiss=false(1,limit); forcedHolds=false(1,limit);
    distances=inf(1,limit); interventions=false(1,limit);
    controller=apf_adp_init(); waypoint=p.target; lastActorSource=NaN;
    actorOnlyAsync=strcmp(architecture,'actor_async');
    isAsync=ismember(architecture,{'async','actor_async'});
    cache=RTFrozenActorCache(); obsoleteGenerationDrops=0;
    future=[]; requestWall=NaN; requestTime=NaN; requestMode=NaN; requestMissCounted=false;
    requests=0; completed=0; staleDrops=0; failedJobs=0; missedJobDeadlines=0;
    activeJobs=0; skippedReleases=0; acceptAges=[]; workerTimes=[];
    appliedEvents=struct('sourceTime',{},'appliedTime',{},'reason',{},'from',{},'to',{});
    count=0; collision=false; viabilityLost=false; termination='flight_timeout';
    previousCommand=zeros(3,1); emergencyStop=false; clock=tic;
    while count<limit
        release=count*p.dt;
        if paced, rtadp_wait_release(clock,release); end
        started=toc(clock); timer=tic;
        releases(count+1)=started; lateness(count+1)=max(0,started-release);
        if actorOnlyAsync
            if ~isempty(future) && isfile(future.result)
                age=toc(clock)-requestWall;
                result=rtadp_ipc_fetch(future); completed=completed+1;
                workerTimes(end+1)=result.computeSeconds*1000; %#ok<AGROW>
                if age>protocol.actorResultLifetimeSeconds
                    staleDrops=staleDrops+1;
                    if ~requestMissCounted, missedJobDeadlines=missedJobDeadlines+1; end
                elseif controller.mode==2 && result.generation==controller.activations
                    cache.Action=result.action; lastActorSource=requestTime;
                    acceptAges(end+1)=age; %#ok<AGROW>
                else
                    obsoleteGenerationDrops=obsoleteGenerationDrops+1;
                end
                future=[];
            end
            if ~isempty(future) && ~requestMissCounted && toc(clock)-requestWall>protocol.actorResultLifetimeSeconds
                missedJobDeadlines=missedJobDeadlines+1; requestMissCounted=true;
            end
            if mod(count,cfg.physicsStepsPerDecision)==0
                if controller.mode~=2, cache.Action=zeros(3,1); lastActorSource=NaN; end
                cache.HasObservation=false;
                oldEvents=numel(controller.events);
                [waypoint,controller,~]=apf_adp_decide(controller,states(:,1:count+1),release, ...
                    obstacles,p,cfg,s,cache);
                for e=oldEvents+1:numel(controller.events)
                    event=controller.events(e);
                    appliedEvents(end+1)=struct('sourceTime',event.time,'appliedTime',release, ...
                        'reason',event.reason,'from',event.from,'to',event.to); %#ok<AGROW>
                end
                if controller.mode==2 && cache.HasObservation
                    if isempty(future)
                        activeJobs=activeJobs+1; delay=0;
                        if strcmp(fault,'actor_burst') && ismember(activeJobs,protocol.faultActorActiveJobs)
                            delay=protocol.faultActorDelaySeconds;
                        end
                        requestTime=release; requestWall=toc(clock); requestMissCounted=false;
                        requests=requests+1;
                        future=rtadp_ipc_submit_actor(pool,cache.LastObservation,release,controller.activations,delay);
                    else, skippedReleases=skippedReleases+1; end
                end
            end
        elseif strcmp(architecture,'async')
            if ~isempty(future) && isfile(future.result)
                age=toc(clock)-requestWall;
                try
                    result=rtadp_ipc_fetch(future); completed=completed+1;
                    workerTimes(end+1)=result.computeSeconds*1000; %#ok<AGROW>
                    if age<=protocol.actorResultLifetimeSeconds
                        oldEvents=numel(controller.events); controller=result.controller;
                        waypoint=result.waypoint; acceptAges(end+1)=age; %#ok<AGROW>
                        if controller.mode==2, lastActorSource=requestTime; end
                        for e=oldEvents+1:numel(controller.events)
                            event=controller.events(e);
                            appliedEvents(end+1)=struct('sourceTime',event.time,'appliedTime',release, ...
                                'reason',event.reason,'from',event.from,'to',event.to); %#ok<AGROW>
                        end
                    else
                        staleDrops=staleDrops+1;
                        if ~requestMissCounted, missedJobDeadlines=missedJobDeadlines+1; end
                    end
                catch exception
                    failedJobs=failedJobs+1;
                    warning('RTADP:WorkerResult','Worker result failed: %s',exception.message);
                end
                future=[];
            end
            if ~isempty(future) && ~requestMissCounted && toc(clock)-requestWall>protocol.actorResultLifetimeSeconds
                missedJobDeadlines=missedJobDeadlines+1; requestMissCounted=true;
            end
            if mod(count,cfg.physicsStepsPerDecision)==0
                if isempty(future)
                    delay=0;
                    if controller.mode==2
                        activeJobs=activeJobs+1;
                        if strcmp(fault,'actor_burst') && ismember(activeJobs,protocol.faultActorActiveJobs)
                            delay=protocol.faultActorDelaySeconds;
                        end
                    end
                    requestTime=release; requestWall=toc(clock); requestMode=controller.mode; %#ok<NASGU>
                    requestMissCounted=false; requests=requests+1;
                    future=rtadp_ipc_submit(pool,controller, ...
                        states(:,1:count+1),release,obstacles,p,delay);
                else
                    skippedReleases=skippedReleases+1;
                end
            end
        elseif mod(count,cfg.physicsStepsPerDecision)==0
            oldEvents=numel(controller.events);
            [waypoint,controller,~]=apf_adp_decide(controller,states(:,1:count+1),release, ...
                obstacles,p,cfg,s,actor);
            if controller.mode==2, lastActorSource=release; end
            for e=oldEvents+1:numel(controller.events)
                event=controller.events(e);
                appliedEvents(end+1)=struct('sourceTime',event.time,'appliedTime',release, ...
                    'reason',event.reason,'from',event.from,'to',event.to); %#ok<AGROW>
            end
        end
        useBrake=controller.mode==5 || emergencyStop || (isAsync ...
            && controller.mode==2 && (isnan(lastActorSource) || release-lastActorSource>protocol.actorStaleBrakeSeconds));
        if useBrake, nominal=-state(4:6)/p.dt;
        else, nominal=adp3d_leader_command(state,waypoint,obstacles,p); end
        if isempty(backup) || isempty(backup.commands) || norm(backup.states(:,1)-state)>1e-8
            [backup,ok]=adp3d_braking_plan(state,obstacles,p,cfg.safety);
            if ~ok, viabilityLost=true; emergencyStop=true; end
        end
        if emergencyStop
            executed=-state(4:6)/p.dt;
            [next,actual]=motion_step3d(state,executed,zeros(3,1),p);
            [hit,minDistance]=check_swept3d(state,next,obstacles,p);
            step=struct('executedCommand',executed,'actualAcceleration',actual, ...
                'collision',hit,'minimumObstacleDistance',minDistance,'intervention',true);
            nextBackup=[];
        else
            [next,nextBackup,step]=adp3d_safe_step(state,nominal,backup,obstacles,p,cfg.safety);
        end
        cpuMilliseconds=1000*toc(timer);
        finished=toc(clock);
        % Complete elapsed 50ms cycles continue physically under the previous output.
        missed=0;
        if paced, missed=max(0,floor((finished-release)/p.dt)); end
        if missed>0
            for missedIndex=1:missed
                if count>=limit, break; end
                [held,actual]=motion_step3d(state,previousCommand,zeros(3,1),p);
                [hit,minDistance]=check_swept3d(state,held,obstacles,p);
                count=count+1; states(:,count+1)=held;
                commands(:,count)=previousCommand; accelerations(:,count)=actual;
                forcedHolds(count)=true; deadlineMiss(count)=true; distances(count)=minDistance;
                modeLog(count)=controller.mode; cycles(count)=cpuMilliseconds;
                finishes(count)=finished;
                state=held; collision=collision || hit;
                if collision, termination='collision_during_missed_update'; break; end
            end
            backup=[];
            if collision, break; end
            continue; % discard the late acceleration computed from the old state
        end
        count=count+1; states(:,count+1)=next; commands(:,count)=step.executedCommand;
        accelerations(:,count)=step.actualAcceleration; modeLog(count)=controller.mode;
        braking(count)=useBrake; cycles(count)=cpuMilliseconds; finishes(count)=finished;
        deadlineMiss(count)=finished>release+p.dt; interventions(count)=step.intervention;
        distances(count)=step.minimumObstacleDistance;
        backup=nextBackup; state=next; previousCommand=step.executedCommand;
        collision=collision || step.collision;
        if collision, termination='collision'; break; end
        if norm(p.target-state(1:3))<=p.goalTol && norm(state(4:6))<=p.vTol
            termination='goal'; break;
        end
        if (controller.mode==5 || emergencyStop) && norm(state(4:6))<=cfg.safety.stopSpeed
            if emergencyStop, termination='lost_braking_viability'; else, termination=controller.failureReason; end
            break;
        end
    end
    elapsed=toc(clock);
    if ~isempty(future)
        rtadp_ipc_wait(future,10); % outside the measured flight; single job drained before next trial
    end
    relevant=cycles(1:count); relevant=relevant(relevant>0);
    if isempty(acceptAges), acceptAges=NaN; end
    if isempty(workerTimes), workerTimes=NaN; end
    summary=struct('id',item.id,'architecture',architecture,'fault',fault,'paced',paced, ...
        'success',strcmp(termination,'goal'),'termination',termination,'collision',collision, ...
        'flightSeconds',count*p.dt,'wallSeconds',elapsed,'steps',count, ...
        'activations',controller.activations,'actorCallsAccepted',controller.actorCalls, ...
        'requests',requests,'completedJobs',completed,'staleResultsDropped',staleDrops, ...
        'workerRunsActorOnly',actorOnlyAsync,'obsoleteGenerationDrops',obsoleteGenerationDrops, ...
        'missedActorDeadlines',missedJobDeadlines,'failedWorkerJobs',failedJobs, ...
        'skippedActorReleases',skippedReleases,'brakingForStaleActorSteps',sum(braking(1:count) & modeLog(1:count)==2), ...
        'heldCommandStepsAfterMainOverrun',sum(forcedHolds(1:count)), ...
        'mainDeadlineMisses',sum(deadlineMiss(1:count)),'mainCycleP99Ms',prctile(relevant,99), ...
        'mainCycleMaxMs',max(relevant),'startLatenessP99Ms',1000*prctile(lateness(1:count),99), ...
        'acceptedResultMaxAgeMs',1000*max(acceptAges),'workerComputeP99Ms',prctile(workerTimes,99), ...
        'hardObstacleMargin',min(distances(1:count))-p.obsSafe, ...
        'safetyInterventions',sum(interventions(1:count)),'lostBrakingViability',viabilityLost, ...
        'finalGoalDistance',norm(p.target-state(1:3)),'finalSpeed',norm(state(4:6)), ...
        'trainingPerformed',false,'criticLoaded',false);
    episode=struct('item',item,'states',states(:,1:count+1),'commands',commands(:,1:count), ...
        'accelerations',accelerations(:,1:count),'modeLog',modeLog(1:count), ...
        'braking',braking(1:count),'forcedHolds',forcedHolds(1:count), ...
        'cycleMilliseconds',cycles(1:count),'releaseWallSeconds',releases(1:count), ...
        'finishWallSeconds',finishes(1:count),'startLatenessSeconds',lateness(1:count), ...
        'deadlineMiss',deadlineMiss(1:count),'minimumObstacleDistances',distances(1:count), ...
        'appliedEvents',appliedEvents,'controller',controller,'summary',summary);
end
