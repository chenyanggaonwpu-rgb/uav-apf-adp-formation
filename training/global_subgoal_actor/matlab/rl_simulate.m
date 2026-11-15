function [result,plannerCache]=rl_simulate(p,scene,plannerCache,mode,actor,cfg,s,routePolicy,runOptions)
% Adapted from frozen run_simulation3d: same route, formation and goal logic.
% Add leader-only frozen supervisor and a shared joint braking guardian.
    if nargin<9,runOptions=struct();end
    requestedOptions=runOptions;requestedActor=actor;requestedMode=mode;
    assert(~p.adp.enable && p.nUAV==5);
    assert(any(strcmp(mode,{'actor','none','rule'})));
    injectFault=isfield(p,'ablationStaleWaypointFault')&&p.ablationStaleWaypointFault;
    staleWaypointLatched=injectFault;assert(~injectFault||strcmp(routePolicy,'global'));
    controller=apf_adp_init();controller.ruleCalls=0;controller.proposalCalls=0;
    controller.proposalSeconds=0;controller.observationSeconds=0;controller.ruleDirection=zeros(3,1);
    leaderHistory=zeros(6,ceil(p.Tmax/p.dt)+1);
    actorWaypoint=p.target;actorDecisionMs=0;
    continuous=isfield(p,'v4Continuous')&&p.v4Continuous;
    isPlanner=strcmp(routePolicy,'actor_global');
    planner=struct('waypoint',p.target,'previous',nan(3,1),'lastDecision',-inf,'bestDistance',inf,'stallSince',0,'decisions',0);
    assert(~isPlanner||(strcmp(mode,'actor')&&isa(actor,'RLPolicy')&&actor.Net.schema==6));
    wind=zeros(3,1);if isfield(p,'v4WindBias'),wind=p.v4WindBias(:);end
    delay=0;if isfield(p,'v4SensorDelaySteps'),delay=p.v4SensorDelaySteps;end
    rawMeasurements=zeros(6,5,ceil(p.Tmax/p.dt)+1);lastCommand=zeros(3,5);
    filteredPosition=[];filterAlpha=1;
    if isfield(p,'v4FilterAlpha')&&p.noiseStd>0,filterAlpha=p.v4FilterAlpha;end
    moduleClock=tic;
% RUN_SIMULATION3D Synchronous 3-D formation simulation with event timings.
    p=validate_params3d(p);
    if nargin<3, plannerCache=[]; end
    if nargin<2 || isempty(scene), scene=scenario3d(p.sceneId,p.seed,p); end
    if ischar(scene) || isstring(scene), scene=scenario3d(char(scene),p.seed,p); end
    n=p.nUAV;
    steps=ceil(p.Tmax/p.dt);
    [state,offsets,heading]=init_formation3d(p);
    result=struct('success',false,'reason','not_started','time',0, ...
        'state',state,'params',p,'scene',scene,'route',zeros(3,0));
    result.controller=controller;result.controllerID=p.version;
    result.initialFleetState=state;
    if isfield(p,'initialState'),state=p.initialState;result.initialFleetState=state;end
    result.plan=struct('status','not_run');
    result.heightPlans=cell(0,1);
    result.timing=struct('initialPlanSeconds',nan,'preparationSeconds',0,'preparedCacheBytes',0,'replanSeconds',zeros(0,1), ...
        'terminalPlanSeconds',zeros(0,1));
    result.history=struct();
    if strcmp(scene.kind,'generation_failed')
        result.reason='generation_failed'; return;
    end
    obstacles=scene.obstacles;
    obstacleBounds=dv2_obstacle_bounds(obstacles);
    [initialHit,~,~,~]=check_swept3d(state,state,obstacles,p);
    if initialHit
        result.reason='invalid_scene'; return;
    end
    backup=[]; % Each safety decision is made from measured state. Initial braking infeasibility is logged, not silently filtered.
    [~,initialCanStop]=f5_braking_plan(state,obstacles,p,cfg.safety);
    result.initialCanStop=initialCanStop;
    leaderHistory(:,1)=state(:,1);
    planClock=tic;
    if isPlanner
        route=p.target;planInfo=struct('status','actor_global_subgoal_no_leader_rule_path');
    elseif strcmp(routePolicy,'local_apf')
        route=p.target;planInfo=struct('status','deliberately_disabled_local_apf_stress');
    else
        [route,planInfo,plannerCache]=plan_layers3d(state(1:3,1),p.target, ...
            obstacles,p,p.pathClearance,p.pathClearance,plannerCache);
    end
    result.timing.initialPlanSeconds=toc(planClock);
    result.plan=planInfo;result.route=route;result.initialRoute=route;
    result.heightPlans{end+1}=struct('stage','initial','vehicle',1,'info',planInfo);
    if isempty(route)
        if isfield(p,'v4AllowDirectPlannerFallback')&&p.v4AllowDirectPlannerFallback
            route=p.target;result.plan.directFallback=true;result.route=route;result.initialRoute=route;
        else
            result.reason='planning_failed'; return;
        end
    end
    terminalCache=[];
    if p.planning.prebuildGraphs
        preparationClock=tic;
        if strcmp(routePolicy,'global')
            plannerCache=prepare_planning_graphs3d(obstacles,p,p.pathClearance,plannerCache);
        end
        terminalCache=prepare_planning_graphs3d(obstacles,p,p.obsSafe+1.3);
        result.timing.preparationSeconds=toc(preparationClock);
        cacheStorage=whos('plannerCache','terminalCache');
        result.timing.preparedCacheBytes=sum([cacheStorage.bytes]);
    end
    memory=struct('trail',zeros(2,steps+1),'cum',zeros(1,steps+1), ...
        'count',1,'psi',heading,'hold',false,'holdPsi',heading, ...
        'holdAlpha',0,'holdStartRefs',zeros(3,n),'offsets',offsets);
    stochasticSeed=p.seed;
    if isfield(p,'stochasticSeed'), stochasticSeed=p.stochasticSeed; end
    rng(stochasticSeed+23001,'twister');
    positionNoise=p.noiseStd*randn(3,n,steps+1);
    memory.trail(:,1)=state(1:2,1)+positionNoise(1:2,1,1);
    disturbanceCount=ceil(steps/p.disturbanceHoldSteps);
    rng(stochasticSeed+47001,'twister');
    raw=randn(3,n,disturbanceCount);
    mag=sqrt(sum(raw.^2,1));
    disturbance=p.disturbanceMax*raw./max(mag,eps) .* ...
        rand(1,n,disturbanceCount);
    if isfield(p,'robustExperiment')
        e=p.robustExperiment;disturbance=zeros(3,n,steps);
        first=e.startStep;last=first+round(e.duration/p.dt)-1;
        if e.amplitude>0,disturbance(:,:,first:last)=repmat(e.amplitude*e.direction(:),1,n,last-first+1);end
    end
    hist.measuredState=zeros(6,n,steps);hist.rawMeasuredState=zeros(6,n,steps);hist.nominalCommand=zeros(3,n,steps);
    hist.backupAvailable=false(steps,1);hist.nominalBrakeCertified=false(steps,1);
    hist.actorAction=zeros(3,steps);hist.actorCallCount=zeros(steps,1);
    hist.actorMode=zeros(steps,1);hist.actorDecisionMs=zeros(steps,1);
    hist.jointBrake=false(steps,1);hist.command=zeros(3,n,steps);
    hist.actualAcceleration=zeros(3,n,steps);hist.goalDistance=zeros(steps,1);
    hist.t=zeros(steps,1);
    hist.state=zeros(6,n,steps);
    hist.reference=zeros(3,n,steps);
    hist.waypoint=zeros(3,steps);
    hist.formationError=zeros(steps,1);
    hist.obstacleClearance=inf(steps,1);
    hist.pairDistance=inf(steps,1);
    hist.baseNorm=zeros(steps,1);
    hist.adpNorm=zeros(steps,1);
    hist.effectiveAdpNorm=zeros(steps,1);
    hist.adpActorWeightNorm=zeros(steps,1);
    hist.adpCriticWeightNorm=zeros(steps,1);
    hist.adpLearningSkipped=false(steps,1);
    hist.adpPredictionGain=zeros(steps,1);
    hist.adpFitError=zeros(steps,1);
    hist.safetyNorm=zeros(steps,1);
    hist.commandNorm=zeros(steps,1);
    hist.actualAccelNorm=zeros(steps,1);
    hist.cycleMs=zeros(steps,1);
    hist.safetyIterations=zeros(steps,1);
    hist.safetyTriggered=false(steps,1);
    hist.predictedSafe=false(steps,1);
    hist.stateAdvanced=false(steps,1);
    hist.tangentDegenerate=false(steps,1);
    hist.referenceBlocked=false(steps,1);
    hist.holdAlpha=zeros(steps,1);
    hist.noise=positionNoise(:,:,1:steps);
    hist.disturbance=zeros(3,n,steps);
    recordAdpVectors=false;
    if recordAdpVectors
        hist.adpPreState=zeros(6,n,steps);
        hist.adpControlReference=zeros(3,n,steps);
        hist.adpControlVelocityReference=zeros(3,n,steps);
        hist.adpControlPositionRate=zeros(3,n,steps);
        hist.adpBaseVector=zeros(3,n,steps);
        hist.adpRawVector=zeros(3,n,steps);
        hist.adpEffectiveVector=zeros(3,n,steps);
        hist.adpCommandVector=zeros(3,n,steps);
        hist.adpActorWeights=zeros(18,1,steps);
        hist.adpCriticWeights=zeros(6,1,steps);
    end
    routeIndex=1;
    terminalRoutes=cell(1,n);
    terminalIndex=ones(1,n);
    terminalPlanned=false;
    holdEnteredAt=inf;
    successHold=0;
    bestWaypointDistance=inf;
    stagnant=0;
    pathLength=0;
    verticalTravel=0;
    ascent=zeros(1,n); descent=zeros(1,n); fleetPath=zeros(1,n);
    commandEffort=0;
    result.reason='timeout';
    kStop=steps;
    observer=rl_new_observer(cfg,p);
    record=struct('action_index',zeros(1,0),'actions',zeros(3,0,'single'),'times',zeros(1,0), ...
        'requested_subgoals',zeros(3,0),'executed_subgoals',zeros(3,0), ...
        'projection_distance',zeros(1,0),'repeated_subgoal',false(1,0),'reselection_reasons',{{}},'projection_methods',{{}});
    captured=false;startK=1;restoring=false;
    if isfield(requestedOptions,'resume_file')&&~isempty(requestedOptions.resume_file)
        loaded=load(requestedOptions.resume_file,'snapshot');snapshot=loaded.snapshot;
        assert(snapshot.schema==5&&isfield(snapshot.variables,'backup'),'V4:IncompleteSnapshot');
        names=fieldnames(snapshot.variables);
        for ii=1:numel(names)
            assert(isvarname(names{ii}));eval([names{ii} '=snapshot.variables.(names{ii});']);
        end
        mode=requestedMode;actor=requestedActor;startK=snapshot.step;restoring=true;captured=true;
        rng(snapshot.rng);
        observer=rl_new_observer(cfg,p);rl_restore_observer(observer,snapshot.observer);
        if isa(actor,'RLPolicy')
            policyState=actor.Stream.State;rl_restore_observer(actor,snapshot.observer);actor.Stream.State=policyState;
            if isfield(requestedOptions,'resume_policy_stream')&&requestedOptions.resume_policy_stream
                assert(isfield(snapshot,'policy_stream'));actor.Stream.State=snapshot.policy_stream;
            end
        end
        % record and planner were restored with the physical state; keep pending credit.
    end
    moduleClock=tic;
    for k=startK:steps
        cycleClock=tic;
        if ~(restoring&&k==startK)
        t=(k-1)*p.dt;
        measured=state;
        measured(1:3,:)=state(1:3,:)+positionNoise(:,:,k);
        rawMeasurements(:,:,k)=measured;
        if delay>0
            earlier=max(1,k-delay);oldMeasurement=rawMeasurements(:,:,earlier);
            % Delayed position sensor, fresh original-protocol velocity sensor.
            % Trapezoidal dead reckoning is causal and shared by all policies.
            measured(1:3,:)=oldMeasurement(1:3,:)+(k-earlier)*p.dt*.5*(oldMeasurement(4:6,:)+measured(4:6,:));
        end
        if filterAlpha<1
            if isempty(filteredPosition),filteredPosition=measured(1:3,:);else
                prediction=filteredPosition+p.dt*measured(4:6,:);
                filteredPosition=prediction+filterAlpha*(measured(1:3,:)-prediction);
            end
            measured(1:3,:)=filteredPosition;
        end
        hist.rawMeasuredState(:,:,k)=rawMeasurements(:,:,k);
        leaderHistory(:,k)=measured(:,1);
        if memory.hold
            memory.holdAlpha=min(1,(t-holdEnteredAt)/p.terminalBlendTime);
        end
        [refs,vrefs,heading,referenceBlocked]= ...
            formation_reference3d(measured,memory,obstacles,p);
        memory.psi=heading;
        while routeIndex<size(route,2) && ...
                norm(measured(1:3,1)-route(:,routeIndex))<p.waypointTol
            routeIndex=routeIndex+1;
            bestWaypointDistance=inf;
            stagnant=0;
        end
        waypoint=route(:,routeIndex);
        if memory.hold, waypoint=p.target; end
        if staleWaypointLatched,waypoint=p.target;end
        if isPlanner&&~memory.hold,waypoint=planner.waypoint;end
        nominalWaypoint=waypoint;
        controlRefs=refs;controlVrefs=vrefs;controlPositionRates=vrefs;
        if memory.hold && memory.holdAlpha>=1 && ~terminalPlanned
            terminalPlanned=true;
            for i=2:n
                terminalClock=tic;
                [terminalRoutes{i},terminalPlanInfo]=plan_layers3d(measured(1:3,i), ...
                    refs(:,i),obstacles,p,p.obsSafe+1.3,p.obsSafe+0.15,terminalCache);
                result.timing.terminalPlanSeconds(end+1,1)=toc(terminalClock);
                result.heightPlans{end+1}=struct('stage','terminal','vehicle',i,'info',terminalPlanInfo);
            end
        end
        if terminalPlanned
            for i=2:n
                local=terminalRoutes{i};
                if isempty(local), continue; end
                while terminalIndex(i)<size(local,2) && ...
                        norm(measured(1:3,i)-local(:,terminalIndex(i)))<p.waypointTol
                    terminalIndex(i)=terminalIndex(i)+1;
                end
                if terminalIndex(i)<size(local,2)
                    controlRefs(:,i)=local(:,terminalIndex(i));
                    controlPositionRates(:,i)=zeros(3,1);
                    delta=controlRefs(:,i)-measured(1:3,i);
                    controlVrefs(:,i)=min(p.vmax,sqrt(2*0.6*p.amax*norm(delta)))* ...
                        delta/max(norm(delta),eps);
                end
            end
        end
        end % a restored snapshot is already at this exact pre-context phase
        if ~captured && isfield(requestedOptions,'capture_file') && ...
                ~isempty(requestedOptions.capture_file) && ~memory.hold && ...
                mod(k-1,cfg.physicsStepsPerDecision)==0 && ...
                rl_capture_due(controller,leaderHistory(:,1:k),t,p,s,requestedOptions)
            names={'p','scene','plannerCache','planner','record','isPlanner','controller','leaderHistory','actorWaypoint','continuous', ...
                'wind','delay','rawMeasurements','lastCommand','filteredPosition','filterAlpha', ...
                'state','offsets','heading','result','obstacles','terminalCache','memory','backup', ...
                'positionNoise','disturbance','hist','routeIndex','terminalRoutes','terminalIndex', ...
                'terminalPlanned','holdEnteredAt','successHold','bestWaypointDistance','stagnant', ...
                'pathLength','verticalTravel','ascent','descent','fleetPath','commandEffort','kStop', ...
                'injectFault','staleWaypointLatched','route','n','steps','refs','vrefs','referenceBlocked','waypoint','nominalWaypoint', ...
                'controlRefs','controlVrefs','controlPositionRates','t','k','measured','cfg','s','routePolicy'};
            variables=struct();
            for ii=1:numel(names),variables.(names{ii})=eval(names{ii});end
            snapshot=struct('schema',5,'phase','before_context_and_policy_action','step',k, ...
                'time',t,'variables',variables,'observer',rl_observer_state(observer),'rng',rng);
            if isa(actor,'RLPolicy'),snapshot.policy_stream=actor.Stream.State;end
            target=requestedOptions.capture_file;parent=fileparts(target);if ~isfolder(parent),mkdir(parent);end
            temporary=[target '.part'];save(temporary,'snapshot','-v7');movefile(temporary,target,'f');
            captured=true;
            if isfield(requestedOptions,'capture_only')&&requestedOptions.capture_only
                result.reason='snapshot_captured';kStop=max(1,k-1);break;
            end
        end
        observer.setContext(measured,nominalWaypoint,t,obstacles,p,controller,lastCommand);
        if isa(actor,'RLPolicy')||isa(actor,'LocalRLPolicy')||isa(actor,'GlobalEscapePolicy')
            actor.Observations=observer.Observations;actor.ObservationTimes=observer.ObservationTimes;
            actor.Positions=observer.Positions;actor.GoalDistances=observer.GoalDistances;
            actor.LastControl=observer.LastControl;actor.Context=observer.Context;
            if isa(actor,'RLPolicy')||isa(actor,'GlobalEscapePolicy'),actor.GlobalFeatures=observer.GlobalFeatures;end
        end
        actorDecisionMs=0;
        if ~strcmp(mode,'none') && ~memory.hold && mod(k-1,cfg.physicsStepsPerDecision)==0
            decisionClock=tic;
            previousMode=controller.mode;
            if isPlanner
                [actorWaypoint,planner,controller,actorDiagnostic]=rl_planner_decide(planner,controller,actor,measured,t,obstacles,p,cfg.planner);
                if actorDiagnostic.decided
                    record.action_index(end+1)=size(observer.Observations,2)-1;
                    record.actions(:,end+1)=single(actorDiagnostic.action);record.times(end+1)=t;
                    record.requested_subgoals(:,end+1)=actorDiagnostic.requested;
                    record.executed_subgoals(:,end+1)=actorDiagnostic.executed;
                    record.projection_distance(end+1)=actorDiagnostic.projection_distance;
                    record.repeated_subgoal(end+1)=actorDiagnostic.repeated;
                    record.reselection_reasons{end+1}=actorDiagnostic.reason;
                    record.projection_methods{end+1}=actorDiagnostic.method;
                end
            elseif continuous
                if controller.mode~=2,controller.mode=2;controller.modeSince=t;controller.activations=controller.activations+1;end
                if strcmp(mode,'rule')
                    observation=adp3d_observation(measured(:,1),obstacles,p,cfg.scanRange);
                    [action,controller.ruleDirection]=fc_rule_action(observation,cfg,p.obsSafe,controller.ruleDirection);
                    controller.ruleCalls=controller.ruleCalls+1;
                else
                    action=v4_numeric_action(actor,zeros(32,1));controller.actorCalls=controller.actorCalls+1;
                end
                [actorWaypoint,~]=adp3d_decode_action(measured(:,1),action,cfg.maxOffset);
                controller.proposalCalls=controller.proposalCalls+1;actorDiagnostic=struct('action',action);
            else
                [actorWaypoint,controller,actorDiagnostic]=rl_fc_decide(controller,leaderHistory(:,1:k), ...
                    t,obstacles,p,cfg,s,actor,mode);
            end
            if ~isPlanner||actorDiagnostic.decided,hist.actorAction(:,k)=actorDiagnostic.action;end
            if ~isPlanner&&controller.mode==2
                record.action_index(end+1)=size(observer.Observations,2)-1;
                record.actions(:,end+1)=single(actorDiagnostic.action);record.times(end+1)=t;
            end
            if ~isPlanner && previousMode==2 && controller.mode~=2 && controller.mode~=5 && strcmp(routePolicy,'global')
                if injectFault && ~strcmp(controller.events(end).reason,'elapsed_budget_handoff'),staleWaypointLatched=false;end
                rejoinClock=tic;
                [newRoute,rejoinInfo,plannerCache]=plan_layers3d(measured(1:3,1),p.target, ...
                    obstacles,p,p.pathClearance,p.obsSafe+0.15,plannerCache);
                result.timing.replanSeconds(end+1,1)=toc(rejoinClock);
                result.heightPlans{end+1}=struct('stage','actor_rejoin','vehicle',1,'info',rejoinInfo);
                if isempty(newRoute)
                    controller.mode=5;controller.failureReason='rejoin_planning_failed';
                else
                    route=newRoute;routeIndex=1;waypoint=route(:,1);
                    stagnant=0;bestWaypointDistance=inf;
                end
            end
            actorDecisionMs=1000*toc(decisionClock);
        end
        if memory.hold && controller.mode==2
            controller.events(end+1)=struct('time',t,'from',2,'to',3, ...
                'reason','formation_terminal_hold','goalDistance',norm(p.target-state(1:3,1)), ...
                'speed',norm(state(4:6,1)),'position',state(1:3,1));
            controller.mode=3;controller.modeSince=t;controller.historySince=t;
        end
        if controller.mode==2 && ~memory.hold,waypoint=actorWaypoint;end
        controlObstacles=dv2_near_obstacles(measured(1:3,:),obstacles,obstacleBounds,max(8,p.rho0+2));
        [base,controlInfo]=f5_control(measured,controlRefs,controlVrefs,waypoint,controlObstacles,p,memory.hold);
        adpControl=zeros(3,n);
        candidate=limit_vector3d(base+adpControl,p.amax);
        if p.safety.enable
            [command,safetyInfo]=dv2_fast_resolve_safety3d(measured,candidate,controlObstacles,p);
        else
            command=candidate;
            safetyInfo=struct('iterations',0,'predictedSafe',false, ...
                'triggered',false,'correctionNorm',0);
        end
        learningInfo=struct('skipped',false,'predictionGain',0,'fitError',0);
        appliedWind=wind;
        if isfield(p,'v4WindIntervals')
            active=any(t>=p.v4WindIntervals(:,1)&t<p.v4WindIntervals(:,2));if ~active,appliedWind=zeros(3,1);end
        end
        stepDisturbance=disturbance(:,:,ceil(k/p.disturbanceHoldSteps))+appliedWind;
        oldState=state;
        if controller.mode==5,command=limit_vector3d(-measured(4:6,:)/p.dt,p.amax);end
        hist.nominalCommand(:,:,k)=candidate;
        if p.safety.enable
            [command,backup,jointInfo]=ua_select_command(measured,command,backup,controlObstacles,p,cfg.safety);
        else
            jointInfo=struct('intervention',false,'backupAvailable',false,'canStop',false);
        end
        command=limit_vector3d(command,p.amax);
        [state,actualAccel]=motion_step3d(state,command,stepDisturbance,p);lastCommand=command;
        hist.measuredState(:,:,k)=measured;hist.actorCallCount(k)=controller.actorCalls;
        hist.backupAvailable(k)=jointInfo.backupAvailable;hist.nominalBrakeCertified(k)=jointInfo.canStop;
        moved=true;leaderHistory(:,k+1)=state(:,1);
        [collision,minObstacle,minPair,boundaryHit]= ...
            check_swept3d(oldState,state,obstacles,p);
        if moved
            pathLength=pathLength+norm(state(1:3,1)-oldState(1:3,1));
            verticalTravel=verticalTravel+abs(state(3,1)-oldState(3,1));
            dz=state(3,:)-oldState(3,:);
            ascent=ascent+max(dz,0); descent=descent+max(-dz,0);
            fleetPath=fleetPath+vecnorm(state(1:3,:)-oldState(1:3,:));
            commandEffort=commandEffort+p.dt*sum(command.^2,'all');
            memory.count=memory.count+1;
            memory.trail(:,memory.count)=state(1:2,1)+positionNoise(1:2,1,k+1);
            % Velocity feedback is noise-free in this protocol. Integrating its
            % horizontal magnitude avoids turning position jitter into travel.
            memory.cum(memory.count)=memory.cum(memory.count-1)+ ...
                p.dt*norm(state(4:5,1));
            measuredNext=state;
            measuredNext(1:3,:)=state(1:3,:)+positionNoise(:,:,k+1);
            [newRefs,~,~,~]=formation_reference3d(measuredNext,memory,obstacles,p);
            % No old residual ADP, Critic evaluation or parameter update.
        else
            newRefs=refs;
        end
        formError=max(vecnorm(state(1:3,2:n)-newRefs(:,2:n)));
        leaderError=norm(state(1:3,1)-p.target);
        maxSpeed=max(vecnorm(state(4:6,:)));
        if moved && ~memory.hold && norm(measuredNext(1:3,1)-p.target)<=p.holdTriggerDist
            memory.hold=true;
            memory.holdPsi=heading;
            memory.holdStartRefs=newRefs;
            memory.holdAlpha=0;
            holdEnteredAt=t+p.dt;
        end
        if moved && ~memory.hold && strcmp(routePolicy,'global') && (controller.mode~=2||continuous)
            waypointDistance=norm(measuredNext(1:3,1)-nominalWaypoint);
            if waypointDistance<bestWaypointDistance-0.15
                bestWaypointDistance=waypointDistance;stagnant=0;
            else
                stagnant=stagnant+p.dt;
            end
            if stagnant>=p.replanAfter
                replanClock=tic;
                [newRoute,newPlanInfo,plannerCache]=plan_layers3d(measured(1:3,1), ...
                    p.target,obstacles,p,p.pathClearance,p.obsSafe+0.15,plannerCache);
                result.timing.replanSeconds(end+1,1)=toc(replanClock);
                result.heightPlans{end+1}=struct('stage','replan','vehicle',1,'info',newPlanInfo);
                if isempty(newRoute)
                    result.reason='replan_failed';
                else
                    route=newRoute;routeIndex=1;
                    stagnant=0;bestWaypointDistance=inf;
                end
            end
        end
        good=memory.hold && leaderError<=p.goalTol && ...
            formError<=p.formTol && maxSpeed<=p.vTol && ~referenceBlocked;
        if isfield(p,'robustExperiment')
            good=good && t+p.dt >= (p.robustExperiment.startStep-1)*p.dt+p.robustExperiment.duration+5;
        end
        if good, successHold=successHold+p.dt; else, successHold=0; end
        hist.actorMode(k)=controller.mode;hist.actorDecisionMs(k)=actorDecisionMs;
        hist.jointBrake(k)=jointInfo.intervention;hist.command(:,:,k)=command;
        hist.actualAcceleration(:,:,k)=actualAccel;hist.goalDistance(k)=leaderError;
        hist.t(k)=t+p.dt;
        hist.state(:,:,k)=state;
        hist.reference(:,:,k)=newRefs;
        hist.waypoint(:,k)=waypoint;
        hist.formationError(k)=formError;
        hist.obstacleClearance(k)=minObstacle;
        hist.pairDistance(k)=minPair;
        hist.baseNorm(k)=mean(vecnorm(base));
        hist.adpNorm(k)=mean(vecnorm(adpControl));
        hist.effectiveAdpNorm(k)=mean(vecnorm(candidate-limit_vector3d(base,p.amax)));
        hist.adpActorWeightNorm(k)=norm(p.adp.wA(:));
        hist.adpCriticWeightNorm(k)=norm(p.adp.wC);
        hist.adpLearningSkipped(k)=learningInfo.skipped;
        hist.adpPredictionGain(k)=learningInfo.predictionGain;
        hist.adpFitError(k)=learningInfo.fitError;
        hist.safetyNorm(k)=safetyInfo.correctionNorm;
        hist.commandNorm(k)=mean(vecnorm(command));
        hist.actualAccelNorm(k)=mean(vecnorm(actualAccel));
        hist.safetyIterations(k)=safetyInfo.iterations;
        hist.safetyTriggered(k)=safetyInfo.triggered || jointInfo.intervention;
        hist.predictedSafe(k)=safetyInfo.predictedSafe;
        hist.stateAdvanced(k)=moved;
        hist.tangentDegenerate(k)=any(controlInfo.tangentDegenerate);
        hist.referenceBlocked(k)=referenceBlocked;
        hist.holdAlpha(k)=memory.holdAlpha;
        hist.disturbance(:,:,k)=stepDisturbance;
        if recordAdpVectors
            hist.adpPreState(:,:,k)=measured;
            hist.adpControlReference(:,:,k)=controlInfo.adpContext.position;
            hist.adpControlVelocityReference(:,:,k)=controlInfo.adpContext.velocity;
            hist.adpControlPositionRate(:,:,k)=controlInfo.adpContext.positionRate;
            hist.adpBaseVector(:,:,k)=base;
            hist.adpRawVector(:,:,k)=adpControl;
            hist.adpEffectiveVector(:,:,k)=candidate-limit_vector3d(base,p.amax);
            hist.adpCommandVector(:,:,k)=command;
            hist.adpActorWeights(:,:,k)=p.adp.wA(:);
            hist.adpCriticWeights(:,:,k)=p.adp.wC;
        end
        hist.cycleMs(k)=1000*toc(cycleClock);
        if ~all(isfinite(state),'all')
            result.reason='invalid_state';
        elseif collision
            if boundaryHit, result.reason='boundary_violation';
            else, result.reason='collision'; end
        elseif controller.mode==5 && maxSpeed<=cfg.safety.stopSpeed
            result.reason=controller.failureReason;
        elseif successHold>=p.holdDuration
            result.reason='success';result.success=true;
        end
        if ~strcmp(result.reason,'timeout')
            kStop=k;break;
        end
    end
    if isa(actor,'RLPolicy')
        terminalMeasured=state;terminalMeasured(1:3,:)=state(1:3,:)+positionNoise(:,:,kStop+1);
        actor.setContext(terminalMeasured,p.target,hist.t(kStop),obstacles,p,controller,lastCommand);
    end
    observer.setContext(terminalMeasuredForRecord(state,positionNoise,kStop),p.target,hist.t(kStop),obstacles,p,controller,lastCommand,true);
    record.observations=observer.Observations;record.observation_times=observer.ObservationTimes;
    record.positions=single(observer.Positions);record.global_features=observer.GlobalFeatures;
    record.global_observation_schema=1;record.actor_schema=6;record.task_protocol='global_subgoal_v1';
    result.planner=planner;result.leaderRulePlannerCalls=sum(cellfun(@(x)x.vehicle==1&&~strcmp(x.stage,'initial'),result.heightPlans));
    if ~isPlanner&&strcmp(routePolicy,'global'),result.leaderRulePlannerCalls=result.leaderRulePlannerCalls+1;end
    record.capture_written=captured;record.resume_step=startK;
    result.trainingRecord=record;
    if injectFault,result.injectedFault=struct('released',~staleWaypointLatched,'globalPlannerEnabled',true);end
    result.controller=controller;result.initialState=leaderHistory(:,1);
    result.routePolicy=routePolicy;result.mode=mode;result.wallSeconds=toc(moduleClock);
    result.time=hist.t(kStop);
    result.state=state;
    result.params=p;
    result.route=route;
    result.history=trim_history(hist,kStop);
    result.metrics=struct('leaderPathLength',pathLength, ...
        'formationRms',sqrt(mean(hist.formationError(1:kStop).^2)), ...
        'minObstacleClearance',min(hist.obstacleClearance(1:kStop)), ...
        'minPairDistance',min(hist.pairDistance(1:kStop)), ...
        'leaderHeightSpan',max([p.start(3);squeeze(hist.state(3,1,1:kStop))])- ...
            min([p.start(3);squeeze(hist.state(3,1,1:kStop))]), ...
        'leaderVerticalTravel',verticalTravel,'leaderAscent',ascent(1), ...
        'leaderDescent',descent(1),'fleetAscent',sum(ascent), ...
        'fleetDescent',sum(descent),'fleetPathLength',sum(fleetPath), ...
        'commandEffort',commandEffort);
end

function trimmed=trim_history(hist,count)
    names=fieldnames(hist);
    trimmed=struct();
    for i=1:numel(names)
        name=names{i};value=hist.(name);
        if ndims(value)==3
            trimmed.(name)=value(:,:,1:count);
        elseif size(value,2)>1 && size(value,1)==3
            trimmed.(name)=value(:,1:count);
        else
            trimmed.(name)=value(1:count,:);
        end
    end
end

function measured=terminalMeasuredForRecord(state,noise,kStop)
    measured=state;measured(1:3,:)=state(1:3,:)+noise(:,:,kStop+1);
end
