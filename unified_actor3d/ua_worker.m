function ua_worker(output,shard,shards)
    ua_setup();data=load(fullfile(output,'catalogue.mat'),'items','jobs','protocol');
    [actor,cfg,s,model]=ua_load_actor();originalWeights=actor.Learnables;
    assert(strcmp(model.modelSha256,data.protocol.model.modelSha256));
    for k=1:20,adp3d_actor_action(actor,zeros(32,1));end
    selection=shard:shards:numel(data.jobs);done=0;started=tic;
    for index=selection
        job=data.jobs{index};item=data.items{job.itemIndex};
        recordPath=fullfile(output,'records',[job.id '.json']);
        if isfile(recordPath),done=done+1;continue;end
        q=item.parameters;q.adp.enable=false;q.safety.enable=job.safety;
        q.noiseStd=job.noiseStd;q.stochasticSeed=job.noiseSeed;q.disturbanceMax=0;
        q.controller=struct('kind','unified_frozen_actor','method',job.method,'routePolicy',job.routePolicy);
        if strcmp(job.campaign,'robust')
            q.robustExperiment=struct('startStep',job.gustStartStep,'duration',job.gustDuration, ...
                'amplitude',job.gustAmplitude,'direction',job.gustDirection);
        end
        errorText='';timer=tic;
        try
            if strcmp(job.campaign,'single')
                mode='apf';if strcmp(job.method,'actor'),mode='automatic';end
                [episode,summary]=apf_adp_rollout(item.singleCase,mode,actor,cfg,s);
                result=singleResult(episode,summary,q,model);audit=struct('passed',true,'steps',summary.steps,'dynamicsMaximumError',0);
                % Independently audit the scalar plant as well.
                for k=1:summary.steps
                    expected=motion_step3d(episode.states(:,k),episode.commands(:,k),zeros(3,1),q);
                    audit.dynamicsMaximumError=max(audit.dynamicsMaximumError,max(abs(expected-episode.states(:,k+1))));
                end
                assert(audit.dynamicsMaximumError<1e-10);
            else
                result=ua_simulate(q,item.scene,[],job.method,actor,cfg,s,job.routePolicy);
                result.model=model;result.model.actorEnabled=strcmp(job.method,'actor');result.model.actorCalls=result.controller.actorCalls;
                audit=ua_audit(result);
            end
            assert(isequaln(originalWeights,actor.Learnables),'UA:WeightsChanged');
        catch exception
            errorText=getReport(exception,'extended','hyperlinks','off');
            result=struct('success',false,'reason','execution_error','time',0,'history',struct(), ...
                'model',model,'params',q,'controller',struct('actorCalls',0,'activations',0,'ruleCalls',0));
            audit=struct('passed',false,'steps',0,'dynamicsMaximumError',NaN);
            fprintf('UA_ERROR %s %s\n',job.id,exception.message);
        end
        wallSeconds=toc(timer);
        flightPath=fullfile(output,'flight',[job.id '.mat']);
        save(flightPath,'result','job','item','audit','wallSeconds','errorText','-v7');
        row=summarize(result,job,audit,wallSeconds,model);row.flightSha256=ua_sha256(flightPath);
        writejson(recordPath,row);done=done+1;
        writejson(fullfile(output,'progress',sprintf('worker_%02d.json',shard)), ...
            struct('shard',shard,'shards',shards,'done',done,'assigned',numel(selection), ...
                'lastTrial',job.id,'lastReason',result.reason,'lastWallSeconds',wallSeconds, ...
                'elapsedSeconds',toc(started),'timestamp',datestr(now,31)));
        fprintf('UA_TRIAL shard=%d %d/%d %s reason=%s calls=%d wall=%.3f\n',shard,done,numel(selection),job.id,result.reason,row.actorCalls,wallSeconds);
    end
    writejson(fullfile(output,'progress',sprintf('complete_%02d.json',shard)), ...
        struct('complete',true,'shard',shard,'trials',done,'weightsUnchanged',isequaln(originalWeights,actor.Learnables),'seconds',toc(started)));
    fprintf('UA_WORKER_COMPLETE shard=%d trials=%d\n',shard,done);
end
function row=summarize(r,j,a,wall,model)
    row=struct('id',j.id,'campaign',j.campaign,'mapID',j.mapID,'family',j.family,'method',j.method, ...
        'safety',j.safety,'routePolicy',j.routePolicy,'kind',j.kind,'tag',j.tag, ...
        'noiseStd',j.noiseStd,'noiseSeed',j.noiseSeed,'gustAmplitude',j.gustAmplitude,'gustDirection',j.direction, ...
        'gustStartStep',j.gustStartStep,'gustDuration',j.gustDuration,'matchedReference',j.matchedReference, ...
        'success',r.success,'reason',r.reason,'flightSeconds',r.time,'wallSeconds',wall, ...
        'controllerID',model.controllerID,'modelSha256',model.modelSha256,'weightsSha256',model.weightsSha256, ...
        'runtimeSourceSha256',model.runtimeSourceSha256, ...
        'actorLoaded',true,'actorCalls',r.controller.actorCalls,'activations',r.controller.activations, ...
        'ruleCalls',r.controller.ruleCalls,'legacyADPCalls',0,'trainingPerformed',false, ...
        'auditPassed',a.passed,'steps',a.steps,'dynamicsMaximumError',a.dynamicsMaximumError, ...
        'minObstacleDistance',NaN,'minPairDistance',NaN,'maxSpeed',NaN,'maxCommand',NaN, ...
        'maxActualAcceleration',NaN,'pathLength',NaN,'formationRms',NaN,'safetySteps',0,'jointBrakeSteps',0, ...
        'degenerateSteps',0,'maxStallSeconds',0,'leaderControlReversals',0,'cycleP99Ms',NaN,'cycleMaxMs',NaN, ...
        'obstacleSafetyViolation',false,'pairSafetyViolation',false,'surfaceContact',false,'pulseDistance',NaN);
    h=r.history;if ~isfield(h,'state') || isempty(h.state),return;end
    row.minObstacleDistance=min(h.obstacleClearance);row.minPairDistance=min(h.pairDistance);
    row.maxSpeed=max(vecnorm(reshape(h.state(4:6,:,:),3,[])));
    row.maxCommand=max(vecnorm(reshape(h.command,3,[])));
    row.maxActualAcceleration=max(vecnorm(reshape(h.actualAcceleration,3,[])));
    row.safetySteps=nnz(h.safetyTriggered);row.jointBrakeSteps=nnz(h.jointBrake);
    row.degenerateSteps=nnz(h.tangentDegenerate);
    row.pathLength=r.metrics.leaderPathLength;row.formationRms=r.metrics.formationRms;
    row.cycleP99Ms=prctile(h.cycleMs,99);row.cycleMaxMs=max(h.cycleMs);
    row.obstacleSafetyViolation=row.minObstacleDistance<=r.params.obsSafe;
    row.pairSafetyViolation=row.minPairDistance<=r.params.uavSafe;
    row.surfaceContact=row.minObstacleDistance<=0;
    speed=vecnorm(reshape(h.state(4:6,1,:),3,[]));
    away=h.goalDistance(:)'>r.params.holdTriggerDist;
    slow=speed<.08 & away;run=0;longest=0;
    for k=1:numel(slow),if slow(k),run=run+1;longest=max(longest,run);else,run=0;end,end
    row.maxStallSeconds=longest*r.params.dt;
    u=reshape(h.command(:,1,:),3,[]);changes=0;
    for axis=1:3
        significant=u(axis,:);significant=significant(abs(significant)>.2);
        changes=changes+nnz(diff(sign(significant))~=0);
    end
    row.leaderControlReversals=changes;
    if strcmp(j.campaign,'robust') && j.gustStartStep<=size(h.state,3)
        if j.gustStartStep==1,state=r.initialFleetState;else,state=h.state(:,:,j.gustStartStep-1);end
        [~,row.pulseDistance]=check_swept3d(state,state,r.scene.obstacles,r.params);
    end
end
function result=singleResult(e,s,p,model)
    count=s.steps;h=struct();h.state=reshape(e.states(:,2:end),6,1,count);
    h.t=(1:count)'*p.dt;h.command=reshape(e.commands,3,1,count);
    h.actualAcceleration=reshape(e.accelerations,3,1,count);
    h.obstacleClearance=e.minimumObstacleDistances(:);h.pairDistance=inf(count,1);
    h.safetyTriggered=e.interventions(:);h.jointBrake=h.safetyTriggered;
    h.tangentDegenerate=false(count,1);h.goalDistance=vecnorm(e.states(1:3,2:end)-p.target)';
    h.cycleMs=e.controlCycleMilliseconds(:);h.formationError=zeros(count,1);
    result=struct('success',s.success,'reason',s.termination,'time',s.flightSeconds,'params',p, ...
        'history',h,'episode',e,'singleSummary',s,'model',model, ...
        'controller',struct('actorCalls',s.actorCalls,'activations',s.activations,'ruleCalls',0), ...
        'metrics',struct('leaderPathLength',s.pathLengthMeters,'formationRms',0));
end
function writejson(file,value)
    temp=[file '.part'];fid=fopen(temp,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value,PrettyPrint=true));fclose(fid);movefile(temp,file,'f');
end
