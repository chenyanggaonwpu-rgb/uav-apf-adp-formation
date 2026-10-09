function run_normal_seed_study3d(studyDir,worker,phase)
% RUN_NORMAL_SEED_STUDY3D Generation barrier followed by complete frozen flights.
    snap=fullfile(studyDir,'source_snapshot');
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'),fullfile(snap,'experiments3d'));
    assert(startsWith(which('run_simulation3d'),snap) && startsWith(which('normal_scene3d'),snap));
    stored=load(fullfile(studyDir,'protocol.mat'),'protocol');protocol=stored.protocol;
    range=protocol.ranges(worker,:);directory=fullfile(studyDir,'workers',sprintf('worker_%02d',worker));
    if ~isfolder(directory),mkdir(directory);end
    if strcmp(phase,'flight'),assert(isfile(fullfile(studyDir,'pre_run_freeze.json')));end
    rows=struct([]);proposals=struct([]);timer=tic;write_status('running');
    for seed=range(1):range(2)
        mapFile=fullfile(studyDir,'maps',sprintf('seed_%06d.mat',seed));
        if strcmp(phase,'generate')
            if isfile(mapFile)
                d=load(mapFile,'generationRow','q','scene');row=d.generationRow;q=d.q;scene=d.scene;
            else
                clock=tic;q=protocol.p;q.seed=seed;
                [scene,metadata,audit]=normal_scene3d(seed,q,protocol.generation);
                q.randomObstacleCount=metadata.obstacleCount;
                mapHash=normal_seed_study_io3d('map_hash',q,scene);check=metadata.check;
                duration=nan;expanded=0;
                if isfield(check.witness,'duration'),duration=check.witness.duration;end
                if isfield(check.witness,'expandedNodes'),expanded=check.witness.expandedNodes;end
                row=struct('seed',seed,'mapFile',string(sprintf('maps/seed_%06d.mat',seed)), ...
                    'mapHash',string(mapHash),'accepted',metadata.accepted,'attempts',audit.attempts, ...
                    'rejectedMaps',audit.rejectedMaps,'proposalSeed',metadata.proposalSeed, ...
                    'obstacleCount',metadata.obstacleCount,'sphereCount',metadata.sphereCount, ...
                    'cylinderCount',metadata.cylinderCount,'boxCount',metadata.boxCount, ...
                    'nominalVolumeFraction',metadata.nominalVolumeFraction, ...
                    'estimatedUnionVolumeFraction',metadata.estimatedUnionVolumeFraction, ...
                    'initialClearance',check.initialClearance,'terminalClearance',check.terminalClearance, ...
                    'endpointClearance',check.endpointClearance,'generationReason',string(check.reason), ...
                    'witnessStatus',string(check.feasibilityStatus),'witnessDuration',duration, ...
                    'witnessExpandedNodes',expanded,'generationSeconds',toc(clock));
                generationRow=row;temporary=[mapFile,'.tmp'];
                save(temporary,'q','scene','metadata','audit','mapHash','generationRow','-v7');movefile(temporary,mapFile,'f');
            end
            assert(q.seed==seed && row.seed==seed && scene.generationAudit.requestedSeed==seed);
            for k=1:numel(scene.generationAudit.proposals)
                a=scene.generationAudit.proposals{k};c=a.check;expanded=0;duration=nan;
                if isfield(c.witness,'expandedNodes'),expanded=c.witness.expandedNodes;end
                if isfield(c.witness,'duration'),duration=c.witness.duration;end
                proposals=[proposals;struct('seed',seed,'attempt',k,'proposalSeed',a.proposalSeed, ...
                    'obstacleCount',a.samplingMetadata.obstacleCount,'accepted',c.accepted, ...
                    'reason',string(c.reason),'feasibilityStatus',string(c.feasibilityStatus), ...
                    'initialClearance',c.initialClearance,'terminalClearance',c.terminalClearance, ...
                    'witnessExpandedNodes',expanded,'witnessDuration',duration)]; %#ok<AGROW>
            end
        elseif strcmp(phase,'flight')
            trials=fullfile(directory,'trials');if ~isfolder(trials),mkdir(trials);end
            file=fullfile(trials,sprintf('normal_%06d.mat',seed));
            if isfile(file)
                d=load(file,'row','errorText');row=d.row;assert(isempty(d.errorText),'UAV3D:TrialError','Unresolved execution error at seed %d.',seed);
            else
                d=load(mapFile);q=d.q;scene=d.scene;mapHash=d.mapHash;row=d.generationRow;
                row.success=false;row.reason="execution_error";row.planStatus="not_run";row.fixedPlanStatus="not_run";
                row.initialPlanFound=false;row.initialLayerCount=numel(q.heightLayers);row.peakLayerCount=row.initialLayerCount;
                row.addedCount=0;row.adaptiveAttempted=false;row.addedHeights="";
                row.initialPlanSeconds=nan;row.time=0;row.path=nan;row.verticalTravel=nan;row.formationRms=nan;
                row.minObstacleClearance=nan;row.minPairDistance=nan;row.cycles=0;row.replanCount=0;row.safetyTriggers=0;
                row.finalLeaderError=nan;row.finalFormationError=nan;row.finalMaxSpeed=nan;
                errorText='';r=[];clock=tic;
                try
                    r=run_simulation3d(q,scene);row.success=r.success;row.reason=string(r.reason);row.time=r.time;
                    row.planStatus=string(r.plan.status);row.initialPlanFound=ismember(row.planStatus,["direct","path"]);
                    row.initialPlanSeconds=r.timing.initialPlanSeconds;row.replanCount=numel(r.timing.replanSeconds);
                    if isfield(r.plan,'heightSelection')
                        hs=r.plan.heightSelection;row.fixedPlanStatus=string(hs.fixedStatus);row.initialLayerCount=numel(hs.layers);
                        row.peakLayerCount=row.initialLayerCount;row.addedCount=numel(hs.addedHeights);row.adaptiveAttempted=hs.attempted;
                        if ~isempty(hs.addedHeights),row.addedHeights=strjoin(compose('%.12g',hs.addedHeights),';');end
                        for k=1:numel(r.heightPlans),row.peakLayerCount=max(row.peakLayerCount,numel(r.heightPlans{k}.info.heightLayers));end
                    end
                    if isfield(r,'metrics')
                        row.path=r.metrics.leaderPathLength;row.verticalTravel=r.metrics.leaderVerticalTravel;
                        row.formationRms=r.metrics.formationRms;row.minObstacleClearance=r.metrics.minObstacleClearance;row.minPairDistance=r.metrics.minPairDistance;
                    end
                    if isfield(r.history,'t') && ~isempty(r.history.t)
                        row.cycles=numel(r.history.t);row.safetyTriggers=nnz(r.history.safetyTriggered);
                        row.finalLeaderError=norm(r.state(1:3,1)-q.target);row.finalFormationError=r.history.formationError(end);
                        row.finalMaxSpeed=max(vecnorm(r.state(4:6,:)));
                    end
                catch ex
                    errorText=getReport(ex,'extended','hyperlinks','off');row.success=false;row.reason="execution_error";
                end
                row.wallSeconds=toc(clock);temporary=[file,'.tmp'];
                save(temporary,'q','scene','mapHash','row','r','errorText','-v7');movefile(temporary,file,'f');
                if ~isempty(errorText)
                    fid=fopen(strrep(file,'.mat','.error.txt'),'w','n','UTF-8');fprintf(fid,'%s',errorText);fclose(fid);
                    error('UAV3D:TrialError','Execution error at seed %d; raw data retained.',seed);
                end
            end
        else
            error('UAV3D:StudyPhase','Unknown phase %s.',phase);
        end
        rows=[rows;row]; %#ok<AGROW>
        normal_seed_study_io3d('table',fullfile(directory,[phase,'.csv']),struct2table(rows));
        if strcmp(phase,'generate')
            normal_seed_study_io3d('table',fullfile(directory,'proposals.csv'),struct2table(proposals));
        end
        write_status('running');
        if mod(numel(rows),5)==0,fprintf('NORMAL_PROGRESS phase=%s worker=%d completed=%d/%d seed=%d wall=%.1f\n',phase,worker,numel(rows),range(2)-range(1)+1,seed,toc(timer));end
    end
    write_status('complete');fprintf('NORMAL_WORKER_COMPLETE phase=%s worker=%d count=%d\n',phase,worker,numel(rows));
    function write_status(status)
        accepted=0;success=0;attempts=0;
        if ~isempty(rows)
            accepted=nnz([rows.accepted]);attempts=sum([rows.attempts]);
            if isfield(rows,'success'),success=nnz([rows.success]);end
        end
        value=struct('phase',phase,'worker',worker,'status',status,'scheduled',range(2)-range(1)+1, ...
            'completed',numel(rows),'accepted',accepted,'success',success,'attempts',attempts,'wallSeconds',toc(timer));
        normal_seed_study_io3d('json',fullfile(directory,[phase,'_status.json']),value);
    end
end
