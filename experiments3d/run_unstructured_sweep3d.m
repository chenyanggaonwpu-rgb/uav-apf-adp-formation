function run_unstructured_sweep3d(studyDir,workerIndex)
% RUN_UNSTRUCTURED_SWEEP3D Full flights, retaining every unconditioned map.
    snap=fullfile(studyDir,'source_snapshot');
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'),fullfile(snap,'experiments3d'));
    assert(startsWith(which('run_simulation3d'),snap));
    saved=load(fullfile(studyDir,'protocol.mat'),'protocol'); protocol=saved.protocol;
    manifest=readtable(fullfile(studyDir,'map_manifest.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    range=protocol.ranges(workerIndex,:); directory=fullfile(studyDir,'workers',sprintf('worker_%02d',workerIndex));
    trials=fullfile(directory,'trials');if ~isfolder(trials),mkdir(trials);end
    rows=struct([]); timer=tic; write_status('running');
    for seed=range(1):range(2)
        file=fullfile(trials,sprintf('random_%06d.mat',seed));
        if isfile(file)
            stored=load(file,'row');row=stored.row;assert(row.seed==seed);
        else
            m=manifest(manifest.seed==seed,:); assert(height(m)==1);
            frozen=load(fullfile(studyDir,char(m.mapFile)));q=frozen.q;scene=frozen.scene;mapHash=frozen.mapHash;
            r=[];errorText='';trialClock=tic;
            row=struct('seed',seed,'obstacleCount',m.obstacleCount,'sphereCount',m.sphereCount, ...
                'cylinderCount',m.cylinderCount,'boxCount',m.boxCount,'nominalVolumeFraction',m.nominalVolumeFraction, ...
                'initialHardSafe',logical(m.initialHardSafe),'leaderEndpointsPlanClear',logical(m.leaderEndpointsPlanClear), ...
                'leaderTargetHardSafe',logical(m.leaderTargetHardSafe),'nominalTerminalHardSafe',logical(m.nominalTerminalHardSafe), ...
                'mapHash',string(mapHash),'success',false,'reason',"execution_error",'planStatus',"not_run", ...
                'fixedPlanStatus',"not_run",'initialPlanFound',false,'initialLayerCount',5,'peakLayerCount',5, ...
                'addedHeights',"",'addedCount',0,'adaptiveAttempted',false,'missingIntervals',0,'budgetRejected',0, ...
                'capacity',0,'availableNodes',0,'edges',0,'initialPlanSeconds',nan,'time',0,'path',nan, ...
                'verticalTravel',nan,'formationRms',nan,'minObstacleClearance',nan,'minPairDistance',nan, ...
                'cycles',0,'replanCount',0,'safetyTriggers',0,'finalLeaderError',nan, ...
                'finalFormationError',nan,'finalMaxSpeed',nan,'wallSeconds',nan);
            try
                r=run_simulation3d(q,scene);
                row.success=r.success;row.reason=string(r.reason);row.time=r.time;
                row.initialPlanSeconds=r.timing.initialPlanSeconds;row.replanCount=numel(r.timing.replanSeconds);
                row.planStatus=string(r.plan.status);row.initialPlanFound=ismember(row.planStatus,["direct","path"]);
                if isfield(r.plan,'heightSelection')
                    hs=r.plan.heightSelection;row.fixedPlanStatus=string(hs.fixedStatus);
                    row.initialLayerCount=numel(hs.layers);row.peakLayerCount=row.initialLayerCount;
                    row.addedCount=numel(hs.addedHeights);row.adaptiveAttempted=hs.attempted;
                    if ~isempty(hs.addedHeights),row.addedHeights=strjoin(compose('%.12g',hs.addedHeights),';');end
                    row.missingIntervals=hs.missingIntervals;row.budgetRejected=hs.budgetRejected;
                    row.capacity=r.plan.capacity;row.availableNodes=r.plan.availableNodes;row.edges=r.plan.edges;
                    for k=1:numel(r.heightPlans)
                        row.peakLayerCount=max(row.peakLayerCount,numel(r.heightPlans{k}.info.heightLayers));
                    end
                end
                if isfield(r,'metrics')
                    row.path=r.metrics.leaderPathLength;row.verticalTravel=r.metrics.leaderVerticalTravel;
                    row.formationRms=r.metrics.formationRms;row.minObstacleClearance=r.metrics.minObstacleClearance;
                    row.minPairDistance=r.metrics.minPairDistance;
                end
                if isfield(r.history,'t') && ~isempty(r.history.t)
                    row.cycles=numel(r.history.t);row.safetyTriggers=nnz(r.history.safetyTriggered);
                    row.finalLeaderError=norm(r.state(1:3,1)-q.target);row.finalFormationError=r.history.formationError(end);
                    row.finalMaxSpeed=max(vecnorm(r.state(4:6,:)));
                end
            catch ex
                row.success=false;row.reason="execution_error";errorText=getReport(ex,'extended','hyperlinks','off');
            end
            row.wallSeconds=toc(trialClock);
            save(file,'q','r','scene','row','mapHash','errorText','-v7');
            if ~isempty(errorText)
                fid=fopen(strrep(file,'.mat','.error.txt'),'w','n','UTF-8');fprintf(fid,'%s',errorText);fclose(fid);
            end
        end
        rows=[rows;row]; %#ok<AGROW>
        % Small enough to checkpoint the summary after every map.
        writetable(struct2table(rows),fullfile(directory,'trials.csv'));write_status('running');
        if mod(numel(rows),5)==0 || row.reason=="execution_error"
            fprintf('UNSTRUCTURED_PROGRESS worker=%d completed=%d/%d success=%d seed=%d reason=%s wall=%.1fs\n', ...
                workerIndex,numel(rows),range(2)-range(1)+1,nnz([rows.success]),seed,row.reason,toc(timer));
        end
    end
    write_status('complete');fprintf('UNSTRUCTURED_WORKER_COMPLETE worker=%d count=%d success=%d\n',workerIndex,numel(rows),nnz([rows.success]));
    function write_status(status)
        succeeded=0;errors=0;if ~isempty(rows),succeeded=nnz([rows.success]);errors=nnz([rows.reason]=="execution_error");end
        value=struct('worker',workerIndex,'status',status,'scheduled',range(2)-range(1)+1, ...
            'completed',numel(rows),'success',succeeded,'executionErrors',errors,'wallSeconds',toc(timer));
        fid=fopen(fullfile(directory,'status.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
    end
end
