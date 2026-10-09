function run_midpoint_sweep3d(studyDir,workerIndex)
% RUN_MIDPOINT_SWEEP3D Resumable complete flights on immutable original maps.
    snap=fullfile(studyDir,'source_snapshot');
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'),fullfile(snap,'experiments3d'));
    assert(startsWith(which('run_simulation3d'),snap));
    stored=load(fullfile(studyDir,'protocol.mat'),'protocol'); protocol=stored.protocol;
    manifest=readtable(fullfile(studyDir,'map_manifest.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    range=protocol.ranges(workerIndex,:);
    workerDir=fullfile(studyDir,'workers',sprintf('worker_%02d',workerIndex));
    trialDir=fullfile(workerDir,'trials'); if ~isfolder(trialDir), mkdir(trialDir); end
    rows=struct([]); total=tic; write_status('running');
    for seed=range(1):range(2)
        file=fullfile(trialDir,sprintf('adaptive_%06d.mat',seed));
        if isfile(file)
            saved=load(file,'row'); row=saved.row; assert(row.seed==seed);
        else
            map=load(fullfile(studyDir,char(manifest.mapFile(manifest.seed==seed))));
            scene=map.scene; mapHash=map.mapHash; originalQ=map.originalQ;
            q=originalQ; q.version=protocol.version; q.adaptiveHeight=protocol.p.adaptiveHeight;
            oldRow=map.oldRow; r=[]; errorText=''; timer=tic;
            row=struct('seed',seed,'family',string(scene.family),'baselineSuccess',oldRow.success, ...
                'baselineReason',string(oldRow.reason),'mapHash',string(mapHash), ...
                'success',false,'reason',"execution_error",'planStatus',"not_run", ...
                'fixedPlanStatus',"not_run",'initialPlanFound',false,'initialLayerCount',5, ...
                'peakLayerCount',5,'addedHeights',"",'addedCount',0,'adaptiveAttempted',false, ...
                'missingIntervals',0,'budgetRejected',0,'capacity',0,'availableNodes',0,'edges',0, ...
                'initialPlanSeconds',nan,'time',0,'path',nan,'verticalTravel',nan,'formationRms',nan, ...
                'minObstacleClearance',nan,'minPairDistance',nan,'cycles',0,'replanCount',0, ...
                'safetyTriggers',0,'finalLeaderError',nan,'finalFormationError',nan,'finalMaxSpeed',nan,'wallSeconds',nan);
            try
                r=run_simulation3d(q,scene);
                row.success=r.success; row.reason=string(r.reason); row.time=r.time;
                row.initialPlanSeconds=r.timing.initialPlanSeconds;
                row.replanCount=numel(r.timing.replanSeconds); row.planStatus=string(r.plan.status);
                row.initialPlanFound=ismember(row.planStatus,["direct","path"]);
                hs=r.plan.heightSelection;
                row.fixedPlanStatus=string(hs.fixedStatus); row.initialLayerCount=numel(hs.layers);
                row.addedCount=numel(hs.addedHeights); row.adaptiveAttempted=hs.attempted;
                if ~isempty(hs.addedHeights)
                    row.addedHeights=strjoin(compose('%.12g',hs.addedHeights),';');
                end
                row.missingIntervals=hs.missingIntervals; row.budgetRejected=hs.budgetRejected;
                row.capacity=r.plan.capacity; row.availableNodes=r.plan.availableNodes; row.edges=r.plan.edges;
                row.peakLayerCount=row.initialLayerCount;
                for i=1:numel(r.heightPlans)
                    row.peakLayerCount=max(row.peakLayerCount,numel(r.heightPlans{i}.info.heightLayers));
                end
                if isfield(r,'metrics')
                    row.path=r.metrics.leaderPathLength; row.verticalTravel=r.metrics.leaderVerticalTravel;
                    row.formationRms=r.metrics.formationRms;
                    row.minObstacleClearance=r.metrics.minObstacleClearance; row.minPairDistance=r.metrics.minPairDistance;
                end
                if isfield(r.history,'t') && ~isempty(r.history.t)
                    row.cycles=numel(r.history.t); row.safetyTriggers=nnz(r.history.safetyTriggered);
                    row.finalLeaderError=norm(r.state(1:3,1)-q.target);
                    row.finalFormationError=r.history.formationError(end);
                    row.finalMaxSpeed=max(vecnorm(r.state(4:6,:)));
                end
            catch ex
                row.success=false; row.reason="execution_error";
                errorText=getReport(ex,'extended','hyperlinks','off');
            end
            row.wallSeconds=toc(timer);
            save(file,'q','r','scene','row','mapHash','originalQ','oldRow','errorText','-v7');
            if ~isempty(errorText)
                fid=fopen(strrep(file,'.mat','.error.txt'),'w','n','UTF-8'); fprintf(fid,'%s',errorText); fclose(fid);
            end
        end
        rows=[rows;row]; %#ok<AGROW>
        if mod(numel(rows),5)==0 || seed==range(2)
            writetable(struct2table(rows),fullfile(workerDir,'trials.csv')); write_status('running');
        end
        if mod(numel(rows),25)==0 || row.reason=="execution_error"
            fprintf('MIDPOINT_PROGRESS worker=%d completed=%d/%d success=%d seed=%d reason=%s elapsed=%.1fs\n', ...
                workerIndex,numel(rows),range(2)-range(1)+1,nnz([rows.success]),seed,row.reason,toc(total));
        end
    end
    write_status('complete');
    fprintf('MIDPOINT_WORKER_COMPLETE worker=%d count=%d success=%d wall=%.1fs\n',workerIndex,numel(rows),nnz([rows.success]),toc(total));
    function write_status(status)
        succeeded=0; if ~isempty(rows), succeeded=nnz([rows.success]); end
        value=struct('worker',workerIndex,'status',status,'scheduled',range(2)-range(1)+1, ...
            'completed',numel(rows),'success',succeeded,'wallSeconds',toc(total));
        fid=fopen(fullfile(workerDir,'status.json'),'w','n','UTF-8'); fprintf(fid,'%s',jsonencode(value)); fclose(fid);
    end
end
