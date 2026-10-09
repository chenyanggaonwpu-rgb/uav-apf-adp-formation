function workerDir=run_seed_sweep3d(studyDir,workerIndex)
% RUN_SEED_SWEEP3D Execute one frozen seed range with one MAT per seed.
% Resuming a range reuses completed MAT rows; no outcome-based seed filtering.
    data=load(fullfile(studyDir,'protocol.mat'),'protocol');
    protocol=data.protocol;
    assert(isequal(protocol.p.heightLayers,[5 10 15 20 25]), ...
        'UAV3D:FixedLayers','Do not insert or adapt height layers in this sweep.');
    validateattributes(workerIndex,{'numeric'}, ...
        {'scalar','integer','>=',1,'<=',protocol.nWorkers});
    range=protocol.ranges(workerIndex,:);
    workerDir=fullfile(studyDir,'workers',sprintf('worker_%02d',workerIndex));
    if ~isfolder(workerDir), mkdir(workerDir); end
    trialDir=fullfile(workerDir,'trials');
    if ~isfolder(trialDir), mkdir(trialDir); end
    oldPath=path; cleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    snap=fullfile(studyDir,'source_snapshot');
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'));
    expected=fullfile(snap,'src3d','run_simulation3d.m');
    if ~strcmpi(which('run_simulation3d'),expected)
        error('UAV3D:SnapshotPath','Simulation must use frozen source.');
    end
    started=tic;
    rows=struct([]);
    reportStatus('running');
    for seed=range(1):range(2)
        trialId=sprintf('C_random_%06d',seed);
        trialFile=fullfile(trialDir,[trialId '.mat']);
        if isfile(trialFile)
            stored=load(trialFile,'row');
            row=stored.row;
        else
            q=protocol.p;
            q.seed=seed; q.sceneId='random'; q.noiseStd=0; q.disturbanceMax=0;
            q.adp.enable=true; q.safety.enable=true;
            q.adp.wA=zeros(3,6); q.adp.wC=zeros(6,1);
            q.animate=false;
            r=[]; scene=[]; errorText='';
            row=emptyRow(seed,trialId);
            row.layerCount=numel(q.heightLayers);
            if isfield(q,'verticalCostWeight'), row.verticalCostWeight=q.verticalCostWeight; end
            trialClock=tic;
            try
                scene=scenario3d('random',seed,q);
                if isfield(scene,'family'), row.family=string(scene.family); end
                row.generatedObstacles=numel(scene.obstacles);
                row.generationOK=~strcmp(scene.kind,'generation_failed');
                r=run_simulation3d(q,scene);
                row.reason=string(r.reason); row.success=r.success;
                row.time=r.time;
                row.planStatus=string(r.plan.status);
                row.initialPlanSeconds=r.timing.initialPlanSeconds;
                for name={'availableNodes','edges'}
                    if isfield(r.plan,name{1}), row.(name{1})=r.plan.(name{1}); end
                end
                row.replanCount=numel(r.timing.replanSeconds);
                if isfield(r,'metrics')
                    row.path=r.metrics.leaderPathLength;
                    row.formationRms=r.metrics.formationRms;
                    row.minObstacleClearance=r.metrics.minObstacleClearance;
                    row.minPairDistance=r.metrics.minPairDistance;
                    row.heightSpan=r.metrics.leaderHeightSpan;
                    row.verticalTravel=r.metrics.leaderVerticalTravel;
                end
                if isfield(r.history,'t') && ~isempty(r.history.t)
                    row.cycles=numel(r.history.t);
                    row.safetyTriggers=nnz(r.history.safetyTriggered);
                    row.finalLeaderError=norm(r.state(1:3,1)-q.target);
                    row.finalFormationError=r.history.formationError(end);
                    row.finalMaxSpeed=max(vecnorm(r.state(4:6,:)));
                end
            catch exception
                row.reason="execution_error";
                errorText=getReport(exception,'extended','hyperlinks','off');
            end
            row.wallSeconds=toc(trialClock);
            save(trialFile,'row','r','scene','q','errorText','-v7.3');
            if ~isempty(errorText)
                fid=fopen(fullfile(trialDir,[trialId '.error.txt']),'w');
                fprintf(fid,'%s',errorText); fclose(fid);
            end
        end
        rows=[rows;row]; %#ok<AGROW>
        writetable(struct2table(rows),fullfile(workerDir,'trials.csv'));
        reportStatus('running');
        if ~row.success || mod(numel(rows),25)==0 || seed==range(1)
            fprintf('worker=%d seed=%d reason=%s completed=%d/%d wall=%.1f s\n', ...
                workerIndex,seed,row.reason,numel(rows),range(2)-range(1)+1, ...
                toc(started));
        end
    end
    reportStatus('complete');
    fprintf('WORKER_COMPLETE=%d count=%d success=%d wall=%.1f s\n', ...
        workerIndex,numel(rows),nnz([rows.success]),toc(started));

    function reportStatus(status)
        succeeded=0;
        if ~isempty(rows), succeeded=nnz([rows.success]); end
        summary=struct('worker',workerIndex,'status',status,'range',range, ...
            'completed',numel(rows),'scheduled',range(2)-range(1)+1, ...
            'success',succeeded,'wallSeconds',toc(started), ...
            'updatedLocal',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
        fid=fopen(fullfile(workerDir,'status.json'),'w','n','UTF-8');
        fprintf(fid,'%s',jsonencode(summary)); fclose(fid);
    end
end

function row=emptyRow(seed,trialId)
    row=struct('trialId',string(trialId),'seed',seed,'group',"C", ...
        'generationOK',false,'generatedObstacles',0, ...
        'family',"",'layerCount',0,'verticalCostWeight',0, ...
        'reason',"execution_error",'success',false,'planStatus',"not_run", ...
        'time',nan,'path',nan,'formationRms',nan, ...
        'minObstacleClearance',nan,'minPairDistance',nan, ...
        'heightSpan',nan,'verticalTravel',nan, ...
        'initialPlanSeconds',nan,'availableNodes',nan,'edges',nan, ...
        'replanCount',nan,'cycles',0,'safetyTriggers',0, ...
        'finalLeaderError',nan,'finalFormationError',nan, ...
        'finalMaxSpeed',nan,'wallSeconds',nan);
end
