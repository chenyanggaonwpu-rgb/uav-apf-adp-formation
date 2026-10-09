function run_margin_grid49_aux3d(studyDir,workerIndex,seedList)
% RUN_MARGIN_GRID49_3D Complete 49 paired flights on every frozen map.
    snap=fullfile(studyDir,'source_snapshot');
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'),fullfile(snap,'experiments3d'));
    assert(startsWith(which('run_simulation3d'),snap));
    protocol=jsondecode(fileread(fullfile(studyDir,'protocol.json')));
    manifest=readtable(fullfile(studyDir,'map_manifest.csv'),'TextType','string');
    selected=sortrows(manifest(ismember(manifest.seed,seedList),:),'seed','descend');
    directory=fullfile(studyDir,'auxiliary_runs',sprintf('aux_%02d',workerIndex));
    trialDir=fullfile(directory,'pending_results');planDir=fullfile(directory,'initial_plans');
    if ~isfolder(trialDir),mkdir(trialDir);end
    if ~isfolder(planDir),mkdir(planDir);end
    rows=struct([]);timer=tic;total=height(selected)*49;write_status('running');
    for index=1:height(selected)
        m=selected(index,:);
        owner=mod(m.seed-2001,protocol.nWorkers)+1;
        ownerTrials=fullfile(studyDir,'workers',sprintf('worker_%02d',owner),'trials');
        if ~isempty(dir(fullfile(ownerTrials,sprintf('*_s%06d.mat',m.seed))))
            fprintf('AUX_SKIP_ALREADY_STARTED seed=%d\n',m.seed);continue;
        end
        mapAborted=false;
        frozen=load(fullfile(studyDir,char(m.mapFile)));
        scene=frozen.scene;mapHash=frozen.mapHash;
        for a=1:numel(protocol.pathValues)
            if ~isempty(dir(fullfile(ownerTrials,sprintf('*_s%06d.mat',m.seed))))
                mapAborted=true;break;
            end
            jobs=struct([]);
            for b=1:numel(protocol.referenceValues)
                cid=sprintf('p%02d_r%02d',a,b);
                jobs=[jobs;struct('jobId',string(sprintf('%s_s%06d',cid,m.seed)), ...
                    'stage',"margin_grid49",'configId',string(cid),'seed',m.seed, ...
                    'enabled',false,'pathXY',protocol.pathValues(a),'refXY',protocol.referenceValues(b), ...
                    'planZ',protocol.pathValues(a),'refZ',protocol.referenceValues(b), ...
                    'layerPolicy',"adaptive_midpoint_v2",'noiseStd',0,'disturbanceMax',0, ...
                    'disturbanceHoldSteps',1,'repeat',1,'stochasticSeed',m.seed)]; %#ok<AGROW>
            end
            pending=false;
            for b=1:numel(jobs)
                pending=pending || ~isfile(fullfile(trialDir,char(jobs(b).jobId+".mat")));
            end
            preparedCache=[];planSeconds=0;
            if pending && logical(m.initialHardSafe)
                q=frozen.q;q.pathClearance=protocol.pathValues(a);q=validate_params3d(q);
                clockPlan=tic;
                [route,info,preparedCache]=plan_layers3d(q.start,q.target,scene.obstacles, ...
                    q,q.pathClearance,q.pathClearance,[]);
                planSeconds=toc(clockPlan);
                % Cache is captured BEFORE any flight and is never replaced by a flight's final cache.
                planFile=fullfile(planDir,sprintf('p%02d_s%06d.mat',a,m.seed));
                save(planFile,'route','info','planSeconds','mapHash','-v7');
            end
            for b=1:numel(jobs)
                job=jobs(b);file=fullfile(trialDir,char(job.jobId+".mat"));
                if isfile(file)
                    d=load(file,'row');row=d.row;
                    assert(row.jobId==job.jobId && row.pathXY==job.pathXY && row.refXY==job.refXY);
                else
                    q=frozen.q;q.pathClearance=job.pathXY;q.desiredClear=job.refXY;
                    q.directionalClearance=struct('enable',false,'planningZ',job.pathXY,'referenceZ',job.refXY);
                    q.stochasticSeed=job.stochasticSeed;q.adp.wA=zeros(3,6);q.adp.wC=zeros(6,1);
                    r=[];errorText='';clockTrial=tic;
                    try
                        r=run_simulation3d(q,scene,preparedCache);
                    catch ex
                        errorText=getReport(ex,'extended','hyperlinks','off');
                    end
                    row=clearance_trial_record3d(job,q,scene,mapHash,r,errorText,toc(clockTrial));
                    row.pathIndex=a;row.referenceIndex=b;row.obstacleCount=m.obstacleCount;
                    row.initialHardSafe=logical(m.initialHardSafe);
                    row.leaderEndpointsPlanClear=min(m.leaderStartClearance,m.leaderTargetClearance)>=job.pathXY;
                    row.initialPlanFound=ismember(row.planStatus,["direct","path"]);
                    row.initialLayerCount=5;row.addedCount=0;row.adaptiveAttempted=false;
                    if ~isempty(r) && isfield(r.plan,'heightSelection')
                        row.initialLayerCount=numel(r.plan.heightSelection.layers);
                        row.addedCount=numel(r.plan.heightSelection.addedHeights);
                        row.adaptiveAttempted=r.plan.heightSelection.attempted;
                    end
                    row.sharedInitialPlanSeconds=planSeconds;row.initialGraphCacheUsed=~isempty(preparedCache);
                    row.inputMapSha256=string(m.mapMatSha256);
                    save(file,'job','q','scene','mapHash','r','row','errorText','-v7');
                end
                rows=[rows;row]; %#ok<AGROW>
                write_status('running');
                if ~isempty(errorText_if_new(row))
                    writetable(struct2table(rows),fullfile(directory,'trials.csv'));
                    error('UAV3D:GridExecution','Execution error in %s; raw evidence retained.',row.jobId);
                end
            end
            writetable(struct2table(rows),fullfile(directory,'trials.csv'));
            fprintf('MARGIN_GRID_PROGRESS worker=%d completed=%d/%d success=%d seed=%d path=%.2f wall=%.1fs\n', ...
                workerIndex,numel(rows),total,nnz([rows.success]),m.seed,protocol.pathValues(a),toc(timer));
        end
        if ~mapAborted
            ready=struct('seed',m.seed,'owner',owner,'auxiliary',workerIndex,'completeTasks',49);
            readyFile=fullfile(directory,sprintf('ready_%06d.json',m.seed));
            fid=fopen([readyFile '.tmp'],'w','n','UTF-8');fprintf(fid,'%s',jsonencode(ready));fclose(fid);
            movefile([readyFile '.tmp'],readyFile,'f');
        end
    end

    write_status('complete');
    fprintf('MARGIN_GRID_WORKER_COMPLETE worker=%d records=%d success=%d wall=%.1fs\n', ...
        workerIndex,numel(rows),nnz([rows.success]),toc(timer));
    function write_status(status)
        count=0;errors=0;if ~isempty(rows),count=nnz([rows.success]);errors=nnz([rows.hasExecutionError]);end
        s=struct('worker',workerIndex,'status',status,'scheduled',total,'completed',numel(rows), ...
            'success',count,'executionErrors',errors,'wallSeconds',toc(timer));
        file=fullfile(directory,'status.json');fid=fopen([file '.tmp'],'w','n','UTF-8');
        fprintf(fid,'%s',jsonencode(s));fclose(fid);movefile([file '.tmp'],file,'f');
    end
end
function value=errorText_if_new(row)
    value='';if row.hasExecutionError,value='execution_error';end
end
