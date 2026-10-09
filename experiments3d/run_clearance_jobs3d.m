function run_clearance_jobs3d(studyDir,jobsFile,workerIndex,nWorkers)
% RUN_CLEARANCE_JOBS3D Frozen-map, resumable experiment worker.
    oldPath=path; guard=onCleanup(@() path(oldPath)); %#ok<NASGU>
    snap=fullfile(studyDir,'source_snapshot');
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'),fullfile(snap,'experiments3d'));
    assert(startsWith(which('run_simulation3d'),snap));
    stored=load(fullfile(studyDir,'protocol.mat'),'protocol'); protocol=stored.protocol;
    jobs=readtable(jobsFile,'TextType','string');
    jobs=jobs(mod((1:height(jobs))'-1,nWorkers)+1==workerIndex,:);
    [~,stageName]=fileparts(jobsFile);
    workerDir=fullfile(studyDir,'stages',stageName,sprintf('worker_%02d',workerIndex));
    trialDir=fullfile(workerDir,'trials'); if ~isfolder(trialDir), mkdir(trialDir); end
    rows=struct([]); totalClock=tic;
    plannerPool=containers.Map('KeyType','char','ValueType','any'); lastPlanProfile='';
    for i=1:height(jobs)
        job=table2struct(jobs(i,:));
        trialFile=fullfile(trialDir,char(job.jobId+".mat"));
        if isfile(trialFile)
            existing=load(trialFile,'row'); row=existing.row;
            assert(row.jobId==job.jobId && row.configId==job.configId);
        else
            map=load(fullfile(studyDir,char(job.mapFile)),'scene','mapHash');
            scene=map.scene; mapHash=map.mapHash;
            q=protocol.p; q.seed=job.seed; q.stochasticSeed=job.stochasticSeed;
            q.pathClearance=job.pathXY; q.desiredClear=job.refXY;
            q.directionalClearance=struct('enable',logical(job.enabled), ...
                'planningZ',job.planZ,'referenceZ',job.refZ);
            q.noiseStd=job.noiseStd; q.disturbanceMax=job.disturbanceMax;
            q.disturbanceHoldSteps=job.disturbanceHoldSteps;
            if job.layerPolicy=="add_window_center"
                assert(isfield(scene,'gateCenter'));
                q.heightLayers=unique([q.heightLayers,scene.gateCenter]);
            end
            q.adp.wA=zeros(3,6); q.adp.wC=zeros(6,1);
            r=[]; errorText=''; clockTrial=tic;
            try
                profile=sprintf('%d_%g_%g_%s',job.enabled,job.pathXY,job.planZ,job.layerPolicy);
                if ~strcmp(profile,lastPlanProfile)
                    plannerPool=containers.Map('KeyType','char','ValueType','any'); lastPlanProfile=profile;
                end
                key=char(string(job.seed)); cached=[];
                if isKey(plannerPool,key) && job.stage~="timing", cached=plannerPool(key); end
                [r,newCache]=run_simulation3d(q,scene,cached);
                if ~isempty(newCache) && job.stage~="timing"
                    if plannerPool.Count>=4 && ~isKey(plannerPool,key)
                        plannerPool=containers.Map('KeyType','char','ValueType','any');
                    end
                    plannerPool(key)=newCache;
                end
            catch exception
                errorText=getReport(exception,'extended','hyperlinks','off');
            end
            wallSeconds=toc(clockTrial);
            row=clearance_trial_record3d(job,q,scene,mapHash,r,errorText,wallSeconds);
            save(trialFile,'job','q','scene','mapHash','r','row','errorText','-v7');
            if ~isempty(errorText)
                fid=fopen(fullfile(trialDir,char(job.jobId+".error.txt")),'w','n','UTF-8');
                fprintf(fid,'%s',errorText); fclose(fid);
            end
        end
        rows=[rows;row]; %#ok<AGROW>
        if mod(i,10)==0 || i==height(jobs)
            writetable(struct2table(rows),fullfile(workerDir,'trials.csv'));
            status=struct('stage',stageName,'worker',workerIndex,'scheduled',height(jobs), ...
                'completed',numel(rows),'success',nnz([rows.success]),'wallSeconds',toc(totalClock));
            fid=fopen(fullfile(workerDir,'status.json'),'w','n','UTF-8');
            fprintf(fid,'%s',jsonencode(status)); fclose(fid);
            fprintf('CLEARANCE_PROGRESS stage=%s worker=%d completed=%d/%d success=%d wall=%.1fs\n', ...
                stageName,workerIndex,i,height(jobs),nnz([rows.success]),toc(totalClock));
        end
    end
    fprintf('CLEARANCE_WORKER_COMPLETE stage=%s worker=%d count=%d\n',stageName,workerIndex,height(jobs));
end
