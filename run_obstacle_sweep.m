function outputDir = run_obstacle_sweep(varargin)
% RUN_OBSTACLE_SWEEP Parallel, resumable 3-factor Monte Carlo experiment.
%   run_obstacle_sweep
%   run_obstacle_sweep('ResumeDir','F:\...\results\obstacle_sweep_...')
%   See docs/OBSTACLE_SWEEP.md for experiment definitions and plotting.
    parser=inputParser;
    addParameter(parser,'ObstacleCounts',[4 8 12 16 20]);
    addParameter(parser,'BandWidths',[10 15 20 25 30]);
    addParameter(parser,'MinGaps',[1 2 3 4 5]);
    addParameter(parser,'Seeds',1:200);
    addParameter(parser,'Workers',4);
    addParameter(parser,'PoolType','Threads');
    addParameter(parser,'UseParallel',true);
    addParameter(parser,'BatchSize',100);
    addParameter(parser,'MaxNewTrials',Inf);
    addParameter(parser,'ResumeDir','');
    addParameter(parser,'MakePlots',true);
    addParameter(parser,'BaseParams',[]);
    parse(parser,varargin{:}); o=parser.Results;
    o.PoolType=validatestring(o.PoolType,{'Threads','Processes'});
    root=fileparts(mfilename('fullpath'));
    oldPath=path; oldDir=pwd;
    restore=onCleanup(@() restore_environment(oldPath,oldDir)); %#ok<NASGU>
    addpath(fullfile(root,'src'),fullfile(root,'config'),fullfile(root,'experiments'));
    validateattributes(o.Workers,{'numeric'},{'scalar','integer','positive','finite'});
    validateattributes(o.BatchSize,{'numeric'},{'scalar','integer','positive','finite'});
    validateattributes(o.MaxNewTrials,{'numeric'},{'scalar','positive'});
    assert(isinf(o.MaxNewTrials)||o.MaxNewTrials==fix(o.MaxNewTrials), ...
        'UAV:SweepOptions','MaxNewTrials must be an integer or Inf.');
    if isempty(o.ResumeDir)
        for name={'ObstacleCounts','BandWidths','MinGaps','Seeds'}
            x=o.(name{1});
            validateattributes(x,{'numeric'},{'vector','nonempty','real','finite','nonnegative'});
            assert(numel(unique(x))==numel(x),'UAV:SweepOptions','Duplicate levels.');
        end
        assert(all(o.ObstacleCounts==fix(o.ObstacleCounts)) && ...
            all(o.Seeds==fix(o.Seeds)) && all(o.Seeds<=2^32-1), ...
            'UAV:SweepOptions','Counts/seeds must be valid integers.');
        baseParams=o.BaseParams;
        if isempty(baseParams), baseParams=default_params(); end
        baseParams=validate_params(baseParams);
        baseParams.animate=false; baseParams.debug=false;
        assert(baseParams.vmax==3,'UAV:SweepSpeed', ...
            'This experiment is specified at 3 m/s; check the base parameters.');
        outputDir=create_run_output_dir(baseParams.resultsRoot,'obstacle_sweep',baseParams.version);
        mkdir(fullfile(outputDir,'chunks'));
        snapshot=fullfile(outputDir,'source_snapshot'); mkdir(snapshot);
        copyfile(fullfile(root,'src'),fullfile(snapshot,'src'));
        copyfile(fullfile(root,'config'),fullfile(snapshot,'config'));
        copyfile(fullfile(root,'experiments'),fullfile(snapshot,'experiments'));
        copyfile(fullfile(root,'run_obstacle_sweep.m'),fullfile(snapshot,'run_obstacle_sweep.m'));
        [n,b,g]=ndgrid(o.ObstacleCounts,o.BandWidths,o.MinGaps);
        combinations=table((1:numel(n)).',n(:),b(:),g(:),'VariableNames', ...
            {'combinationId','nObstacles','corridorBand','marginObs'});
        % Seed-major order balances early coverage across all parameter cells.
        [ci,ss]=ndgrid(1:height(combinations),o.Seeds);
        jobs=combinations(ci(:),:);
        jobs.seed=ss(:); jobs.jobId=(1:height(jobs)).';
        cfg=struct('schemaVersion',1,'createdAt',char(datetime('now')), ...
            'matlabVersion',version,'baseParams',baseParams, ...
            'seeds',o.Seeds,'combinations',combinations,'jobs',jobs, ...
            'sourceHashes',obstacle_sweep_manifest(snapshot), ...
            'generator','twister','note',sprintf('%d attempted seeds per combination; generator feasibility filtering is retained.',numel(o.Seeds)));
        save(fullfile(outputDir,'experiment.mat'),'cfg','-v7');
        writetable(cfg.sourceHashes,fullfile(outputDir,'source_sha256.csv'));
        writetable(jobs,fullfile(outputDir,'planned_jobs.csv'));
    else
        outputDir=char(o.ResumeDir);
        loaded=load(fullfile(outputDir,'experiment.mat'),'cfg'); cfg=loaded.cfg;
        snapshot=fullfile(outputDir,'source_snapshot');
        assert(isequal(cfg.sourceHashes,obstacle_sweep_manifest(snapshot)), ...
            'UAV:SweepSnapshot','Frozen experiment source has changed.');
        assert(strcmp(cfg.matlabVersion,version),'UAV:SweepVersion','Resume with the same MATLAB version.');
        original={cfg.combinations.nObstacles,cfg.combinations.corridorBand, ...
            cfg.combinations.marginObs,cfg.seeds};
        names={'ObstacleCounts','BandWidths','MinGaps','Seeds'};
        for k=1:numel(names)
            if ~ismember(names{k},parser.UsingDefaults)
                assert(isequal(sort(unique(o.(names{k})(:))),sort(unique(original{k}(:)))), ...
                    'UAV:SweepResumeOptions','Cannot change experiment levels on resume.');
            end
        end
        assert(isempty(o.BaseParams),'UAV:SweepResumeOptions','Resume uses saved base parameters.');
    end
    outputDir=char(java.io.File(outputDir).getCanonicalPath());
    snapshot=fullfile(outputDir,'source_snapshot');
    % A lock prevents two processes from writing the same experiment.
    lockFile=java.io.File(fullfile(outputDir,'RUNNING.lock'));
    assert(lockFile.createNewFile(),'UAV:SweepLocked', ...
        'RUNNING.lock exists. If the previous MATLAB process crashed, remove this lock after checking it has stopped.');
    lockCleanup=onCleanup(@() lockFile.delete()); %#ok<NASGU>
    fid=fopen(fullfile(outputDir,'RUNNING.lock'),'w');
    fprintf(fid,'MATLAB PID: %d\nStarted: %s\n',feature('getpid'),char(datetime('now'))); fclose(fid);
    addpath(fullfile(snapshot,'src'),fullfile(snapshot,'config'),fullfile(snapshot,'experiments'),'-begin');
    cd(snapshot);
    assert(startsWith(which('run_simulation'),snapshot),'UAV:SweepPath','Wrong source path.');
    files=dir(fullfile(outputDir,'chunks','chunk_*.mat'));
    rows=table();
    for k=1:numel(files)
        loaded=load(fullfile(files(k).folder,files(k).name),'chunk');
        rows=[rows;loaded.chunk]; %#ok<AGROW>
    end
    if ~isempty(rows)
        assert(numel(unique(rows.jobId))==height(rows),'UAV:SweepDuplicate','Duplicate jobs in checkpoint.');
        assert(all(ismember(rows.jobId,cfg.jobs.jobId)),'UAV:SweepCheckpoint','Unexpected checkpoint IDs.');
        fields={'jobId','combinationId','seed','nObstacles','corridorBand','marginObs'};
        assert(isequal(rows(:,fields),cfg.jobs(rows.jobId,fields)), ...
            'UAV:SweepCheckpoint','Checkpoint does not match the experiment design.');
    end
    remaining=cfg.jobs;
    if ~isempty(rows), remaining=remaining(~ismember(remaining.jobId,rows.jobId),:); end
    count=min(height(remaining),o.MaxNewTrials);
    remaining=remaining(1:count,:);
    fprintf('OUTPUT_DIR=%s\n',outputDir);
    fprintf('SWEEP %d combinations, %d planned trials; %d cached; %d new; vmax=%.1f\n', ...
        height(cfg.combinations),height(cfg.jobs),height(rows),height(remaining),cfg.baseParams.vmax);
    runTimer=tic; previousCount=height(rows); usedWorkers=1;
    if o.UseParallel && ~isempty(remaining)
        assert(license('test','Distrib_Computing_Toolbox'),'UAV:SweepParallel','Parallel toolbox is unavailable.');
        pool=gcp('nocreate');
        if isempty(pool)
            if strcmp(o.PoolType,'Threads')
                pool=parpool('Threads',o.Workers);
            else
                cluster=parcluster('Processes');
                cluster.NumThreads=1;
                pool=parpool(cluster,o.Workers);
            end
            poolCleanup=onCleanup(@() delete(pool)); %#ok<NASGU>
        else
            expectedClass='parallel.ThreadPool';
            if strcmp(o.PoolType,'Processes'), expectedClass='parallel.ProcessPool'; end
            assert(isa(pool,expectedClass)&&pool.NumWorkers==o.Workers, ...
                'UAV:SweepPool','Use a pool with the requested type and number of workers.');
        end
        usedWorkers=pool.NumWorkers;
        if strcmp(o.PoolType,'Processes')
            f=parfevalOnAll(pool,@worker_setup,0,snapshot);
            wait(f);
            for k=1:numel(f), if ~isempty(f(k).Error), throw(f(k).Error); end, end
        end
    end
    for first=1:o.BatchSize:height(remaining)
        batch=remaining(first:min(first+o.BatchSize-1,height(remaining)),:);
        result=cell(height(batch),1); baseParams=cfg.baseParams;
        if o.UseParallel
            parfor k=1:height(batch)
                result{k}=obstacle_sweep_trial(batch(k,:),baseParams);
            end
        else
            for k=1:height(batch), result{k}=obstacle_sweep_trial(batch(k,:),baseParams); end
        end
        chunk=vertcat(result{:});
        dest=fullfile(outputDir,'chunks',sprintf('chunk_%08d_%08d.mat',min(chunk.jobId),max(chunk.jobId)));
        assert(~isfile(dest),'UAV:SweepOverwrite','Checkpoint already exists.');
        tmp=[dest '.tmp']; save(tmp,'chunk','-v7'); movefile(tmp,dest);
        rows=[rows;chunk]; %#ok<AGROW>
        update_outputs(outputDir,rows,cfg,runTimer,previousCount,usedWorkers);
        if any(chunk.outcome=="execution_error")
            error('UAV:SweepExecutionError','Unexpected errors recorded. Inspect trials.csv before continuing.');
        end
        if isfile(fullfile(outputDir,'STOP_REQUESTED.txt'))
            fprintf('STOP_REQUESTED: stopped after saving the current batch.\n'); break;
        end
    end
    if ~isempty(rows)
        update_outputs(outputDir,rows,cfg,runTimer,previousCount,usedWorkers);
        write_obstacle_sweep_readout(outputDir);
        if o.MakePlots, plot_obstacle_sweep(outputDir); end
    end
    fprintf('SWEEP_RETURNED=%d/%d\n',height(rows),height(cfg.jobs));
end

function worker_setup(snapshot)
    cd(snapshot);
    addpath(fullfile(snapshot,'src'),fullfile(snapshot,'config'),fullfile(snapshot,'experiments'),'-begin');
    maxNumCompThreads(1);
    assert(startsWith(which('run_simulation'),snapshot),'UAV:SweepPath','Worker source mismatch.');
end

function update_outputs(outputDir,rows,cfg,timer,previousCount,workers)
    rows=sortrows(rows,'jobId');
    summary=obstacle_sweep_summary(rows,cfg.combinations,cfg.seeds);
    writetable(rows,fullfile(outputDir,'trials.csv'));
    writetable(summary,fullfile(outputDir,'summary.csv'));
    done=height(rows); total=height(cfg.jobs); elapsed=toc(timer);
    throughput=(done-previousCount)/max(elapsed,eps);
    remaining_s=(total-done)/max(throughput,eps);
    progress=struct('updatedAt',char(datetime('now')),'completed',done,'planned',total, ...
        'fraction',done/total,'workers',workers,'sessionElapsed_s',elapsed, ...
        'trialsPerSecond',throughput,'estimatedRemaining_s',remaining_s, ...
        'successes',sum(rows.outcome=="success"),'collisions',sum(rows.outcome=="collision"), ...
        'timeouts',sum(rows.outcome=="timeout"), ...
        'generationFailures',sum(rows.outcome=="generation_failed"), ...
        'executionErrors',sum(rows.outcome=="execution_error"),'complete',done==total);
    fid=fopen(fullfile(outputDir,'progress.json'),'w','n','UTF-8');
    assert(fid>=0,'UAV:SweepWrite','Cannot write progress.');
    fprintf(fid,'%s\n',jsonencode(progress,PrettyPrint=true)); fclose(fid);
    fprintf('%s DONE %d/%d (%.1f%%), %.2f trial/s, ETA %.1f min; S=%d C=%d T=%d G=%d E=%d\n', ...
        char(datetime('now','Format','HH:mm:ss')),done,total,100*done/total,throughput,remaining_s/60, ...
        progress.successes,progress.collisions,progress.timeouts,progress.generationFailures,progress.executionErrors);
end

function restore_environment(oldPath,oldDir)
    cd(oldDir); path(oldPath);
end
