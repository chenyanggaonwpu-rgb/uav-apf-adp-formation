function outputDir=run_study3d(mode)
% RUN_STUDY3D Frozen-map A/B/C and separate single-factor perturbation pilot.
% 'pilot': six fixed maps + random seeds 1:6 for ablation, three fixed maps
% for each noise and acceleration level. 'full' uses random seeds 1:150.
    if nargin<1, mode='pilot'; end
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path;
    cleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'));
    p=default_params3d();
    fixedIds={'S1_sphere','S2_cylinder','S3_overpass', ...
        'S4_space_s','S5_choice','S6_terminal'};
    if strcmp(mode,'pilot')
        randomSeeds=1:6;
    elseif strcmp(mode,'full')
        randomSeeds=1:150;
    else
        error('UAV3D:StudyMode','Use pilot or full.');
    end
    noiseLevels=[0 0.02 0.05 0.10];
    disturbanceLevels=[0 0.05 0.10 0.20];
    perturbIds={'S1_sphere','S3_overpass','S6_terminal'};
    protocol=struct('mode',mode,'fixedIds',{fixedIds}, ...
        'randomSeeds',randomSeeds,'noiseLevels',noiseLevels, ...
        'disturbanceLevels',disturbanceLevels,'perturbIds',{perturbIds}, ...
        'groups',{{'A','B','C'}},'dt',p.dt,'Tmax',p.Tmax, ...
        'mapRule','Fixed IDs and random seeds specified before results; no filtering by outcomes.', ...
        'noiseModel','Independent Gaussian position noise per axis and cycle; velocity feedback exact.', ...
        'disturbanceModel','Uniform random direction and uniform magnitude up to bound; held one cycle.', ...
        'learningRule','Shared actor/critic reset to zero on every map and group trial.');
    outputDir=create_run_output_dir(p.resultsRoot,'3d_study',p.version);
    mkdir(fullfile(outputDir,'trials'));
    mkdir(fullfile(outputDir,'source_snapshot'));
    mkdir(fullfile(outputDir,'source_snapshot','src3d'));
    mkdir(fullfile(outputDir,'source_snapshot','config'));
    mkdir(fullfile(outputDir,'source_snapshot','src'));
    copyfile(fullfile(root,'src3d','*.m'), ...
        fullfile(outputDir,'source_snapshot','src3d'));
    copyfile(fullfile(root,'config','*.m'), ...
        fullfile(outputDir,'source_snapshot','config'));
    copyfile(fullfile(root,'src','create_run_output_dir.m'), ...
        fullfile(outputDir,'source_snapshot','src'));
    copyfile(fullfile(root,'experiments3d','run_study3d.m'), ...
        fullfile(outputDir,'source_snapshot'));
    save(fullfile(outputDir,'protocol.mat'),'protocol','p');
    rows=struct([]);
    fixedScenes=cell(size(fixedIds));
    for i=1:numel(fixedIds)
        fixedScenes{i}=scenario3d(fixedIds{i},p.seed,p);
    end
    randomScenes=cell(size(randomSeeds));
    for i=1:numel(randomSeeds)
        randomScenes{i}=scenario3d('random',randomSeeds(i),p);
    end
    save(fullfile(outputDir,'frozen_maps.mat'),'fixedScenes','randomScenes');
    for i=1:numel(fixedScenes)
        for g={'A','B','C'}
            execute(fixedScenes{i},g{1},0,0,sprintf('%s_%s',g{1},fixedIds{i}));
        end
    end
    for i=1:numel(randomScenes)
        for g={'A','B','C'}
            execute(randomScenes{i},g{1},0,0, ...
                sprintf('%s_random_%03d',g{1},randomSeeds(i)));
        end
    end
    for i=1:numel(perturbIds)
        s=fixedScenes{find(strcmp(fixedIds,perturbIds{i}),1)};
        for level=noiseLevels
            execute(s,'C',level,0, ...
                sprintf('noise_%s_%03d',s.id,round(1000*level)));
        end
        for level=disturbanceLevels(2:end)
            execute(s,'C',0,level, ...
                sprintf('disturbance_%s_%03d',s.id,round(1000*level)));
        end
    end
    writetable(struct2table(rows),fullfile(outputDir,'trials.csv'));
    fprintf('3-D study complete: %d scheduled trials in %s\n',numel(rows),outputDir);

    function execute(scene,group,noiseStd,disturbanceMax,trialId)
        q=default_params3d();
        q.seed=scene.seed;
        q.noiseStd=noiseStd;
        q.disturbanceMax=disturbanceMax;
        q.adp.enable=~strcmp(group,'A');
        q.safety.enable=strcmp(group,'C');
        reason='execution_error';
        r=[];
        errorText='';
        try
            r=run_simulation3d(q,scene);
            reason=r.reason;
            save(fullfile(outputDir,'trials',[trialId '.mat']),'r','-v7.3');
        catch exception
            errorText=getReport(exception,'extended','hyperlinks','off');
            fid=fopen(fullfile(outputDir,'trials',[trialId '.error.txt']),'w');
            fprintf(fid,'%s',errorText);fclose(fid);
        end
        row=struct('trialId',string(trialId),'mapId',string(scene.id), ...
            'mapKind',string(scene.kind),'seed',scene.seed, ...
            'group',string(group),'noiseStd',noiseStd, ...
            'disturbanceMax',disturbanceMax,'reason',string(reason), ...
            'success',strcmp(reason,'success'),'time',nan,'path',nan, ...
            'formationRms',nan,'minObstacleClearance',nan, ...
            'minPairDistance',nan,'heightSpan',nan,'verticalTravel',nan, ...
            'initialPlanSeconds',nan,'replanCount',nan, ...
            'maxReplanSeconds',nan,'terminalPlanCount',nan, ...
            'maxTerminalPlanSeconds',nan,'cycleMeanMs',nan, ...
            'cycleP95Ms',nan,'cycleP99Ms',nan,'cycleMaxMs',nan, ...
            'over50Fraction',nan,'safetyTriggers',nan);
        if ~isempty(r)
            row.time=r.time;
            row.initialPlanSeconds=r.timing.initialPlanSeconds;
            row.replanCount=numel(r.timing.replanSeconds);
            row.terminalPlanCount=numel(r.timing.terminalPlanSeconds);
            if ~isempty(r.timing.replanSeconds)
                row.maxReplanSeconds=max(r.timing.replanSeconds);
            end
            if ~isempty(r.timing.terminalPlanSeconds)
                row.maxTerminalPlanSeconds=max(r.timing.terminalPlanSeconds);
            end
            if isfield(r,'metrics')
                row.path=r.metrics.leaderPathLength;
                row.formationRms=r.metrics.formationRms;
                row.minObstacleClearance=r.metrics.minObstacleClearance;
                row.minPairDistance=r.metrics.minPairDistance;
                row.heightSpan=r.metrics.leaderHeightSpan;
                row.verticalTravel=r.metrics.leaderVerticalTravel;
            end
            if isfield(r.history,'cycleMs') && ~isempty(r.history.cycleMs)
                cycles=r.history.cycleMs;
                row.cycleMeanMs=mean(cycles);
                row.cycleP95Ms=prctile(cycles,95);
                row.cycleP99Ms=prctile(cycles,99);
                row.cycleMaxMs=max(cycles);
                row.over50Fraction=mean(cycles>50);
                row.safetyTriggers=nnz(r.history.safetyTriggered);
            end
        end
        rows=[rows;row]; %#ok<AGROW>
        writetable(struct2table(rows),fullfile(outputDir,'trials.csv'));
        fprintf('%s: %s, time=%.2f s, p99=%.2f ms\n',trialId,reason, ...
            row.time,row.cycleP99Ms);
    end
end
