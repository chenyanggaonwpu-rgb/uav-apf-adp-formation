function outputDir=run_environment_validation3d(firstSeed,lastSeed)
% RUN_ENVIRONMENT_VALIDATION3D Predeclared 2.1.0 geometry/flight pilot.
% Default: five fixed maps, three contrasts and all random seeds 1:16.
% Failures are retained. This small challenge pilot is not a general success rate.
    if nargin<1, firstSeed=1; end
    if nargin<2, lastSeed=16; end
    validateattributes(firstSeed,{'numeric'},{'scalar','integer','positive'});
    validateattributes(lastSeed,{'numeric'},{'scalar','integer','>=',firstSeed});
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path; cleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'));
    p=default_params3d(); p.noiseStd=0; p.disturbanceMax=0;
    outputDir=create_run_output_dir(fullfile(root,'results'),'environment3d',p.version);
    snap=fullfile(outputDir,'source_snapshot'); mkdir(snap);
    for sub={'config','src3d','tests'}
        mkdir(fullfile(snap,sub{1}));
        copyfile(fullfile(root,sub{1},'*.m'),fullfile(snap,sub{1}));
    end
    copyfile(mfilename('fullpath')+".m",snap);
    files=dir(fullfile(snap,'**','*.m'));
    sourceFiles=strings(numel(files),1); sha256=strings(numel(files),1);
    for i=1:numel(files)
        file=fullfile(files(i).folder,files(i).name);
        sourceFiles(i)=extractAfter(string(file),strlength(string(snap))+1);
        sha256(i)=file_hash(file);
    end
    writetable(table(sourceFiles,sha256),fullfile(outputDir,'source_sha256.csv'));
    fixedIds={'S7_underpass','S8_window','S9_altitude_choice', ...
        'S10_layer_gap','S11_blocked'};
    protocol=struct('version',p.version,'fixedIds',{fixedIds}, ...
        'randomSeeds',firstSeed:lastSeed,'heightLayers',p.heightLayers, ...
        'verticalCostWeight',p.verticalCostWeight,'p',p, ...
        'contrasts',{{'S9 weight 0','S9 weight 2','S10 add layer 13 m'}}, ...
        'randomFamilyRule','seed modulo four: mixed, underpass, window, staggered windows', ...
        'denominator','All predeclared seeds including generation/planning/control failures.', ...
        'mapFilter','Only endpoint reservation and bounded obstacle separation; no reachability or flight outcome filtering.', ...
        'energyMeaning','Geometric ascent/descent and squared acceleration effort are proxies, not electrical energy.', ...
        'createdLocal',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
    save(fullfile(outputDir,'protocol.mat'),'protocol');
    write_json(fullfile(outputDir,'protocol.json'),protocol);
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'));
    assert(startsWith(which('run_simulation3d'),snap));
    mkdir(fullfile(outputDir,'trials')); rows=struct([]);
    fprintf('ENVIRONMENT_STUDY_DIR=%s\n',outputDir);
    for i=1:numel(fixedIds)
        execute(fixedIds{i},p.seed,p.verticalCostWeight,p.heightLayers, ...
            ['C_' fixedIds{i}]);
    end
    execute('S9_altitude_choice',p.seed,0,p.heightLayers,'S9_weight_0');
    execute('S9_altitude_choice',p.seed,2,p.heightLayers,'S9_weight_2');
    execute('S10_layer_gap',p.seed,p.verticalCostWeight, ...
        sort([p.heightLayers 13]),'S10_added_13m');
    for seed=firstSeed:lastSeed
        execute('random',seed,p.verticalCostWeight,p.heightLayers, ...
            sprintf('C_random_%06d',seed));
    end
    assert(numel(rows)==8+lastSeed-firstSeed+1);
    write_json(fullfile(outputDir,'status.json'),struct('complete',true, ...
        'scheduled',numel(rows),'completed',numel(rows), ...
        'randomSeeds',firstSeed:lastSeed));
    fprintf('Environment pilot complete: %d/%d scheduled trials, no outcome filtering.\n', ...
        numel(rows),numel(rows));

    function execute(id,seed,weight,layers,trialId)
        q=p; q.seed=seed; q.sceneId=id; q.verticalCostWeight=weight;
        q.heightLayers=layers; q.adp.wA=zeros(3,6); q.adp.wC=zeros(6,1);
        q.adp.enable=true; q.safety.enable=true; q.animate=false;
        r=[]; scene=[]; errorText=''; reason='execution_error';
        trialClock=tic;
        try
            scene=scenario3d(id,seed,q);
            r=run_simulation3d(q,scene); reason=r.reason;
        catch exception
            errorText=getReport(exception,'extended','hyperlinks','off');
        end
        wallSeconds=toc(trialClock);
        save(fullfile(outputDir,'trials',[trialId '.mat']), ...
            'r','q','scene','errorText','-v7.3');
        family=''; kind='';
        if ~isempty(scene), family=scene.family; kind=scene.kind; end
        row=struct('trialId',string(trialId),'sceneId',string(id), ...
            'mapKind',string(kind),'family',string(family),'seed',seed, ...
            'verticalCostWeight',weight,'layerCount',numel(layers), ...
            'reason',string(reason),'success',strcmp(reason,'success'), ...
            'wallSeconds',wallSeconds,'time',nan,'planStatus',"not_run", ...
            'planLength',nan,'planVerticalTravel',nan,'planCost',nan, ...
            'planCapacity',nan,'leaderPath',nan,'leaderAscent',nan, ...
            'leaderDescent',nan,'leaderVerticalTravel',nan,'fleetAscent',nan, ...
            'fleetDescent',nan,'fleetPath',nan,'commandEffort',nan, ...
            'formationRms',nan,'minObstacleClearance',nan,'minPairDistance',nan, ...
            'initialPlanSeconds',nan,'replans',nan,'safetyTriggers',nan, ...
            'cycleP99Ms',nan,'cycleMaxMs',nan);
        if ~isempty(r)
            row.time=r.time; row.planStatus=string(r.plan.status);
            row.initialPlanSeconds=r.timing.initialPlanSeconds;
            row.replans=numel(r.timing.replanSeconds);
            if isfield(r.plan,'pathLength')
                row.planLength=r.plan.pathLength; row.planVerticalTravel=r.plan.verticalTravel;
                row.planCost=r.plan.pathCost; row.planCapacity=r.plan.capacity;
            end
            if isfield(r,'metrics')
                m=r.metrics;
                row.leaderPath=m.leaderPathLength; row.leaderAscent=m.leaderAscent;
                row.leaderDescent=m.leaderDescent; row.leaderVerticalTravel=m.leaderVerticalTravel;
                row.fleetAscent=m.fleetAscent; row.fleetDescent=m.fleetDescent;
                row.fleetPath=m.fleetPathLength; row.commandEffort=m.commandEffort;
                row.formationRms=m.formationRms;
                row.minObstacleClearance=m.minObstacleClearance;
                row.minPairDistance=m.minPairDistance;
                row.safetyTriggers=nnz(r.history.safetyTriggered);
                row.cycleP99Ms=prctile(r.history.cycleMs,99);
                row.cycleMaxMs=max(r.history.cycleMs);
            end
        end
        rows=[rows;row]; %#ok<AGROW>
        writetable(struct2table(rows),fullfile(outputDir,'trials.csv'));
        write_json(fullfile(outputDir,'status.json'),struct('complete',false, ...
            'scheduled',8+lastSeed-firstSeed+1,'completed',numel(rows)));
        fprintf('%s | %s | %s | %.2f s | ascent %.2f m | wall %.1f s\n', ...
            trialId,family,reason,row.time,row.leaderAscent,wallSeconds);
    end
end

function write_json(file,value)
    fid=fopen(file,'w','n','UTF-8'); guard=onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s',jsonencode(value,PrettyPrint=true));
end

function sha=file_hash(file)
    fid=fopen(file,'r'); bytes=fread(fid,inf,'*uint8'); fclose(fid);
    digest=java.security.MessageDigest.getInstance('SHA-256');
    digest.update(bytes); raw=typecast(digest.digest(),'uint8');
    sha=string(lower(reshape(dec2hex(raw,2)',1,[])));
end
