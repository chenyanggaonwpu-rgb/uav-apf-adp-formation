function studyDir=prepare_seed_sweep3d(firstSeed,lastSeed,nWorkers)
% PREPARE_SEED_SWEEP3D Freeze a predeclared seed sweep of the current C setup.
% studyDir=prepare_seed_sweep3d(151,1150,4)
    if nargin<3, nWorkers=1; end
    validateattributes(firstSeed,{'numeric'},{'scalar','integer','nonnegative'});
    validateattributes(lastSeed,{'numeric'},{'scalar','integer','>=',firstSeed});
    validateattributes(nWorkers,{'numeric'},{'scalar','integer','positive'});
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path; cleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'config'),fullfile(root,'src'));
    p=default_params3d();
    p.sceneId='random'; p.noiseStd=0; p.disturbanceMax=0; p.animate=false;
    p.adp.enable=true; p.safety.enable=true;
    p.adp.wA=zeros(3,6); p.adp.wC=zeros(6,1);
    if ~isequal(p.heightLayers,[5 10 15 20 25])
        error('UAV3D:FixedLayers','This experiment requires the original five fixed height layers.');
    end
    studyDir=create_run_output_dir(fullfile(root,'results'),'seed_sweep3d',p.version);
    snap=fullfile(studyDir,'source_snapshot');
    mkdir(snap); mkdir(fullfile(snap,'src3d')); mkdir(fullfile(snap,'config'));
    copyfile(fullfile(root,'src3d','*.m'),fullfile(snap,'src3d'));
    copyfile(fullfile(root,'config','*.m'),fullfile(snap,'config'));
    copyfile(fullfile(root,'experiments3d','run_seed_sweep3d.m'),snap);
    copyfile(mfilename('fullpath')+".m",snap);
    seeds=firstSeed:lastSeed;
    nWorkers=min(nWorkers,numel(seeds));
    cuts=round(linspace(0,numel(seeds),nWorkers+1));
    ranges=zeros(nWorkers,2);
    for i=1:nWorkers
        ranges(i,:)=[seeds(cuts(i)+1),seeds(cuts(i+1))];
    end
    protocol=struct('firstSeed',firstSeed,'lastSeed',lastSeed, ...
        'seeds',seeds,'nWorkers',nWorkers,'ranges',ranges,'p',p, ...
        'group','C','noiseStd',0,'disturbanceMax',0, ...
        'sceneId','random','heightPolicy','fixed_5_10_15_20_25_no_insertion', ...
        'mapFamilyRule','seed modulo four: mixed, underpass, window, staggered_windows', ...
        'mapFiltering','No path or flight outcome filtering; generator rules are frozen in scenario3d.m.', ...
        'purpose','Measure current fixed-five-layer end-to-end success rate.', ...
        'denominator','All predeclared seeds; generation, planning and execution failures retained.', ...
        'sourceRule','All simulation functions are loaded from this immutable source snapshot.', ...
        'timingRule','Concurrent independent MATLAB processes; cycle times are not a real-time benchmark.', ...
        'createdLocal',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
    save(fullfile(studyDir,'protocol.mat'),'protocol');
    fid=fopen(fullfile(studyDir,'protocol.json'),'w','n','UTF-8');
    fprintf(fid,'%s',jsonencode(protocol,PrettyPrint=true)); fclose(fid);
    mkdir(fullfile(studyDir,'workers'));
    fprintf('STUDY_DIR=%s\n',studyDir);
    disp(ranges);
end
