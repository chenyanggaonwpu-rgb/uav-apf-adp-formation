function studyDir=prepare_clearance_study3d(sourceCommit)
% PREPARE_CLEARANCE_STUDY3D Immutable maps, numerical snapshot and protocol.
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path; guard=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'));
    baseline=fullfile(root,'results','seed_sweep3d_v2.1.0_20261005_113155_970');
    old=load(fullfile(baseline,'protocol.mat'),'protocol');
    generationP=old.protocol.p;
    p=validate_params3d(default_params3d()); p.sceneId='random'; p.animate=false;
    studyDir=create_run_output_dir(fullfile(root,'results'),'clearance_boundary3d',p.version);
    snap=fullfile(studyDir,'source_snapshot'); mkdir(snap);
    for sub={'config','src3d','experiments3d','tests'}
        mkdir(fullfile(snap,sub{1}));
        copyfile(fullfile(root,sub{1},'*.m'),fullfile(snap,sub{1}));
    end
    mkdir(fullfile(snap,'src'));
    copyfile(fullfile(root,'experiments3d','clearance_study.py'),fullfile(snap,'experiments3d'));
    copyfile(fullfile(root,'src','create_run_output_dir.m'),fullfile(snap,'src'));
    mkdir(fullfile(studyDir,'maps')); mkdir(fullfile(studyDir,'stages'));
    copyfile(fullfile(baseline,'trials_all_1000.csv'),fullfile(studyDir,'baseline_trials.csv'));
    copyfile(fullfile(baseline,'trials_sha256.csv'),fullfile(studyDir,'baseline_mat_sha256.csv'));
    protocol=struct('version',p.version,'sourceCommit',sourceCommit,'baselineTag', ...
        'baseline-3d-v2.1.0-20261006','baselineDir',baseline,'p',p, ...
        'generationP',generationP,'mapSeeds',1:2000, ...
        'screenSeeds',1:8,'refinementSeeds',1:40,'nominalSeeds',1:1000, ...
        'robustnessSeeds',41:80,'holdoutSeeds',1001:2000, ...
        'pathXYLevels',[0.61 1 2 3 4.2 5.6], ...
        'refXYLevels',[0.61 1 2 3 4], ...
        'zLevels',[0.61 0.8 1.1 1.5 2 3.2 4.2], ...
        'noiseBoundary',[0 0.02 0.05 0.1 0.2 0.35 0.5 1 2], ...
        'disturbanceBoundary',[0 0.1 0.2 0.4 0.8 1.5 3], ...
        'geometryWidths',[1 1.25 1.5 2 3 4 6 8 10], ...
        'geometryCenters',[10 11.25 12.5 13.75 15], ...
        'screenRule','All 210 coarse factor combinations plus unchanged baseline, same eight maps.', ...
        'selectionRule','Exclude execution errors; rank end-to-end success, worst-family success, then safe distances. Retain multiple separated parameter regions and boundary contrasts.', ...
        'holdoutRule','Select and freeze the final candidate before inspecting seeds 1001-2000; baseline and candidate share every map and stochastic sequence.', ...
        'hardSafetyRule','obsSafe=0.6, uavSafe=1, safety layer enabled, vmax=amax=3; success conditions unchanged.', ...
        'geometryRule','Minkowski margin ellipsoid with horizontal/vertical radii; true physical distances and collision checks unchanged.', ...
        'recordRule','All jobs, failures, raw trajectories, map hashes, source hashes and selection decisions retained; no outcome-filtered maps.', ...
        'timingRule','Parallel timings are descriptive only. Final timing benchmark runs alone.', ...
        'createdLocal',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
    save(fullfile(studyDir,'protocol.mat'),'protocol','-v7');
    write_json(fullfile(studyDir,'protocol.json'),protocol);
    manifests=struct([]);
    for seed=1:2000
        if seed<=1000
            worker=find(seed>=old.protocol.ranges(:,1) & seed<=old.protocol.ranges(:,2));
            raw=fullfile(baseline,'workers',sprintf('worker_%02d',worker),'trials', ...
                sprintf('C_random_%06d.mat',seed));
            stored=load(raw,'scene'); scene=stored.scene; origin=raw;
        else
            % Generation parameters are frozen at the old protocol values.
            scene=scenario3d('random',seed,generationP); origin='frozen_2.1.0_generator';
        end
        mapHash=geometry_hash(scene,generationP.workspace);
        mapFile=fullfile('maps',sprintf('seed_%06d.mat',seed));
        save(fullfile(studyDir,mapFile),'scene','mapHash','generationP','-v7');
        manifests=[manifests;struct('seed',seed,'family',string(scene.family), ...
            'mapFile',string(mapFile),'mapHash',string(mapHash),'origin',string(origin))]; %#ok<AGROW>
        if mod(seed,250)==0, fprintf('MAPS_FROZEN=%d/2000\n',seed); end
    end
    % Controlled vertical aperture family, including mathematically closed
    % and open-but-unsampled cases. These are not population success seeds.
    id=900000;
    for width=protocol.geometryWidths
        for center=protocol.geometryCenters
            id=id+1;
            scene=scenario3d('S8_window',25,generationP);
            scene.seed=id; scene.id=sprintf('gate_w_%g_z_%g',width,center);
            scene.family='boundary_window'; scene.gateWidth=width; scene.gateCenter=center;
            scene.obstacles=[obstacle3d('box',[35;0;0],[45;80;center-width/2]), ...
                obstacle3d('box',[35;0;center+width/2],[45;80;30])];
            mapHash=geometry_hash(scene,generationP.workspace); mapFile=fullfile('maps',sprintf('seed_%06d.mat',id));
            save(fullfile(studyDir,mapFile),'scene','mapHash','generationP','-v7');
            manifests=[manifests;struct('seed',id,'family',string(scene.family), ...
                'mapFile',string(mapFile),'mapHash',string(mapHash),'origin',"predeclared_window_grid")]; %#ok<AGROW>
        end
    end
    writetable(struct2table(manifests),fullfile(studyDir,'map_manifest.csv'));
    files=[dir(fullfile(snap,'**','*.m'));dir(fullfile(snap,'**','*.py'))]; rows=struct([]);
    for file=files'
        filePath=fullfile(file.folder,file.name);
        rows=[rows;struct('relativePath',string(filePath(numel(snap)+2:end)), ...
            'sha256',string(file_hash(filePath)))]; %#ok<AGROW>
    end
    writetable(struct2table(rows),fullfile(studyDir,'source_sha256.csv'));
    fprintf('CLEARANCE_STUDY_DIR=%s\n',studyDir);
end

function digest=geometry_hash(scene,workspace)
    bytes=unicode2native(jsonencode(struct('workspace',workspace, ...
        'obstacles',scene.obstacles)),'UTF-8');
    md=java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes);
    digest=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
end
function digest=file_hash(path)
    fid=fopen(path,'rb'); bytes=fread(fid,inf,'*uint8'); fclose(fid);
    md=java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes);
    digest=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
end
function write_json(path,value)
    fid=fopen(path,'w','n','UTF-8'); fprintf(fid,'%s',jsonencode(value,PrettyPrint=true)); fclose(fid);
end
