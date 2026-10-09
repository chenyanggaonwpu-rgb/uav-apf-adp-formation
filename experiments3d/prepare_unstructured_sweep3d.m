function studyDir=prepare_unstructured_sweep3d(sourceCommit)
% PREPARE_UNSTRUCTURED_SWEEP3D Freeze all 200 maps before any trial is run.
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'));
    p=default_params3d(); p.animate=false; p.sceneId='unstructured_random';
    assert(strcmp(p.version,'2.3.1') && p.pathClearance==4.2 && p.desiredClear==4);
    assert(p.adaptiveHeight.enable && p.adaptiveHeight.maxAddedLayers==8);
    options=struct('countRange',[1 40],'sphereRadius',[1 6], ...
        'cylinderRadius',[1 6],'cylinderHeight',[2 24], ...
        'boxWidth',[2 20],'boxDepth',[2 20],'boxHeight',[2 24]);
    studyDir=create_run_output_dir(fullfile(root,'results'),'unstructured_random3d',p.version);
    mkdir(fullfile(studyDir,'maps')); mkdir(fullfile(studyDir,'workers')); mkdir(fullfile(studyDir,'validation'));
    snapshot=fullfile(studyDir,'source_snapshot'); mkdir(snapshot);
    for folder={'config','src3d','experiments3d'}, mkdir(fullfile(snapshot,folder{1})); end
    copyfile(fullfile(root,'src3d','*.m'),fullfile(snapshot,'src3d'));
    copyfile(fullfile(root,'config','*.m'),fullfile(snapshot,'config'));
    names={'unstructured_scene3d.m','prepare_unstructured_sweep3d.m','run_unstructured_sweep3d.m', ...
        'audit_unstructured_sweep3d.m','midpoint_sha256_file3d.m','unstructured_sweep.py', ...
        'test_unstructured_scene3d.m'};
    for k=1:numel(names), copyfile(fullfile(root,'experiments3d',names{k}),fullfile(snapshot,'experiments3d')); end
    hashes=struct([]); listing=dir(fullfile(snapshot,'**','*'));
    for k=1:numel(listing)
        if listing(k).isdir, continue; end
        file=fullfile(listing(k).folder,listing(k).name);
        relative=strrep(extractAfter(string(file),strlength(string(snapshot))+1),'\','/');
        hashes=[hashes;struct('file',relative,'sha256',string(midpoint_sha256_file3d(file)))]; %#ok<AGROW>
    end
    writetable(struct2table(hashes),fullfile(studyDir,'source_sha256.csv'));
    manifests=struct([]); catalog=struct([]); seeds=2001:2200;
    for seed=seeds
        [scene,metadata]=unstructured_scene3d(seed,p,options);
        q=p; q.seed=seed; q.randomObstacleCount=metadata.obstacleCount;
        [state,offsets,psi]=init_formation3d(q); rot=[cos(psi) -sin(psi);sin(psi) cos(psi)];
        terminal=[q.target(1:2)+rot*offsets;q.target(3)*ones(1,q.nUAV)];
        startClear=min_clearance(state(1:3,:),scene.obstacles);
        leaderStart=min_clearance(q.start,scene.obstacles); leaderTarget=min_clearance(q.target,scene.obstacles);
        terminalClear=min_clearance(terminal,scene.obstacles);
        bytes=unicode2native(jsonencode(struct('workspace',q.workspace,'obstacles',scene.obstacles)),'UTF-8');
        md=java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes);
        mapHash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
        relative=sprintf('maps/seed_%06d.mat',seed); file=fullfile(studyDir,relative);
        save(file,'scene','q','metadata','mapHash','-v7');
        record=struct('seed',seed,'mapFile',string(relative),'mapHash',string(mapHash), ...
            'mapMatSha256',string(midpoint_sha256_file3d(file)), ...
            'obstacleCount',metadata.obstacleCount,'sphereCount',metadata.sphereCount, ...
            'cylinderCount',metadata.cylinderCount,'boxCount',metadata.boxCount, ...
            'nominalVolume',metadata.nominalVolume,'nominalVolumeFraction',metadata.nominalVolumeFraction, ...
            'initialMinObstacleClearance',startClear,'initialHardSafe',startClear>q.obsSafe, ...
            'leaderStartClearance',leaderStart,'leaderTargetClearance',leaderTarget, ...
            'leaderEndpointsPlanClear',min(leaderStart,leaderTarget)>=q.pathClearance, ...
            'leaderTargetHardSafe',leaderTarget>q.obsSafe,'nominalTerminalMinClearance',terminalClear, ...
            'nominalTerminalHardSafe',terminalClear>q.obsSafe);
        manifests=[manifests;record]; %#ok<AGROW>
        for k=1:numel(scene.obstacles)
            o=scene.obstacles(k); radius=nan;
            if strcmp(o.type,'sphere')
                c=o.center; sides=2*o.r*ones(3,1); radius=o.r; volume=4*pi*o.r^3/3;
            elseif strcmp(o.type,'cylinder')
                c=[o.center;(o.zmin+o.zmax)/2]; sides=[2*o.r;2*o.r;o.zmax-o.zmin]; radius=o.r; volume=pi*o.r^2*sides(3);
            else
                c=o.center; sides=2*o.halfSize; volume=prod(sides);
            end
            catalog=[catalog;struct('seed',seed,'index',k,'type',string(o.type), ...
                'x',c(1),'y',c(2),'z',c(3),'radius',radius, ...
                'sizeX',sides(1),'sizeY',sides(2),'sizeZ',sides(3),'volume',volume)]; %#ok<AGROW>
        end
    end
    manifest=struct2table(manifests); assert(height(manifest)==200);
    writetable(manifest,fullfile(studyDir,'map_manifest.csv'));
    writetable(struct2table(catalog),fullfile(studyDir,'obstacle_catalog.csv'));
    protocol=struct('version','2.3.1','experiment','unstructured_random_200','p',p,'generation',options, ...
        'seeds',seeds,'nWorkers',4,'ranges',[2001 2050;2051 2100;2101 2150;2151 2200], ...
        'sourceCommit',sourceCommit,'createdLocal',char(datetime('now','TimeZone','Asia/Shanghai','Format','yyyy-MM-dd HH:mm:ss')), ...
        'mapRule','All 200 predeclared seeds frozen before trials; no replacement, endpoint reservation, spacing constraint or reachability filtering.', ...
        'randomRule','Independent uniform count and type; uniform bounded sizes; centres conditionally uniform on the shape-specific containment region; overlap allowed.', ...
        'denominatorRule','Primary success / 200 includes occupied initial scenes and planning failures; separately show success / initially hard-safe maps. Execution errors invalidate inference.', ...
        'heightRule','Unchanged 2.3.1: try base 5/10/15/20/25 m then add uncovered local interval midpoints, at most 8 additions.', ...
        'timingRule','Four concurrent single-compute-thread MATLAB processes; timing is not a hard real-time benchmark.', ...
        'illustrationRule','Smallest successful seed, smallest failure with a flight history, and smallest planning-failed seed; never chosen to alter the statistics.');
    save(fullfile(studyDir,'protocol.mat'),'protocol');
    fid=fopen(fullfile(studyDir,'protocol.json'),'w','n','UTF-8'); fprintf(fid,'%s',jsonencode(protocol,PrettyPrint=true)); fclose(fid);
    runtime=fullfile(root,'.runtime','unstructured_random_20261008'); if ~isfolder(runtime),mkdir(runtime);end
    fid=fopen(fullfile(runtime,'study_path.txt'),'w','n','UTF-8'); fprintf(fid,'%s',studyDir); fclose(fid);
    fprintf('UNSTRUCTURED_PREPARED=%s maps=%d occupied_initial=%d obstacles=%d\n', ...
        studyDir,height(manifest),nnz(~manifest.initialHardSafe),height(struct2table(catalog)));
end
function result=min_clearance(points,obstacles)
    result=inf;
    for o=obstacles
        for k=1:size(points,2), result=min(result,obstacle_distance3d(points(:,k),o)); end
    end
end
