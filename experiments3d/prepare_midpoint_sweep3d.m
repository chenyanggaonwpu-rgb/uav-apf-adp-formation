function studyDir=prepare_midpoint_sweep3d(sourceCommit)
% PREPARE_MIDPOINT_SWEEP3D Freeze all original 1000 maps and midpoint policy.
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'));
    baseline=fullfile(root,'results','seed_sweep3d_v2.1.0_20261005_113155_970');
    old=load(fullfile(baseline,'protocol.mat'),'protocol'); oldProtocol=old.protocol;
    assert(isequal(oldProtocol.seeds,1:1000) && oldProtocol.nWorkers==4);
    p=oldProtocol.p; p.version='2.3.1';
    p.adaptiveHeight=struct('enable',true,'maxAddedLayers',8,'intervalTolerance',1e-7);
    assert(p.pathClearance==4.2 && p.desiredClear==4.0);
    studyDir=create_run_output_dir(fullfile(root,'results'),'adaptive_midpoint3d',p.version);
    mkdir(fullfile(studyDir,'maps')); mkdir(fullfile(studyDir,'workers'));
    mkdir(fullfile(studyDir,'validation'));
    snapshot=fullfile(studyDir,'source_snapshot'); mkdir(snapshot);
    for folder={'config','src3d','experiments3d'}
        mkdir(fullfile(snapshot,folder{1}));
    end
    copyfile(fullfile(root,'src3d','*.m'),fullfile(snapshot,'src3d'));
    copyfile(fullfile(root,'config','*.m'),fullfile(snapshot,'config'));
    sourceNames={'prepare_midpoint_sweep3d.m','run_midpoint_sweep3d.m', ...
        'audit_midpoint_sweep3d.m','midpoint_sha256_file3d.m','midpoint_sweep.py'};
    for i=1:numel(sourceNames)
        copyfile(fullfile(root,'experiments3d',sourceNames{i}),fullfile(snapshot,'experiments3d'));
    end
    hashes=struct([]); listing=dir(fullfile(snapshot,'**','*'));
    for i=1:numel(listing)
        if listing(i).isdir, continue; end
        file=fullfile(listing(i).folder,listing(i).name);
        relative=strrep(extractAfter(string(file),strlength(string(snapshot))+1),'\','/');
        hashes=[hashes;struct('file',relative,'sha256',string(midpoint_sha256_file3d(file)))]; %#ok<AGROW>
    end
    writetable(struct2table(hashes),fullfile(studyDir,'source_sha256.csv'));
    manifests=struct([]); previousRows=struct([]);
    for seed=1:1000
        oldWorker=find(seed>=oldProtocol.ranges(:,1) & seed<=oldProtocol.ranges(:,2));
        assert(numel(oldWorker)==1);
        oldMat=fullfile(baseline,'workers',sprintf('worker_%02d',oldWorker),'trials',sprintf('C_random_%06d.mat',seed));
        original=load(oldMat,'q','scene','row');
        scene=original.scene; originalQ=original.q; oldRow=original.row;
        assert(originalQ.seed==seed && scene.seed==seed && strcmp(scene.kind,'random'));
        assert(oldRow.success==strcmp(oldRow.reason,'success'));
        bytes=unicode2native(jsonencode(struct('workspace',originalQ.workspace,'obstacles',scene.obstacles)),'UTF-8');
        md=java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes);
        mapHash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
        relative=sprintf('maps/seed_%06d.mat',seed);
        save(fullfile(studyDir,relative),'scene','originalQ','oldRow','mapHash','oldMat','-v7');
        manifests=[manifests;struct('seed',seed,'family',string(scene.family), ...
            'mapFile',string(relative),'mapHash',string(mapHash),'baselineMat',string(oldMat), ...
            'baselineMatSha256',string(midpoint_sha256_file3d(oldMat)), ...
            'baselineSuccess',oldRow.success,'baselineReason',string(oldRow.reason))]; %#ok<AGROW>
        previousRows=[previousRows;oldRow]; %#ok<AGROW>
    end
    manifest=struct2table(manifests); baselineRows=struct2table(previousRows);
    assert(nnz(manifest.baselineSuccess)==564 && nnz(manifest.baselineReason=="planning_failed")==436);
    writetable(manifest,fullfile(studyDir,'map_manifest.csv'));
    writetable(baselineRows,fullfile(studyDir,'baseline_results.csv'));
    protocol=struct('version','2.3.1','p',p,'seeds',1:1000,'nWorkers',4, ...
        'ranges',oldProtocol.ranges,'baselineDir',baseline,'baselineSuccess',564, ...
        'sourceCommit',sourceCommit,'createdLocal',char(datetime('now','TimeZone','Asia/Shanghai','Format','yyyy-MM-dd HH:mm:ss')), ...
        'mapRule','Original stored geometry; never regenerate maps or remove failed seeds.', ...
        'parameterRule','All old parameters retained except version and adaptiveHeight policy.', ...
        'heightRule','Try base five layers first. On failure add uncovered local vertical interval midpoints; max 8 added layers.', ...
        'probeRule','Centres, 5x5 obstacle-footprint samples and 3x3 overlap samples; broad footprints first; all final graph edges checked.', ...
        'denominatorRule','All 1000 predeclared original seeds; flight success requires original terminal hold and safety criteria.', ...
        'timingRule','Four concurrent single-compute-thread MATLAB processes; timing is not a hard real-time benchmark.');
    save(fullfile(studyDir,'protocol.mat'),'protocol');
    fid=fopen(fullfile(studyDir,'protocol.json'),'w','n','UTF-8');
    fprintf(fid,'%s',jsonencode(protocol,PrettyPrint=true)); fclose(fid);
    runtime=fullfile(root,'.runtime','adaptive_midpoint_20261008');
    fid=fopen(fullfile(runtime,'study_path.txt'),'w','n','UTF-8'); fprintf(fid,'%s',studyDir); fclose(fid);
    fprintf('MIDPOINT_STUDY_PREPARED=%s maps=%d old_success=%d\n',studyDir,height(manifest),nnz(manifest.baselineSuccess));
end
