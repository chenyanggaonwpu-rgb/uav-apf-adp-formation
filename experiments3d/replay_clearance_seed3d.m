function [r,outputDir]=replay_clearance_seed3d(seed,makeVideo,studyDir)
% REPLAY_CLEARANCE_SEED3D Frozen map, frozen numerical code, selected preset.
% [r,folder]=replay_clearance_seed3d(1004,true) additionally exports an MP4.
    if nargin<2, makeVideo=false; end
    if nargin<3, studyDir=[]; end
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path; guard=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'config'),fullfile(root,'src'));
    [p,studyDir]=clearance_candidate_params3d(studyDir);
    snap=fullfile(studyDir,'source_snapshot');
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'));
    assert(startsWith(which('run_simulation3d'),snap));
    assert(isscalar(seed) && isfinite(seed) && seed==floor(seed));
    mapFile=fullfile(studyDir,'maps',sprintf('seed_%06d.mat',seed));
    assert(isfile(mapFile),'UAV3D:MapNotFrozen','Requested seed is absent from the frozen map library.');
    map=load(mapFile,'scene','mapHash'); scene=map.scene; mapHash=map.mapHash;
    bytes=unicode2native(jsonencode(struct('workspace',p.workspace,'obstacles',scene.obstacles)),'UTF-8');
    md=java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes);
    digest=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
    manifest=readtable(fullfile(studyDir,'map_manifest.csv'),'Delimiter',',','VariableNamingRule','preserve','TextType','string');
    expected=manifest.mapHash(manifest.seed==seed);
    assert(numel(expected)==1 && strcmp(digest,expected) && strcmp(digest,mapHash), ...
        'UAV3D:MapChanged','Frozen map geometry does not match its manifest.');
    p.seed=seed; p.stochasticSeed=seed;
    r=run_simulation3d(p,scene);
    outputDir=create_run_output_dir(fullfile(root,'results'),'clearance_replay3d',p.version);
    save(fullfile(outputDir,'replay.mat'),'r','p','scene','mapHash','-v7');
    if isfield(r.history,'t') && ~isempty(r.history.t)
        plot_result3d(r,outputDir);
        if makeVideo, render_3d_animation(r,fullfile(outputDir,'flight_3d.mp4')); end
    end
    record=struct('seed',seed,'success',r.success,'reason',r.reason, ...
        'configId',p.evidenceConfigId,'sourceCommit',p.evidenceCommit, ...
        'mapHash',mapHash,'mapFile',mapFile,'sourceSnapshot',snap, ...
        'pathXY',p.pathClearance,'refXY',p.desiredClear, ...
        'planZ',p.directionalClearance.planningZ,'refZ',p.directionalClearance.referenceZ, ...
        'outputDir',outputDir,'requestedVideo',logical(makeVideo));
    fid=fopen(fullfile(outputDir,'replay_protocol.json'),'w','n','UTF-8');
    fprintf(fid,'%s',jsonencode(record,PrettyPrint=true)); fclose(fid);
    fprintf('CLEARANCE_REPLAY seed=%d reason=%s output=%s\n',seed,r.reason,outputDir);
end
