function result=main_unknown3d(options)
% MAIN_UNKNOWN3D Rectified stereo -> private maps -> five/single UAV flight.
% options.config accepts um_config(); options.caseId selects a DEVELOPMENT
% scene. The environment is passed only to simulation sensors and audit.
    if nargin<1,options=struct();end
    assert(isstruct(options) && isscalar(options) && ...
        all(ismember(fieldnames(options),{'config','caseId','video'})), ...
        'UM:Options','Use options.config, options.caseId and optional options.video.');
    project=fileparts(mfilename('fullpath'));oldPath=path;cleanup=onCleanup(@()path(oldPath));
    addpath(fullfile(project,'unknown_map3d'),fullfile(project,'src3d'));
    c=um_config();c.sensorMode='stereo';c.controlMode='apf';c.nUAV=5;c.maxSeconds=240;
    if isfield(options,'config'),c=um_validate_config(options.config);end
    caseId='sphere';if isfield(options,'caseId'),caseId=options.caseId;end
    cases=um_cases(c,'development');index=find(cellfun(@(x)strcmp(x.id,caseId),cases));
    assert(numel(index)==1,'UM:CaseId','Unknown development caseId');item=cases{index};
    directory=fullfile(project,'results','unknown_map3d',['run_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))]);
    assert(~exist(directory,'dir'),'UM:OutputExists');mkdir(directory);source=fullfile(directory,'source');mkdir(source);
    folders={'unknown_map3d','src3d'};manifest=struct('file',{},'sha256',{});
    for i=1:numel(folders)
        copyfile(fullfile(project,folders{i}),fullfile(source,folders{i}));
        files=dir(fullfile(source,folders{i},'**','*'));
        for j=1:numel(files)
            if files(j).isdir,continue;end
            file=fullfile(files(j).folder,files(j).name);
            manifest(end+1)=struct('file',erase(file,[source filesep]),'sha256',um_file_hash(file)); %#ok<AGROW>
        end
    end
    for entry={'main_unknown3d.m','main.m','VERSION'}
        copyfile(fullfile(project,entry{1}),fullfile(source,entry{1}));
        manifest(end+1)=struct('file',entry{1},'sha256',um_file_hash(fullfile(source,entry{1}))); %#ok<AGROW>
    end
    save(fullfile(directory,'input.mat'),'item','-v7');
    metadata=struct('sourceSha256',manifest,'inputSha256',um_file_hash(fullfile(directory,'input.mat')), ...
        'matlab',version,'matlabRelease',version('-release'),'platform',computer, ...
        'visionToolbox',ver('vision'),'gitBeforeRun',um_git_head(project), ...
        'scope','Simulation only; calibrated steerable stereo and bounded launch bubble.');
    fid=fopen(fullfile(directory,'manifest.json'),'w');fwrite(fid,jsonencode(metadata,PrettyPrint=true));fclose(fid);
    clear um_*;rehash;addpath(fullfile(source,'unknown_map3d'),fullfile(source,'src3d'));
    assert(startsWith(which('um_simulate'),source),'UM:SourceBinding');
    clock=tic;result=um_simulate(item);result.outputDirectory=directory;result.wallSeconds=toc(clock);
    save(fullfile(directory,'result.mat'),'result','-v7.3');
    row=um_result_row(result);row.wallSeconds=result.wallSeconds;writetable(struct2table(row),fullfile(directory,'summary.csv'));
    um_plot_result(result,directory);
    if isfield(options,'video') && options.video,um_render_video(result,fullfile(directory,'flight.mp4'));end
    fprintf('\nUnknown-map stereo: success=%d, reason=%s, time=%.2fs, audit=%d\n', ...
        result.success,result.reason,result.time,result.audit.passed);
    fprintf('Saved: %s\n',directory);
end
