function output=run_all_unified(output,shard,shards)
% Reproduce all 3840 preregistered trials from the committed geometry pack.
% Add this directory to MATLAB's path, then run run_all_unified(). For parallel
% execution use separate MATLAB sessions and the SAME output with shard=1..N.
    project=fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(project,'unified_actor3d'));ua_setup();
    if nargin<1 || isempty(output)
        output=create_run_output_dir(fullfile(project,'results'),'unified_actor','2.3.4-unified-actor-20261029');
    end
    if nargin<2,shard=1;end
    if nargin<3,shards=1;end
    if ~isfolder(output),mkdir(output);end
    for folder={'inputs','flight','traces','records','progress','reports'},if ~isfolder(fullfile(output,folder{1})),mkdir(fullfile(output,folder{1}));end,end
    if ~isfile(fullfile(output,'catalogue.mat'))
        copyfile(fullfile(fileparts(mfilename('fullpath')),'catalogue.mat'),fullfile(output,'catalogue.mat'));
    end
    for attempt=1:5
        try
            ua_worker(output,shard,shards);return;
        catch exception
            names={exception.stack.name};ioFailure=any(strcmp(names,'ua_worker>writejson'));
            if ~ioFailure || attempt==5,rethrow(exception);end
            % Windows readers may temporarily hold a progress file. Preserve
            % the error and resume; already saved trial records are skipped.
            file=fullfile(output,sprintf('worker_%02d_io_attempt_%02d.txt',shard,attempt));
            fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',getReport(exception,'extended','hyperlinks','off'));fclose(fid);
            warning('UA:ProgressIOResume','Progress output failed; resuming saved tasks: %s',exception.message);
            pause(.5);
        end
    end
end
