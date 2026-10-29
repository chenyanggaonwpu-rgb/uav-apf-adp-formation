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
    ua_worker(output,shard,shards);
end
