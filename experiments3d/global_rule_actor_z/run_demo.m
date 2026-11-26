function outDir = run_demo(outDir)
% Run the packaged single-window and multi-window cases and save trajectories.
    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    campaign = fullfile(root,'training','global_subgoal_actor');
    if nargin < 1 || isempty(outDir)
        stamp = char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
        outDir = fullfile(root,'demo_output',stamp);
    end
    casesFile = fullfile(here,'cases','smoke_cases.json');
    run_bench(campaign,casesFile,outDir,[],[],true);
    summary = jsondecode(fileread(fullfile(outDir,'summary.json')));
    assert(summary.completed == summary.cases,'UAV:DemoIncomplete', ...
        'Only %d of %d cases completed. See %s.', ...
        summary.completed,summary.cases,fullfile(outDir,'flights.jsonl'));
    fprintf('Demo output: %s\n',outDir);
end
