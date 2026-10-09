function outputDir=benchmark_2d_3d()
% BENCHMARK_2D_3D Same MATLAB process/hardware, separate declared scenes.
% This is a runtime comparison, not a matched-map algorithm comparison.
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path;
    cleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'));
    p2=default_params();p2.animate=false;p2.Tmax=60;p2.seed=25;
    p3=default_params3d();p3.Tmax=60;p3.seed=25;
    warm2=p2;warm2.Tmax=0.1;warm2.nObstacles=0;
    warm3=p3;warm3.Tmax=0.1;
    run_simulation(warm2);
    run_simulation3d(warm3,scenario3d('S0_empty',25,warm3));
    outputDir=create_run_output_dir(p3.resultsRoot,'dimension_benchmark',p3.version);
    cases={'2d_empty','3d_empty','2d_default16','3d_sphere','3d_overpass'};
    results=cell(size(cases));
    p2.nObstacles=0;
    results{1}=run_simulation(p2);
    results{2}=run_simulation3d(p3,scenario3d('S0_empty',25,p3));
    p2.nObstacles=16;
    results{3}=run_simulation(p2);
    results{4}=run_simulation3d(p3,scenario3d('S1_sphere',25,p3));
    results{5}=run_simulation3d(p3,scenario3d('S3_overpass',25,p3));
    rows=struct([]);
    for i=1:numel(cases)
        r=results{i};cycles=r.history.cycleMs;
        row=struct('caseId',string(cases{i}),'reason',string(r.reason), ...
            'cycles',numel(cycles),'meanMs',mean(cycles), ...
            'p95Ms',prctile(cycles,95),'p99Ms',prctile(cycles,99), ...
            'maxMs',max(cycles),'over50Fraction',mean(cycles>50));
        rows=[rows;row]; %#ok<AGROW>
        fprintf('%s: %s p99=%.2f ms max=%.2f ms\n', ...
            cases{i},r.reason,row.p99Ms,row.maxMs);
    end
    writetable(struct2table(rows),fullfile(outputDir,'summary.csv'));
    save(fullfile(outputDir,'raw_results.mat'),'results','cases','p2','p3','-v7.3');
    copyfile(fullfile(root,'src','run_simulation.m'),outputDir);
    copyfile(fullfile(root,'experiments3d','benchmark_2d_3d.m'),outputDir);
end
