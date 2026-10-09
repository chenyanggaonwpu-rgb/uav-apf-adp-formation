function outputDir=recheck_s3_timing3d()
% RECHECK_S3_TIMING3D Supplemental isolated repeat; never replaces full data.
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path;
    cleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'));
    p=default_params3d();
    scene=scenario3d('S3_overpass',p.seed,p);
    warm=p;warm.Tmax=0.1;
    run_simulation3d(warm,scenario3d('S0_empty',warm.seed,warm));
    outputDir=create_run_output_dir(p.resultsRoot,'s3_timing_recheck',p.version);
    levels=[0 0.02 0.05 0.10];
    rows=struct([]);
    for index=1:numel(levels)
        q=p;q.noiseStd=levels(index);
        r=run_simulation3d(q,scene);
        cycles=r.history.cycleMs;
        row=struct('noiseStd',levels(index),'reason',string(r.reason), ...
            'cycles',numel(cycles),'p95Ms',prctile(cycles,95), ...
            'p99Ms',prctile(cycles,99),'maxMs',max(cycles), ...
            'over50Cycles',nnz(cycles>50));
        rows=[rows;row]; %#ok<AGROW>
        save(fullfile(outputDir,sprintf('noise_%03d.mat',round(1000*levels(index)))), ...
            'r','-v7.3');
        fprintf('S3 timing recheck sigma %.2f: %s p99 %.2f ms max %.2f ms over50 %d\n', ...
            levels(index),r.reason,row.p99Ms,row.maxMs,row.over50Cycles);
    end
    writetable(struct2table(rows),fullfile(outputDir,'summary.csv'));
    copyfile(fullfile(root,'experiments3d','recheck_s3_timing3d.m'),outputDir);
end
