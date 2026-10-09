function validate_midpoint_pilot3d(outputDir)
% VALIDATE_MIDPOINT_PILOT3D Regress unchanged flight fields and exercise fallback.
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'),fullfile(root,'tests'));
    if ~isfolder(outputDir), mkdir(outputDir); end
    test_adaptive_height3d(); test_environment3d(); test_directional_clearance3d(); test_3d();
    baseline=fullfile(root,'results','seed_sweep3d_v2.1.0_20261005_113155_970');
    records=struct([]);
    for seed=1:16
        old=load(fullfile(baseline,'workers','worker_01','trials',sprintf('C_random_%06d.mat',seed)));
        q=old.q; q.version='2.3.1';
        q.adaptiveHeight=struct('enable',false,'maxAddedLayers',8,'intervalTolerance',1e-7);
        fixed=run_simulation3d(q,old.scene);
        assert(fixed.success==old.r.success && strcmp(fixed.reason,old.r.reason));
        maxDifference=0;
        names=fieldnames(old.r.history);
        for i=1:numel(names)
            name=names{i}; if strcmp(name,'cycleMs'), continue; end
            x=old.r.history.(name); y=fixed.history.(name);
            assert(isequal(size(x),size(y)));
            assert(isequaln(x,y),'UAV3D:LegacyRegression','seed %d history %s changed.',seed,name);
        end
        assert(isequaln(fixed.state,old.r.state));
        q.adaptiveHeight.enable=true; r=run_simulation3d(q,old.scene);
        added=""; if ~isempty(r.plan.addedHeightLayers), added=strjoin(compose('%.12g',r.plan.addedHeightLayers),';'); end
        row=struct('seed',seed,'baselineSuccess',old.r.success,'baselineReason',string(old.r.reason), ...
            'fixedRegressionExact',true,'newSuccess',r.success,'newReason',string(r.reason), ...
            'layers',numel(r.plan.heightLayers),'addedHeights',added);
        records=[records;row]; %#ok<AGROW>
        save(fullfile(outputDir,sprintf('pilot_%06d.mat',seed)),'q','r','fixed','row','-v7');
        writetable(struct2table(records),fullfile(outputDir,'pilot.csv'));
        fprintf('MIDPOINT_PILOT seed=%d old=%s new=%s added=%s\n',seed,old.r.reason,r.reason,row.addedHeights);
    end
    save(fullfile(outputDir,'pilot_complete.mat'),'records');
    fprintf('MIDPOINT_PILOT_COMPLETE success=%d/16 exact_legacy=16/16\n',nnz([records.newSuccess]));
end
