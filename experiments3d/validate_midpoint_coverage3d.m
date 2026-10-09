function validate_midpoint_coverage3d()
% VALIDATE_MIDPOINT_COVERAGE3D Generic occluded-centre test and retained misses.
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'),fullfile(root,'tests'));
    test_adaptive_height3d();
    rt=fullfile(root,'.runtime','adaptive_midpoint_20261008');
    attempt=strtrim(fileread(fullfile(rt,'attempt01_study_path.txt')));
    baseline=fullfile(root,'results','seed_sweep3d_v2.1.0_20261005_113155_970');
    seeds=[];
    for w=1:4
        rows=readtable(fullfile(attempt,'workers',sprintf('worker_%02d',w),'trials.csv'), ...
            'Delimiter',',','TextType','string','VariableNamingRule','preserve');
        seeds=[seeds;rows.seed(~rows.success)]; %#ok<AGROW>
    end
    seeds=sort(seeds); records=struct([]);
    for seed=seeds'
        old=load(fullfile(baseline,'workers',sprintf('worker_%02d',ceil(seed/250)), ...
            'trials',sprintf('C_random_%06d.mat',seed)),'q','scene');
        q=old.q; q.version='2.3.1'; q.adaptiveHeight=struct('enable',true,'maxAddedLayers',8,'intervalTolerance',1e-7);
        [route,info]=plan_layers3d(q.start,q.target,old.scene.obstacles,q);
        margin=inf; points=[q.start,route];
        for i=1:size(points,2)-1
            margin=min(margin,segment_clearance3d(points(:,i),points(:,i+1),old.scene.obstacles));
        end
        assert(~isempty(route) && margin>=q.pathClearance-1e-6,'Coverage repair failed seed %d',seed);
        added="";if ~isempty(info.addedHeightLayers),added=strjoin(compose('%.12g',info.addedHeightLayers),';');end
        records=[records;struct('seed',seed,'status',string(info.status), ...
            'addedHeights',added,'layers',numel(info.heightLayers),'routeMinClearance',margin, ...
            'budgetRejected',info.heightSelection.budgetRejected)]; %#ok<AGROW>
        fprintf('COVERAGE_REPAIR seed=%d path=%s layers=%d minClear=%.8f\n',seed,info.status,numel(info.heightLayers),margin);
    end
    writetable(struct2table(records),fullfile(rt,'coverage_repair.csv'));
    fprintf('COVERAGE_REPAIR_TESTS_PASSED cases=%d\n',numel(seeds));
end
