function diagnose_environment_gaps3d(studyDir)
% DIAGNOSE_ENVIRONMENT_GAPS3D Separate diagnostic, not the default planner.
% Add the mid-height of known structural wall openings to failed pilot maps.
% The original five-layer outcomes and denominator are never replaced.
    oldPath=path; guard=onCleanup(@() path(oldPath)); %#ok<NASGU>
    snap=fullfile(studyDir,'source_snapshot');
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'));
    assert(startsWith(which('run_simulation3d'),snap));
    trials=readtable(fullfile(studyDir,'trials.csv'),'TextType','string');
    selected=trials(trials.sceneId=="random" & trials.reason=="planning_failed",:);
    diagDir=fullfile(studyDir,'gap_diagnostics'); mkdir(diagDir);
    copyfile(mfilename('fullpath')+".m",diagDir);
    rows=struct([]);
    for i=1:height(selected)
        id=char(selected.trialId(i));
        base=load(fullfile(studyDir,'trials',[id '.mat']));
        q=base.q; scene=base.scene;
        candidates=wall_mid_heights(scene.obstacles,q);
        q.heightLayers=unique([q.heightLayers,candidates]);
        q.adp.wA=zeros(3,6); q.adp.wC=zeros(6,1);
        q=validate_params3d(q);
        [witness,planInfo]=plan_layers3d(q.start,q.target,scene.obstacles,q);
        witnessClearance=inf;
        points=[q.start,witness];
        for j=1:size(points,2)-1
            witnessClearance=min(witnessClearance, ...
                segment_clearance3d(points(:,j),points(:,j+1),scene.obstacles));
        end
        continuousWitness=~isempty(witness) && witnessClearance>=q.pathClearance-1e-6;
        r=run_simulation3d(q,scene);
        save(fullfile(diagDir,[id '.mat']), ...
            'r','q','scene','candidates','witness','planInfo','witnessClearance','-v7.3');
        row=struct('trialId',string(id),'seed',scene.seed,'family',string(scene.family), ...
            'baselineReason',selected.reason(i),'baselineLayerCount',numel(base.q.heightLayers), ...
            'addedHeights',strjoin(compose('%.9g',candidates),';'), ...
            'layerCount',numel(q.heightLayers),'capacity',planInfo.capacity, ...
            'planStatus',string(planInfo.status),'continuousWitness',continuousWitness, ...
            'witnessClearance',witnessClearance,'flightReason',string(r.reason), ...
            'flightSuccess',r.success,'time',r.time,'minObstacleClearance',nan, ...
            'minPairDistance',nan);
        if isfield(r,'metrics')
            row.minObstacleClearance=r.metrics.minObstacleClearance;
            row.minPairDistance=r.metrics.minPairDistance;
        end
        rows=[rows;row]; %#ok<AGROW>
        writetable(struct2table(rows),fullfile(diagDir,'diagnostics.csv'));
        fprintf('Gap diagnostic seed %d: %d layers | witness %d | %s\n', ...
            scene.seed,numel(q.heightLayers),continuousWitness,r.reason);
    end
    fid=fopen(fullfile(diagDir,'status.json'),'w','n','UTF-8');
    fprintf(fid,'%s',jsonencode(struct('complete',true,'scheduled',height(selected), ...
        'completed',numel(rows),'rule','Window centre insertion is a separate diagnostic; baseline trials unchanged.'), ...
        PrettyPrint=true)); fclose(fid);
end

function heights=wall_mid_heights(obstacles,p)
    heights=zeros(1,0);
    for a=1:numel(obstacles)
        low=obstacles(a);
        if ~strcmp(low.type,'box') || low.zmin~=p.workspace(1,3), continue; end
        for b=1:numel(obstacles)
            high=obstacles(b);
            if ~strcmp(high.type,'box') || high.zmax~=p.workspace(2,3), continue; end
            sameFootprint=norm(low.center(1:2)-high.center(1:2))<1e-9 && ...
                norm(low.halfSize(1:2)-high.halfSize(1:2))<1e-9;
            if sameFootprint && high.zmin-low.zmax>=2*p.pathClearance
                heights(end+1)=(high.zmin+low.zmax)/2; %#ok<AGROW>
            end
        end
    end
    heights=unique(heights);
end
