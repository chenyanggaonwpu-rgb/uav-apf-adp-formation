function summary=test_normal_scene3d(outputDir)
% TEST_NORMAL_SCENE3D Geometry regressions, honest bounded failures and frozen fresh seeds.
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'config'),fullfile(root,'src3d'),fullfile(root,'experiments3d'));
    if nargin<1, outputDir=fullfile(root,'.runtime','normal_scene_20261009','focused_tests'); end
    if ~isfolder(outputDir), mkdir(outputDir); end
    mapsDir=fullfile(outputDir,'maps'); if ~isfolder(mapsDir), mkdir(mapsDir); end
    p=default_params3d(); rules=p.normalScene;
    empty=struct('type',{},'center',{},'r',{},'zmin',{},'zmax',{},'halfSize',{});
    scene=struct('obstacles',empty);
    report=check_normal_scene3d(scene,p,rules);
    assert(report.accepted && report.taskFeasible && strcmp(report.feasibilityStatus,'certified'));
    assert(report.witness.duration<=p.Tmax && report.witness.minPairDistance>p.uavSafe);
    assert(report.witness.settleDuration==p.terminalBlendTime+p.holdDuration);
    tight=p; tight.Tmax=report.witness.duration-0.01;
    timed=check_normal_scene3d(scene,tight,rules);
    assert(~timed.accepted && strcmp(timed.feasibilityStatus,'time_budget_not_certified'));
    % Protect a follower, not just the leader.
    initial=init_formation3d(p); unsafe=scene;
    unsafe.obstacles=obstacle3d('sphere',initial(1:3,5),0.5);
    report=check_normal_scene3d(unsafe,p,rules);
    assert(~report.accepted && strcmp(report.reason,'initial_geometry'));
    assert(obstacle_distance3d(p.start,unsafe.obstacles)>rules.startClearance);
    % Endpoints can be clear while the first departure crosses an obstacle.
    direction=(p.target-p.start)/norm(p.target-p.start); blocked=scene;
    blocked.obstacles=obstacle3d('sphere',p.start+3.5*direction,0.5);
    report=check_normal_scene3d(blocked,p,rules);
    assert(report.initialValid && ~report.departureValid && strcmp(report.reason,'departure_not_certified'));
    terminal=p.target+(initial(1:3,:)-p.start); unsafe=scene;
    unsafe.obstacles=obstacle3d('sphere',terminal(:,5),0.5);
    report=check_normal_scene3d(unsafe,p,rules);
    assert(report.initialValid && ~report.terminalValid && strcmp(report.reason,'terminal_geometry'));
    % A safe state is not a continuous-motion certificate.
    obstacle=obstacle3d('sphere',[25;25;10],1);
    assert(obstacle_distance3d([20;25;10],obstacle)>rules.witnessClearance);
    assert(~certify_formation_segment3d([20;25;10],[30;25;10],zeros(3,1), ...
        obstacle,p,rules.witnessClearance,rules.boundaryClearance));
    % Exhausted independent search remains unknown, with no false infeasibility claim.
    blocked=scene; blocked.obstacles=obstacle3d('sphere',[40;36;10],6);
    limited=rules; limited.maxExpandedNodes=1;
    report=check_normal_scene3d(blocked,p,limited);
    assert(report.localValid && ~report.accepted && ~report.taskFeasible && ...
        strcmp(report.feasibilityStatus,'search_budget_exhausted'));
    localOnly=rules; localOnly.requireWitness=false;
    report=check_normal_scene3d(blocked,p,localOnly);
    assert(report.accepted && ~report.taskFeasible && strcmp(report.feasibilityStatus,'not_checked'));
    limited=rules; limited.maxExpandedNodes=1000; wall=scene;
    wall.obstacles=obstacle3d('box',[36;0;0],[44;80;30]);
    report=check_normal_scene3d(wall,p,limited);
    assert(report.localValid && ~report.accepted && ~report.taskFeasible);
    % Never silently remove obstacles or continue sampling beyond the budget.
    impossible=rules; impossible.terminalClearance=150; impossible.maxAttempts=3;
    [failed,m,audit]=normal_scene3d(900001,p,impossible);
    assert(strcmp(failed.kind,'generation_failed') && ~m.accepted && audit.attempts==3 && ...
        numel(audit.proposals)==3 && audit.rejectedMaps==3 && isnan(audit.acceptedProposalSeed));
    fixtureDir=fullfile(outputDir,'bounded_failure_fixture'); mkdir(fixtureDir);
    save_scene_generation3d(failed,fixtureDir);
    % Distance batching must agree with the scalar closed-solid geometry.
    points=[20 40 45;20 36 41;10 10 18];
    obstacles=[obstacle3d('sphere',[40;36;10],4), ...
        obstacle3d('cylinder',[40;36],5,18,26), ...
        obstacle3d('box',[36;20;18],[44;50;26])];
    batch=point_clearance_batch3d(points,obstacles);
    for k=1:size(points,2)
        values=arrayfun(@(o) obstacle_distance3d(points(:,k),o),obstacles);
        assert(abs(batch(k)-min(values))<1e-12);
    end
    rows=struct([]); totalAttempts=0;
    for seed=3001:3012
        fprintf('NORMAL_MAP_TEST seed=%d\n',seed);
        q=p; q.seed=seed;
        [generated,metadata,audit]=normal_scene3d(seed,q);
        assert(metadata.accepted && metadata.check.accepted && metadata.check.taskFeasible);
        assert(metadata.check.initialClearance>=rules.startClearance && ...
            metadata.check.terminalClearance>=rules.terminalClearance && ...
            metadata.check.departureClearanceLowerBound>=rules.departureClearance && ...
            metadata.check.arrivalClearanceLowerBound>=rules.departureClearance);
        witness=metadata.check.witness; offsets=witness.offsets;
        assert(witness.duration<=q.Tmax && max(witness.segmentPeakSpeeds)<=q.vmax && ...
            witness.maxAcceleration<=q.amax && witness.minPairDistance>q.uavSafe);
        assert(norm(witness.points(:,1)-q.start)<1e-12 && norm(witness.points(:,end)-q.target)<1e-12);
        for k=1:size(witness.points,2)-1
            a=zeros(6,5); b=a;
            a(1:3,:)=witness.points(:,k)+offsets; b(1:3,:)=witness.points(:,k+1)+offsets;
            [hit,clearance,pair]=check_swept3d(a,b,generated.obstacles,q);
            assert(~hit && clearance>=rules.witnessClearance-1e-7 && pair>q.uavSafe);
            length=norm(witness.points(:,k+1)-witness.points(:,k));
            peak=witness.segmentPeakSpeeds(k); duration=witness.segmentDurations(k);
            assert(abs(length-(peak*duration-peak^2/q.amax))<1e-9);
        end
        if seed==3001
            first=generated; again=normal_scene3d(seed,q); assert(isequaln(first,again));
            changed=q; changed.pathClearance=5.6; changed.desiredClear=1.2;
            changed.heightLayers=[4 8 12 16 24];
            assert(isequaln(first,normal_scene3d(seed,changed)), ...
                'Environment rules must not move with tuned planner/reference margins.');
            assert(isequaln(first,scenario3d('random',seed,q)));
            assert(isequaln(first,unstructured_scene3d(seed,q)));
        end
        save(fullfile(mapsDir,sprintf('seed_%06d.mat',seed)),'generated','metadata','audit','q','-v7.3');
        rows=[rows;struct('seed',seed,'attempts',audit.attempts, ...
            'rejectedMaps',audit.rejectedMaps,'obstacleCount',metadata.obstacleCount, ...
            'initialClearance',metadata.check.initialClearance,'terminalClearance',metadata.check.terminalClearance, ...
            'departureLowerBound',metadata.check.departureClearanceLowerBound, ...
            'arrivalLowerBound',metadata.check.arrivalClearanceLowerBound, ...
            'unionOccupancyEstimate',metadata.estimatedUnionVolumeFraction, ...
            'witnessDuration',witness.duration,'witnessExpandedNodes',witness.expandedNodes, ...
            'witnessPathLength',witness.pathLength)]; %#ok<AGROW>
        totalAttempts=totalAttempts+audit.attempts;
    end
    writetable(struct2table(rows),fullfile(outputDir,'normal_map_checks.csv'));
    summary=struct('passed',true,'acceptedMaps',12,'seedRange',[3001 3012], ...
        'proposals',totalAttempts,'rejectedMaps',totalAttempts-12, ...
        'sampleGenerationAcceptancePercent',100*12/totalAttempts, ...
        'flightSuccessRate','Not measured by geometry tests.');
    file=fopen(fullfile(outputDir,'focused_test_receipt.json'),'w','n','UTF-8');
    fprintf(file,'%s',jsonencode(summary,PrettyPrint=true)); fclose(file);
    fprintf('NORMAL_SCENE_TEST_PASSED maps=12 proposals=%d rejected=%d\n',totalAttempts,totalAttempts-12);
end
