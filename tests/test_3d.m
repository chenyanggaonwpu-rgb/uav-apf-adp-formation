function test_3d()
% TEST_3D Geometric, planning, dynamics and end-to-end 3-D checks.
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path;
    cleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'));
    p=default_params3d();
    p=validate_params3d(p);
    assert(isequal(size(p.adp.wA),[3 6]) && numel(p.adp.wA)+numel(p.adp.wC)==24);
    sphereScene=scenario3d('S1_sphere',25,p);
    sphereObstacle=sphereScene.obstacles;
    assert(abs(obstacle_distance3d([45;36;10],sphereObstacle)-1)<1e-10);
    assert(segment_clearance3d([35;36;10],[45;36;10],sphereObstacle)<0, ...
        'A segment that passes through a sphere must be rejected.');
    cyl=scenario3d('S2_cylinder',25,p);
    c=cyl.obstacles;
    assert(abs(obstacle_distance3d([46;36;10],c)-1)<1e-10);
    assert(abs(obstacle_distance3d([40;36;18],c)-2)<1e-10);
    assert(abs(obstacle_distance3d([46;36;18],c)-sqrt(5))<1e-10);
    assert(segment_clearance3d([40;36;20],[40;36;-2],c)<0, ...
        'The swept segment must catch a cylinder top-to-bottom crossing.');
    old=[15 19;10 10;10 10;0 0;0 0;0 0];
    new=old;new(1,:)=[19 15];
    [hit,~,pair]=check_swept3d(old,new,struct('type',{},'center',{}, ...
        'r',{},'zmin',{},'zmax',{}),p);
    assert(hit && pair<1e-9,'Synchronous inter-UAV crossing must be caught.');
    [next,actual]=motion_step3d(zeros(6,1),[3;3;3],zeros(3,1),p);
    assert(norm(actual)<=p.amax+1e-12 && norm(next(4:6))<=p.vmax+1e-12);
    [nearState,~,~]=init_formation3d(p);
    nearState(:,1)=[45.2;36;10;-3;0;0];
    [repairCommand,repairInfo]=resolve_safety3d(nearState,zeros(3,5), ...
        sphereObstacle,p);
    assert(repairInfo.triggered && repairInfo.predictedSafe && repairCommand(1,1)>0, ...
        'A repairable approach must trigger a bounded outward correction.');
    [unsafeState,~,~]=init_formation3d(p);
    unsafeState(:,1)=[44.65;36;10;-3;0;0];
    [safeCommand,safetyInfo]=resolve_safety3d(unsafeState,zeros(3,5), ...
        sphereObstacle,p);
    predicted=motion_step3d(unsafeState,safeCommand,zeros(3,5),p);
    [predictedHit,~,~,~]=check_swept3d(unsafeState,predicted, ...
        sphereObstacle,p);
    assert(~safetyInfo.predictedSafe && predictedHit && ...
        safetyInfo.iterations<=5, ...
        'Unavoidable post-limit collision must be reported, not executed.');
    p.target=[40;36;10];
    [force,~,degenerate]=apf_repulsion3d([44.5;36;10],p.target, ...
        sphereObstacle,p);
    assert(degenerate && all(isfinite(force)), ...
        'Projected tangent degeneracy must have a finite braking path.');
    p=default_params3d();
    p.pathClearance=4.2; % Historical overpass and live-replan regression geometry.
    p.directionalClearance.planningZ=p.pathClearance;
    overpass=scenario3d('S3_overpass',25,p);
    [route,info,cache]=plan_layers3d(p.start,p.target,overpass.obstacles,p, ...
        p.pathClearance);
    assert(strcmp(info.status,'path') && max(route(3,:))>=20, ...
        'The finite-height wall should require an ascent.');
    [cachedRoute,cachedInfo]=plan_layers3d(p.start,p.target,overpass.obstacles, ...
        p,p.pathClearance,p.pathClearance,cache);
    assert(isequal(route,cachedRoute) && cachedInfo.buildSeconds==0, ...
        'Repeated searches must reuse the frozen static graph.');
    points=[p.start,route];
    for k=1:size(points,2)-1
        assert(segment_clearance3d(points(:,k),points(:,k+1), ...
            overpass.obstacles)>=p.pathClearance-1e-6, ...
            'Every simplified segment must meet the declared clearance.');
    end
    p.Tmax=60;
    result=run_simulation3d(p,scenario3d('S0_empty',25,p));
    assert(result.success && strcmp(result.reason,'success'));
    assert(all(isfinite(result.history.state),'all'));
    assert(max(vecnorm(squeeze(result.history.state(4:6,1,:))))<=p.vmax+1e-9);
    p.noiseStd=0.02;
    noisy=run_simulation3d(p,scenario3d('S1_sphere',25,p));
    assert(noisy.success && noisy.metrics.formationRms<4, ...
        'Measured position jitter must not accumulate as horizontal travel.');
    p.noiseStd=0;
    p.seed=18;
    p.verticalCostWeight=0; % This historical regression uses the 2.0.0 geometry/cost.
    replanCase=run_simulation3d(p,scenario3d('random_legacy',18,p));
    assert(replanCase.success && numel(replanCase.timing.replanSeconds)>=1, ...
        'A safe live state inside 4.2 m planning inflation must reconnect.');
    fprintf('3-D tests passed. Empty-map completion %.2f s; overpass %d waypoints.\n', ...
        result.time,size(route,2));
end
