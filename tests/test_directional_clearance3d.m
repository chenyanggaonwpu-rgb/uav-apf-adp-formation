function test_directional_clearance3d()
% TEST_DIRECTIONAL_CLEARANCE3D Physical geometry, margin slices and regressions.
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path; guard=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'));
    p=default_params3d();
    p.adaptiveHeight.enable=false; % Preserve the fixed-layer directional-margin contrast.
    s=obstacle3d('sphere',[40;40;15],3);
    c=obstacle3d('cylinder',[40;40],3,10,20);
    b=obstacle3d('box',[37;37;10],[43;43;20]);
    points=[46 40 43 40;40 40 40 40;15 21 15 15];
    for o=[s,c,b]
        actual=directional_clearance3d(points,o,[2;2;2]);
        expected=arrayfun(@(i) obstacle_distance3d(points(:,i),o)/2,1:size(points,2));
        assert(max(abs(actual-expected))<1e-9,'Isotropic margins must recover physical distance scaling.');
    end
    assert(abs(directional_clearance3d([40;40;19],s,[4;4;1])-1)<1e-8);
    assert(abs(directional_clearance3d([47;40;15],s,[4;4;1])-1)<1e-8);
    assert(directional_clearance3d([40;40;18.5],s,[4;4;1])<1);
    assert(segment_directional_clearance3d([20;40;15],[60;40;15],s,[4;4;1])<0);
    assert(segment_directional_clearance3d([20;40;19.1],[60;40;19.1],s,[4;4;1])>1);
    rng(9123,'twister'); a=[30;30;8]+20*rand(3,24); ends=[30;30;8]+20*rand(3,24);
    for o=[s,c,b]
        batch=directional_segments_free3d(a,ends,o,[3;3;0.8]);
        exact=arrayfun(@(i) segment_directional_clearance3d(a(:,i),ends(:,i),o,[3;3;0.8]),1:24);
        assert(all(batch'==(exact>=1-1e-9)),'Batched certificates must agree with independent swept minimization.');
    end
    for o=[s,c,b]
        for z=[8.9 12 15 21.1]
            f=reference_footprints3d(o,z,3,0.8);
            if isempty(f), continue; end
            if strcmp(f.type,'circle')
                point=[f.center+[f.padding+1e-6;0];z];
            else
                point=[f.center+[f.half(1)+f.padding+1e-6;0];z];
            end
            assert(directional_clearance3d(point,o,[3;3;0.8])>=1-2e-6, ...
                'Reference footprint must agree with the planning dilation.');
        end
    end
    p.pathClearance=4.2; % Historical isotropic-versus-directional layer-gap contrast.
    p.directionalClearance.planningZ=p.pathClearance;
    scene=scenario3d('S10_layer_gap',25,p);
    [oldRoute,~]=plan_layers3d(p.start,p.target,scene.obstacles,p);
    assert(isempty(oldRoute));
    q=p; q.directionalClearance.enable=true;
    q.directionalClearance.planningZ=2; q.directionalClearance.referenceZ=2;
    [route,info,cache]=plan_layers3d(q.start,q.target,scene.obstacles,q);
    assert(~isempty(route) && any(route(3,:)==10 | route(3,:)==15));
    assert(info.graphClearance==p.pathClearance);
    trajectory=[q.start,route];
    for i=1:size(trajectory,2)-1
        assert(segment_directional_clearance3d(trajectory(:,i),trajectory(:,i+1), ...
            scene.obstacles,[q.pathClearance;q.pathClearance;2])>=1-1e-7);
    end
    window=scenario3d('S8_window',25,p);
    [~,~,cache]=plan_layers3d(q.start,q.target,window.obstacles,q);
    assert(~isempty(cache));
    changed=q; changed.directionalClearance.planningZ=1;
    caught=false;
    try
        plan_layers3d(q.start,q.target,window.obstacles,changed, ...
            changed.pathClearance,changed.pathClearance,cache);
    catch exception
        caught=strcmp(exception.identifier,'UAV3D:PlannerCache');
    end
    assert(caught,'Changing Z margin must invalidate the graph cache.');
    scene=scenario3d('S11_blocked',25,p);
    q.pathClearance=0.61; q.desiredClear=0.61;
    q.directionalClearance.planningZ=0.61; q.directionalClearance.referenceZ=0.61;
    [route,~]=plan_layers3d(q.start,q.target,scene.obstacles,q);
    assert(isempty(route),'A sealed map must remain sealed at the smallest admissible margins.');
    fprintf('DIRECTIONAL_CLEARANCE_TESTS_PASSED\n');
end
