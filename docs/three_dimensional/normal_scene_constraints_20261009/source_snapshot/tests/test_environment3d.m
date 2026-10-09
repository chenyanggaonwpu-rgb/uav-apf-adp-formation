function test_environment3d()
% TEST_ENVIRONMENT3D Closed box geometry, suspended solids and layer gaps.
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path; cleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'));
    p=default_params3d();
    p.pathClearance=4.2; % Historical layer-gap fixture, independent of current defaults.
    p.directionalClearance.planningZ=p.pathClearance;
    p=validate_params3d(p);
    p.adaptiveHeight.enable=false; % Retain the historical fixed-layer negative control.
    b=obstacle3d('box',[36;0;20],[44;80;30]);
    [d,n]=obstacle_distance3d([45;40;25],b);
    assert(abs(d-1)<1e-10 && norm(n-[1;0;0])<1e-10);
    [d,n]=obstacle_distance3d([40;40;18],b);
    assert(abs(d-2)<1e-10 && norm(n-[0;0;-1])<1e-10);
    assert(abs(obstacle_distance3d([45;81;31],b)-sqrt(3))<1e-10);
    assert(abs(obstacle_distance3d([40;40;25],b)+4)<1e-10);
    assert(abs(segment_clearance3d([20;40;25],[60;40;25],b)+4)<1e-5, ...
        'A swept crossing must see the interior, not just safe endpoints.');
    assert(abs(segment_clearance3d([20;40;18],[60;40;18],b)-2)<1e-5);
    c=obstacle3d('cylinder',[40;36],5,18,26);
    [d,n]=obstacle_distance3d([40;36;10],c);
    assert(abs(d-8)<1e-10 && norm(n-[0;0;-1])<1e-10);
    assert(segment_clearance3d([40;36;10],[40;36;28],c)<0);
    roof=scenario3d('S7_underpass',25,p);
    [route,info]=plan_layers3d(p.start,p.target,roof.obstacles,p);
    assert(strcmp(info.status,'direct') && all(route(3,:)==10));
    assert(segment_clearance3d([5;40;25],[75;40;25],roof.obstacles)<0, ...
        'The highest layer must be blocked by a suspended roof.');
    window=scenario3d('S8_window',25,p);
    [route,info,cache]=plan_layers3d(p.start,p.target,window.obstacles,p);
    assert(strcmp(info.status,'path') && any(route(3,:)==15));
    assert_route([p.start,route],window.obstacles,p.pathClearance);
    [again,~]=plan_layers3d(p.start,p.target,window.obstacles,p, ...
        p.pathClearance,p.pathClearance,cache);
    assert(isequal(route,again));
    q=p; q.verticalCostWeight=p.verticalCostWeight+1;
    mismatch=false;
    try
        plan_layers3d(p.start,p.target,window.obstacles,q, ...
            p.pathClearance,p.pathClearance,cache);
    catch exception
        mismatch=strcmp(exception.identifier,'UAV3D:PlannerCache');
    end
    assert(mismatch,'A graph with stale altitude penalties must not be reused.');
    gap=scenario3d('S10_layer_gap',25,p);
    [route,info]=plan_layers3d(p.start,p.target,gap.obstacles,p);
    assert(isempty(route) && strcmp(info.status,'no_path'));
    witness=[p.start,[30;30;13],[50;44;13],p.target];
    assert_route(witness,gap.obstacles,p.pathClearance);
    q=p; q.heightLayers=sort([p.heightLayers 13]);
    [route,info]=plan_layers3d(q.start,q.target,gap.obstacles,q);
    assert(strcmp(info.status,'path') && any(route(3,:)==13));
    assert_route([q.start,route],gap.obstacles,q.pathClearance);
    blocked=scenario3d('S11_blocked',25,p);
    [route,info]=plan_layers3d(p.start,p.target,blocked.obstacles,p);
    assert(isempty(route) && strcmp(info.status,'no_path'));
    for seed=1:4
        a=scenario3d('random_structured',seed,p); bscene=scenario3d('random_structured',seed,p);
        assert(isequaln(a,bscene) && strcmp(a.kind,'random') && ...
            numel(a.obstacles)==p.randomObstacleCount);
        if seed>1
            assert(segment_clearance3d([5;40;25],[75;40;25],a.obstacles)<0);
        end
    end
    mixed=scenario3d('random_structured',1,p);
    assert(all(ismember({'sphere','cylinder','box'},{mixed.obstacles.type})));
    assert(any(arrayfun(@(o) strcmp(o.type,'cylinder') && o.zmin>0,mixed.obstacles)));
    [state,offsets,psi]=init_formation3d(p);
    state(1:3,:)=state(1:3,:)+[38;30;12];
    memory=struct('psi',psi,'hold',true,'holdPsi',0,'holdAlpha',1, ...
        'holdStartRefs',zeros(3,5),'offsets',offsets);
    localBox=obstacle3d('box',[36;20;18],[44;50;26]);
    [refs,~,~,isBlocked]=formation_reference3d(state,memory,localBox,p);
    assert(~isBlocked && all(refs(3,:)==22), ...
        'Horizontal reference repair must keep synchronized height and clear the box.');
    for i=2:5
        assert(obstacle_distance3d(refs(:,i),localBox)>=p.desiredClear-1e-6);
    end
    fprintf('3-D environment tests passed: roof, window, fixed-layer gap and closed-solid checks.\n');
end

function assert_route(points,obstacles,clearance)
    for i=1:size(points,2)-1
        assert(segment_clearance3d(points(:,i),points(:,i+1),obstacles)>=clearance-1e-6);
    end
end
