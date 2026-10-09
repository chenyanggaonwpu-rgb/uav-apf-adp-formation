function test_adaptive_height3d()
% TEST_ADAPTIVE_HEIGHT3D Missing sections, nonuniform layers and safety checks.
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path; guard=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'));
    p=default_params3d();
    p.pathClearance=4.2; % Historical layer-gap fixture, independent of current defaults.
    p.directionalClearance.planningZ=p.pathClearance;
    p=validate_params3d(p); base=p.heightLayers;
    gap=scenario3d('S10_layer_gap',25,p);
    intervals=available_height_intervals3d([40;40],gap.obstacles,p);
    assert(max(abs(intervals-[12.2,13.8]),[],'all')<1e-9);
    selection=adaptive_height_layers3d(p,gap.obstacles);
    assert(isequal(selection.addedHeights,13) && isequal(selection.layers,[5 10 13 15 20 25]));
    disabled=p; disabled.adaptiveHeight.enable=false;
    [route,~]=plan_layers3d(p.start,p.target,gap.obstacles,disabled);
    assert(isempty(route),'The historical fixed-five-layer negative control must fail.');
    [route,info,cache]=plan_layers3d(p.start,p.target,gap.obstacles,p);
    assert(~isempty(route) && isequal(info.addedHeightLayers,13));
    assert(isequal(p.heightLayers,base),'A temporary layer must not mutate base configuration.');
    points=[p.start,route];
    for i=1:size(points,2)-1
        assert(segment_clearance3d(points(:,i),points(:,i+1),gap.obstacles)>=p.pathClearance-1e-6);
    end
    [again,againInfo]=plan_layers3d(p.start,p.target,gap.obstacles,p,p.pathClearance,p.pathClearance,cache);
    assert(isequal(route,again) && againInfo.buildSeconds==0 && isequal(againInfo.addedHeightLayers,13));
    changed=p; changed.adaptiveHeight.maxAddedLayers=7; caught=false;
    try
        plan_layers3d(p.start,p.target,gap.obstacles,changed,p.pathClearance,p.pathClearance,cache);
    catch ex
        caught=strcmp(ex.identifier,'UAV3D:PlannerCache');
    end
    assert(caught,'A changed adaptive policy must invalidate cached working layers.');
    covered=scenario3d('S8_window',25,p);
    [route,info]=plan_layers3d(p.start,p.target,covered.obstacles,p);
    assert(~isempty(route) && isempty(info.addedHeightLayers) && ~info.heightSelection.attempted);
    blocked=scenario3d('S11_blocked',25,p);
    [route,info]=plan_layers3d(p.start,p.target,blocked.obstacles,p);
    assert(isempty(route) && isempty(info.addedHeightLayers),'Insertion cannot open a sealed wall.');
    % A pillar hides the centre section; the same window remains open to either side.
    hidden=gap.obstacles; hidden(end+1)=obstacle3d('cylinder',[40;40],2,0,30);
    assert(isempty(available_height_intervals3d([40;40],hidden,p)));
    [route,info]=plan_layers3d(p.start,p.target,hidden,p);
    assert(~isempty(route) && any(abs(info.addedHeightLayers-13)<1e-8), ...
        'Off-centre free sections must remain visible when the footprint centre is blocked.');
    points=[p.start,route];
    for i=1:size(points,2)-1
        assert(segment_clearance3d(points(:,i),points(:,i+1),hidden)>=p.pathClearance-1e-6);
    end
    % Two distinct missed windows require two working heights, not one global centre.
    walls=[obstacle3d('box',[28;0;0],[34;80;7]),obstacle3d('box',[28;0;17],[34;80;30]), ...
        obstacle3d('box',[49;0;0],[55;80;13]),obstacle3d('box',[49;0;23],[55;80;30])];
    selection=adaptive_height_layers3d(p,walls);
    assert(any(abs(selection.addedHeights-12)<1e-8) && any(abs(selection.addedHeights-18)<1e-8));
    limited=p; limited.adaptiveHeight.maxAddedLayers=1;
    selected=adaptive_height_layers3d(limited,walls);
    assert(numel(selected.addedHeights)==1 && selected.budgetRejected>0);
    % Independent dense point classification checks the analytic local intervals.
    solids=[obstacle3d('sphere',[40;40;15],3), ...
        obstacle3d('cylinder',[40;40],3,10,20), ...
        obstacle3d('box',[37;37;10],[43;43;20])];
    heights=linspace(4.3,25.7,503);
    for directional=[false,true]
        q=p; q.directionalClearance.enable=directional;
        q.directionalClearance.planningZ=2; q.directionalClearance.referenceZ=2;
        for o=solids
            for xy={[40;40],[44;40],[46;40]}
                xy=xy{1}; bands=available_height_intervals3d(xy,o,q);
                admitted=false(size(heights));
                for j=1:size(bands,1)
                    admitted=admitted | (heights>=bands(j,1) & heights<=bands(j,2));
                end
                if directional
                    distances=directional_clearance3d([repmat(xy,1,numel(heights));heights],o,[q.pathClearance;q.pathClearance;2]);
                    expected=distances>=1;
                else
                    expected=arrayfun(@(z) obstacle_distance3d([xy;z],o)>=q.pathClearance,heights);
                end
                assert(isequal(admitted,expected),'Section intervals disagree with independent physical/margin distances.');
            end
        end
    end
    fprintf('ADAPTIVE_HEIGHT_TESTS_PASSED: gap=13 m, covered, sealed, two-window, cache, budget and all solids.\n');
end
