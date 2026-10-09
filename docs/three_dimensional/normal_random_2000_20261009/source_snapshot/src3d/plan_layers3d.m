function [waypoints,info,cache]=plan_layers3d(startPoint,goalPoint,obstacles,p,clearance,startJoinClearance,cache)
% PLAN_LAYERS3D Fixed-layer planning with bounded missing-interval fallback.
% Working layers are local to this call/map; p.heightLayers is not modified.
    if nargin<5, clearance=p.pathClearance; end
    if nargin<6, startJoinClearance=clearance; end
    if nargin<7, cache=[]; end
    enabled=isfield(p,'adaptiveHeight') && p.adaptiveHeight.enable;
    base=reshape(p.heightLayers,1,[]);
    selection=struct('enable',enabled,'attempted',false,'baseLayers',base, ...
        'layers',base,'addedHeights',zeros(1,0),'probeXY',zeros(2,0), ...
        'sections',zeros(0,4),'decisions',strings(0,1),'missingIntervals',0, ...
        'budgetRejected',0,'maxAddedLayers',0,'intervalTolerance',1e-7, ...
        'fixedStatus','not_run','policyVersion','midpoint_footprint_v2');
    if enabled
        selection.maxAddedLayers=p.adaptiveHeight.maxAddedLayers;
        selection.intervalTolerance=p.adaptiveHeight.intervalTolerance;
    end
    cachedAdaptive=enabled && ~isempty(cache) && isfield(cache,'heightSelection') && ...
        ~isempty(cache.heightSelection.addedHeights);
    if cachedAdaptive
        selection=cache.heightSelection;
        if ~isfield(selection,'policyVersion') || ~strcmp(selection.policyVersion,'midpoint_footprint_v2') || ...
                ~isequal(selection.baseLayers,base) || ...
                selection.maxAddedLayers~=p.adaptiveHeight.maxAddedLayers || ...
                selection.intervalTolerance~=p.adaptiveHeight.intervalTolerance
            error('UAV3D:PlannerCache','Adaptive height policy changed.');
        end
        local=p; local.heightLayers=selection.layers;
        [waypoints,info,cache]=plan_fixed_layers3d(startPoint,goalPoint,obstacles, ...
            local,clearance,startJoinClearance,cache);
    else
        [waypoints,info,cache]=plan_fixed_layers3d(startPoint,goalPoint,obstacles, ...
            p,clearance,startJoinClearance,cache);
        selection.fixedStatus=info.status;
        if enabled && isempty(waypoints) && ~strcmp(info.status,'connector_out_of_bounds')
            firstBuild=info.buildSeconds; firstSearch=info.searchSeconds;
            selection=adaptive_height_layers3d(p,obstacles,clearance);
            selection.fixedStatus=info.status;
            if ~isempty(selection.addedHeights)
                local=p; local.heightLayers=selection.layers;
                [waypoints,info,cache]=plan_fixed_layers3d(startPoint,goalPoint,obstacles, ...
                    local,clearance,startJoinClearance,[]);
                info.buildSeconds=info.buildSeconds+firstBuild;
                info.searchSeconds=info.searchSeconds+firstSearch;
            end
        end
    end
    info.heightSelection=selection;
    info.heightLayers=selection.layers; info.addedHeightLayers=selection.addedHeights;
    if ~isempty(cache), cache.heightSelection=selection; end
end
