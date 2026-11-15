function bank=prepare_planning_graphs3d(obstacles,p,clearance,previous)
% PREPARE_PLANNING_GRAPHS3D Prepare static base/fallback graphs before control.
% Base and adaptive graphs stay separate: preparing extra layers must not
% make them active until the same legacy base-plan failure occurs.
    if nargin<4,previous=[];end
    if isstruct(previous) && isfield(previous,'preparedGraphs')
        bank=previous;
        if ~isequaln(bank.signature,planning_graph_signature3d(obstacles,p,clearance))
            error('UAV3D:PlannerCache','Prepared graph configuration or map changed.');
        end
        plan_fixed_layers3d(p.start,p.target,obstacles,p,clearance,clearance,bank.baseCache,true);
        validate_policy(bank,p);
        return;
    end
    enabled=isfield(p,'adaptiveHeight') && p.adaptiveHeight.enable;
    bank=struct('preparedGraphs',true,'baseCache',[],'adaptiveCache',[], ...
        'adaptiveSelection',[],'activeAdaptive',false,'heightSelection',[], ...
        'adaptiveEnabled',enabled,'adaptiveOptions',[],'selectionStart',p.start, ...
        'selectionTarget',p.target,'signature',planning_graph_signature3d(obstacles,p,clearance));
    if enabled,bank.adaptiveOptions=p.adaptiveHeight;end
    previousAdaptive=~isempty(previous) && isfield(previous,'heightSelection') && ...
        ~isempty(previous.heightSelection.addedHeights);
    base=[];
    if ~previousAdaptive,base=previous;end
    [~,~,bank.baseCache]=plan_fixed_layers3d(p.start,p.target,obstacles,p, ...
        clearance,clearance,base,true);
    if enabled
        bank.adaptiveSelection=adaptive_height_layers3d(p,obstacles,clearance);
        if ~isempty(bank.adaptiveSelection.addedHeights)
            local=p;local.heightLayers=bank.adaptiveSelection.layers;
            adaptive=[];
            if previousAdaptive,adaptive=previous;end
            [~,~,bank.adaptiveCache]=plan_fixed_layers3d(p.start,p.target,obstacles, ...
                local,clearance,clearance,adaptive,true);
        end
    end
    if previousAdaptive
        if ~enabled || ~isequal(previous.heightSelection.layers,bank.adaptiveSelection.layers)
            error('UAV3D:PlannerCache','Adaptive height policy changed.');
        end
        bank.activeAdaptive=true;bank.heightSelection=previous.heightSelection;
    end
end

function validate_policy(bank,p)
    enabled=isfield(p,'adaptiveHeight') && p.adaptiveHeight.enable;
    if bank.adaptiveEnabled~=enabled || ...
            (enabled && (~isequal(bank.adaptiveOptions,p.adaptiveHeight) || ...
            ~isequal(bank.selectionStart,p.start) || ~isequal(bank.selectionTarget,p.target)))
        error('UAV3D:PlannerCache','Prepared adaptive policy changed.');
    end
end
