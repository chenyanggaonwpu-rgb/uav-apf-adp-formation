function signature=planning_graph_signature3d(obstacles,p,clearance)
% Only geometry/planning inputs belong to a static graph's signature.
% Online ADP weights and measured vehicle states must not invalidate it.
    vertical=0;if isfield(p,'verticalCostWeight'),vertical=p.verticalCostWeight;end
    directional=struct('enable',false);
    if isfield(p,'directionalClearance'),directional=p.directionalClearance;end
    adaptive=struct('enable',false);
    if isfield(p,'adaptiveHeight'),adaptive=p.adaptiveHeight;end
    signature=struct('obstacles',obstacles,'workspace',p.workspace, ...
        'gridResolution',p.pathGridResolution,'heightLayers',p.heightLayers, ...
        'graphClearance',clearance,'pathClearance',p.pathClearance, ...
        'verticalCostWeight',vertical,'directional',directional,'adaptive',adaptive, ...
        'selectionStart',p.start,'selectionTarget',p.target);
end
