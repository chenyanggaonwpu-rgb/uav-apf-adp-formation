function metrics=route_cost3d(points,p)
% ROUTE_COST3D Geometric travel proxies, not battery energy or rotor power.
    weight=0;
    if isfield(p,'verticalCostWeight'), weight=p.verticalCostWeight; end
    delta=diff(points,1,2);
    dz=delta(3,:);
    metrics=struct('pathLength',sum(vecnorm(delta)), ...
        'ascent',sum(max(dz,0)),'descent',sum(max(-dz,0)), ...
        'verticalTravel',sum(abs(dz)),'cost',0);
    metrics.cost=metrics.pathLength+weight*metrics.verticalTravel;
end
