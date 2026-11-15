function [p,scene]=v4_decode_case(item,base)
    p=base;p.start=item.start(:);p.target=item.goal(:);p.workspace=double(item.workspace);
    p.Tmax=item.mission_seconds;p.vmax=item.vmax;p.amax=item.amax;p.seed=item.seed;
    p.noiseStd=item.noise_std;p.disturbanceMax=0;p.animate=false;p.adp.enable=false;
    p.v4WindBias=item.wind_bias(:);p.v4SensorDelaySteps=item.sensor_delay_steps;
    p.v4FilterAlpha=.15;
    p.v4WindIntervals=[0 inf];if isfield(item,'wind_intervals_seconds'),p.v4WindIntervals=double(item.wind_intervals_seconds);end
    % Prebuilding only changes caching; the same graphs are built on demand.
    p.planning.prebuildGraphs=false;
    p.heightLayers=unique([6:6:p.workspace(2,3)-3,p.start(3),p.target(3)]);
    p.v4AllowDirectPlannerFallback=true;
    obstacles=struct('type',{},'center',{},'r',{},'zmin',{},'zmax',{},'halfSize',{});
    raw=item.obstacles;if isstruct(raw),raw=num2cell(raw);end
    for k=1:numel(raw)
        o=raw{k};
        switch o.type
            case 'box',obstacles(end+1)=obstacle3d('box',o.center(:)-o.halfSize(:),o.center(:)+o.halfSize(:)); %#ok<AGROW>
            case 'sphere',obstacles(end+1)=obstacle3d('sphere',o.center(:),o.r); %#ok<AGROW>
            case 'cylinder',obstacles(end+1)=obstacle3d('cylinder',o.center(:),o.r,o.zmin,o.zmax); %#ok<AGROW>
        end
    end
    scene=struct('id',item.id,'kind','v4_complex_mixed','seed',item.seed,'obstacles',obstacles);
end
