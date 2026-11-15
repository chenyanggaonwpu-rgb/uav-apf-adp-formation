function [point,info]=rl_project_subgoal(action,position,previous,obstacles,p,settings)
% Geometry-only feasibility repair. No route search or target heuristic.
    raw=double(action(:));assert(numel(raw)==3&&all(isfinite(raw)));
    lo=p.workspace(1,:)';hi=p.workspace(2,:)';
    requested=lo+(max(-1,min(1,raw))+1).*.5.*(hi-lo);
    clearance=max(settings.projection_clearance,p.obsSafe+.05);
    low=lo+clearance+1e-5;high=hi-clearance-1e-5;
    point=max(low,min(high,requested));method='identity';
    if norm(point-requested)>1e-8,method='boundary_clip';end
    near=norm(point-position)<settings.minimum_subgoal_distance;
    if near
        direction=point-position;
        if norm(direction)<1e-8,direction=raw;end
        if norm(direction)<1e-8,direction=[1;0;0];end
        point=max(low,min(high,position+settings.minimum_subgoal_distance*direction/norm(direction)));
        method='near_point_repair';
    end
    for iteration=1:48
        minimum=inf;normal=zeros(3,1);
        for o=obstacles
            [distance,outward]=obstacle_distance3d(point,o);
            if distance<minimum,minimum=distance;normal=outward;end
        end
        if minimum>=clearance,break;end
        point=max(low,min(high,point+(clearance-minimum+1e-4)*normal));method='signed_distance_projection';
    end
    if point_clearance_batch3d(point,obstacles)<clearance || norm(point-position)<settings.minimum_subgoal_distance-1e-6
        % Nearest feasible coarse point to the request; does not rank by goal.
        [x,y,z]=ndgrid(low(1):4:high(1),low(2):4:high(2),low(3):4:high(3));
        candidates=[x(:)';y(:)';z(:)'];
        valid=point_clearance_batch3d(candidates,obstacles)>=clearance & ...
            vecnorm(candidates-position)>=settings.minimum_subgoal_distance;
        candidates=candidates(:,valid);assert(~isempty(candidates),'Planner:NoFeasibleSubgoal');
        [~,index]=min(sum((candidates-requested).^2,1));point=candidates(:,index);method='nearest_feasible_grid_point';
    end
    assert(all(point>=low)&all(point<=high)&&point_clearance_batch3d(point,obstacles)>=clearance-1e-8);
    repeated=all(isfinite(previous))&&norm(point-previous)<.5;
    info=struct('raw_action',raw,'requested',requested,'executed',point, ...
        'projection_distance',norm(point-requested),'method',method,'near_requested',near,'repeated',repeated);
end
