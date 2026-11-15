function local=dv2_near_obstacles(points,obstacles,bounds,radius)
% Preserve original obstacle order. Exclude only solids beyond ALL aircraft.
% radius >= 8 exceeds APF support (rho0=6), one-step repair range, and the
% <=1.6 m nominal braking path at vmax=amax=3, dt=.05. Global navigation,
% reference projection, observations and true collision checks use ALL solids.
    keep=false(1,numel(obstacles));
    for i=1:size(points,2)
        q=max(abs(points(:,i)-bounds.center)-bounds.half,0);
        keep=keep | sum(q.^2,1)<=(radius+1e-9)^2;
    end
    local=obstacles(keep);
end
