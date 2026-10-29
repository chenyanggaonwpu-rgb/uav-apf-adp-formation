function [criticPhi,actorPhi] = adp_tracking_features3d(state,i,context,p)
% Signed local tracking coordinates for the Actor; quadratic value basis.
% The transition uses one control objective, including terminal waypoints.
    errorPosition=(context.position(:,i)-state(1:3,i))/p.formErrScale;
    if ~context.trackPosition(i),errorPosition=zeros(3,1);end
    errorVelocity=(context.velocity(:,i)-state(4:6,i))/p.vmax;
    point=state(1:3,i);clearance=inf;
    for j=1:numel(context.obstacles)
        clearance=min(clearance,obstacle_distance3d(point,context.obstacles(j)));
    end
    obsRisk=max(0,1-max(clearance,0)/p.rho0);
    others=state(1:3,:);others(:,i)=[];
    pairRisk=max(0,1-min(vecnorm(point-others))/p.uavAvoidSafe);
    boundary=min([point-p.workspace(1,:)';p.workspace(2,:)'-point]);
    boundaryRisk=max(0,1-max(boundary,0)/(p.obsSafe+p.safety.margin));
    criticPhi=[dot(errorPosition,errorPosition);dot(errorVelocity,errorVelocity); ...
        2*dot(errorPosition,errorVelocity);obsRisk^2;pairRisk^2;boundaryRisk^2];
    % Smooth compression bounds the policy inputs without a flat cutoff.
    actorPhi=[errorPosition/sqrt(1+dot(errorPosition,errorPosition)); ...
        errorVelocity/sqrt(1+dot(errorVelocity,errorVelocity))];
end
