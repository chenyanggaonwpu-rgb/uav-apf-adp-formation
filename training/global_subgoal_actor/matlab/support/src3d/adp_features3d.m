function phi=adp_features3d(state,i,refs,obstacles,p)
% ADP_FEATURES3D Six interpretable scalar features, as in the 2-D model.
    position=state(1:3,i);velocity=state(4:6,i);
    clearance=inf;
    for k=1:numel(obstacles)
        clearance=min(clearance,obstacle_distance3d(position,obstacles(k)));
    end
    risk=max(0,min(1,1-max(clearance,0)/p.rho0));
    distance=norm(p.target-position);
    goal=min(1,distance/p.goalScale);
    speed=min(1,norm(velocity)/p.vmax);
    formation=0;
    if i>1, formation=min(1,norm(position-refs(:,i))/p.formErrScale); end
    alignment=0;
    if norm(velocity)>0.05 && distance>0.05
        alignment=0.5*(1-dot(velocity,p.target-position)/ ...
            (norm(velocity)*distance));
    end
    phi=[risk;goal;speed;formation;alignment;1];
end
