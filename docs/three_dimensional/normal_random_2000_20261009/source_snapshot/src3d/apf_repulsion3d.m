function [rep,risk,degenerate] = apf_repulsion3d(position,goal,obstacles,p)
% APF_REPULSION3D Surface-normal repulsion and projected path tangent.
    rep=zeros(3,1);risk=0;degenerate=false;
    toGoal=goal-position;
    goalDistance=norm(toGoal);
    factor=max(p.minGoalRepFactor,1-exp(-p.goalDecay*goalDistance^2));
    for k=1:numel(obstacles)
        [d,normal]=obstacle_distance3d(position,obstacles(k));
        if d>p.rho0, continue; end
        risk=max(risk,max(0,min(1,1-d/p.rho0)));
        magnitude=p.kRep*max(0,(p.rho0-d)/p.rho0)*factor;
        rep=rep+magnitude*normal;
        if p.useTangential && d>0 && d<p.rho0/2 && goalDistance>1e-3
            projected=toGoal-dot(toGoal,normal)*normal;
            if norm(projected)<1e-6
                projected=p.target-position;
                projected=projected-dot(projected,normal)*normal;
            end
            if norm(projected)<1e-6
                degenerate=true; % No arbitrary 3-D rotation: brake/replan upstream.
            else
                rep=rep+p.kTang*(1-d/p.rho0)*projected/norm(projected);
            end
        end
    end
end
