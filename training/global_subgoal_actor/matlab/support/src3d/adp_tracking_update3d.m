function [p,info] = adp_tracking_update3d(oldState,newState,base,adpControl,obstacles,p,context)
% Online value evaluation and bounded greedy residual-policy fitting.
% Critic: PSD tracking quadratic plus obstacle/pair/boundary costs; zero start.
% Actor: 3x6 signed-error map, fitted to a one-step learned-value backup.
    info=struct('skipped',false,'predictionGain',0,'fitError',0);
    if ~p.adp.enable,return;end
    if context.safetyIntervened
        % The local policy model does not contain the safety repair operator.
        info.skipped=true;return;
    end
    nextContext=context;
    nextContext.position=context.position+p.dt*context.positionRate;
    nextContext.predictionState=motion_step3d(oldState,context.appliedCommand,zeros(3,p.nUAV),p);
    adp=p.adp;criticDelta=zeros(6,1);actorDelta=zeros(3,6);
    agents=1:p.nUAV;
    if isfield(adp,'controlLeader') && ~adp.controlLeader
        agents(agents==p.leaderIdx)=[];
    end
    numberAgents=numel(agents);
    if numberAgents==0,return;end
    for i=agents
        [oldPhi,actorPhi]=adp_tracking_features3d(oldState,i,context,p);
        newPhi=adp_tracking_features3d(newState,i,nextContext,p);
        baseline=limit_vector3d(base(:,i),p.amax);
        effective=context.appliedCommand(:,i)-baseline;
        cost=stage_cost(newState,i,newPhi,effective,obstacles,p);
        td=cost+adp.gamma*(adp.wC'*newPhi)-adp.wC'*oldPhi;
        if ~isfield(adp,'criticLearning') || adp.criticLearning
            criticDelta=criticDelta+adp.lrC*td*oldPhi/(adp.criticFeatureFloor+dot(oldPhi,oldPhi));
        end
        objective=@(u) backup(oldState,i,base(:,i),u,nextContext,obstacles,p);
        current=adpControl(:,i);center=objective(current);
        gradient=zeros(3,1);curvature=zeros(3,1);
        for ax=1:3
            offset=zeros(3,1);offset(ax)=adp.eps;
            upper=objective(current+offset);lower=objective(current-offset);
            gradient(ax)=(upper-lower)/(2*adp.eps);
            curvature(ax)=(upper-2*center+lower)/(adp.eps^2);
        end
        % Positive proximal curvature keeps a nonconvex/limited backup finite.
        floorCurvature=2*p.dt*adp.qCtrl;
        proposal=current-gradient./max(curvature,floorCurvature);
        proposal=adp_residual_limit3d(proposal,base(:,i),p);
        candidates=[zeros(3,1),current,proposal,0.5*(current+proposal)];
        values=zeros(1,4);
        for j=1:4,values(j)=objective(candidates(:,j));end
        best=min(values);eligible=find(values<=best+1e-12);
        [~,smallest]=min(vecnorm(candidates(:,eligible)));
        target=candidates(:,eligible(smallest));
        rawError=target/adp.gain-adp.wA*actorPhi;
        actorDelta=actorDelta+adp.lrA*rawError*actorPhi'/(adp.actorFeatureFloor+dot(actorPhi,actorPhi));
        info.predictionGain=info.predictionGain+max(0,values(1)-best)/numberAgents;
        info.fitError=info.fitError+norm(target-current)/numberAgents;
    end
    valueWeights=adp.wC+criticDelta/numberAgents;
    block=[valueWeights(1),valueWeights(3);valueWeights(3),valueWeights(2)];
    [vectors,diagonal]=eig(block);
    block=vectors*max(0,diagonal)*vectors';
    valueWeights(1:3)=[block(1,1);block(2,2);block(1,2)];
    valueWeights(4:6)=max(0,valueWeights(4:6));
    p.adp.wC=limit_weights(valueWeights,adp.maxWeightNorm);
    actorWeights=adp.wA+actorDelta/numberAgents;
    if ismember(adp.mode,{'tracking_value_v3','tracking_value_v4'})
        actorWeights=adp_project_actor3d(actorWeights);
    end
    p.adp.wA=limit_weights(actorWeights,adp.maxWeightNorm);
end

function value=backup(state,i,base,residual,context,obstacles,p)
    next=context.predictionState;
    residual=adp_residual_limit3d(residual,base,p);
    command=limit_vector3d(base+residual,p.amax);
    next(4:6,i)=limit_vector3d(state(4:6,i)+p.dt*command,p.vmax);
    next(1:3,i)=state(1:3,i)+p.dt*next(4:6,i);
    phi=adp_tracking_features3d(next,i,context,p);
    effective=command-limit_vector3d(base,p.amax);
    value=stage_cost(next,i,phi,effective,obstacles,p)+p.adp.gamma*(p.adp.wC'*phi);
end

function cost=stage_cost(~,~,phi,effective,~,p)
    cost=p.dt*(p.adp.qForm*phi(1)+p.adp.qVelocity*phi(2)+ ...
        p.adp.qRisk*sum(phi(4:6))+ ...
        p.adp.qCtrl*dot(effective,effective));
end

function w=limit_weights(w,maximum)
    if any(~isfinite(w),'all'),error('UAV3D:ADPDiverged','Nonfinite ADP weights.');end
    magnitude=norm(w(:));
    if magnitude>maximum,w=w*maximum/magnitude;end
end
