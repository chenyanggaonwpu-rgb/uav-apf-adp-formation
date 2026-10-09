function p=adp_update3d(oldState,newState,base,adpControl,oldRefs,newRefs,obstacles,p)
% ADP_UPDATE3D Shared 6-feature critic and 3-axis actor, reset per map.
    if ~p.adp.enable, return; end
    adp=p.adp;
    criticDelta=zeros(6,1);actorDelta=zeros(3,6);
    for i=1:p.nUAV
        oldPhi=adp_features3d(oldState,i,oldRefs,obstacles,p);
        newPhi=adp_features3d(newState,i,newRefs,obstacles,p);
        stageCost=adp.qRisk*oldPhi(1)+adp.qForm*oldPhi(4)+ ...
            adp.qCtrl*dot(adpControl(:,i),adpControl(:,i))+adp.qTime*p.dt;
        td=stageCost+adp.gamma*(adp.wC'*newPhi)-adp.wC'*oldPhi;
        scale=1+dot(oldPhi,oldPhi);
        criticDelta=criticDelta+adp.lrC*td*oldPhi/scale;
        gradient=zeros(3,1);
        for ax=1:3
            perturb=zeros(3,1);perturb(ax)=adp.eps;
            plus=one_agent_step(oldState,i,base(:,i)+adpControl(:,i)+perturb,p);
            minus=one_agent_step(oldState,i,base(:,i)+adpControl(:,i)-perturb,p);
            fp=adp_features3d(plus,i,newRefs,obstacles,p);
            fm=adp_features3d(minus,i,newRefs,obstacles,p);
            gradient(ax)=adp.gamma*(adp.wC'*(fp-fm))/(2*adp.eps);
        end
        gradient=gradient+2*adp.qCtrl*adpControl(:,i);
        actorDelta=actorDelta-adp.lrA*adp.gain*gradient*oldPhi'/scale;
    end
    p.adp.wC=limit_weights(adp.wC+criticDelta/p.nUAV,adp.maxWeightNorm);
    p.adp.wA=limit_weights(adp.wA+actorDelta/p.nUAV,adp.maxWeightNorm);
end

function next=one_agent_step(state,i,control,p)
    next=state;
    command=limit_vector3d(control,p.amax);
    next(4:6,i)=limit_vector3d(state(4:6,i)+p.dt*command,p.vmax);
    next(1:3,i)=state(1:3,i)+p.dt*next(4:6,i);
end

function w=limit_weights(w,maximum)
    if any(~isfinite(w),'all')
        error('UAV3D:ADPDiverged','ADP weights became nonfinite.');
    end
    magnitude=norm(w(:));
    if magnitude>maximum, w=w*maximum/magnitude; end
end
