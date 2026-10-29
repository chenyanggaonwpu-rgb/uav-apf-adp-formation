function audit=f5_audit(result,item,cfg)
% Independent batch audit of saved executed states, commands and swept paths.
    if ~isfield(result.history,'state') || isempty(result.history.state)
        audit=struct('steps',0,'passed',true,'dynamicsError',0,'minimumCertifiedMargin',NaN);return;
    end
    p=item.parameters;h=result.history;[initial,~,~]=init_formation3d(p);
    old=cat(3,initial,h.state(:,:,1:end-1));next=h.state;
    velocity=old(4:6,:,:)+p.dt*h.command;
    lengths=sqrt(sum(velocity.^2,1));velocity=velocity.*min(1,p.vmax./max(lengths,eps));
    expected=old;expected(4:6,:,:)=velocity;expected(1:3,:,:)=old(1:3,:,:)+p.dt*velocity;
    error=max(abs(expected-next),[],'all');assert(error<1e-10,'F5:DynamicsAudit');
    assert(max(sqrt(sum(h.command.^2,1)),[],'all')<=p.amax+1e-10);
    assert(max(sqrt(sum(next(4:6,:,:).^2,1)),[],'all')<=p.vmax+1e-10);
    a=reshape(old(1:3,:,:),3,[]);b=reshape(next(1:3,:,:),3,[]);
    lo=p.workspace(1,:)'+cfg.safety.boundaryBuffer;hi=p.workspace(2,:)'-cfg.safety.boundaryBuffer;
    assert(all(a>lo & a<hi,'all') && all(b>lo & b<hi,'all'));
    lower=point_clearance_batch3d((a+b)/2,item.scene.obstacles)-vecnorm(b-a)/2-p.obsSafe;
    uncertain=find(lower<=cfg.safety.tolerance);margin=min(lower);
    for index=uncertain
        [ok,m]=adp3d_certify_segment(a(:,index),b(:,index),item.scene.obstacles,p,cfg.safety);
        assert(ok,'F5:ObstacleAudit');lower(index)=m;
    end
    margin=min(lower);
    for i=1:p.nUAV-1
        for j=i+1:p.nUAV
            r=reshape(old(1:3,i,:)-old(1:3,j,:),3,[]);
            dr=reshape(next(1:3,i,:)-next(1:3,j,:),3,[])-r;
            alpha=max(0,min(1,-sum(r.*dr,1)./max(sum(dr.^2,1),eps)));
            pairMargin=min(vecnorm(r+dr.*alpha))-p.uavSafe;
            assert(pairMargin>cfg.safety.tolerance,'F5:PairAudit');margin=min(margin,pairMargin);
        end
    end
    audit=struct('steps',numel(h.t),'passed',true,'dynamicsError',error,'minimumCertifiedMargin',margin);
end
