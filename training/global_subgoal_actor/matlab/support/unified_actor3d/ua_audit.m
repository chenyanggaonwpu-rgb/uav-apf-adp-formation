function audit=ua_audit(result)
% Vectorized independent reconstruction of every stored true plant step.
    h=result.history;p=result.params;
    if ~isfield(h,'state') || isempty(h.state)
        audit=struct('passed',true,'steps',0,'dynamicsMaximumError',0,'commandLimitPassed',true,'speedLimitPassed',true);return;
    end
    previous=cat(3,result.initialFleetState,h.state(:,:,1:end-1));
    u=h.command;lengths=sqrt(sum(u.^2,1));clipped=u.*min(1,p.amax./max(lengths,eps));
    acceleration=clipped+h.disturbance;
    velocity=previous(4:6,:,:)+p.dt*acceleration;speed=sqrt(sum(velocity.^2,1));
    velocity=velocity.*min(1,p.vmax./max(speed,eps));
    predicted=previous;predicted(4:6,:,:)=velocity;predicted(1:3,:,:)=previous(1:3,:,:)+p.dt*velocity;
    error=max(abs(predicted-h.state),[],'all');
    actualError=max(abs(acceleration-h.actualAcceleration),[],'all');
    measured=previous;measured(1:3,:,:)=previous(1:3,:,:)+h.noise;
    measurementError=max(abs(measured-h.measuredState),[],'all');
    commandOK=max(lengths,[],'all')<=p.amax+1e-10;
    speedOK=max(sqrt(sum(h.state(4:6,:,:).^2,1)),[],'all')<=p.vmax+1e-10;
    passed=error<1e-10 && actualError<1e-10 && measurementError<1e-10 && commandOK && speedOK;
    audit=struct('passed',passed,'steps',numel(h.t),'dynamicsMaximumError',error, ...
        'accelerationMaximumError',actualError,'measurementMaximumError',measurementError, ...
        'commandLimitPassed',commandOK,'speedLimitPassed',speedOK);
    assert(passed,'UA:IndependentAuditFailed');
end
