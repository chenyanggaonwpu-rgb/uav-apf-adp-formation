function ua_pilot(output)
    project=ua_setup();if ~isfolder(output),mkdir(output);end
    [actor,cfg,s,model]=ua_load_actor();original=actor.Learnables;
    p=default_params3d();scene=scenario3d('S1_sphere',p.seed,p);
    observed=run_simulation3d(p,scene);assert(observed.model.actorLoaded && observed.model.legacyADPCalls==0);
    assert(observed.success,'UA:DefaultPilotFailed');
    % No-noise numerical equivalence with the frozen five-UAV integration.
    reference=f5_simulate(p,scene,[],'automatic',actor,cfg,s,'global');
    assert(isequaln(observed.history.state,reference.history.state),'UA:NominalTrajectoryDrift');
    assert(isequaln(observed.history.command,reference.history.command),'UA:NominalCommandDrift');
    cs=f5_cases('pilot',p,cfg);trap=cs{end};
    trap.parameters.controller=struct('kind','unified_frozen_actor','method','actor','routePolicy',trap.routePolicy);
    active=run_simulation3d(trap.parameters,trap.scene);
    assert(active.controller.actorCalls>0 && active.model.actorLoaded,'UA:ActorNeverCalled');
    old=f5_simulate(trap.parameters,trap.scene,[],'automatic',actor,cfg,s,trap.routePolicy);
    assert(isequaln(active.history.state,old.history.state),'UA:ActiveTrajectoryDrift');
    assert(isequaln(original,actor.Learnables),'UA:ActorChanged');
    % Inject a known pulse and audit true discrete dynamics independently.
    p.controller.method='none';p.noiseStd=.02;p.stochasticSeed=2026102901;
    p.robustExperiment=struct('startStep',80,'duration',2,'amplitude',.3,'direction',[0;1;0]);
    noisy=run_simulation3d(p,scene);previous=noisy.initialFleetState;maximumError=0;
    for k=1:numel(noisy.history.t)
        predicted=motion_step3d(previous,noisy.history.command(:,:,k),noisy.history.disturbance(:,:,k),p);
        maximumError=max(maximumError,max(abs(predicted-noisy.history.state(:,:,k)),[],'all'));
        previous=noisy.history.state(:,:,k);
    end
    assert(maximumError==0 && any(noisy.history.disturbance(:)~=0),'UA:PulseMissing');
    assert(max(vecnorm(reshape(noisy.history.command,3,[])))<=p.amax+1e-10);
    receipt=struct('passed',true,'model',model,'defaultReason',observed.reason, ...
        'nominalStateAndCommandExact',true,'activeStateAndCommandExact',true, ...
        'activeActorCalls',active.controller.actorCalls,'activeReason',active.reason, ...
        'noiseReason',noisy.reason,'dynamicsMaximumError',maximumError,'weightsUnchanged',true);
    fid=fopen(fullfile(output,'pilot_receipt.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(receipt,PrettyPrint=true));fclose(fid);
    save(fullfile(output,'pilot_evidence.mat'),'observed','reference','active','old','noisy','receipt','-v7');
    fprintf('UA_PILOT_PASS calls=%d noisy=%s\n',active.controller.actorCalls,noisy.reason);
end
