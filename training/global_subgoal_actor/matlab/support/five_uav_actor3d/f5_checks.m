function receipt=f5_checks(p,cfg)
    empty=struct('type',{},'center',{},'r',{},'zmin',{},'zmax',{},'halfSize',{});
    state=zeros(6,5);state(1:3,:)=[20 22 40 50 60;20 20 40 50 60;10 10 10 10 10];
    state(4,1)=3;state(4,2)=-3;
    [~,ok]=f5_braking_plan(state,empty,p,cfg.safety);
    assert(~ok,'F5:MustRejectHeadOnBackup');
    state(1,2)=26;[backup,ok]=f5_braking_plan(state,empty,p,cfg.safety);assert(ok);
    for k=1:size(backup.commands,3)
        assert(f5_certify(backup.states(:,:,k),backup.states(:,:,k+1),empty,p,cfg.safety));
    end
    minPair=inf;interventions=0;
    for k=1:80
        nominal=zeros(3,5);nominal(1,1)=3;nominal(1,2)=-3;
        [state,backup,detail]=f5_safe_step(state,nominal,backup,empty,p,cfg.safety);
        assert(~detail.collision);minPair=min(minPair,detail.pairDistance);
        interventions=interventions+detail.intervention;
    end
    assert(interventions>0 && minPair>p.uavSafe);
    stream=RandStream('mt19937ar','Seed',58001);
    obstacles=[obstacle3d('sphere',[40;40;10],4), ...
        obstacle3d('box',[30;20;5],[38;30;15]),obstacle3d('cylinder',[55;50],3,0,20)];
    for sample=1:50
        a=[80;80;30].*rand(stream,3,5);b=a+4*randn(stream,3,5);
        expected=true;
        for i=1:5,expected=expected && adp3d_certify_segment(a(:,i),b(:,i),obstacles,p,cfg.safety);end
        actual=f5_static_cert(a,b,obstacles,p,cfg.safety);
        assert(actual==expected,'F5:BatchCertificateEquivalence');
    end
    receipt=struct('unsafeHeadOnBackupRejected',true,'safeBackupEverySegmentChecked',true, ...
        'headOnPhysicalSteps',80,'headOnInterventions',interventions,'minimumPairDistance',minPair, ...
        'batchCertificateEquivalenceSamples',250, ...
        'trainingPerformed',false,'criticEvaluated',false);
end
