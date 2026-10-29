function [next,backup,info] = adp3d_safe_step(state,nominal,backup,obstacles,p,settings)
% 只有“下一段安全且下一状态仍能刹停”时执行名义控制。
% 拒绝时执行上一时刻已经核验的后备第一条命令，不冻结状态、不清零速度。
    assert(p.disturbanceMax==0 && p.noiseStd==0,'ADP3D:DisturbanceUnsupported');
    if isempty(backup.commands)
        [backup,ok]=adp3d_braking_plan(state,obstacles,p,settings);
        assert(ok,'ADP3D:NoBackup','当前状态无可核验制动后备。');
    end
    assert(backup.certified && norm(backup.states(:,1)-state)<1e-8, ...
        'ADP3D:BackupMismatch','当前状态与制动后备不一致。');
    candidate=motion_step3d(state,nominal,zeros(3,1),p);
    [segmentSafe,margin]=adp3d_certify_segment(state(1:3),candidate(1:3),obstacles,p,settings);
    futureSafe=false;
    if segmentSafe
        [newBackup,futureSafe]=adp3d_braking_plan(candidate,obstacles,p,settings);
    end
    intervention=~(segmentSafe && futureSafe);
    if ~intervention
        command=nominal;
        backup=newBackup;
    else
        command=backup.commands(:,1);
        backup.commands(:,1)=[];
        backup.states(:,1)=[];
    end
    [next,actualAcceleration]=motion_step3d(state,command,zeros(3,1),p);
    assert(norm(next-backup.states(:,1))<1e-8,'ADP3D:BackupExecutionMismatch');
    % 原工程碰撞检查作为独立诊断；准入依据为上面的保守证书。
    [collision,minObstacle,~,boundaryHit]=check_swept3d(state,next,obstacles,p);
    info=struct('intervention',intervention,'nominalCommand',nominal, ...
        'executedCommand',command,'actualAcceleration',actualAcceleration, ...
        'candidateSegmentSafe',segmentSafe,'candidateCanStop',futureSafe, ...
        'candidateMarginLowerBound',margin,'collision',collision, ...
        'minimumObstacleDistance',minObstacle,'boundaryHit',boundaryHit);
end
