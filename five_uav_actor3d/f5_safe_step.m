function [next,backup,info]=f5_safe_step(state,nominal,backup,obstacles,p,s,independentDiagnostic)
% Rejection advances the real plant using the previous verified joint backup.
    assert(p.nUAV==5 && p.noiseStd==0 && p.disturbanceMax==0);
    if isempty(backup.commands)
        [backup,ok]=f5_braking_plan(state,obstacles,p,s);assert(ok,'F5:NoBackup');
    end
    assert(backup.certified && norm(backup.states(:,:,1)-state,'fro')<1e-8,'F5:BackupMismatch');
    nominal=limit_vector3d(nominal,p.amax);
    candidate=motion_step3d(state,nominal,zeros(3,p.nUAV),p);
    [segmentSafe,margin]=f5_certify(state,candidate,obstacles,p,s);canStop=false;
    if segmentSafe,[newBackup,canStop]=f5_braking_plan(candidate,obstacles,p,s);end
    intervention=~(segmentSafe && canStop);
    if intervention
        command=backup.commands(:,:,1);backup.commands(:,:,1)=[];backup.states(:,:,1)=[];
    else
        command=nominal;backup=newBackup;
    end
    [next,actualAcceleration]=motion_step3d(state,command,zeros(3,p.nUAV),p);
    assert(norm(next-backup.states(:,:,1),'fro')<1e-8,'F5:BackupExecutionMismatch');
    if nargin<7 || independentDiagnostic
        [hit,minObstacle,minPair,boundary]=check_swept3d(state,next,obstacles,p);
    else
        % The calling simulator immediately performs the unchanged independent
        % swept check and records its result. Do not compute it twice.
        hit=false;minObstacle=NaN;minPair=NaN;boundary=false;
    end
    info=struct('intervention',intervention,'segmentSafe',segmentSafe,'canStop',canStop, ...
        'margin',margin,'command',command,'acceleration',actualAcceleration, ...
        'collision',hit,'obstacleDistance',minObstacle,'pairDistance',minPair,'boundaryHit',boundary);
end
