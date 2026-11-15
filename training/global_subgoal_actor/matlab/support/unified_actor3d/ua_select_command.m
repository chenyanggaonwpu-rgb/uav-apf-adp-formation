function [command,backup,info]=ua_select_command(measured,nominal,backup,obstacles,p,s)
% Safety uses the same measured state as feedback. It selects a command only;
% the caller ALWAYS advances the true plant, including external acceleration.
    nominal=limit_vector3d(nominal,p.amax);
    if isempty(backup) || isempty(backup.commands) || ...
            norm(backup.states(:,:,1)-measured,'fro')>=1e-8
        [backup,backupAvailable]=f5_braking_plan(measured,obstacles,p,s);
    else
        backupAvailable=backup.certified;
    end
    candidate=motion_step3d(measured,nominal,zeros(3,p.nUAV),p);
    [segmentSafe,margin]=f5_certify(measured,candidate,obstacles,p,s);canStop=false;
    if segmentSafe,[newBackup,canStop]=f5_braking_plan(candidate,obstacles,p,s);end
    intervention=~(segmentSafe && canStop);
    if ~intervention
        command=nominal;backup=newBackup;
    elseif backupAvailable && ~isempty(backup.commands)
        command=backup.commands(:,:,1);backup.commands(:,:,1)=[];backup.states(:,:,1)=[];
    else
        % Missing a nominal certificate never freezes position or zeroes speed.
        command=limit_vector3d(-measured(4:6,:)/p.dt,p.amax);backup=[];
    end
    info=struct('intervention',intervention,'backupAvailable',backupAvailable, ...
        'segmentSafe',segmentSafe,'canStop',canStop,'margin',margin);
end
