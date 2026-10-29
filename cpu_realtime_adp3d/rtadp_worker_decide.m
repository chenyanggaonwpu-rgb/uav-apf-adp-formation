function result=rtadp_worker_decide(resident,controller,history,time,obstacles,p,delaySeconds)
% Actual process-worker call; only forward inference in the immutable Actor.
    timer=tic;
    data=resident.Value;
    [waypoint,after,diagnostic]=apf_adp_decide(controller,history,time,obstacles,p, ...
        data.cfg,data.settings,data.actor);
    computeSeconds=toc(timer);
    if delaySeconds>0, pause(delaySeconds); end
    result=struct('waypoint',waypoint,'controller',after,'diagnostic',diagnostic, ...
        'sourceTime',time,'computeSeconds',computeSeconds,'workerTotalSeconds',toc(timer));
end
