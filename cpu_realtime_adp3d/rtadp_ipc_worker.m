function rtadp_ipc_worker(project,ipc)
% A separate MATLAB CPU process. No Parallel Toolbox pool, GPU, or training.
    addpath(fullfile(project,'cpu_realtime_adp3d'));
    data=rtadp_worker_init(project); resident=struct('Value',data);
    info=rtadp_worker_status(resident);
    save(fullfile(ipc,'worker_ready.pending.mat'),'info','-v7');
    rtadp_atomic_publish(fullfile(ipc,'worker_ready.pending.mat'),fullfile(ipc,'worker_ready.mat'));
    fprintf('ACTOR_CPU_WORKER_READY\n');
    sequence=1;
    while ~isfile(fullfile(ipc,'STOP.txt'))
        stem=fullfile(ipc,sprintf('job_%06d',sequence));
        request=[stem '_request.mat'];
        if ~isfile(request), pause(0.002); continue; end
        try
            if isfile([stem '_cancel.txt'])
                reply=struct('ok',false,'message','Job cancelled before execution.');
            else
                packet=load(request,'payload'); payload=packet.payload;
                if isfield(payload,'observation')
                    timer=tic;
                    action=adp3d_actor_action(data.actor,payload.observation);
                    computeSeconds=toc(timer);
                    if payload.delaySeconds>0, pause(payload.delaySeconds); end
                    result=struct('action',action,'sourceTime',payload.time,'generation',payload.generation, ...
                        'computeSeconds',computeSeconds,'workerTotalSeconds',toc(timer));
                else
                    result=rtadp_worker_decide(resident,payload.controller,payload.history,payload.time, ...
                        payload.obstacles,payload.parameters,payload.delaySeconds);
                end
                reply=struct('ok',true,'result',result);
            end
        catch exception
            reply=struct('ok',false,'message',exception.message,'identifier',exception.identifier);
        end
        temporary=[stem '_result.pending.mat'];
        save(temporary,'reply','-v7'); rtadp_atomic_publish(temporary,[stem '_result.mat']);
        sequence=sequence+1;
    end
    info=rtadp_worker_status(resident);
    save(fullfile(ipc,'worker_final.mat'),'info','-v7');
    assert(info.weightsUnchanged);
    fprintf('ACTOR_CPU_WORKER_STOPPED_WEIGHTS_UNCHANGED\n');
end
