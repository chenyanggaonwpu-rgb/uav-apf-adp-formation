function future=rtadp_ipc_submit_actor(ipc,observation,time,generation,delaySeconds)
    stem=rtadp_ipc_nextstem(ipc);
    payload=struct('observation',observation,'time',time,'generation',generation,'delaySeconds',delaySeconds);
    temporary=[stem '_request.pending.mat']; request=[stem '_request.mat'];
    save(temporary,'payload','-v7'); rtadp_atomic_publish(temporary,request);
    future=struct('request',request,'result',[stem '_result.mat'],'cancel',[stem '_cancel.txt']);
end
