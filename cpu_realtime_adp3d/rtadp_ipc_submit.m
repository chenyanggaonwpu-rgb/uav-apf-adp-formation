function future=rtadp_ipc_submit(ipc,controller,history,time,obstacles,p,delaySeconds)
% One outstanding job at a time; atomic MAT-file handoff between local processes.
    stem=rtadp_ipc_nextstem(ipc);
    payload=struct('controller',controller,'history',history,'time',time, ...
        'obstacles',obstacles,'parameters',p,'delaySeconds',delaySeconds);
    temporary=[stem '_request.pending.mat']; request=[stem '_request.mat'];
    save(temporary,'payload','-v7'); rtadp_atomic_publish(temporary,request);
    future=struct('request',request,'result',[stem '_result.mat'],'cancel',[stem '_cancel.txt']);
end
