function data=rtadp_worker_init(project)
% Executed once in the local CPU process worker, before any paced flight.
    addpath(fullfile(project,'cpu_realtime_adp3d'));
    [actor,cfg,s,details]=rtadp_load_actor(project);
    weights=actor.Learnables; first=[];
    for k=1:15
        timer=tic; adp3d_actor_action(actor,zeros(32,1)); first(k)=1000*toc(timer); %#ok<AGROW>
    end
    adp3d_setup(cfg);
    checks=apf_adp_checks(actor,cfg,s);
    assert(isequaln(weights,actor.Learnables));
    data=struct('actor',actor,'weights',weights,'cfg',cfg,'settings',s, ...
        'details',details,'warmupMilliseconds',first,'checks',checks);
end
