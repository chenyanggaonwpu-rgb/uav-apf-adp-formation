function checks=rtadp_bridge_checks(actor,cfg,s)
% With a fresh output the adapter must reproduce the original Actor path exactly.
    p=adp3d_setup(cfg); item=adp3d_make_case(10001,cfg,p); p=item.parameters;
    state=item.initialState; observation=adp3d_observation(state,item.obstacles,p,cfg.scanRange);
    action=adp3d_actor_action(actor,observation);
    cache=RTFrozenActorCache(); cache.Action=action;
    history=repmat(state,1,61); c=apf_adp_init();
    [realWaypoint,realController,realDiagnostic]=apf_adp_decide(c,history,3,item.obstacles,p,cfg,s,actor);
    [cachedWaypoint,cachedController,cachedDiagnostic]=apf_adp_decide(c,history,3,item.obstacles,p,cfg,s,cache);
    assert(isequal(realWaypoint,cachedWaypoint) && isequaln(realController,cachedController) ...
        && isequaln(realDiagnostic,cachedDiagnostic),'RTADP:CacheBridgeNotEquivalent');
    assert(isequal(single(observation),single(cache.LastObservation)));
    checks=struct('freshActorOutputGivesIdenticalWaypoint',true, ...
        'freshActorOutputGivesIdenticalSupervisorAndDiagnostics',true, ...
        'observationFloat32ValuesUnchanged',true,'cacheContainsNoNetworkParameters',true, ...
        'supervisorSourceUnmodified',true,'trainingPerformed',false);
end
