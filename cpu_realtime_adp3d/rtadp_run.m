function report=rtadp_run(project,output,phase)
% No training path exists in this entry. All learned, APF, safety and supervisor
% parameters are loaded from the frozen trained version and never tuned here.
    addpath(fullfile(project,'cpu_realtime_adp3d'));
    protocol=rtadp_protocol(project);
    [actor,cfg,s,modelInfo]=rtadp_load_actor(project); p=adp3d_setup(cfg);
    assert(p.dt==0.05 && cfg.physicsStepsPerDecision*p.dt==0.2);
    originalWeights=actor.Learnables; originalCfg=cfg; originalSettings=s;
    cold=zeros(1,15);
    for k=1:numel(cold)
        timer=tic; adp3d_actor_action(actor,zeros(32,1)); cold(k)=1000*toc(timer);
    end
    apf_adp_checks(actor,cfg,s);
    bridgeChecks=rtadp_bridge_checks(actor,cfg,s);
    catalog=apf_adp_cases('development',cfg,s); cases=cell(1,numel(protocol.caseIds));
    for k=1:numel(cases)
        match=find(cellfun(@(x)strcmp(x.id,protocol.caseIds{k}),catalog));
        assert(numel(match)==1); cases{k}=catalog{match};
    end
    if strcmp(phase,'pilot')
        protocol.pipelineRepeats=3; protocol.denseRepeats=2; protocol.inferenceRepeats=10;
        protocol.clockBudgetSeconds=3;
    end
    save(fullfile(output,'protocol.mat'),'protocol','cfg','s','modelInfo','cold','cases','-v7');
    writeJson('protocol.json',struct('protocol',protocol,'modelInfo',modelInfo, ...
        'frozenSupervisorSettings',s,'coldAndWarmInferenceMs',cold));
    writeJson('fresh_action_bridge_checks.json',bridgeChecks);
    [benchmarks,forward]=rtadp_benchmark(actor,cfg,s,protocol,output);
    blackouts=rtadp_blackout_faults(cfg,output);
    pool=fullfile(output,'ipc'); resident=[];
    ready=fullfile(pool,'worker_ready.mat'); waiting=tic;
    while ~isfile(ready)
        assert(toc(waiting)<180,'RTADP:WorkerStartupTimeout'); pause(0.05);
    end
    startup=load(ready,'info'); workerInfo=startup.info;
    assert(workerInfo.weightsUnchanged && ~workerInfo.criticLoaded);
    warmProtocol=protocol; warmProtocol.clockBudgetSeconds=0.4;
    rtadp_paced_flight(cases{2},'serial',actor,resident,pool,cfg,s,warmProtocol,'none',false);
    rtadp_paced_flight(cases{2},'async',actor,resident,pool,cfg,s,warmProtocol,'none',true);
    % Warm the process boundary and every actual function before measuring flights.
    c=apf_adp_init(); warm=rtadp_ipc_submit(pool,c, ...
        cases{1}.initialState,0,cases{1}.obstacles,cases{1}.parameters,0);
    rtadp_ipc_wait(warm,10); rtadp_ipc_fetch(warm);
    warm=rtadp_ipc_submit_actor(pool,zeros(32,1),0,0,0); rtadp_ipc_wait(warm,10); rtadp_ipc_fetch(warm);
    planned=cell(numel(protocol.flightCaseIndices),3);
    for k=1:size(planned,1)
        planned(k,:)={protocol.flightCaseIndices(k),protocol.flightArchitectures{k},protocol.flightFaults{k}};
    end
    if strcmp(phase,'pilot'), planned={1,'async','none'}; end
    if ismember(phase,{'ipc_validation','actor_validation'}), planned=planned(ismember(planned(:,2),{'async','actor_async'}),:); end
    summaries={};
    for k=1:size(planned,1)
        item=cases{planned{k,1}}; architecture=planned{k,2}; fault=planned{k,3};
        fprintf('PACED_START %d/%d %s %s fault=%s\n',k,size(planned,1),item.id,architecture,fault);
        [episode,summary]=rtadp_paced_flight(item,architecture,actor,resident,pool,cfg,s,protocol,fault,true);
        if strcmp(item.id,'sphere_detour_40006') && summary.mainDeadlineMisses==0 && ~strcmp(phase,'pilot')
            reference=load(fullfile(project,'results','neural_adp_supervisor', ...
                'development_20261016_202051','sphere_detour_40006_automatic.mat'),'automatic');
            assert(isequal(episode.states,reference.automatic.states) && isequal(episode.commands,reference.automatic.commands), ...
                'RTADP:NormalAPFTrajectoryChanged');
        end
        save(fullfile(output,sprintf('paced_%02d_%s_%s.mat',k,item.id,architecture)), ...
            'episode','summary','-v7');
        summaries{end+1}=summary; %#ok<AGROW>
        flights=struct2table([summaries{:}]); writetable(flights,fullfile(output,'paced_flights.csv'));
        fprintf('PACED_END %d: goal=%d hit=%d deadlineMisses=%d max=%.3fms staleDropped=%d brakeSteps=%d\n', ...
            k,summary.success,summary.collision,summary.mainDeadlineMisses,summary.mainCycleMaxMs, ...
            summary.staleResultsDropped,summary.brakingForStaleActorSteps);
    end
    fid=fopen(fullfile(pool,'STOP.txt'),'w'); fprintf(fid,'stop\n'); fclose(fid);
    waiting=tic;
    while ~isfile(fullfile(pool,'worker_final.mat'))
        assert(toc(waiting)<15,'RTADP:WorkerShutdownTimeout'); pause(0.01);
    end
    workerFinal=load(fullfile(pool,'worker_final.mat'),'info'); workerAfter=workerFinal.info;
    assert(isequaln(originalWeights,actor.Learnables) && workerAfter.weightsUnchanged);
    assert(isequaln(originalCfg,cfg) && isequaln(originalSettings,s),'RTADP:ParametersChanged');
    report=struct('phase',phase,'scope',protocol.scope,'outputDirectory',output, ...
        'actorOnly',true,'criticLoaded',false,'agentLoaded',false,'trainingPerformed',false, ...
        'actorParameters',modelInfo.parameterCount,'mainWeightsUnchanged',true,'workerWeightsUnchanged',true, ...
        'allADPAPFSafetyAndSupervisorParametersUnchanged',true,'forwardInference',forward, ...
        'pacedFlights',height(flights),'pacedGoals',sum(flights.success),'pacedCollisions',sum(flights.collision), ...
        'mainMissedDeadlines',sum(flights.mainDeadlineMisses),'mainMaximumCycleMs',max(flights.mainCycleMaxMs), ...
        'densePipelineCasesOverBudget',sum(benchmarks.over50Ms>0), ...
        'negativeMainBlackoutCollisions',sum(blackouts.collision & blackouts.expectedNegativeControl), ...
        'workerStartup',workerInfo,'workerFinal',workerAfter);
    save(fullfile(output,'cpu_realtime_report.mat'),'report','flights','benchmarks','blackouts','protocol','-v7');
    writeJson('cpu_realtime_report.json',report);
    fprintf('ACTOR_ONLY_CPU_REALTIME_CHECK_COMPLETED\n');
    function writeJson(filename,value)
        fid=fopen(fullfile(output,filename),'w','n','UTF-8');
        guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
        fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
    end
end
