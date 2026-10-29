function [rows,details]=rtadp_benchmark(actor,cfg,s,protocol,output)
% Time complete decision + APF + safety/braking path, not merely the small network.
    base=adp3d_setup(cfg);
    trap=adp3d_make_case(10001,cfg,base);
    weights=actor.Learnables; stages={'normal','trigger','active','exit','safety_intercept'};
    collected={}; inference=zeros(1,protocol.inferenceRepeats);
    observation=adp3d_observation(trap.initialState,trap.obstacles,trap.parameters,cfg.scanRange);
    for k=1:protocol.inferenceRepeats
        timer=tic; adp3d_actor_action(actor,observation); inference(k)=1000*toc(timer);
    end
    for stageIndex=1:numel(stages)
        stage=stages{stageIndex};
        for countIndex=1:numel(protocol.obstacleCounts)
            obstacleCount=protocol.obstacleCounts(countIndex);
            if countIndex>1 && ~ismember(stage,{'normal','active'}), continue; end
            if countIndex==1, repetitions=protocol.pipelineRepeats; else, repetitions=protocol.denseRepeats; end
            p=base; c=apf_adp_init(); time=3;
            obstacles=[obstacle3d('box',[36;0;0],[44;80;10]),obstacle3d('box',[36;0;20],[44;80;30])];
            p.target=[70;40;15]; state=[10;40;15;3;0;0];
            switch stage
                case 'trigger'
                    state=trap.initialState; p=trap.parameters; obstacles=trap.obstacles;
                case 'active'
                    state=trap.initialState; p=trap.parameters; obstacles=trap.obstacles;
                    c.mode=2; c.modeSince=0; c.entryDistance=norm(p.target-state(1:3)); c.activations=1;
                case 'exit'
                    state=[40;40;15;1;0;0]; c.mode=2; c.entryDistance=40;
                    c.modeSince=0; c.clearSince=2; c.activations=1;
                case 'safety_intercept'
                    state=[33.95;40;10;3;0;0]; p.target=[70;40;10];
                    obstacles=obstacle3d('box',[36;0;0],[44;80;30]);
            end
            % Extra static solids at a predeclared far-corner lattice exercise map cost.
            for q=numel(obstacles)+1:obstacleCount
                x=3+3*mod(q-1,5); y=3+3*mod(floor((q-1)/5),5); z=3+4*mod(floor((q-1)/25),6);
                obstacles(end+1)=obstacle3d('sphere',[x;y;z],0.5); %#ok<AGROW>
            end
            history=repmat(state,1,61);
            [backup,ok]=adp3d_braking_plan(state,obstacles,p,cfg.safety); assert(ok);
            samples=zeros(repetitions,1); interventions=0; actorCalls=0;
            for rep=1:repetitions
                timer=tic;
                if strcmp(stage,'safety_intercept')
                    nominal=[3;0;0]; after=c;
                else
                    [waypoint,after,~]=apf_adp_decide(c,history,time,obstacles,p,cfg,s,actor);
                    nominal=adp3d_leader_command(state,waypoint,obstacles,p);
                end
                [~,~,step]=adp3d_safe_step(state,nominal,backup,obstacles,p,cfg.safety);
                samples(rep)=1000*toc(timer);
                interventions=interventions+step.intervention;
                actorCalls=actorCalls+after.actorCalls-c.actorCalls;
                assert(~step.collision);
            end
            row=struct('stage',stage,'obstacles',numel(obstacles),'samples',repetitions, ...
                'medianMs',median(samples),'p99Ms',prctile(samples,99),'maximumMs',max(samples), ...
                'over50Ms',sum(samples>50),'actorCalls',actorCalls,'safetyInterventions',interventions);
            collected{end+1}=row; %#ok<AGROW>
            filename=sprintf('benchmark_%s_%d.mat',stage,numel(obstacles));
            save(fullfile(output,filename),'samples','row','-v7');
            fprintf('BENCH %s solids=%d P99=%.3fms max=%.3fms missed=%d/%d\n', ...
                stage,numel(obstacles),row.p99Ms,row.maximumMs,row.over50Ms,repetitions);
        end
    end
    assert(isequaln(weights,actor.Learnables));
    rows=struct2table([collected{:}]); writetable(rows,fullfile(output,'pipeline_benchmarks.csv'));
    details=struct('actorForwardOnlySamples',numel(inference),'actorForwardMedianMs',median(inference), ...
        'actorForwardP99Ms',prctile(inference,99),'actorForwardMaxMs',max(inference), ...
        'weightsUnchanged',true,'trainingPerformed',false);
    save(fullfile(output,'actor_forward_benchmark.mat'),'details','inference','-v7');
end
