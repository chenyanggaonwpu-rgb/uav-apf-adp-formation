function rl_worker(action,first,second,assets)
    rl_setup();base=load(fullfile(assets,'base.mat'));
    if strcmp(action,'selfcheck')
        [state,~,~]=init_formation3d(base.p);next=motion_step3d(state,zeros(3,5),zeros(3,5),base.p);
        writejson(first,struct('passed',all(isfinite(next),'all'),'release',version('-release'),'fleet',5,'teacher_available',false));return;
    elseif strcmp(action,'encode')
        cases=jsondecode(fileread(first));features=cell(numel(cases),1);
        for k=1:numel(cases)
            if iscell(cases),item=cases{k};else,item=cases(k);end
            [p,scene]=v4_decode_case(item,base.p);features{k}=rl_encode_global_map(scene.obstacles,p);
        end
        writejson(second,struct('passed',true,'features',{features},'schema',1));return;
    elseif strcmp(action,'validate'),validate_maps(first,second,base);return;
    elseif strcmp(action,'once'),execute(first,second,assets,base);return;end
    assert(strcmp(action,'queue'));queue=first;
    while ~isfile(fullfile(queue,'STOP'))
        writejson(fullfile(queue,'heartbeat',[second '.json']),struct('worker',second,'timestamp',datestr(now,31)));
        pending=dir(fullfile(queue,'queued','*.json'));claimed='';lockPath='';
        for k=1:numel(pending)
            original=fullfile(pending(k).folder,pending(k).name);target=fullfile(queue,'claimed',pending(k).name);
            lockPath=[target '.lock'];lockObject=java.io.File(lockPath);
            if ~lockObject.mkdir(),continue;end
            if ~isfile(original),rmdir(lockPath);continue;end
            [ok,~]=movefile(original,target);
            if ok,claimed=target;break;else,rmdir(lockPath);end
        end
        if isempty(claimed),pause(.25);continue;end
        [~,id]=fileparts(claimed);execute(claimed,fullfile(queue,'completed',[id '.mat']),assets,base);
        if isfile(claimed),delete(claimed);end
        if isfolder(lockPath),rmdir(lockPath);end
    end
end
function execute(jobFile,output,assets,base) %#ok<INUSD>
    started=tic;job=jsondecode(fileread(jobFile));
    try
        item=job.scene;[p,scene]=v4_decode_case(item,base.p);cfg=base.cfg;s=base.s;
        assert(p.nUAV==5&&p.dt==.05&&p.Tmax==120&&p.safety.enable);
        p.v4Continuous=false;p.v4AllowDirectPlannerFallback=false;p.disturbanceMax=0;p.disturbanceHoldSteps=10;
        p.stochasticSeed=item.stochastic_seed;p.rlRayNoise=item.ray_noise_std;p.rlRayDropout=item.ray_dropout_probability;
        if isfield(item,'random_acceleration_max'),p.disturbanceMax=item.random_acceleration_max;end
        context=item.context;assert(strcmp(context,'global_planning')&&strcmp(job.task_protocol,'global_subgoal_v1'));
        s.maximumFlightSeconds=120;s.disableNatural=false;s.elapsedTakeover.enable=false;
        cfg.physicsStepsPerDecision=4;cfg.planner=job.planner;
        assert(cfg.planner.decision_seconds>=1&&cfg.planner.decision_seconds<=2);
        route='local_apf';if strcmp(job.arm,'global_rule'),route='global';end
        p.ablationStaleWaypointFault=false;
        rules=default_normal_scene_options3d();rules.requireWitness=false;geometry=check_normal_scene3d(scene,p,rules);assert(geometry.accepted,item.id);
        actor=[];mode=job.mode;
        assert(any(strcmp(mode,{'none','rule','actor'}))&&job.teacher_probability==0,'RL:TeacherForbidden');
        if strcmp(mode,'actor')
            loaded=load(job.model,'actor');assert(strcmp(ua_sha256(job.model),job.model_sha256));
            if isfield(loaded.actor,'schema')
                options=struct('training',false,'teacher_probability',0,'stochastic',job.stochastic,'seed',job.policy_seed,'ray_noise_std',0,'ray_dropout_probability',0);
                if isfield(job,'noise_hold_seconds'),options.noise_hold_seconds=job.noise_hold_seconds;end
                if loaded.actor.schema==6
                    assert(loaded.actor.input_size==3340&&loaded.actor.global_schema==1&&strcmp(loaded.actor.action_semantics,'normalized_global_subgoal_xyz'));
                    actor=RLPolicy(loaded.actor,options,cfg);route='actor_global';
                elseif loaded.actor.schema==5
                    assert(loaded.actor.input_size==3340&&loaded.actor.global_schema==1);actor=GlobalEscapePolicy(loaded.actor,options,cfg);
                elseif loaded.actor.schema==4
                    assert(loaded.actor.input_size==2048&&~job.stochastic);actor=LocalRLPolicy(loaded.actor,options,cfg);
                else,error('RL:UnknownActorSchema','Unsupported Actor schema');end
            else
                assert(~job.stochastic);actor=V4Legacy(loaded.actor);
            end
        end
        runOptions=struct();
        for field={'capture_file','capture_only','capture_active_after','resume_file','resume_policy_stream'}
            if isfield(job,field{1}),runOptions.(field{1})=job.(field{1});end
        end
        result=rl_simulate(p,scene,[],mode,actor,cfg,s,route,runOptions);audit=v4_audit(result);
        record=struct('observations',zeros(128,0,'single'),'action_index',zeros(1,0),'actions',zeros(3,0,'single'),'times',zeros(1,0),'resume_step',1);
        if isfield(result,'trainingRecord'),record=result.trainingRecord;end
        safe=true;steps=0;critical=false;
        if isfield(result.history,'state')&&~isempty(result.history.state)
            safe=min(result.history.obstacleClearance)>p.obsSafe&&min(result.history.pairDistance)>p.uavSafe;
            steps=numel(result.history.t);critical=min(result.history.obstacleClearance)<=0||min(result.history.pairDistance)<=.25||strcmp(result.reason,'boundary_violation');
        end
        events=result.controller.events;effective=false;forced=false;
        if ~isempty(events)
            effective=any(ismember({events.reason},{'local_apf_corridor_clear','terminal_handoff','formation_terminal_hold'}));
            forced=any(strcmp({events.reason},'elapsed_budget_handoff'));
        end
        if isa(actor,'RLPolicy'),assert(actor.TeacherQueries==0);end
        row=struct('id',job.id,'case_id',item.id,'split',item.split,'group',item.group,'context',context,'noise_profile',item.noise_profile, ...
            'arm',job.arm,'success',result.success,'safe',safe,'critical',critical,'reason',result.reason,'steps',steps, ...
            'flight_seconds',result.time,'wall_seconds',toc(started),'audit_passed',audit.passed,'teacher_queries',0, ...
            'transitions',size(record.actions,2),'effective_handoff',effective,'forced_budget_handoff',forced, ...
            'activations',result.controller.activations,'resume_step',record.resume_step,'stochastic',job.stochastic,'policy_seed',job.policy_seed,'task_protocol','global_subgoal_v1','leader_route_policy',route, ...
            'terminal_follower_planning','shared_original','leader_rule_planner_calls',0, ...
            'leader_path_meters',0,'fleet_path_meters',0,'stagnant_seconds',0,'revisit_seconds',0);
        if isfield(result,'metrics')
            row.leader_path_meters=result.metrics.leaderPathLength;row.fleet_path_meters=result.metrics.fleetPathLength;
            row.leader_rule_planner_calls=result.leaderRulePlannerCalls;
            [row.stagnant_seconds,row.revisit_seconds]=rl_motion_metrics(result);
        end
        if strcmp(route,'actor_global'),assert(row.leader_rule_planner_calls==0);end
        if isfield(job,'model_sha256'),row.model_sha256=job.model_sha256;end
        if isfield(job,'capture_file'),row.capture_file=job.capture_file;row.capture_exists=isfile(job.capture_file);end
        temporary=[output '.part'];save(temporary,'result','row','job','audit','record','-v7');movefile(temporary,output,'f');writejson([output '.json'],row);
        fprintf('RL_FLIGHT %s %s transitions=%d wall=%.3f\n',job.id,row.reason,row.transitions,row.wall_seconds);
    catch exception
        writejson([output '.error.json'],struct('id',job.id,'execution_error',true,'message',getReport(exception,'extended','hyperlinks','off')));
        fprintf(2,'RL_ERROR %s %s\n',job.id,exception.message);
    end
end
function validate_maps(input,output,base)
    cases=jsondecode(fileread(input));checks=cell(numel(cases),1);
    for k=1:numel(cases)
        if iscell(cases),item=cases{k};else,item=cases(k);end
        [p,scene]=v4_decode_case(item,base.p);rules=default_normal_scene_options3d();rules.requireWitness=false;
        geometry=check_normal_scene3d(scene,p,rules);assert(geometry.accepted,item.id);
        [state,~,~]=init_formation3d(p);offsets=state(1:3,:)-p.start;
        [~,canStop]=f5_braking_plan(state,scene.obstacles,p,base.cfg.safety);assert(canStop,item.id);
        route=double(item.geometric_witness.points)';minimum=inf;
        for j=1:size(route,2)-1
            [ok,margin]=certify_formation_segment3d(route(:,j),route(:,j+1),offsets,scene.obstacles,p,1.2,1.2,1e-5);
            assert(ok,item.id);minimum=min(minimum,margin);
        end
        checks{k}=struct('id',item.id,'family',item.group,'accepted',true,'initialCanStop',canStop,'minimumWitnessClearance',minimum);
    end
    writejson(output,struct('passed',true,'maps',numel(cases),'checks',{checks},'teacher_used',false));
end
function writejson(path,value)
    parent=fileparts(path);if ~isfolder(parent),mkdir(parent);end
    temporary=[path '.part'];fid=fopen(temporary,'w','n','UTF-8');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value));fclose(fid);movefile(temporary,path,'f');
end
