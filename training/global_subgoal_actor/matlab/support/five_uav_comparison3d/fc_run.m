function fc_run(project,campaign,output,phase,shard,shards)
    data=load(fullfile(campaign,'catalogue_before_flight.mat'),'cases','pilotCases','protocol');
    if strcmp(phase,'pilot'),cases=data.pilotCases;else,cases=data.cases;end
    [actor,cfg,s,modelInfo]=rtadp_load_actor(project);weights=actor.Learnables;
    assert(isequaln(cfg,data.protocol.configuration) && isequaln(s,data.protocol.supervisor));
    for k=1:20,adp3d_actor_action(actor,zeros(32,1));end
    if strcmp(phase,'pilot'),fc_checks(project,output,actor,cfg,s);end
    records=struct([]);pairs=struct([]);auditedSteps=0;
    methods={'none','rule','actor'};policies={'global','local_apf'};
    selected=shard:shards:numel(cases);
    for index=selected
        item=cases{index};outcomes=cell(2,3);
        order=circshift(1:6,[0 mod(index-1,6)]);
        for combination=order
            policyIndex=ceil(combination/3);methodIndex=mod(combination-1,3)+1;
            method=methods{methodIndex};routePolicy=policies{policyIndex};errorText='';auditText='';
            clock=tic;
            try
                result=fc_simulate(item.parameters,item.scene,[],method,actor,cfg,s,routePolicy);
            catch exception
                errorText=getReport(exception,'extended','hyperlinks','off');
                result=struct('success',false,'reason','execution_error','time',0,'history',struct());
                fprintf('FC_ERROR case=%s route=%s method=%s %s\n',item.id,routePolicy,method,exception.message);
            end
            wallSeconds=toc(clock);auditClock=tic;
            try
                audit=f5_audit(result,item,cfg);
            catch exception
                auditText=getReport(exception,'extended','hyperlinks','off');
                count=0;if isfield(result.history,'t'),count=numel(result.history.t);end
                audit=struct('steps',count,'passed',false,'dynamicsError',NaN,'minimumCertifiedMargin',NaN);
                fprintf('FC_AUDIT_ERROR case=%s route=%s method=%s %s\n',item.id,routePolicy,method,exception.message);
            end
            auditSeconds=toc(auditClock);auditedSteps=auditedSteps+audit.steps;
            assert(isequaln(weights,actor.Learnables),'FC:WeightsChanged');
            assert(isequaln(cfg,data.protocol.configuration) && isequaln(s,data.protocol.supervisor));
            filename=sprintf('%s_%s_%s.mat',item.id,routePolicy,method);
            save(fullfile(output,'flight',filename),'result','audit','item','method','routePolicy','wallSeconds','auditSeconds','errorText','auditText','-v7');
            row=fc_summarize(result,item,method,routePolicy,audit,wallSeconds);row.auditSeconds=auditSeconds;
            records=[records;row];outcomes{policyIndex,methodIndex}=result; %#ok<AGROW>
            writetable(struct2table(records),fullfile(output,'trials.csv'));
            fprintf('FC_FLIGHT %d/%d case=%s route=%s method=%s reason=%s time=%.2f activates=%d actor=%d rule=%d audit=%d wall=%.2fs\n', ...
                numel(records),6*numel(selected),item.id,routePolicy,method,result.reason,result.time,row.activations,row.actorCalls,row.ruleCalls,audit.passed,wallSeconds);
        end
        for policyIndex=1:2
            a=outcomes{policyIndex,1};b=outcomes{policyIndex,2};c=outcomes{policyIndex,3};
            pair=struct('id',string(item.id),'family',string(item.family),'routePolicy',string(policies{policyIndex}), ...
                'noneSuccess',a.success,'ruleSuccess',b.success,'actorSuccess',c.success, ...
                'ruleActivations',counter(b,'activations'),'actorActivations',counter(c,'activations'), ...
                'ruleNoChange',exact(a,b),'actorNoChange',exact(a,c));
            pairs=[pairs;pair]; %#ok<AGROW>
        end
        writetable(struct2table(pairs),fullfile(output,'pairs.csv'));
    end
    completion=struct('completed',true,'phase',phase,'shard',shard,'shards',shards,'cases',numel(selected), ...
        'flights',numel(records),'auditedSteps',auditedSteps,'allAuditsPassed',all([records.auditPassed]), ...
        'collisions',sum([records.collision]),'executionErrors',sum(strcmp([records.reason],'execution_error')), ...
        'actorLearnablesUnchanged',isequaln(weights,actor.Learnables),'configurationUnchanged',isequaln(cfg,data.protocol.configuration), ...
        'supervisorUnchanged',isequaln(s,data.protocol.supervisor),'modelInfo',modelInfo,'trainingPerformed',false,'criticLoaded',false);
    save(fullfile(output,'aggregate.mat'),'records','pairs','completion','-v7');
    fc_writejson(fullfile(output,'completion.json'),completion);
    fprintf('FC_COMPLETE shard=%d cases=%d flights=%d steps=%d collisions=%d errors=%d audits=%d\n',shard,numel(selected),numel(records),auditedSteps,completion.collisions,completion.executionErrors,completion.allAuditsPassed);
end
function count=counter(r,name)
    count=0;if isfield(r,'controller'),count=r.controller.(name);end
end
function yes=exact(a,b)
    yes=isfield(a.history,'state') && isfield(b.history,'state') && ...
        isequaln(a.history.state,b.history.state) && isequaln(a.history.command,b.history.command);
end
