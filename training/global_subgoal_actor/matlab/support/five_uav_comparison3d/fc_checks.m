function checks=fc_checks(project,output,actor,cfg,s)
% Meaningful regression: executed states/commands must match sealed flights.
    records=struct([]);names={'window_63001','S8_window'};
    for i=1:numel(names)
        for oldMode={'baseline','automatic'}
            files=dir(fullfile(project,'results','five_uav_actor3d','validation_s*','flight',[names{i} '_' oldMode{1} '.mat']));
            assert(numel(files)==1,'FC:MissingRegressionFlight');
            previous=load(fullfile(files(1).folder,files(1).name),'result','item');
            if strcmp(oldMode{1},'baseline'),method='none';else,method='actor';end
            item=previous.item;
            r=fc_simulate(item.parameters,item.scene,[],method,actor,cfg,s,item.routePolicy);
            audit=f5_audit(r,item,cfg);
            exact=isequaln(r.history.state,previous.result.history.state) && isequaln(r.history.command,previous.result.history.command);
            assert(exact && r.success==previous.result.success && strcmp(r.reason,previous.result.reason),'FC:AdapterRegression');
            records=[records;struct('case',string(names{i}),'method',string(method),'routePolicy',string(item.routePolicy),'exact',exact,'steps',audit.steps)]; %#ok<AGROW>
            save(fullfile(output,[names{i} '_' method '_regression.mat']),'r','audit','exact','-v7');
            fprintf('FC_REGRESSION case=%s method=%s exact=%d\n',names{i},method,exact);
        end
    end
    observation=[1;0;0;zeros(3,1);ones(26,1)];
    [action,~]=fc_rule_action(observation,cfg,.6,zeros(3,1));
    assert(norm(action-[1;0;0])<1e-12);
    observation(7:32)=0;[action,~]=fc_rule_action(observation,cfg,.6,zeros(3,1));
    assert(all(action==0));
    checks=struct('passed',true,'sealedFlightRegressions',records,'boundedRuleContract',true, ...
        'originalSafetyChecks',f5_checks(loadParameters(project),cfg));
    fc_writejson(fullfile(output,'adapter_checks.json'),checks);
end
function p=loadParameters(project)
    data=load(fullfile(project,'results','planning_block_fix_20261013','inputs','trial_010001_S.mat'),'q');p=data.q;p.adp.enable=false;
end
