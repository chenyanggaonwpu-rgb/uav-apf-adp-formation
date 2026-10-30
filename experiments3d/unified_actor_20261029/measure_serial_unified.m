function measure_serial_unified(output)
% Execute only after the concurrent campaign has stopped. All complete control
% cycles include synchronous replanning; disk I/O and plotting stay outside.
    project=fileparts(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(project,'unified_actor3d'));ua_setup();
    if ~isfolder(output),mkdir(output);end
    [actor,cfg,s,model]=ua_load_actor();data=load(fullfile(project,'results','unified_actor_20261029','catalogue.mat'),'items');
    metadataFile=load(s.modelFile,'metadata');metadata=rmfield(metadataFile.metadata,'sourceManifest');
    fid=fopen(fullfile(output,'model_metadata.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(metadata,PrettyPrint=true));fclose(fid);
    catalogue=cell(size(data.items));
    for k=1:numel(data.items)
        item=data.items{k};q=item.parameters;
        if isfield(item,'singleCase'),initial=item.singleCase.initialState;
        elseif isfield(q,'initialState'),initial=q.initialState;
        else,[initial,~,~]=init_formation3d(q);end
        catalogue{k}=struct('id',item.id,'campaign',item.campaign,'family',item.family, ...
            'start',q.start,'target',q.target,'workspace',q.workspace,'obstacles',item.scene.obstacles,'initialState',initial);
    end
    fid=fopen(fullfile(output,'case_geometry.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(catalogue,PrettyPrint=true));fclose(fid);
    chosen={'S0_empty','S1_sphere','S8_window','normal_61036','window_63001'};
    cold=zeros(15,1);for k=1:15,t=tic;adp3d_actor_action(actor,zeros(32,1));cold(k)=1000*toc(t);end
    records=struct([]);allCycles=cell(0,1);timingDetails=cell(0,1);
    for k=1:numel(chosen)
        match=find(cellfun(@(c)strcmp(c.campaign,'integration')&&strcmp(c.id,chosen{k}),data.items));assert(numel(match)==1);
        item=data.items{match};p=item.parameters;p.adp.enable=false;p.safety.enable=true;
        % Prewarm the entire selected map's code path once, separately saved.
        warm=ua_simulate(p,item.scene,[],'actor',actor,cfg,s,item.routePolicy);
        save(fullfile(output,['warm_' chosen{k} '.mat']),'warm','-v7');
        for repeat=1:3
            result=ua_simulate(p,item.scene,[],'actor',actor,cfg,s,item.routePolicy);
            c=result.history.cycleMs;row=stats(c);row.caseID=chosen{k};row.dimension='3D five UAV';row.repeat=repeat;
            row.actorCalls=result.controller.actorCalls;row.reason=result.reason;
            row.initialPlanSeconds=result.timing.initialPlanSeconds;row.preparationSeconds=result.timing.preparationSeconds;
            row.replanEvents=numel(result.timing.replanSeconds);row.terminalPlanEvents=numel(result.timing.terminalPlanSeconds);
            row.maxReplanSeconds=max([0;result.timing.replanSeconds]);row.maxTerminalPlanSeconds=max([0;result.timing.terminalPlanSeconds]);
            records=[records;row];allCycles{end+1,1}=c;timingDetails{end+1,1}=result.timing; %#ok<AGROW>
            writematrix(c,fullfile(output,sprintf('%s_r%d_cycle_ms.csv',chosen{k},repeat)));
            save(fullfile(output,sprintf('%s_r%d.mat',chosen{k},repeat)),'result','-v7');
            fprintf('SERIAL %s repeat=%d p99=%.3f max=%.3f over50=%d\n',chosen{k},repeat,row.p99Ms,row.maxMs,row.over50);
        end
    end
    for obstacleCount=[0 15]
        p=default_params();p.nObstacles=obstacleCount;p.Tmax=120;p.animate=false;p.seed=5;
        p.noiseStd=0;p.disturbanceMax=0;warm=run_simulation(p); %#ok<NASGU>
        for repeat=1:3
            r=run_simulation(p);c=r.history.cycleMs;row=stats(c);
            row.caseID=sprintf('legacy_2d_obs%d',obstacleCount);row.dimension='2D historical reference';row.repeat=repeat;
            row.actorCalls=0;row.reason=r.reason;row.initialPlanSeconds=NaN;row.preparationSeconds=NaN;
            row.replanEvents=NaN;row.terminalPlanEvents=NaN;row.maxReplanSeconds=NaN;row.maxTerminalPlanSeconds=NaN;
            records=[records;row];allCycles{end+1,1}=c; %#ok<AGROW>
            save(fullfile(output,sprintf('%s_r%d.mat',row.caseID,repeat)),'r','-v7');
            writematrix(c,fullfile(output,sprintf('%s_r%d_cycle_ms.csv',row.caseID,repeat)));
        end
    end
    writetable(struct2table(records),fullfile(output,'serial_timing.csv'));
    environment=struct('matlab',version,'computer',computer,'singleComputationalThread',true, ...
        'concurrentCampaignStopped',true,'warmFlightsExcluded',true,'inferenceWarmupMs',cold, ...
        'diskAndPlotsInCycle',false,'synchronousEventPlanningInCycle',true,'model',model);
    save(fullfile(output,'timing_arrays.mat'),'allCycles','records','environment','timingDetails','-v7');
    fid=fopen(fullfile(output,'environment.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(environment,PrettyPrint=true));fclose(fid);
    set(groot,'defaultFigureVisible','off');smokeResult=main();
    assert(smokeResult.success && smokeResult.model.parameterCount==6467 && ~smokeResult.params.adp.enable,'UA:MainSmokeFailed');
    save(fullfile(output,'main_entry_smoke.mat'),'smokeResult','-v7');
    fprintf('SERIAL_TIMING_COMPLETE flights=%d\n',numel(records));
end
function row=stats(c)
    row=struct('cycles',numel(c),'meanMs',mean(c),'p95Ms',prctile(c,95),'p99Ms',prctile(c,99), ...
        'maxMs',max(c),'over50',nnz(c>50),'over50Fraction',mean(c>50));
end
