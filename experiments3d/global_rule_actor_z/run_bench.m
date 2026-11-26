function run_bench(campaign,casesFile,outDir,arm,modelPath,saveTrajectories)
% Step-0 benchmark. The default arm is global_xy_z with the archived schema-6 Actor.
% rule uses the full-map route and APF. gate is the original learned XYZ.
% xy_z uses rule XY and Actor Z. rule_xy uses rule XY and current altitude.
% xy_z_apfoff uses the same XY+Z with the leader APF bypassed.
% global_xy_z uses the full rule route with certified Actor Z substitutions.
% global_gate permits G's verified short segments along the full rule route.
% Historical hop, apf and apfoff arm IDs are also accepted.
    if nargin<4 || isempty(arm),arm='global_xy_z';end
    if nargin<5,modelPath=[];end
    if nargin<6,saveTrajectories=false;end
    assert(ismember(arm,{'rule','apf','gate','hop','apfoff','xy_z','rule_xy','xy_z_apfoff','global_xy_z','global_gate'}), ...
        'UAV:UnknownBenchArm','Unknown benchmark arm %s.',arm);
    useDefaultModel=strcmp(arm,'global_xy_z') && isempty(modelPath);
    if useDefaultModel
        modelPath=fullfile(campaign,'assets','reference_models','retained149.mat');
    end
    addpath(fullfile(campaign,'matlab'));
    rl_setup();
    here=fileparts(mfilename('fullpath'));addpath(here);
    base=load(fullfile(campaign,'assets','base.mat'));
    cfgJson=jsondecode(fileread(fullfile(campaign,'config','local_cpu_long.json')));
    planner=cfgJson.environment;
    cases=jsondecode(fileread(casesFile));
    if isstruct(cases)&&numel(cases)>1,cases=num2cell(cases);end
    if ~iscell(cases),cases={cases};end
    net=struct();modelSha='';
    if ismember(arm,{'gate','hop','apfoff','xy_z','xy_z_apfoff','global_xy_z','global_gate'})
        assert(~isempty(modelPath) && isfile(modelPath),'UAV:MissingStep0Actor', ...
            'Supply the schema-6 Actor MAT file as modelPath.');
        modelSha=ua_sha256(modelPath);
        if useDefaultModel
            assert(strcmp(modelSha,'b2c051cacb795849dcfe223e85230f2657a9e17ab201809b33f974bef16c674a'), ...
                'UAV:Step0ActorHashMismatch','The default Actor MAT does not match the archived benchmark model.');
        end
        loaded=load(modelPath,'actor');net=loaded.actor;
    end
    if ~isfolder(outDir),mkdir(outDir);end
    logPath=fullfile(outDir,'flights.jsonl');
    if isfile(logPath),delete(logPath);end
    fid=fopen(logPath,'w','n','UTF-8');assert(fid>=0);
    cleanup=onCleanup(@()fclose(fid));
    allCycles=zeros(0,1);
    completed=0;success=0;unsafe=0;times=zeros(0,1);succTimes=zeros(0,1);

    for k=1:numel(cases)
        item=cases{k};
        caseStarted=tic;
        try
            [row,cyc,result]=step0_bench_flight(item,base,planner,arm,net,modelSha);
            if saveTrajectories,step0_save_trajectory(result,item.id,arm,outDir);end
            fprintf(fid,'%s\n',jsonencode(row));
            allCycles=[allCycles;cyc]; %#ok<AGROW>
            completed=completed+1;success=success+row.safe_success;
            unsafe=unsafe+double(~row.safe);
            times(end+1)=row.flight_seconds; %#ok<AGROW>
            if row.safe_success,succTimes(end+1)=row.flight_seconds;end %#ok<AGROW>
            fprintf('BENCH %s %s %s ok=%d t=%.1f ms_med=%.1f wall=%.1fs\n',arm,item.id, ...
                row.reason,row.safe_success,row.flight_seconds,row.ms_median,toc(caseStarted));
        catch exception
            fprintf(fid,'%s\n',jsonencode(struct('id',item.id,'error', ...
                getReport(exception,'extended','hyperlinks','off'))));
            fprintf(2,'BENCH_ERROR %s %s\n',item.id,exception.message);
        end
        summary=struct('arm',arm,'cases',numel(cases),'completed',completed, ...
            'success',success,'unsafe',unsafe,'model_sha256',modelSha, ...
            'mean_flight_seconds',mean(times),'mean_success_seconds',mean(succTimes), ...
            'cycles',numel(allCycles));
        fid2=fopen(fullfile(outDir,'summary.json.part'),'w','n','UTF-8');
        fprintf(fid2,'%s\n',jsonencode(summary));fclose(fid2);
        movefile(fullfile(outDir,'summary.json.part'),fullfile(outDir,'summary.json'),'f');
    end
    save(fullfile(outDir,'cycles.mat'),'allCycles','-v7');
    fprintf('BENCH_DONE %s %d/%d success=%d unsafe=%d\n',arm,completed,numel(cases),success,unsafe);
end
