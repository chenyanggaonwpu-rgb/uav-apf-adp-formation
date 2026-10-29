function ua_prepare_campaign(output)
% Freeze every requested 2.3.4 campaign before formal flights; outcomes never
% select maps. Original experiment geometry and absolute gust times are reused.
    project=ua_setup();if ~isfolder(output),mkdir(output);end
    [~,cfg,s,model]=ua_load_actor();items=cell(0,1);jobs=cell(0,1);
    for folder={'inputs','flight','traces','records','progress','reports'},mkdir(fullfile(output,folder{1}));end
    production=load(fullfile(project,'results','planning_block_fix_20261013','inputs','trial_010001_S.mat'),'q');
    base=production.q;base.adp.enable=false;base.adp.wA=zeros(3,6);base.adp.wC=zeros(6,1);
    for study={'adp_repair_regression_20261011_r2','adp_repair_extension_20261011_r2'}
        root=fullfile(project,'交付文件','ADP排查与修复_20261012','results',study{1});
        maps=jsondecode(fileread(fullfile(root,'maps.json')));
        for k=1:numel(maps)
            file=fullfile(root,maps(k).mapFile);v=load(file,'q','scene');
            item=make(sprintf('repair_%06d',maps(k).seed),'repair',maps(k).population,v.q,v.scene,'global');
            item.originalMapFile=file;item.originalMapSha256=ua_sha256(file);idx=addItem(item);four(idx);
        end
    end
    cases=jsondecode(fileread(fullfile(project,'交付文件','APF切向退化动态检查_20261029','cases.json')));
    for k=1:numel(cases)
        c=cases(k);p=default_params3d();p.start=c.start(:);p.target=c.target(:);p.seed=20261029;
        [initial,~,~]=init_formation3d(p);initial(4:6,:)=repmat(c.velocity(:),1,p.nUAV);p.initialState=initial;
        policy='global';if c.forceDirectRoute,policy='local_apf';end
        scene=struct('id',c.id,'kind','fixed','seed',p.seed,'obstacles',c.obstacle);
        item=make(c.id,'dynamic',[c.suite '_' c.family],p,scene,policy);idx=addItem(item);four(idx);
    end
    maps=jsondecode(fileread(fullfile(project,'交付文件','噪声与外扰鲁棒性实验_20261029','maps.json')));
    oldJobs=jsondecode(fileread(fullfile(project,'交付文件','噪声与外扰鲁棒性实验_20261029','jobs.json')));
    for k=1:numel(maps)
        c=maps(k);p=default_params3d();p.start=c.start(:);p.target=c.target(:);p.seed=20261029;
        scene=struct('id',c.id,'kind','fixed','seed',p.seed,'obstacles',c.obstacle);
        idx=addItem(make(c.id,'robust',c.family,p,scene,'global'));
        selected=oldJobs(strcmp({oldJobs.map_id},c.id) & strcmp({oldJobs.group},'S'));
        assert(numel(selected)==38);
        for e=selected(:)'
            for a=0:1
                for guard=0:1
                    job=newJob(idx,method(a),guard,'global',e.tag);
                    job.noiseStd=e.sigma;job.noiseSeed=e.noise_seed;job.gustAmplitude=e.amplitude;
                    job.gustDirection=e.vector(:);job.gustStartStep=e.start_step;job.gustDuration=2;
                    job.kind=e.kind;job.repeat=e.repeat;job.direction=e.direction;
                    job.matchedReference=sprintf('robust_%s_%s_%s_S%d',c.id,extractReference(e),method(a),guard);
                    jobs{end+1,1}=job;
                end
            end
        end
    end
    f5=f5_cases('formal',base,cfg);assert(numel(f5)==128);
    for k=1:numel(f5)
        v=f5{k};idx=addItem(make(v.id,'integration',v.family,v.parameters,v.scene,v.routePolicy));
        for a=0:1,jobs{end+1,1}=newJob(idx,method(a),true,v.routePolicy,'B');end
    end
    v=load(fullfile(project,'results','five_uav_comparison3d','campaign_20261017_210011','catalogue_before_flight.mat'),'cases');
    assert(numel(v.cases)==144);
    for k=1:numel(v.cases)
        c=v.cases{k};idx=addItem(make(c.id,'factorial',c.family,c.parameters,c.scene,'global'));
        order=circshift(1:6,[0 mod(k-1,6)]);methods={'none','rule','actor'};policies={'global','local_apf'};
        for combination=order
            m=methods{mod(combination-1,3)+1};policy=policies{ceil(combination/3)};
            jobs{end+1,1}=newJob(idx,m,true,policy,'B');
        end
    end
    single=apf_adp_cases('heldout',cfg,s);assert(numel(single)==40);
    for k=1:numel(single)
        c=single{k};item=make(c.id,'single',c.family,c.parameters,struct('id',c.id,'kind','fixed','obstacles',c.obstacles),'local_apf');
        item.singleCase=c;idx=addItem(item);
        for a=0:1,jobs{end+1,1}=newJob(idx,method(a),true,'local_apf','B');end
    end
    assert(numel(jobs)==3840);
    index=struct([]);
    for k=1:numel(items)
        item=items{k};file=fullfile(output,'inputs',[item.campaign '_' item.id '.mat']);save(file,'item','-v7');
        index=[index;struct('index',k,'id',string(item.id),'campaign',string(item.campaign), ...
            'family',string(item.family),'inputSha256',string(ua_sha256(file)))]; %#ok<AGROW>
    end
    protocol=struct('controllerID',model.controllerID,'model',model,'configuration',cfg,'supervisor',s, ...
        'counts',struct('repair',616,'dynamic',200,'robust',1824,'integration',256,'factorial',864,'single',80), ...
        'totalTrials',numel(jobs),'catalogueItems',numel(items),'frozenBeforeFormalFlights',true, ...
        'ablationArms',{{'none_S0','none_S1','actor_S0','actor_S1'}}, ...
        'defaultMethod','actor','defaultSafety',true,'actorPerMapReset',false,'onlineLearning',false, ...
        'noise','Independent per-axis position noise each step; same random stream for all paired arms; velocity feedback remains exact', ...
        'gust','Common 3D acceleration applied to all five vehicles after the command norm limit, for 2 seconds', ...
        'trigger','Reuse original preregistered absolute pulse step for exact timing comparability; real distance at pulse is reported', ...
        'truthUse','True state for plant and evaluation only; supervisor history, feedback and safety use measured state', ...
        'safetyOff','Disable predictive repair AND joint braking guardian; independent true swept check stays enabled', ...
        'safetyOn','Predictive repair plus joint braking selection; actual disturbed plant always advances', ...
        'success','Five vehicles: original leader goal, worst formation error, maximum speed, unblocked reference and 1-second hold. Robust trials also observe through pulse end plus 5 seconds', ...
        'singleSuccess','Historical one-vehicle diagnostic uses its original goal/speed gate; reported separately', ...
        'recovery','All-vehicle position error <=0.5 m and velocity error <=0.25 m/s relative to matched no-gust run for 1 continuous second within 5 seconds after gust end', ...
        'timing','Concurrent batch latency is diagnostic only; serial timing measured separately', ...
        'scope','Known-map 2.3.4; excludes historical 2D and unrelated unknown-map 3.0 development');
    save(fullfile(output,'catalogue.mat'),'items','jobs','protocol','-v7');
    writejson(fullfile(output,'protocol.json'),protocol);writejson(fullfile(output,'jobs.json'),jobs);
    writetable(struct2table(index),fullfile(output,'catalogue.csv'));
    fprintf('UA_CATALOGUE_FROZEN items=%d jobs=%d\n',numel(items),numel(jobs));
    function idx=addItem(item),items{end+1,1}=item;idx=numel(items);end
    function four(idx)
        for a=0:1,for guard=0:1,jobs{end+1,1}=newJob(idx,method(a),guard,items{idx}.routePolicy,'B');end,end
    end
    function job=newJob(idx,m,guard,policy,tag)
        item=items{idx};suffix='';if strcmp(item.campaign,'factorial'),suffix=['_' policy];end
        id=sprintf('%s_%s_%s_%s_S%d%s',item.campaign,item.id,tag,m,guard,suffix);
        job=struct('id',id,'itemIndex',idx,'campaign',item.campaign,'mapID',item.id,'family',item.family, ...
            'method',m,'safety',logical(guard),'routePolicy',policy,'tag',tag,'kind','baseline', ...
            'noiseStd',0,'noiseSeed',2026102901,'gustAmplitude',0,'gustDirection',zeros(3,1), ...
            'gustStartStep',1,'gustDuration',2,'repeat',0,'direction','none','matchedReference','');
    end
end
function item=make(id,campaign,family,p,scene,policy)
    p.adp.enable=false;p.adp.wA=zeros(3,6);p.adp.wC=zeros(6,1);p.animate=false;
    if ~isfield(p,'planning'),p.planning=struct('prebuildGraphs',true);end
    item=struct('id',id,'campaign',campaign,'family',family,'parameters',p,'scene',scene,'routePolicy',policy);
end
function m=method(a),if a,m='actor';else,m='none';end,end
function tag=extractReference(e)
    if e.sigma==0,tag='B';else,tag=sprintf('N%03d_n%02d',round(e.sigma*1000),e.repeat);end
end
function writejson(file,value),fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value,PrettyPrint=true));fclose(fid);end
