function report = adp3d_evaluate(modelFile,split,cfg)
% 保留场景上比较 APF 与确定性 Actor；不更新网络、不筛掉失败。
% report=adp3d_evaluate('.../actor_candidate.mat','validation');
% test 只在完成调参后作最终评估；不能用 test 成绩选模型。
    if nargin<2, split='validation'; end
    if nargin<3, cfg=adp3d_config; end
    p=adp3d_setup(cfg);
    switch split
        case 'validation', seeds=cfg.validationSeeds;
        case 'test', seeds=cfg.testSeeds;
        otherwise, error('ADP3D:BadSplit','split 必须为 validation 或 test。');
    end
    loaded=load(modelFile);
    if isfield(loaded,'actorNet')
        actorNet=loaded.actorNet;
        assert(strcmp(loaded.metadata.interfaceVersion,cfg.interfaceVersion));
        assert(loaded.metadata.scanRange==cfg.scanRange && loaded.metadata.maxOffset==cfg.maxOffset ...
            && loaded.metadata.decisionPeriod==p.dt*cfg.physicsStepsPerDecision, ...
            'ADP3D:EvaluationInterfaceMismatch');
        manifest=loaded.metadata.sourceManifest;
    else
        actorNet=getModel(getActor(loaded.checkpoint.agent));
        assert(strcmp(loaded.checkpoint.cfg.interfaceVersion,cfg.interfaceVersion));
        assert(loaded.checkpoint.cfg.scanRange==cfg.scanRange ...
            && loaded.checkpoint.cfg.maxOffset==cfg.maxOffset ...
            && loaded.checkpoint.cfg.physicsStepsPerDecision==cfg.physicsStepsPerDecision, ...
            'ADP3D:EvaluationInterfaceMismatch');
        manifest=loaded.checkpoint.sourceManifest;
    end
    assert(isequaln(manifest,adp3d_source_manifest(cfg)), ...
        'ADP3D:EvaluationSourceMismatch','模型对应的源码与当前版本不同。');
    catalog=adp3d_case_catalog(seeds,cfg,p);
    if ~isfolder(cfg.outputRoot), mkdir(cfg.outputRoot); end
    outputDirectory=[tempname(cfg.outputRoot) '_' split];
    mkdir(outputDirectory);
    rows={};
    for mode={'apf','actor'}
        for k=1:numel(catalog)
            manager=Adp3dEpisodeManager(cfg,catalog(k),fullfile(outputDirectory,mode{1}));
            manager.ControlMode=mode{1};
            [observation,info]=manager.reset(); done=false;
            while ~done
                if strcmp(mode{1},'actor'), action=adp3d_actor_action(actorNet,observation);
                else, action=zeros(3,1); end
                [observation,~,done,info]=manager.step(action,info);
            end
            % 每个场景独立文件，完整保存全部轨迹。
            episode=info; summary=manager.LastSummary; %#ok<NASGU>
            save(fullfile(outputDirectory,sprintf('%s_seed_%d.mat',mode{1},catalog(k).seed)), ...
                'episode','summary','-v7');
            row=manager.LastSummary;
            row.controller=mode{1};
            row.safetyInterventionFraction=sum(info.interventions)/numel(info.interventions);
            rows{end+1}=row; %#ok<AGROW>
        end
    end
    allRows=struct2table([rows{:}]);
    writetable(allRows,fullfile(outputDirectory,'all_cases.csv'));
    report=struct('split',split,'modelFile',modelFile,'outputDirectory',outputDirectory, ...
        'scope','single UAV, static known-map window-wall family', ...
        'seeds',seeds,'casesPerController',numel(seeds));
    for mode={'apf','actor'}
        selected=strcmp(allRows.controller,mode{1});
        report.(mode{1})=struct('successRate',mean(allRows.success(selected)), ...
            'collisionCount',sum(allRows.collision(selected)), ...
            'medianFinalGoalDistance',median(allRows.finalGoalDistance(selected)), ...
            'meanInterventionFraction',mean(allRows.safetyInterventionFraction(selected)));
    end
    save(fullfile(outputDirectory,'evaluation.mat'),'report','allRows','cfg','-v7');
    fprintf('%s: APF %.1f%%; Actor %.1f%%; collisions %d / %d\n',split, ...
        100*report.apf.successRate,100*report.actor.successRate, ...
        report.apf.collisionCount,report.actor.collisionCount);
end
