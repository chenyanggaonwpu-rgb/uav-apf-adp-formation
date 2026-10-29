function result = adp3d_train(cfg,resumeFile)
% ADP3D_TRAIN 显式调用才开始训练；配置 diagnostic 仅运行 2x8 步连通性检查。
% 正式：cfg=adp3d_config; result=adp3d_train(cfg);
% 续训：cfg.maxEpisodes=4000; result=adp3d_train(cfg,'.../checkpoint_000025.mat');
    if nargin<1, cfg=adp3d_config; end
    if nargin<2, resumeFile=''; end
    p=adp3d_setup(cfg);
    assert(isempty(intersect(cfg.trainSeeds,[cfg.validationSeeds cfg.testSeeds])));
    assert(isempty(intersect(cfg.validationSeeds,cfg.testSeeds)));
    if ~isfolder(cfg.outputRoot), mkdir(cfg.outputRoot); end
    [~,uniqueName]=fileparts(tempname(cfg.outputRoot));
    runDirectory=fullfile(cfg.outputRoot,[cfg.mode '_' datestr(now,'yyyymmdd_HHMMSS') '_' uniqueName]);
    mkdir(runDirectory);
    sourceManifest=adp3d_source_manifest(cfg);
    completedEpisodes=0;
    if isempty(resumeFile)
        rng(cfg.seed,'twister');
        [agent,networkInfo]=adp3d_build_agent(cfg,p);
        catalog=adp3d_case_catalog(cfg.trainSeeds,cfg,p);
        savedSchedule=[];
        savedRng=rng;
    else
        loaded=load(resumeFile,'checkpoint');
        checkpoint=loaded.checkpoint;
        assert(strcmp(cfg.mode,checkpoint.cfg.mode), ...
            'ADP3D:DiagnosticResume','诊断模型不能作为正式训练断点。');
        omit={'moduleRoot','projectRoot','outputRoot','plots','maxEpisodes'};
        assert(isequaln(rmfield(cfg,omit),rmfield(checkpoint.cfg,omit)), ...
            'ADP3D:ResumeConfigMismatch','续训配置与断点不匹配。');
        assert(isequaln(sourceManifest,checkpoint.sourceManifest), ...
            'ADP3D:ResumeSourceMismatch','源码已改变；请保留旧版本或启动新实验。');
        agent=checkpoint.agent; catalog=checkpoint.catalog;
        networkInfo=checkpoint.networkInfo;
        completedEpisodes=checkpoint.completedEpisodes;
        savedSchedule=checkpoint.scheduleState;
        savedRng=checkpoint.rngState;
    end
    assert(cfg.maxEpisodes>completedEpisodes,'ADP3D:NoEpisodesRemaining');
    [env,manager]=adp3d_create_env(cfg,catalog,fullfile(runDirectory,'episodes'));
    if ~isempty(savedSchedule), manager.restoreSchedule(savedSchedule); end
    rng(savedRng);
    save(fullfile(runDirectory,'configuration.mat'),'cfg','catalog','sourceManifest','networkInfo','-v7');
    writeJson(fullfile(runDirectory,'configuration.json'),cfg);
    writeJson(fullfile(runDirectory,'source_manifest.json'),sourceManifest);
    beforeWeights=adp3d_agent_weights(agent);
    writeCheckpoint(completedEpisodes,struct(),'initial');
    options=rlTrainingOptions('MaxEpisodes',cfg.maxEpisodes-completedEpisodes, ...
        'MaxStepsPerEpisode',cfg.maxDecisions,'ScoreAveragingWindowLength',25, ...
        'StopTrainingCriteria','EpisodeCount', ...
        'StopTrainingValue',cfg.maxEpisodes-completedEpisodes, ...
        'SaveAgentCriteria','Custom','SaveAgentValue',@checkpointCallback, ...
        'SaveAgentDirectory',fullfile(runDirectory,'toolbox_checkpoints'), ...
        'Verbose',true,'Plots',cfg.plots,'UseParallel',false);
    fprintf('ADP3D mode=%s; Actor=%d, Critic=%d+%d parameters; device=%s\n', ...
        cfg.mode,networkInfo.actorParameters,networkInfo.critic1Parameters, ...
        networkInfo.critic2Parameters,cfg.device);
    fprintf('Output: %s\n',runDirectory);
    lastCompletedEpisode=completedEpisodes;
    clockStart=tic;
    try
        nativeStats=train(agent,env,options);
    catch failure
        partialEpisode=manager.LastInfo; %#ok<NASGU>
        save(fullfile(runDirectory,'interrupted_episode.mat'),'partialEpisode','-v7');
        writeCheckpoint(lastCompletedEpisode,struct('message',failure.message),'interrupted');
        rethrow(failure);
    end
    trainingWallSeconds=toc(clockStart);
    % 保存纯数据；工具箱原始结果的 TrainingOptions 含嵌套回调，不能连带序列化整个训练上下文。
    trainingStats=struct('EpisodeIndex',nativeStats.EpisodeIndex, ...
        'EpisodeReward',nativeStats.EpisodeReward,'EpisodeSteps',nativeStats.EpisodeSteps, ...
        'AverageReward',nativeStats.AverageReward,'EpisodeQ0',nativeStats.EpisodeQ0, ...
        'ActorGradientSteps',nativeStats.ActorGradientSteps, ...
        'CriticGradientSteps',nativeStats.CriticGradientSteps);
    afterWeights=adp3d_agent_weights(agent);
    deltas=struct('actor',norm(afterWeights.actor-beforeWeights.actor), ...
        'critic1',norm(afterWeights.critic1-beforeWeights.critic1), ...
        'critic2',norm(afterWeights.critic2-beforeWeights.critic2));
    actualEpisodes=numel(trainingStats.EpisodeIndex);
    totalDecisions=sum(trainingStats.EpisodeSteps);
    completed=completedEpisodes+actualEpisodes;
    writeCheckpoint(completed,trainingStats,'final');
    result=struct('runDirectory',runDirectory,'mode',cfg.mode, ...
        'formalTraining',strcmp(cfg.mode,'train'),'completedEpisodes',completed, ...
        'episodesThisRun',actualEpisodes,'decisionsThisRun',totalDecisions, ...
        'trainingWallSeconds',trainingWallSeconds, ...
        'decisionsPerSecond',totalDecisions/max(trainingWallSeconds,eps), ...
        'parameterChangeL2',deltas,'networkInfo',networkInfo, ...
        'actorGradientSteps',sum(trainingStats.ActorGradientSteps), ...
        'criticGradientSteps',sum(trainingStats.CriticGradientSteps), ...
        'lastEpisode',manager.LastSummary,'policyValidated',false);
    save(fullfile(runDirectory,'training_result.mat'),'result','trainingStats', ...
        'beforeWeights','afterWeights','-v7');
    writeJson(fullfile(runDirectory,'training_result.json'),result);
    if strcmp(cfg.mode,'train')
        % 导出只是训练后的候选 Actor；必须再在保留种子上验证，不能直接宣称可部署。
        adp3d_export_actor(agent,cfg,sourceManifest, ...
            fullfile(runDirectory,'actor_candidate.mat'),completed);
    end

    function flag=checkpointCallback(stats)
        % R2024a 的自定义保存回调收到截至当前回合的 EpisodeIndex 向量。
        episode=completedEpisodes+stats.EpisodeIndex(end);
        lastCompletedEpisode=episode;
        if mod(episode,cfg.checkpointEvery)==0
            writeCheckpoint(episode,stats,sprintf('checkpoint_%06d',episode));
        end
        flag=false; % 我们同时保存环境抽样状态，代替工具箱的 agent-only 文件。
    end

    function writeCheckpoint(episode,stats,label)
        if isstruct(stats)
            remove=intersect(fieldnames(stats),{'TrainingOptions','SimulationInfo'});
            stats=rmfield(stats,remove);
        end
        checkpoint=struct('agent',agent,'cfg',cfg,'catalog',catalog, ...
            'networkInfo',networkInfo,'sourceManifest',sourceManifest, ...
            'completedEpisodes',episode,'scheduleState',manager.scheduleState(), ...
            'rngState',rng,'stats',stats,'policyValidated',false); %#ok<NASGU>
        temporary=fullfile(runDirectory,[label '.pending.mat']);
        save(temporary,'checkpoint','-v7.3');
        movefile(temporary,fullfile(runDirectory,[label '.mat']));
    end
end

function writeJson(file,value)
    fid=fopen(file,'w','n','UTF-8'); assert(fid>=0);
    cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end
