function cfg = adp3d_config(mode)
% ADP3D_CONFIG 三维 TD3 配置。这里只生成配置，不启动训练。
    if nargin<1, mode='train'; end
    assert(ismember(string(mode),["train","diagnostic"]));
    cfg.interfaceVersion='adp3d-window-v1';
    cfg.moduleRoot=fileparts(mfilename('fullpath'));
    candidateRoot=fileparts(cfg.moduleRoot);
    if isfile(fullfile(candidateRoot,'config','default_params3d.m'))
        cfg.projectRoot=candidateRoot;
        cfg.outputRoot=fullfile(cfg.projectRoot,'results','neural_adp3d');
    elseif isfile(fullfile(cfg.moduleRoot,'runtime_source','config','default_params3d.m'))
        cfg.projectRoot=fullfile(cfg.moduleRoot,'runtime_source');
        cfg.outputRoot=fullfile(cfg.moduleRoot,'runs');
    else
        error('ADP3D:ProjectLayout','请将 neural_adp3d 放在三维项目目录内，或使用含 runtime_source 的便携包。');
    end
    cfg.mode=char(mode);
    cfg.seed=20261015;
    cfg.device='cpu';
    cfg.observationSize=32;
    cfg.actionSize=3;
    cfg.hiddenUnits=[64 64];
    cfg.scanRange=12;
    cfg.maxOffset=8;
    cfg.physicsStepsPerDecision=4; % 原物理 dt=0.05 s，网络每 0.2 s 决策。
    cfg.maxDecisions=300;
    cfg.maxEpisodes=2000;
    cfg.trainSeeds=1:64;
    cfg.validationSeeds=10001:10016;
    cfg.testSeeds=20001:20016;
    cfg.logEpisodes=true;
    cfg.plots='training-progress';
    cfg.checkpointEvery=25;
    cfg.safety=struct('tolerance',1e-6,'maxSubdivisionDepth',12, ...
        'boundaryBuffer',0.6,'stopSpeed',1e-10);
    cfg.reward=struct('progress',1,'timePerSecond',0.2, ...
        'intervention',0.5,'actionPerSecond',0.01, ...
        'success',150,'timeout',-25,'stall',-15,'collision',-200);
    cfg.stallSeconds=8;
    cfg.stallDisplacement=0.1;
    cfg.stallSpeed=0.08;
    cfg.td3=struct('discountTimeSeconds',40,'bufferLength',100000, ...
        'miniBatchSize',128,'warmStartSteps',1000,'actorLearnRate',1e-4, ...
        'criticLearnRate',3e-4,'targetSmoothFactor',0.005, ...
        'policyUpdateFrequency',2,'learningFrequency',1,'maxMiniBatchPerEpoch',1, ...
        'explorationStd',0.25, ...
        'explorationStdMin',0.05,'explorationDecay',1e-5, ...
        'targetNoiseStd',0.15,'targetNoiseLimit',0.3);
    if strcmp(mode,'diagnostic')
        cfg.maxEpisodes=2;
        cfg.maxDecisions=8;
        cfg.trainSeeds=1:4;
        cfg.td3.miniBatchSize=8;
        cfg.td3.warmStartSteps=8;
        cfg.td3.bufferLength=1000;
        cfg.checkpointEvery=1;
        cfg.plots='none';
    end
end
