function [agent,networkInfo,observationInfo,actionInfo] = adp3d_build_agent(cfg,p)
% TD3 的双 Critic、延迟 Actor 更新、目标网络和平滑噪声由 RL 工具箱执行。
    [actorNet,c1,c2,networkInfo]=adp3d_create_networks(cfg);
    observationInfo=rlNumericSpec([32 1],'Name','observation','DataType','single');
    actionInfo=rlNumericSpec([3 1],'LowerLimit',-ones(3,1), ...
        'UpperLimit',ones(3,1),'Name','action');
    actor=rlContinuousDeterministicActor(actorNet,observationInfo,actionInfo, ...
        'ObservationInputNames','observation','UseDevice',cfg.device);
    critic1=rlQValueFunction(c1,observationInfo,actionInfo, ...
        'ObservationInputNames','observation','ActionInputNames','action','UseDevice',cfg.device);
    critic2=rlQValueFunction(c2,observationInfo,actionInfo, ...
        'ObservationInputNames','observation','ActionInputNames','action','UseDevice',cfg.device);
    t=cfg.td3;
    opts=rlTD3AgentOptions;
    opts.SampleTime=p.dt*cfg.physicsStepsPerDecision;
    opts.DiscountFactor=exp(-opts.SampleTime/t.discountTimeSeconds);
    opts.ExperienceBufferLength=t.bufferLength;
    opts.MiniBatchSize=t.miniBatchSize;
    opts.NumWarmStartSteps=t.warmStartSteps;
    opts.LearningFrequency=t.learningFrequency;
    opts.MaxMiniBatchPerEpoch=t.maxMiniBatchPerEpoch;
    opts.NumEpoch=1;
    opts.TargetSmoothFactor=t.targetSmoothFactor;
    opts.PolicyUpdateFrequency=t.policyUpdateFrequency;
    opts.TargetUpdateFrequency=t.policyUpdateFrequency;
    opts.ActorOptimizerOptions=rlOptimizerOptions('LearnRate',t.actorLearnRate,'GradientThreshold',1);
    optimizer=rlOptimizerOptions('LearnRate',t.criticLearnRate,'GradientThreshold',1);
    opts.CriticOptimizerOptions=[optimizer optimizer];
    opts.ExplorationModel.StandardDeviation=t.explorationStd;
    opts.ExplorationModel.StandardDeviationMin=t.explorationStdMin;
    opts.ExplorationModel.StandardDeviationDecayRate=t.explorationDecay;
    opts.TargetPolicySmoothModel.StandardDeviation=t.targetNoiseStd;
    opts.TargetPolicySmoothModel.LowerLimit=-t.targetNoiseLimit;
    opts.TargetPolicySmoothModel.UpperLimit=t.targetNoiseLimit;
    opts.ResetExperienceBufferBeforeTraining=false;
    opts.InfoToSave=struct('ExperienceBuffer',true,'Optimizer',true, ...
        'PolicyState',true,'Target',true);
    agent=rlTD3Agent(actor,[critic1 critic2],opts);
end
