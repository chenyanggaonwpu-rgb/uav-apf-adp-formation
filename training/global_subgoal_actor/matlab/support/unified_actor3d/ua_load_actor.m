function [actor,cfg,s,details]=ua_load_actor(assetName)
% One immutable model for every map. Verify both its training binding and the
% current inference interface; runtime changes have their own source manifest.
    if nargin<1,assetName='escape_actor_v2_20261104';end
    project=ua_setup();asset=fullfile(project,'controller_assets',assetName);
    manifest=jsondecode(fileread(fullfile(asset,'asset_manifest.json')));
    modelFile=fullfile(asset,'actor_candidate.mat');configFile=fullfile(asset,'configuration.mat');
    assert(strcmp(ua_sha256(modelFile),manifest.modelSha256),'UA:ModelHashMismatch');
    assert(strcmp(ua_sha256(configFile),manifest.configurationSha256),'UA:ConfigHashMismatch');
    config=load(configFile,'cfg');cfg=config.cfg;
    vars=whos('-file',modelFile);
    assert(isequal(sort({vars.name}),{'actorNet','metadata'}),'UA:UnexpectedModelContents');
    timer=tic;model=load(modelFile,'actorNet','metadata');loadSeconds=toc(timer);
    if isfield(manifest,'schemaVersion') && manifest.schemaVersion==2
        binding=model.metadata.trainingSourceManifest;
        entries=[binding.runtime.entries(:);binding.training(:)];
        assert(numel(entries)==manifest.trainingFiles,'UA:TrainingFileCountMismatch');
        for k=1:numel(entries)
            sourceFile=fullfile(asset,'training_reference',entries(k).path);
            assert(isfile(sourceFile) && strcmp(ua_sha256(sourceFile),entries(k).sha256), ...
                'UA:TrainingSourceBindingMismatch');
        end
        bound=binding.runtime.entries;interfacePrefix='neural_adp3d/';
        assert(model.metadata.parameterCount==manifest.parameterCount && ...
            strcmp(model.metadata.weightsSha256,manifest.weightsSha256),'UA:MetadataMismatch');
        assert(model.metadata.observationSize==cfg.observationSize && ...
            model.metadata.actionSize==cfg.actionSize && model.metadata.scanRange==cfg.scanRange && ...
            model.metadata.maxOffset==cfg.maxOffset && ...
            model.metadata.decisionPeriod==cfg.physicsStepsPerDecision*0.05,'UA:InterfaceMismatch');
        assert(~model.metadata.inferenceUsesExpert && model.metadata.fiveUAVTraining && ...
            model.metadata.actorActiveStagesOnly,'UA:TrainingProtocolMismatch');
        assert(model.metadata.training.initialActorTrainingEpisodes==manifest.training.initialActorTrainingEpisodes && ...
            model.metadata.training.newRLTrainingEpisodes==manifest.training.attemptedTD3Episodes && ...
            model.metadata.training.selectedRLEpisode==manifest.training.selectedRLEpisode && ...
            strcmp(model.metadata.training.selectedValidation.tag,manifest.training.selectedValidationTag), ...
            'UA:TrainingAccountingMismatch');
    else
        assert(model.metadata.completedTrainingEpisodes==manifest.completedTrainingEpisodes, ...
            'UA:TrainingEpisodeMismatch');
        auditCfg=cfg;auditCfg.moduleRoot=fullfile(asset,'training_reference','module');
        auditCfg.projectRoot=fullfile(asset,'training_reference','project');
        assert(isequaln(model.metadata.sourceManifest,adp3d_source_manifest(auditCfg)), ...
            'UA:TrainingSourceBindingMismatch');
        bound=model.metadata.sourceManifest.module;interfacePrefix='';
    end
    for name={'adp3d_actor_action.m','adp3d_observation.m','adp3d_decode_action.m'}
        entry=bound(strcmp({bound.path},[interfacePrefix name{1}]));assert(numel(entry)==1);
        assert(strcmp(ua_sha256(which(name{1}(1:end-2))),entry.sha256),'UA:InferenceInterfaceChanged');
    end
    actor=model.actorNet;assert(isempty(actor.State),'UA:StatefulActor');
    digest=java.security.MessageDigest.getInstance('SHA-256');count=0;
    for k=1:height(actor.Learnables)
        v=extractdata(actor.Learnables.Value{k});
        assert(isa(v,'single') && ~isa(v,'gpuArray') && all(isfinite(v(:))),'UA:InvalidWeights');
        digest.update(typecast(v(:),'uint8'));count=count+numel(v);
    end
    weightsSha256=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
    assert(count==manifest.parameterCount && strcmp(weightsSha256,manifest.weightsSha256),'UA:WeightsMismatch');
    cfg.moduleRoot=fullfile(project,'neural_adp3d');cfg.projectRoot=project;
    s=apf_adp_config();s.modelFile=modelFile;
    runtime=ua_runtime_manifest();
    details=struct('controllerID',manifest.controllerID,'modelFile',modelFile, ...
        'modelSha256',manifest.modelSha256,'weightsSha256',weightsSha256,'parameterCount',count, ...
        'sourceCheckpoint',manifest.sourceCheckpoint, ...
        'trainingBindingVerified',true,'inferenceInterfaceVerified',true, ...
        'cpuOnly',true,'actorLoaded',true,'criticLoaded',false,'agentLoaded',false, ...
        'trainingPerformed',false,'legacyADPCalls',0,'loadSeconds',loadSeconds, ...
        'runtimeSourceSha256',runtime.sha256);
    if isfield(manifest,'schemaVersion') && manifest.schemaVersion==2
        details.actorVersion='V2';details.training=manifest.training;
        details.actorAssetID=manifest.controllerID;
        if isfield(s,'controllerID'),details.controllerID=s.controllerID;end
        details.defaultElapsedTakeover=s.elapsedTakeover;
    else
        details.completedTrainingEpisodes=model.metadata.completedTrainingEpisodes;
    end
end
