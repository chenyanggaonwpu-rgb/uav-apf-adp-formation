function [actor,cfg,s,details]=ua_load_actor()
% One immutable model for every map. Verify both its training binding and the
% current inference interface; runtime changes have their own source manifest.
    project=ua_setup();asset=fullfile(project,'controller_assets','frozen_actor_20261015');
    manifest=jsondecode(fileread(fullfile(asset,'asset_manifest.json')));
    modelFile=fullfile(asset,'actor_candidate.mat');configFile=fullfile(asset,'configuration.mat');
    assert(strcmp(ua_sha256(modelFile),manifest.modelSha256),'UA:ModelHashMismatch');
    assert(strcmp(ua_sha256(configFile),manifest.configurationSha256),'UA:ConfigHashMismatch');
    config=load(configFile,'cfg');cfg=config.cfg;
    vars=whos('-file',modelFile);
    assert(all(ismember({vars.name},{'actorNet','metadata'})),'UA:UnexpectedModelContents');
    timer=tic;model=load(modelFile,'actorNet','metadata');loadSeconds=toc(timer);
    auditCfg=cfg;auditCfg.moduleRoot=fullfile(asset,'training_reference','module');
    auditCfg.projectRoot=fullfile(asset,'training_reference','project');
    assert(isequaln(model.metadata.sourceManifest,adp3d_source_manifest(auditCfg)), ...
        'UA:TrainingSourceBindingMismatch');
    bound=model.metadata.sourceManifest.module;
    for name={'adp3d_actor_action.m','adp3d_observation.m','adp3d_decode_action.m'}
        entry=bound(strcmp({bound.path},name{1}));assert(numel(entry)==1);
        assert(strcmp(ua_sha256(which(name{1}(1:end-2))),entry.sha256),'UA:InferenceInterfaceChanged');
    end
    actor=model.actorNet;assert(isempty(actor.State),'UA:StatefulActor');
    digest=java.security.MessageDigest.getInstance('SHA-256');count=0;
    for k=1:height(actor.Learnables)
        v=extractdata(actor.Learnables.Value{k});assert(isa(v,'single') && ~isa(v,'gpuArray'));
        digest.update(typecast(v(:),'uint8'));count=count+numel(v);
    end
    weightsSha256=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
    assert(count==6467 && strcmp(weightsSha256,manifest.weightsSha256),'UA:WeightsMismatch');
    cfg.moduleRoot=fullfile(project,'neural_adp3d');cfg.projectRoot=project;
    s=apf_adp_config();s.modelFile=modelFile;
    runtime=ua_runtime_manifest();
    details=struct('controllerID',manifest.controllerID,'modelFile',modelFile, ...
        'modelSha256',manifest.modelSha256,'weightsSha256',weightsSha256,'parameterCount',count, ...
        'trainingBindingVerified',true,'inferenceInterfaceVerified',true, ...
        'cpuOnly',true,'actorLoaded',true,'criticLoaded',false,'agentLoaded',false, ...
        'trainingPerformed',false,'legacyADPCalls',0,'loadSeconds',loadSeconds, ...
        'runtimeSourceSha256',runtime.sha256);
end
