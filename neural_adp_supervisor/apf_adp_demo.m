function [automatic,baseline] = apf_adp_demo(caseId)
% Example: addpath('F:\UAV project\UAV_APF_ADP_Formation\neural_adp_supervisor'); apf_adp_demo
% Uses development scenes only. Does not train or edit the model/main formation controller.
    if nargin<1, caseId='approach_window_10001'; end
    s=apf_adp_config(); addpath(fullfile(s.projectRoot,'neural_adp3d'));
    cfg=adp3d_config('train'); p=adp3d_setup(cfg); %#ok<NASGU>
    model=load(s.modelFile,'actorNet','metadata');
    assert(isequaln(model.metadata.sourceManifest,adp3d_source_manifest(cfg)));
    cases=apf_adp_cases('development',cfg,s);
    match=find(cellfun(@(x)strcmp(x.id,caseId),cases));
    assert(numel(match)==1,'APFADP:UnknownDemo','Use a declared development case ID.');
    item=cases{match}; outputRoot=fullfile(s.projectRoot,'results','neural_adp_supervisor');
    if ~isfolder(outputRoot), mkdir(outputRoot); end
    outputDirectory=[tempname(outputRoot) '_demo']; mkdir(outputDirectory);
    [baseline,~]=apf_adp_rollout(item,'apf',model.actorNet,cfg,s);
    [automatic,~]=apf_adp_rollout(item,'automatic',model.actorNet,cfg,s);
    save(fullfile(outputDirectory,'demo.mat'),'automatic','baseline','cfg','s','-v7');
    apf_adp_plot(automatic,baseline,fullfile(outputDirectory,'automatic_flow'));
    disp(automatic.summary);
    fprintf('Results saved in %s\n',outputDirectory);
end
