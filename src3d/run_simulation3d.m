function [result,plannerCache]=run_simulation3d(p,scene,plannerCache)
% The production known-map entry always uses the shared frozen Actor.
% Actor-off/rule are explicit experimental controls, selected in p.controller.
    project=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(project,'unified_actor3d'));ua_setup();
    if nargin<1 || isempty(p),p=default_params3d();end
    if nargin<2 || isempty(scene),scene=scenario3d(p.sceneId,p.seed,p);end
    if nargin<3,plannerCache=[];end
    method='actor';routePolicy='global';
    if isfield(p,'controller')
        assert(strcmp(p.controller.kind,'unified_frozen_actor'),'UA:UnsupportedController');
        if isfield(p.controller,'method'),method=p.controller.method;end
        if isfield(p.controller,'routePolicy'),routePolicy=p.controller.routePolicy;end
    end
    assert(any(strcmp(method,{'actor','none','rule'})),'UA:UnknownMethod');
    p.adp.enable=false;p.adp.wA=zeros(3,6);p.adp.wC=zeros(6,1);
    p.version='2.3.4-unified-actor-20261029';
    p.controller=struct('kind','unified_frozen_actor','method',method,'routePolicy',routePolicy);
    persistent actor cfg settings details
    if isempty(actor),[actor,cfg,settings,details]=ua_load_actor();end
    [result,plannerCache]=ua_simulate(p,scene,plannerCache,method,actor,cfg,settings,routePolicy);
    result.model=details;result.model.actorEnabled=strcmp(method,'actor');
    result.model.actorCalls=result.controller.actorCalls;
    result.model.legacyADPCalls=0;
end
