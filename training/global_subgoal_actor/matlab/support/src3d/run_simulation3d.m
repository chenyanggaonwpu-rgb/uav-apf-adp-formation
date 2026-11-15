function [result,plannerCache]=run_simulation3d(p,scene,plannerCache)
% The production known-map entry always uses the shared frozen Actor.
% Actor-off/rule are explicit experimental controls, selected in p.controller.
    project=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(project,'unified_actor3d'));ua_setup();
    if nargin<1 || isempty(p),p=default_params3d();end
    if nargin<2 || isempty(scene),scene=scenario3d(p.sceneId,p.seed,p);end
    if nargin<3,plannerCache=[];end
    method='actor';routePolicy='global';elapsedTakeover=[];
    if isfield(p,'controller')
        assert(strcmp(p.controller.kind,'unified_frozen_actor'),'UA:UnsupportedController');
        if isfield(p.controller,'method'),method=p.controller.method;end
        if isfield(p.controller,'routePolicy'),routePolicy=p.controller.routePolicy;end
        if isfield(p.controller,'elapsedTakeover'),elapsedTakeover=p.controller.elapsedTakeover;end
    end
    assert(any(strcmp(method,{'actor','none','rule'})),'UA:UnknownMethod');
    p.adp.enable=false;p.adp.wA=zeros(3,6);p.adp.wC=zeros(6,1);
    p.controller=struct('kind','unified_frozen_actor','method',method,'routePolicy',routePolicy);
    persistent actor cfg settings details
    if isempty(actor),[actor,cfg,settings,details]=ua_load_actor();end
    p.version=details.controllerID;
    runtimeSettings=settings;
    if ~isempty(elapsedTakeover)
        assert(isstruct(elapsedTakeover) && isscalar(elapsedTakeover),'UA:InvalidElapsedTakeover');
        assert(all(isfield(elapsedTakeover,{'enable','afterSeconds','maximumSeconds'})),'UA:IncompleteElapsedTakeover');
        validateattributes(elapsedTakeover.enable,{'logical'},{'scalar'});
        validateattributes(elapsedTakeover.afterSeconds,{'numeric'},{'scalar','finite','nonnegative','<',p.Tmax});
        validateattributes(elapsedTakeover.maximumSeconds,{'numeric'},{'scalar','finite','positive','<=',settings.maximumActiveSeconds});
        runtimeSettings.elapsedTakeover=elapsedTakeover;
        p.controller.elapsedTakeover=elapsedTakeover;
    end
    [result,plannerCache]=ua_simulate(p,scene,plannerCache,method,actor,cfg,runtimeSettings,routePolicy);
    result.elapsedTakeoverSettings=runtimeSettings.elapsedTakeover;
    result.model=details;result.model.actorEnabled=strcmp(method,'actor');
    result.model.actorCalls=result.controller.actorCalls;
    result.model.legacyADPCalls=0;
end
