function [actor,cfg,s,details]=rtadp_load_actor(project)
% All current CPU, five-UAV and known-map entries load the same audited asset.
    addpath(fullfile(project,'unified_actor3d'));
    assert(strcmpi(project,ua_setup()),'UA:ProjectRootMismatch');
    [actor,cfg,s,details]=ua_load_actor();
end
