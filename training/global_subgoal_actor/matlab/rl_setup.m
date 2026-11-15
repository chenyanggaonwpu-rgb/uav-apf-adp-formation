function rl_setup()
    if isdeployed,return;end
    root=fileparts(mfilename('fullpath'));
    folders={'config','src','src3d','neural_adp3d','neural_adp_supervisor','five_uav_actor3d','five_uav_comparison3d','unified_actor3d'};
    for k=1:numel(folders),addpath(fullfile(root,'support',folders{k}));end
    addpath(root);
end
