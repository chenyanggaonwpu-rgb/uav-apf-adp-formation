function project=ua_setup()
% Install only the known-map controller dependencies; never train at runtime.
    project=fileparts(fileparts(mfilename('fullpath')));
    folders={'config','src','src3d','neural_adp3d','neural_adp_supervisor', ...
        'five_uav_actor3d','five_uav_comparison3d','unified_actor3d'};
    for k=1:numel(folders),addpath(fullfile(project,folders{k}));end
end
