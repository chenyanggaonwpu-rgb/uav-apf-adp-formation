function result=main3d(p)
% MAIN3D Run the adaptive finite-layer 3-D demonstration and save all evidence.
    projectRoot=fileparts(mfilename('fullpath'));
    oldPath=path;
    cleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(projectRoot,'config'),fullfile(projectRoot,'src'), ...
        fullfile(projectRoot,'src3d'));
    if nargin<1, p=default_params3d(); end
    p=validate_params3d(p);
    scene=scenario3d(p.sceneId,p.seed,p);
    outputDir=create_run_output_dir(p.resultsRoot,'3d_run',p.version);
    if isfield(scene,'generationAudit'), save_scene_generation3d(scene,outputDir); end
    result=run_simulation3d(p,scene);
    save(fullfile(outputDir,'simulation_result.mat'),'result','-v7.3');
    if isfield(result.history,'t') && ~isempty(result.history.t)
        plot_result3d(result,outputDir);
        writematrix(result.history.cycleMs,fullfile(outputDir,'cycle_ms.csv'));
        if isfield(p,'animate') && p.animate
            render_3d_animation(result,fullfile(outputDir,'flight_3d.mp4'));
        end
    end
    fprintf('3-D scene=%s result=%s time=%.2f s output=%s\n', ...
        scene.id,result.reason,result.time,outputDir);
end
