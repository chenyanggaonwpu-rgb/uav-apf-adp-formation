function render_study_evidence(studyDir)
% RENDER_STUDY_EVIDENCE Rebuild report figures from frozen trial results.
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path;
    cleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'src3d'),fullfile(root,'experiments3d'), ...
        fullfile(root,'docs','three_dimensional'));
    output=fullfile(root,'docs','three_dimensional');
    draw_flow3d(output);
    render_case3d(studyDir,'C_S3_overpass',fullfile(output,'assets','S3'));
    render_case3d(studyDir,'C_random_018',fullfile(output,'assets','random18'));
    plot_tangent_probe3d(fullfile(output,'assets','apf_probe'));
    loaded=load(fullfile(studyDir,'trials','C_S3_overpass.mat'),'r');
    info=loaded.r.plan;
    fprintf('S3 finite layer graph: capacity=%d available=%d edges=%d, heightSpan=%.3f m.\n', ...
        info.capacity,info.availableNodes,info.edges, ...
        loaded.r.metrics.leaderHeightSpan);
end
