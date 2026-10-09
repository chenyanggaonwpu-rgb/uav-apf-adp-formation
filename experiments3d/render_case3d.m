function outputDir=render_case3d(studyDir,trialId,outputDir)
% RENDER_CASE3D Rebuild a two-panel evidence set from one frozen trial MAT.
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path;
    cleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'src3d'));
    source=fullfile(studyDir,'trials',[char(trialId) '.mat']);
    if ~isfile(source), error('UAV3D:MissingCase','Missing trial: %s',source); end
    loaded=load(source,'r');
    if nargin<3 || isempty(outputDir)
        outputDir=fullfile(studyDir,'figures',char(trialId));
    end
    if ~isfolder(outputDir), mkdir(outputDir); end
    plot_result3d(loaded.r,outputDir);
end
