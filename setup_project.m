function cleanup = setup_project()
% SETUP_PROJECT 临时把 src 和 config 加入 MATLAB 路径。
% 用法：cleanup = setup_project(); 保留 cleanup 变量直到工作结束。

    projectRoot = fileparts(mfilename('fullpath'));
    oldPath = path;
    addpath(fullfile(projectRoot, 'src'), fullfile(projectRoot, 'config'), ...
        fullfile(projectRoot, 'tests'));
    cleanup = onCleanup(@() path(oldPath));
end
