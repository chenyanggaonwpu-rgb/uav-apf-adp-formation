function build_worker()
    rl_setup();root=fileparts(fileparts(mfilename('fullpath')));destination=fullfile(root,'worker_windows');
    if ~isfolder(destination),mkdir(destination);end
    mcc('-m',fullfile(root,'matlab','rl_worker.m'),'-d',destination,'-o','uav_rl_worker','-v','-R','-singleCompThread');
    fprintf('PORTABLE_RL_WORKER_COMPILE_COMPLETE\n');
end
