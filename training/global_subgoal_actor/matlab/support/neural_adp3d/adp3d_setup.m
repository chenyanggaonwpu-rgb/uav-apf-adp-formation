function p = adp3d_setup(cfg)
% 在当前 MATLAB 会话中加载原工程；云端可使用随包保存的源代码快照。
    assert(isfile(fullfile(cfg.projectRoot,'config','default_params3d.m')), ...
        'ADP3D:SourceMissing','找不到三维原工程或 runtime_source 快照。');
    addpath(fullfile(cfg.projectRoot,'src'),fullfile(cfg.projectRoot,'src3d'), ...
        fullfile(cfg.projectRoot,'config'));
    assert(strcmpi(which('motion_step3d'),fullfile(cfg.projectRoot,'src3d','motion_step3d.m')), ...
        'ADP3D:WrongSource','motion_step3d 路径与本次指定源代码不一致。');
    p=default_params3d();
    p.nUAV=1;
    p.adp.enable=false;
    assert(p.dt==0.05 && p.noiseStd==0 && p.disturbanceMax==0, ...
        'ADP3D:UnsupportedPhysics','第 1 版仅支持已核验的无扰动静态模型。');
    assert(cfg.observationSize==32 && cfg.actionSize==3);
    assert(cfg.physicsStepsPerDecision>=1 && mod(cfg.physicsStepsPerDecision,1)==0);
end
