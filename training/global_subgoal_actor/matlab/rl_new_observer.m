function observer=rl_new_observer(cfg,p)
    options=struct('training',false,'teacher_probability',0,'stochastic',false, ...
        'seed',p.stochasticSeed+79001,'ray_noise_std',0,'ray_dropout_probability',0);
    if isfield(p,'rlRayNoise'),options.ray_noise_std=p.rlRayNoise;options.ray_dropout_probability=p.rlRayDropout;end
    observer=RLPolicy([],options,cfg);
end
