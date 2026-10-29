function action = adp3d_actor_action(actorNet,observation)
% 冻结网络前向推理，不反向传播、不更新权重。
    validateattributes(observation,{'numeric'},{'size',[32 1],'finite','real'});
    action=double(gather(extractdata(predict(actorNet,dlarray(single(observation),'CB')))));
    assert(isequal(size(action),[3 1]) && all(isfinite(action)));
end
