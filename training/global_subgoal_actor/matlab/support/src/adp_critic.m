function J = adp_critic(phi, params)
% ADP Critic 网络：估计当前状态对应的长期代价 J
    J = params.adp.wC' * phi;         % Critic 权重向量与状态特征做内积
end                                   % 结束 adp_critic 函数
