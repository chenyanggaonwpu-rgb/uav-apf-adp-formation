function weights = adp3d_agent_weights(agent)
% 诊断用：读取真实 agent 内部参数，确认训练确实改变了网络。
    weights.actor=flatten(getModel(getActor(agent)));
    critics=getCritic(agent);
    weights.critic1=flatten(getModel(critics(1)));
    weights.critic2=flatten(getModel(critics(2)));
end

function result=flatten(net)
    result=[];
    for k=1:height(net.Learnables)
        value=gather(extractdata(net.Learnables.Value{k}));
        result=[result;double(value(:))]; %#ok<AGROW>
    end
end
